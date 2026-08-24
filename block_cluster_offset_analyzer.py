#!/usr/bin/env python3
"""Diagnose whether forward-lidar Block candidates exist but are yaw-shifted."""

import argparse
import collections
import importlib.util
import math
import statistics


def load_evaluator(path):
    spec = importlib.util.spec_from_file_location("evaluator_v2", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def percentile(values, fraction):
    values = sorted(values)
    if not values:
        return float("nan")
    position = fraction * (len(values) - 1)
    lower = int(position)
    upper = min(lower + 1, len(values) - 1)
    alpha = position - lower
    return values[lower] * (1.0 - alpha) + values[upper] * alpha


def describe(values):
    if not values:
        return "n=0"
    return (
        f"n={len(values)} min={min(values):.2f} "
        f"p25={percentile(values, 0.25):.2f} "
        f"median={statistics.median(values):.2f} "
        f"p75={percentile(values, 0.75):.2f} max={max(values):.2f}"
    )


def rotate(point, angle_deg):
    angle = math.radians(angle_deg)
    cosine, sine = math.cos(angle), math.sin(angle)
    return (
        cosine * point[0] - sine * point[1],
        sine * point[0] + cosine * point[1],
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("log_path")
    parser.add_argument("--evaluator", required=True)
    parser.add_argument("--epoch-offset", type=float, required=True)
    parser.add_argument("--frame-tol", type=float, default=0.06)
    parser.add_argument("--association-gate", type=float, default=8.0)
    parser.add_argument("--scan-gate", type=float, default=2.0)
    parser.add_argument("--forward-min", type=float, default=0.5)
    args = parser.parse_args()

    evaluator = load_evaluator(args.evaluator)
    records = evaluator.load_records(args.log_path)
    odom_mid = records["odom"][len(records["odom"]) // 2]["stamp"]
    evaluator.align_to_odom(records["gt_block"], args.epoch_offset, odom_mid)
    interpolate_odom = evaluator.make_odom_interpolator(records["odom"])
    interpolate_gt = evaluator.make_gt_interpolator(records["gt_block"])

    frames = collections.defaultdict(list)
    for cluster in records["cluster"]:
        ratio = cluster.get("forward_ratio")
        if ratio is None or ratio < args.forward_min:
            continue
        stamp = cluster.get("measurement_t")
        if stamp is None:
            stamp = cluster["t"] - args.epoch_offset
        frames[round(stamp, 3)].append(cluster)

    observations = []
    scan_frames = []
    label_counts = collections.Counter()
    target_count = 0
    frame_count = 0
    for stamp, clusters in sorted(frames.items()):
        odom = interpolate_odom(stamp)
        gt = interpolate_gt(stamp)
        if odom is None or gt is None:
            continue
        targets = [evaluator.world_to_boat(pose, odom) for pose in gt["poses"]]
        candidates = [(item["x"], item["y"]) for item in clusters]
        if not targets:
            continue
        frame_count += 1
        target_count += len(targets)
        scan_frames.append((targets, candidates))
        pairs, _ = evaluator.maximum_cardinality_matches(
            targets, candidates, args.association_gate
        )
        for target_index, candidate_index in pairs:
            target = targets[target_index]
            candidate = candidates[candidate_index]
            cluster = clusters[candidate_index]
            distance = math.hypot(
                candidate[0] - target[0], candidate[1] - target[1]
            )
            target_angle = math.atan2(target[1], target[0])
            candidate_angle = math.atan2(candidate[1], candidate[0])
            angle_error = math.degrees(
                math.atan2(
                    math.sin(candidate_angle - target_angle),
                    math.cos(candidate_angle - target_angle),
                )
            )
            radius = math.hypot(*target)
            radial_error = (
                math.hypot(*candidate) - radius
            )
            tangential_error = (
                -target[1] * (candidate[0] - target[0])
                + target[0] * (candidate[1] - target[1])
            ) / radius
            observations.append(
                (distance, angle_error, radial_error, tangential_error,
                 cluster["forward_ratio"])
            )
            label_counts[cluster["label"]] += 1

    distances = [item[0] for item in observations]
    angles = [item[1] for item in observations]
    radial = [item[2] for item in observations]
    tangential = [item[3] for item in observations]
    ratios = [item[4] for item in observations]
    print(
        f"frames={frame_count} block_observations={target_count} "
        f"forward_dominant_clusters={sum(len(x[1]) for x in scan_frames)} "
        f"associated_within_{args.association_gate:.0f}m={len(observations)} "
        f"({100.0 * len(observations) / target_count:.1f}%)"
    )
    for gate in (2.0, 5.0, 8.0):
        count = sum(value <= gate for value in distances)
        print(
            f"raw GT coverage <={gate:.0f}m: {count}/{target_count} "
            f"({100.0 * count / target_count:.1f}%)"
        )
    print(f"nearest/matched offset_m: {describe(distances)}")
    print(f"signed angular error_deg: {describe(angles)}")
    print(f"radial error_m: {describe(radial)}")
    print(f"tangential error_m: {describe(tangential)}")
    print(f"forward_ratio: {describe(ratios)}")
    print("matched labels:", dict(label_counts.most_common()))

    scan = []
    angle = -12.0
    while angle <= 12.0001:
        matches = 0
        residuals = []
        for targets, candidates in scan_frames:
            rotated = [rotate(target, angle) for target in targets]
            pairs, _ = evaluator.maximum_cardinality_matches(
                rotated, candidates, args.scan_gate
            )
            matches += len(pairs)
            residuals.extend(
                math.hypot(
                    rotated[left][0] - candidates[right][0],
                    rotated[left][1] - candidates[right][1],
                )
                for left, right in pairs
            )
        scan.append(
            (matches, -statistics.mean(residuals) if residuals else -999.0, angle)
        )
        angle += 0.1
    best_matches, negative_mean, best_angle = max(scan)
    zero = min(scan, key=lambda item: abs(item[2]))
    print(
        f"yaw scan [-12,+12]deg gate={args.scan_gate:.1f}m: "
        f"best={best_angle:+.1f}deg matches={best_matches}/{target_count} "
        f"mean_residual={-negative_mean:.2f}m; "
        f"at_0deg={zero[0]}/{target_count}"
    )


if __name__ == "__main__":
    main()
