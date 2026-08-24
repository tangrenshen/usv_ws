#!/usr/bin/env python3
"""Yaw scan variant of block_cluster_offset_analyzer.py that uses Airy-dominant
clusters as the pseudo-reference instead of gt_block, since the competition has
no ground truth. Both airy and forward clusters are already in base_link
(no odom/world_to_boat needed) -- this mirrors what would actually be available
online: only clusters with a forward_ratio field.

Per frame (grouped by measurement_t), split clusters into:
  - airy reference: forward_ratio <= --airy-max (dominated by the six Airy lidars)
  - forward candidates: forward_ratio >= --forward-min (dominated by forward_lidar)
then scan yaw in [-12, +12]deg, rotate the forward candidates, and count how many
match an airy reference cluster within --scan-gate using one-to-one max matching.
"""

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
    parser.add_argument("--airy-max", type=float, default=0.1)
    parser.add_argument("--forward-min", type=float, default=0.5)
    parser.add_argument("--scan-gate", type=float, default=2.0)
    parser.add_argument("--scan-range-deg", type=float, default=12.0)
    parser.add_argument("--min-range-m", type=float, default=0.0)
    parser.add_argument("--max-range-m", type=float, default=1e9)
    args = parser.parse_args()

    evaluator = load_evaluator(args.evaluator)
    records = evaluator.load_records(args.log_path)

    frames = collections.defaultdict(lambda: ([], []))
    for cluster in records["cluster"]:
        ratio = cluster.get("forward_ratio")
        stamp = cluster.get("measurement_t")
        if ratio is None or stamp is None:
            continue
        r = math.hypot(cluster["x"], cluster["y"])
        if r < args.min_range_m or r > args.max_range_m:
            continue
        key = round(stamp, 3)
        airy_list, fwd_list = frames[key]
        if ratio <= args.airy_max:
            airy_list.append(cluster)
        elif ratio >= args.forward_min:
            fwd_list.append(cluster)

    scan_frames = []
    airy_total = 0
    fwd_total = 0
    usable_frames = 0
    for stamp, (airy_list, fwd_list) in sorted(frames.items()):
        if not airy_list or not fwd_list:
            continue
        usable_frames += 1
        airy_total += len(airy_list)
        fwd_total += len(fwd_list)
        airy_pts = [(c["x"], c["y"]) for c in airy_list]
        fwd_pts = [(c["x"], c["y"]) for c in fwd_list]
        scan_frames.append((airy_pts, fwd_pts))

    print(
        f"usable_frames(both sides present)={usable_frames} "
        f"airy_clusters_total={airy_total} forward_clusters_total={fwd_total}"
    )
    if usable_frames == 0:
        print("no usable frames -- cannot scan")
        return

    per_frame_pairs = [len(a) for a, f in scan_frames]
    print(
        f"airy clusters per usable frame: median={statistics.median(per_frame_pairs):.1f} "
        f"min={min(per_frame_pairs)} max={max(per_frame_pairs)}"
    )

    scan = []
    angle = -args.scan_range_deg
    while angle <= args.scan_range_deg + 0.0001:
        matches = 0
        residuals = []
        for airy_pts, fwd_pts in scan_frames:
            rotated = [rotate(p, angle) for p in fwd_pts]
            pairs, _ = evaluator.maximum_cardinality_matches(
                rotated, airy_pts, args.scan_gate
            )
            matches += len(pairs)
            residuals.extend(
                math.hypot(rotated[left][0] - airy_pts[right][0],
                           rotated[left][1] - airy_pts[right][1])
                for left, right in pairs
            )
        scan.append(
            (matches, -statistics.mean(residuals) if residuals else -999.0, angle)
        )
        angle += 0.1

    best_matches, negative_mean, best_angle = max(scan)
    zero = min(scan, key=lambda item: abs(item[2]))
    total_fwd = sum(len(f) for _, f in scan_frames)
    print(
        f"yaw scan [-{args.scan_range_deg:.0f},+{args.scan_range_deg:.0f}]deg "
        f"gate={args.scan_gate:.1f}m (airy-as-reference): "
        f"best={best_angle:+.1f}deg matches={best_matches}/{total_fwd} "
        f"mean_residual={-negative_mean:.2f}m; "
        f"at_0deg={zero[0]}/{total_fwd}"
    )


if __name__ == "__main__":
    main()
