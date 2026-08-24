#!/usr/bin/env python3
"""Summarize block ground-truth range in the boat coordinate frame."""

import argparse
import importlib.util
import statistics


def load_evaluator(path):
    spec = importlib.util.spec_from_file_location("evaluator_v2", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def percentile(sorted_values, fraction):
    if not sorted_values:
        return 0.0
    position = fraction * (len(sorted_values) - 1)
    lower = int(position)
    upper = min(lower + 1, len(sorted_values) - 1)
    alpha = position - lower
    return (
        sorted_values[lower] * (1.0 - alpha)
        + sorted_values[upper] * alpha
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("log_path")
    parser.add_argument("--evaluator", required=True)
    parser.add_argument("--epoch-offset", type=float, required=True)
    args = parser.parse_args()

    evaluator = load_evaluator(args.evaluator)
    records = evaluator.load_records(args.log_path)
    odom_mid = records["odom"][len(records["odom"]) // 2]["stamp"]
    evaluator.align_to_odom(
        records["gt_block"], args.epoch_offset, odom_mid
    )
    interpolate_odom = evaluator.make_odom_interpolator(records["odom"])

    all_distances = []
    distances_by_index = {}
    covered_frames = 0
    skipped_frames = 0
    counts_per_frame = []

    for frame in records["gt_block"]:
        odom = interpolate_odom(frame["stamp"])
        if odom is None:
            skipped_frames += 1
            continue
        covered_frames += 1
        frame_distances = []
        for index, pose in enumerate(frame["poses"]):
            x, y = evaluator.world_to_boat(pose, odom)
            distance = (x * x + y * y) ** 0.5
            all_distances.append(distance)
            frame_distances.append(distance)
            distances_by_index.setdefault(index, []).append(distance)
        counts_per_frame.append(len(frame_distances))

    all_distances.sort()
    print(
        f"frames covered={covered_frames} skipped={skipped_frames} "
        f"block_observations={len(all_distances)} "
        f"blocks_per_frame_median={statistics.median(counts_per_frame):.0f}"
    )
    print(
        "distance_m "
        f"min={min(all_distances):.2f} "
        f"p25={percentile(all_distances, 0.25):.2f} "
        f"median={statistics.median(all_distances):.2f} "
        f"p75={percentile(all_distances, 0.75):.2f} "
        f"max={max(all_distances):.2f}"
    )

    bucket_edges = (20.0, 40.0, 60.0)
    bucket_counts = [0, 0, 0, 0]
    for distance in all_distances:
        if distance < bucket_edges[0]:
            bucket_counts[0] += 1
        elif distance < bucket_edges[1]:
            bucket_counts[1] += 1
        elif distance < bucket_edges[2]:
            bucket_counts[2] += 1
        else:
            bucket_counts[3] += 1
    labels = ("[0,20)", "[20,40)", "[40,60)", "[60,+inf)")
    for label, count in zip(labels, bucket_counts):
        print(
            f"bucket {label}m: {count}/{len(all_distances)} "
            f"({100.0 * count / len(all_distances):.1f}%)"
        )

    print("per-index median range (GT ordering, not identity-guaranteed):")
    for index, distances in sorted(distances_by_index.items()):
        print(
            f"  index={index}: samples={len(distances)} "
            f"min={min(distances):.2f} "
            f"median={statistics.median(distances):.2f} "
            f"max={max(distances):.2f}"
        )


if __name__ == "__main__":
    main()
