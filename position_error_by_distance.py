#!/usr/bin/env python3
"""Position error broken down by distance bucket, per category. Reuses the
evaluator's own one-to-one matching so the numbers are directly comparable
to the headline position-error figures already reported."""

import argparse
import importlib.util
import math
import statistics
from collections import defaultdict


def load_evaluator(path):
    spec = importlib.util.spec_from_file_location("evaluator_v2", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("log_path")
    parser.add_argument("--evaluator", required=True)
    parser.add_argument("--epoch-offset", type=float, required=True)
    parser.add_argument("--match-tol", type=float, default=2.0)
    return parser.parse_args()


DIST_BUCKETS = [(0.0, 10.0), (10.0, 20.0), (20.0, 40.0), (40.0, 60.0), (60.0, 1e9)]


def bucket_of(d):
    for lo, hi in DIST_BUCKETS:
        if lo <= d < hi:
            return f"{lo:g}-{hi:g}" if hi < 1e8 else f"{lo:g}+"
    return "?"


def main():
    args = parse_args()
    evaluator = load_evaluator(args.evaluator)
    records = evaluator.load_records(args.log_path)
    odom_mid = records["odom"][len(records["odom"]) // 2]["stamp"]
    for record_type, items in records.items():
        if record_type not in {"odom", "cluster"}:
            evaluator.align_to_odom(items, args.epoch_offset, odom_mid)

    interpolate_odom = evaluator.make_odom_interpolator(records["odom"])

    for name, gt_key, detection_key in evaluator.CATEGORIES:
        interpolate_gt = evaluator.make_gt_interpolator(records[gt_key])
        detection_frames, _, _ = evaluator.deduplicate_frames(records[detection_key])

        errors_by_bucket = defaultdict(list)
        sizes_by_bucket = defaultdict(list)  # (fp_max_est) if available - just report count for now

        for frame in detection_frames:
            odom = interpolate_odom(frame["stamp"])
            gt = interpolate_gt(frame["stamp"])
            if odom is None or gt is None:
                continue
            detections = [(pose["x"], pose["y"]) for pose in frame["poses"]]
            targets = [evaluator.world_to_boat(pose, odom) for pose in gt["poses"]]
            pairs, _ = evaluator.maximum_cardinality_matches(detections, targets, args.match_tol)
            for detection_index, target_index in pairs:
                dx, dy = detections[detection_index]
                tx, ty = targets[target_index]
                err = math.hypot(dx - tx, dy - ty)
                dist = math.hypot(tx, ty)
                errors_by_bucket[bucket_of(dist)].append(err)

        print(f"=== {name} ===")
        all_errs = []
        for lo, hi in DIST_BUCKETS:
            b = f"{lo:g}-{hi:g}" if hi < 1e8 else f"{lo:g}+"
            errs = errors_by_bucket[b]
            all_errs.extend(errs)
            if errs:
                print(f"  {b:>8}: n={len(errs):>5} mean={statistics.mean(errs):.3f}m "
                      f"median={statistics.median(errs):.3f}m p90={sorted(errs)[int(0.9*(len(errs)-1))]:.3f}m")
            else:
                print(f"  {b:>8}: n=0")
        if all_errs:
            print(f"  {'overall':>8}: n={len(all_errs):>5} mean={statistics.mean(all_errs):.3f}m "
                  f"median={statistics.median(all_errs):.3f}m")
        print()


if __name__ == "__main__":
    main()
