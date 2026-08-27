#!/usr/bin/env python3
"""FP attribution for all four categories: for each category's unmatched
(false-positive) detections, checks whether they land near a GT target of
a DIFFERENT category (cross-category contamination, same pattern as the
pillar_max_pts precedent) or match nothing (pure noise)."""

import argparse
import importlib.util
import math
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


DIST_BUCKETS = [(0.0, 20.0), (20.0, 40.0), (40.0, 60.0), (60.0, 1e9)]


def dist_bucket(d):
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
    gt_interp = {
        "boat": evaluator.make_gt_interpolator(records["gt"]),
        "buoy": evaluator.make_gt_interpolator(records["gt_buoy"]),
        "pillar": evaluator.make_gt_interpolator(records["gt_pillar"]),
        "block": evaluator.make_gt_interpolator(records["gt_block"]),
    }

    for name, gt_key, detection_key in evaluator.CATEGORIES:
        interpolate_gt = gt_interp[name]
        detection_frames, _, _ = evaluator.deduplicate_frames(records[detection_key])

        fp_total = 0
        match_counts = {"boat": 0, "buoy": 0, "pillar": 0, "block": 0, "none": 0}
        noise_dist_buckets = defaultdict(int)
        # also track buoy-GT proximity for block-category cross-matches
        buoy_neighbor_counts = []  # (num buoys within 1.4m of this block-FP)

        for frame in detection_frames:
            odom = interpolate_odom(frame["stamp"])
            gt = interpolate_gt(frame["stamp"])
            if odom is None or gt is None:
                continue
            detections = [(pose["x"], pose["y"]) for pose in frame["poses"]]
            targets = [evaluator.world_to_boat(pose, odom) for pose in gt["poses"]]
            pairs, _ = evaluator.maximum_cardinality_matches(detections, targets, args.match_tol)
            matched_indices = {i for i, _ in pairs}

            for di, det in enumerate(detections):
                if di in matched_indices:
                    continue  # this one is a TP for its own category
                fp_total += 1
                matched_cat = None
                for cat, interp in gt_interp.items():
                    if cat == name:
                        continue  # already know it doesn't match its own GT
                    other_gt = interp(frame["stamp"])
                    if other_gt is None:
                        continue
                    other_targets = [evaluator.world_to_boat(p, odom) for p in other_gt["poses"]]
                    best = min((math.hypot(det[0] - tx, det[1] - ty) for tx, ty in other_targets),
                               default=1e9)
                    if best <= args.match_tol:
                        matched_cat = cat
                        break
                match_counts[matched_cat if matched_cat else "none"] += 1

                if matched_cat is None:
                    d = math.hypot(det[0], det[1])
                    noise_dist_buckets[dist_bucket(d)] += 1

                if name == "block" and matched_cat == "buoy":
                    buoy_gt = gt_interp["buoy"](frame["stamp"])
                    if buoy_gt is not None:
                        buoy_targets = [evaluator.world_to_boat(p, odom) for p in buoy_gt["poses"]]
                        nearby = sum(1 for tx, ty in buoy_targets
                                     if math.hypot(det[0] - tx, det[1] - ty) <= 1.4)
                        buoy_neighbor_counts.append(nearby)

        print(f"=== {name}: FP total={fp_total} ===")
        for cat in ("boat", "buoy", "pillar", "block", "none"):
            n = match_counts[cat]
            pct = 100.0 * n / fp_total if fp_total else 0.0
            label = "pure noise" if cat == "none" else f"matches {cat} GT"
            print(f"  {label:<16}: {n:>6} ({pct:.1f}%)")

        noise_total = sum(noise_dist_buckets.values())
        if noise_total:
            print("  pure-noise FP distance distribution:")
            for lo, hi in DIST_BUCKETS:
                b = f"{lo:g}-{hi:g}" if hi < 1e8 else f"{lo:g}+"
                n = noise_dist_buckets[b]
                print(f"    {b:>8}: {n:>6} ({100.0*n/noise_total:.1f}%)")

        if buoy_neighbor_counts:
            zero = sum(1 for n in buoy_neighbor_counts if n == 0)
            one = sum(1 for n in buoy_neighbor_counts if n == 1)
            multi = sum(1 for n in buoy_neighbor_counts if n >= 2)
            total = len(buoy_neighbor_counts)
            print(f"  block-FP-matching-buoy: buoys within 1.4m of the FP position "
                  f"(n={total}): 0={zero}({100.0*zero/total:.1f}%) "
                  f"1={one}({100.0*one/total:.1f}%) 2+={multi}({100.0*multi/total:.1f}%)")
        print()


if __name__ == "__main__":
    main()
