#!/usr/bin/env python3
"""forward_lidar single-sensor accuracy: for each category and distance
bucket, split candidates into forward-majority (ratio>=0.5) and
forward-pure (ratio==1.0, no Airy points at all) subsets, and report their
match rate against GT (TP/(TP+FP)). Captured before boat_far_forward_reject
existed, so labels reflect unfiltered original classification."""

import argparse
import importlib.util
import math


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
    parser.add_argument("--cluster-match-tol", type=float, default=0.05)
    return parser.parse_args()


CATEGORY_LABELS = {
    "boat": {"boat", "boat_fallback"},
    "buoy": {"buoy", "buoy_fallback"},
    "pillar": {"pillar"},
    "block": {"block"},
}

DIST_BUCKETS = [(0.0, 20.0), (20.0, 40.0), (40.0, 1e9)]


def dist_bucket(d):
    for lo, hi in DIST_BUCKETS:
        if lo <= d < hi:
            return f"{lo:g}-{hi:g}" if hi < 1e8 else f"{lo:g}+"
    return "?"


def ratio_tiers(ratio):
    tiers = []
    if ratio >= 0.5:
        tiers.append("majority(>=0.5)")
    if ratio >= 0.999:
        tiers.append("pure(==1.0)")
    return tiers


def main():
    args = parse_args()
    evaluator = load_evaluator(args.evaluator)
    records = evaluator.load_records(args.log_path)
    odom_mid = records["odom"][len(records["odom"]) // 2]["stamp"]
    for record_type, items in records.items():
        if record_type not in {"odom", "cluster"}:
            evaluator.align_to_odom(items, args.epoch_offset, odom_mid)

    clusters_by_stamp_cat = {}
    for cluster in records["cluster"]:
        if cluster.get("forward_ratio") is None:
            continue
        category = None
        for name, labels in CATEGORY_LABELS.items():
            if cluster["label"] in labels:
                category = name
                break
        if category is None:
            continue
        stamp = round(cluster["t"] - args.epoch_offset, 6)
        clusters_by_stamp_cat.setdefault((stamp, category), []).append(cluster)

    interpolate_odom = evaluator.make_odom_interpolator(records["odom"])

    # counts[category][tier][distbucket] = [tp, fp]
    counts = {
        cat: {tier: {b: [0, 0] for _, _, b in
                     [(lo, hi, (f"{lo:g}-{hi:g}" if hi < 1e8 else f"{lo:g}+"))
                      for lo, hi in DIST_BUCKETS]}
              for tier in ("majority(>=0.5)", "pure(==1.0)")}
        for cat in CATEGORY_LABELS
    }

    for name, gt_key, detection_key in evaluator.CATEGORIES:
        detection_frames, _, _ = evaluator.deduplicate_frames(records[detection_key])
        interpolate_gt = evaluator.make_gt_interpolator(records[gt_key])

        for frame in detection_frames:
            odom = interpolate_odom(frame["stamp"])
            gt = interpolate_gt(frame["stamp"])
            if odom is None or gt is None:
                continue
            detections = [(pose["x"], pose["y"]) for pose in frame["poses"]]
            targets = [evaluator.world_to_boat(pose, odom) for pose in gt["poses"]]
            pairs, _ = evaluator.maximum_cardinality_matches(detections, targets, args.match_tol)
            matched_detection_indices = {i for i, _ in pairs}

            available = list(clusters_by_stamp_cat.get((round(frame["stamp"], 6), name), ()))
            used = set()
            for detection_index, detection in enumerate(detections):
                options = []
                for cluster_index, cluster in enumerate(available):
                    if cluster_index in used:
                        continue
                    distance = math.hypot(detection[0] - cluster["x"], detection[1] - cluster["y"])
                    if distance <= args.cluster_match_tol:
                        options.append((distance, cluster_index))
                if not options:
                    continue
                _, cluster_index = min(options)
                used.add(cluster_index)
                cluster = available[cluster_index]
                ratio = cluster["forward_ratio"]
                dist_from_origin = math.hypot(cluster["x"], cluster["y"])
                db = dist_bucket(dist_from_origin)
                is_tp = detection_index in matched_detection_indices
                for tier in ratio_tiers(ratio):
                    if db in counts[name][tier]:
                        counts[name][tier][db][0 if is_tp else 1] += 1

    for cat in CATEGORY_LABELS:
        print(f"=== {cat} ===")
        for tier in ("majority(>=0.5)", "pure(==1.0)"):
            print(f"  {tier}:")
            for lo, hi in DIST_BUCKETS:
                b = f"{lo:g}-{hi:g}" if hi < 1e8 else f"{lo:g}+"
                tp, fp = counts[cat][tier][b]
                total = tp + fp
                acc = f"{100.0*tp/total:.1f}%" if total else "-"
                print(f"    {b:>8}: TP={tp:>6} FP={fp:>6} total={total:>6} accuracy={acc}")
        print()


if __name__ == "__main__":
    main()
