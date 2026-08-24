#!/usr/bin/env python3
"""Boat-only forward_ratio distribution + distance-bucketed TP loss, for
deciding whether a forward_ratio reject threshold is a clean win or a
range-tradeoff. Reuses the same matching logic as forward_fp_analyzer.py."""

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


BOAT_LABELS = {"boat", "boat_fallback"}
RATIO_BUCKETS = [
    (0.0, 0.1), (0.1, 0.3), (0.3, 0.5),
    (0.5, 0.7), (0.7, 0.9), (0.9, 1.0001),
]
DIST_BUCKETS = [
    (0.0, 20.0), (20.0, 40.0), (40.0, 60.0), (60.0, 1e9),
]


def bucket_of(value, buckets):
    for lo, hi in buckets:
        if lo <= value < hi:
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

    clusters_by_stamp = {}
    for cluster in records["cluster"]:
        if cluster.get("forward_ratio") is None:
            continue
        if cluster["label"] not in BOAT_LABELS:
            continue
        stamp = round(cluster["t"] - args.epoch_offset, 6)
        clusters_by_stamp.setdefault(stamp, []).append(cluster)

    interpolate_odom = evaluator.make_odom_interpolator(records["odom"])
    interpolate_gt = evaluator.make_gt_interpolator(records["gt"])
    detection_frames, _, _ = evaluator.deduplicate_frames(records["det"])

    tp_ratio_dist = {b: 0 for b in [f"{lo:g}-{hi:g}" for lo, hi in RATIO_BUCKETS]}
    fp_ratio_dist = dict(tp_ratio_dist)
    tp_lost_dist_dist = {b: 0 for b in [
        f"{lo:g}-{hi:g}" if hi < 1e8 else f"{lo:g}+" for lo, hi in DIST_BUCKETS]}
    tp_total_dist_dist = dict(tp_lost_dist_dist)
    fp_lost_dist_dist = dict(tp_lost_dist_dist)
    fp_total_dist_dist = dict(tp_lost_dist_dist)
    fp_unlinked = 0
    fp_linked_forward_dominant = 0
    fp_linked_total = 0

    for frame in detection_frames:
        odom = interpolate_odom(frame["stamp"])
        gt = interpolate_gt(frame["stamp"])
        if odom is None or gt is None:
            continue
        detections = [(pose["x"], pose["y"]) for pose in frame["poses"]]
        targets = [evaluator.world_to_boat(pose, odom) for pose in gt["poses"]]
        pairs, _ = evaluator.maximum_cardinality_matches(detections, targets, args.match_tol)
        matched_detection_indices = {i for i, _ in pairs}

        available = list(clusters_by_stamp.get(round(frame["stamp"], 6), ()))
        used = set()
        for detection_index, detection in enumerate(detections):
            options = []
            for cluster_index, cluster in enumerate(available):
                if cluster_index in used:
                    continue
                distance = math.hypot(detection[0] - cluster["x"], detection[1] - cluster["y"])
                if distance <= args.cluster_match_tol:
                    options.append((distance, cluster_index))
            is_tp = detection_index in matched_detection_indices
            if not options:
                if not is_tp:
                    fp_unlinked += 1
                continue
            _, cluster_index = min(options)
            used.add(cluster_index)
            cluster = available[cluster_index]
            ratio = cluster["forward_ratio"]
            dist_from_origin = math.hypot(cluster["x"], cluster["y"])
            rb = bucket_of(ratio, RATIO_BUCKETS)
            if is_tp:
                if rb in tp_ratio_dist:
                    tp_ratio_dist[rb] += 1
                db = bucket_of(dist_from_origin, DIST_BUCKETS)
                if db in tp_total_dist_dist:
                    tp_total_dist_dist[db] += 1
                    if ratio >= 0.5:
                        tp_lost_dist_dist[db] += 1
            else:
                if rb in fp_ratio_dist:
                    fp_ratio_dist[rb] += 1
                fp_linked_total += 1
                db = bucket_of(dist_from_origin, DIST_BUCKETS)
                if db in fp_total_dist_dist:
                    fp_total_dist_dist[db] += 1
                if ratio >= 0.5:
                    fp_linked_forward_dominant += 1
                    if db in fp_lost_dist_dist:
                        fp_lost_dist_dist[db] += 1

    print("=== boat forward_ratio distribution (linked TP/FP) ===")
    print(f"{'bucket':>10} {'TP':>8} {'FP':>8} {'true:false':>12}")
    for lo, hi in RATIO_BUCKETS:
        b = f"{lo:g}-{hi:g}"
        tp = tp_ratio_dist[b]
        fp = fp_ratio_dist[b]
        ratio_str = f"{tp}:{fp}" if (tp or fp) else "-"
        print(f"{b:>10} {tp:>8} {fp:>8} {ratio_str:>12}")

    print()
    print("=== boat FP composition (linked only) ===")
    print(f"forward-dominant (ratio>=0.5) FP: {fp_linked_forward_dominant}/{fp_linked_total} "
          f"({100.0*fp_linked_forward_dominant/fp_linked_total:.1f}%)" if fp_linked_total else "n/a")
    print(f"unlinked FP (no diagnostic match, ratio unknown, always kept): {fp_unlinked}")

    print()
    print("=== TP/FP loss by distance bucket, if rejecting ratio>=0.5 ===")
    print(f"{'range':>10} {'TP total':>10} {'TP lost':>10} {'TP lost%':>9} "
          f"{'FP total':>10} {'FP removed':>11} {'FP removed%':>12}")
    for lo, hi in DIST_BUCKETS:
        b = f"{lo:g}-{hi:g}" if hi < 1e8 else f"{lo:g}+"
        tp_total = tp_total_dist_dist[b]
        tp_lost = tp_lost_dist_dist[b]
        tp_pct = f"{100.0*tp_lost/tp_total:.1f}%" if tp_total else "-"
        fp_total = fp_total_dist_dist[b]
        fp_lost = fp_lost_dist_dist[b]
        fp_pct = f"{100.0*fp_lost/fp_total:.1f}%" if fp_total else "-"
        print(f"{b:>10} {tp_total:>10} {tp_lost:>10} {tp_pct:>9} "
              f"{fp_total:>10} {fp_lost:>11} {fp_pct:>12}")


if __name__ == "__main__":
    main()
