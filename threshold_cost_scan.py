#!/usr/bin/env python3
"""Global cost evaluation for two threshold-loosening candidates found by
forward_predicate_diagnosis.py: boat_fp_max_sane (near/far separately) and
block_fp_min. For each candidate value, finds clusters (full population,
not forward-filtered) that fail the CURRENT threshold on the one axis being
scanned but pass it at the candidate value, with all other predicates for
that category already satisfied - then classifies each such newly-included
cluster against GT (boat/buoy/pillar/block) via one-to-one matching, and
separately reports what the cluster's OWN current label already is (to
catch cross-category pre-emption, especially relevant for block_fp_min
since block is classified first in the if-else chain)."""

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
    return parser.parse_args()


def boat_other_predicates_ok(c):
    return 0.3 <= c["dz"] <= 1.8 and c["pts"] >= 40


def block_other_predicates_ok(c):
    return (c["fp_max"] <= 3.2 and c["dz"] <= 1.5 and
            c["square"] >= 0.55 and c["pts"] >= 10 and abs(c["z"]) <= 1.0)


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

    # all clusters, any label, grouped by rounded stamp
    clusters_by_stamp = {}
    for cluster in records["cluster"]:
        stamp = round(cluster["t"] - args.epoch_offset, 6)
        clusters_by_stamp.setdefault(stamp, []).append(cluster)

    detection_frames, _, _ = evaluator.deduplicate_frames(records["det"])

    # labels already claimed by a branch checked BEFORE boat in the if-else
    # chain (block -> buoy(_fallback) -> pillar -> boat). A cluster already
    # wearing one of these labels is unaffected by changing boat's
    # threshold - it was never going to reach the boat branch regardless.
    PRE_BOAT_LABELS = {"block", "buoy", "buoy_fallback", "pillar"}

    def classify_new_clusters(select_fn, exclude_preempted_labels=None):
        """select_fn(cluster) -> True if this cluster is a newly-included
        candidate at the scanned value. Returns (match_counts, label_counts,
        total, excluded_count)."""
        match_counts = {"boat": 0, "buoy": 0, "pillar": 0, "block": 0, "none": 0}
        label_counts = {}
        total = 0
        excluded = 0
        for frame in detection_frames:
            odom = interpolate_odom(frame["stamp"])
            if odom is None:
                continue
            available = clusters_by_stamp.get(round(frame["stamp"], 6), ())
            selected = []
            for c in available:
                if not select_fn(c):
                    continue
                if exclude_preempted_labels and c["label"] in exclude_preempted_labels:
                    excluded += 1
                    continue
                selected.append(c)
            if not selected:
                continue
            total += len(selected)
            for c in selected:
                label_counts[c["label"]] = label_counts.get(c["label"], 0) + 1
                matched_cat = None
                for cat, interp in gt_interp.items():
                    gt = interp(frame["stamp"])
                    if gt is None:
                        continue
                    targets = [evaluator.world_to_boat(pose, odom) for pose in gt["poses"]]
                    for tx, ty in targets:
                        if math.hypot(c["x"] - tx, c["y"] - ty) <= args.match_tol:
                            matched_cat = cat
                            break
                    if matched_cat:
                        break
                match_counts[matched_cat if matched_cat else "none"] += 1
        return match_counts, label_counts, total, excluded

    def print_result(title, match_counts, label_counts, total, excluded):
        print(f"  {title}: total_new(at-risk)={total} excluded(pre-empted by earlier branch)={excluded}")
        if total == 0:
            return
        for cat in ("boat", "buoy", "pillar", "block", "none"):
            n = match_counts[cat]
            print(f"    matches {cat:<8} = {n:>6} ({100.0*n/total:.1f}%)")
        top_labels = sorted(label_counts.items(), key=lambda kv: -kv[1])[:6]
        print(f"    current label breakdown (top): "
              + ", ".join(f"{l}={n}" for l, n in top_labels))

    print("=== candidate 1: boat_fp_max_sane ===")
    for scope, dist_ok in [
        ("near(<=20m)", lambda c: math.hypot(c["x"], c["y"]) <= 20.0),
        ("far(>20m)", lambda c: math.hypot(c["x"], c["y"]) > 20.0),
    ]:
        current = 2.5 if scope.startswith("near") else 2.0
        for new_val in (current + 0.5, current + 1.0):
            def sel(c, dist_ok=dist_ok, current=current, new_val=new_val):
                return (dist_ok(c) and boat_other_predicates_ok(c) and
                        current < c["fp_max"] <= new_val)
            mc, lc, tot, exc = classify_new_clusters(sel, PRE_BOAT_LABELS)
            print_result(f"{scope}: {current}->{new_val}", mc, lc, tot, exc)
    print()

    print("=== candidate 2: block_fp_min (block is checked FIRST, nothing to exclude) ===")
    for new_val in (1.5, 1.4, 1.3):
        def sel(c, new_val=new_val):
            return (block_other_predicates_ok(c) and new_val <= c["fp_max"] < 1.65)
        mc, lc, tot, exc = classify_new_clusters(sel)
        print_result(f"1.65->{new_val}", mc, lc, tot, exc)


if __name__ == "__main__":
    main()
