#!/usr/bin/env python3
"""boat/block predicate failure diagnosis, buoy as control.

For forward-majority (ratio>=0.5) clusters, one-to-one match them against
GT targets of the category (same maximum_cardinality_matches methodology
the evaluator uses for published detections, applied here to raw candidate
clusters pre-classification) to get a genuine "this cluster represents
this real target" label - not just "within N meters of some target".

Reports: per-predicate pass rate and ALL-PASS rate, split by whether the
cluster is GT-matched or not (answers "how many of the ALL-PASS failures
are real targets vs noise"); pre-emption check for ALL-PASS clusters; and
for the single lowest-passing predicate, an isolated breakdown of clusters
that fail ONLY that predicate (all others pass), split by GT-match status.
"""

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


BOAT_PREDICATES = [
    ("fp_max>=0.9(boat_fp_min)", lambda c: c["fp_max"] >= 0.9),
    ("fp_max<=sane(2.0/2.5near)", lambda c: c["fp_max"] <= (2.5 if math.hypot(c["x"], c["y"]) <= 20.0 else 2.0)),
    ("dz>=0.3(boat_h_min)", lambda c: c["dz"] >= 0.3),
    ("dz<=1.8(boat_h_max)", lambda c: c["dz"] <= 1.8),
    ("pts>=40(boat_min_pts)", lambda c: c["pts"] >= 40),
]

BLOCK_PREDICATES = [
    ("fp_max>=1.65(block_fp_min)", lambda c: c["fp_max"] >= 1.65),
    ("fp_max<=3.2(block_fp_max)", lambda c: c["fp_max"] <= 3.2),
    ("dz<=1.5(block_h_max)", lambda c: c["dz"] <= 1.5),
    ("square>=0.55(block_square_min)", lambda c: c["square"] >= 0.55),
    ("pts>=10(block_min_pts)", lambda c: c["pts"] >= 10),
    ("|z|<=1.0(block_center_z_max)", lambda c: abs(c["z"]) <= 1.0),
]

BUOY_PREDICATES = [
    ("fp_max<=1.0(buoy_fp_max)", lambda c: c["fp_max"] <= 1.0),
    ("dz<=1.5(buoy_h_max)", lambda c: c["dz"] <= 1.5),
]

CATEGORY_CONFIG = {
    "boat": (BOAT_PREDICATES, {"boat", "boat_fallback"}, "gt"),
    "block": (BLOCK_PREDICATES, {"block"}, "gt_block"),
    "buoy": (BUOY_PREDICATES, {"buoy", "buoy_fallback"}, "gt_buoy"),
}


def main():
    args = parse_args()
    evaluator = load_evaluator(args.evaluator)
    records = evaluator.load_records(args.log_path)
    odom_mid = records["odom"][len(records["odom"]) // 2]["stamp"]
    for record_type, items in records.items():
        if record_type not in {"odom", "cluster"}:
            evaluator.align_to_odom(items, args.epoch_offset, odom_mid)

    interpolate_odom = evaluator.make_odom_interpolator(records["odom"])

    clusters_by_stamp = {}
    for cluster in records["cluster"]:
        if cluster.get("forward_ratio") is None or cluster["forward_ratio"] < 0.5:
            continue
        stamp = round(cluster["t"] - args.epoch_offset, 6)
        clusters_by_stamp.setdefault(stamp, []).append(cluster)

    for name, (predicates, own_labels, gt_key) in CATEGORY_CONFIG.items():
        interpolate_gt = evaluator.make_gt_interpolator(records[gt_key])
        detection_frames, _, _ = evaluator.deduplicate_frames(
            records["det" if name == "boat" else f"det_{name}"]
        )

        pass_counts = {p[0]: [0, 0] for p in predicates}  # [matched, unmatched]
        total = [0, 0]
        all_pass = [0, 0]
        all_pass_labeled = [0, 0]
        all_pass_preempted = [0, 0]
        gt_target_total = 0
        gt_target_matched = 0
        matched_clusters = []  # (results_dict,) for the isolate-worst-predicate pass

        for frame in detection_frames:
            odom = interpolate_odom(frame["stamp"])
            gt = interpolate_gt(frame["stamp"])
            if odom is None or gt is None:
                continue
            targets = [evaluator.world_to_boat(pose, odom) for pose in gt["poses"]]
            if not targets:
                continue
            gt_target_total += len(targets)

            available = clusters_by_stamp.get(round(frame["stamp"], 6), ())
            cluster_positions = [(c["x"], c["y"]) for c in available]
            pairs, _ = evaluator.maximum_cardinality_matches(
                cluster_positions, targets, args.match_tol
            )
            matched_cluster_indices = {ci for ci, _ in pairs}
            gt_target_matched += len(matched_cluster_indices)

            for ci, cluster in enumerate(available):
                is_matched = ci in matched_cluster_indices
                bucket = 0 if is_matched else 1
                total[bucket] += 1
                results = {p[0]: p[1](cluster) for p in predicates}
                for pname, ok in results.items():
                    if ok:
                        pass_counts[pname][bucket] += 1
                if all(results.values()):
                    all_pass[bucket] += 1
                    if cluster["label"] in own_labels:
                        all_pass_labeled[bucket] += 1
                    else:
                        all_pass_preempted[bucket] += 1
                if is_matched:
                    matched_clusters.append(results)

        print(f"=== {name} (forward_ratio>=0.5, one-to-one GT match tol={args.match_tol}m) ===")
        if gt_target_total:
            print(f"  GT targets total={gt_target_total}, matched by some forward-majority cluster="
                  f"{gt_target_matched} ({100.0*gt_target_matched/gt_target_total:.1f}%)")
        print(f"  clusters: gt-matched={total[0]} unmatched(noise)={total[1]}")
        worst_pname, worst_rate = None, 2.0
        for pname in pass_counts:
            m, u = pass_counts[pname]
            tm, tu = total
            rm = m / tm if tm else 0.0
            ru = u / tu if tu else 0.0
            if rm < worst_rate:
                worst_rate, worst_pname = rm, pname
            print(f"    {pname:<32} matched-pass={m:>5}/{tm:<5}({100*rm:.1f}%)  "
                  f"noise-pass={u:>5}/{tu:<5}({100*ru:.1f}%)")
        am, au = all_pass
        tm, tu = total
        print(f"    {'ALL PASS':<32} matched-pass={am:>5}/{tm:<5}({100.0*am/tm if tm else 0:.1f}%)  "
              f"noise-pass={au:>5}/{tu:<5}({100.0*au/tu if tu else 0:.1f}%)")
        print(f"    of ALL-PASS+matched: labeled {name}={all_pass_labeled[0]}, "
              f"pre-empted={all_pass_preempted[0]}")
        print(f"    >>> matched clusters that FAIL all-pass (lost real detections): "
              f"{tm - am}/{tm} ({100.0*(tm-am)/tm if tm else 0:.1f}%)")

        # isolate: among GT-matched clusters, fails ONLY the worst predicate (all others pass)
        if worst_pname is not None:
            only_worst_fail_matched = sum(
                1 for r in matched_clusters
                if not r[worst_pname] and all(ok for p, ok in r.items() if p != worst_pname)
            )
            print(f"    worst single predicate (by matched-cluster pass rate): {worst_pname} ({100*worst_rate:.1f}%)")
            print(f"    matched clusters failing ONLY {worst_pname} (all else pass): "
                  f"{only_worst_fail_matched}/{tm} ({100.0*only_worst_fail_matched/tm if tm else 0:.1f}%)")
        print()


if __name__ == "__main__":
    main()
