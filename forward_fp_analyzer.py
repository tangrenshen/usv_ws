#!/usr/bin/env python3
"""Measure forward-lidar support among one-to-one false-positive detections."""

import argparse
import importlib.util
import math
import statistics


CATEGORY_LABELS = {
    "boat": {"boat", "boat_fallback"},
    "buoy": {"buoy", "buoy_fallback"},
    "pillar": {"pillar"},
    "block": {"block"},
}


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


def cluster_class(label):
    for name, labels in CATEGORY_LABELS.items():
        if label in labels:
            return name
    return None


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
        stamp = round(cluster["t"] - args.epoch_offset, 6)
        category = cluster_class(cluster["label"])
        if category is not None:
            clusters_by_stamp.setdefault((stamp, category), []).append(cluster)

    interpolate_odom = evaluator.make_odom_interpolator(records["odom"])
    all_fp_ratios = []
    all_tp_ratios = []
    all_fp_count = 0
    all_tp_count = 0
    all_target_count = 0
    all_fp_linked_count = 0
    all_tp_linked_count = 0

    for name, gt_key, detection_key in evaluator.CATEGORIES:
        detection_frames, _, _ = evaluator.deduplicate_frames(
            records[detection_key]
        )
        interpolate_gt = evaluator.make_gt_interpolator(records[gt_key])
        false_positive_ratios = []
        true_positive_ratios = []
        false_positive_count = 0
        true_positive_count = 0
        target_count = 0
        false_positive_linked_count = 0
        true_positive_linked_count = 0

        for frame in detection_frames:
            odom = interpolate_odom(frame["stamp"])
            gt = interpolate_gt(frame["stamp"])
            if odom is None or gt is None:
                continue
            detections = [
                (pose["x"], pose["y"]) for pose in frame["poses"]
            ]
            targets = [
                evaluator.world_to_boat(pose, odom) for pose in gt["poses"]
            ]
            target_count += len(targets)
            pairs, _ = evaluator.maximum_cardinality_matches(
                detections, targets, args.match_tol
            )
            matched_detection_indices = {
                detection_index for detection_index, _ in pairs
            }
            true_positive_count += len(matched_detection_indices)
            false_positive_count += (
                len(detections) - len(matched_detection_indices)
            )
            available_clusters = list(
                clusters_by_stamp.get((round(frame["stamp"], 6), name), ())
            )
            used_clusters = set()

            for detection_index, detection in enumerate(detections):
                options = []
                for cluster_index, cluster in enumerate(available_clusters):
                    if cluster_index in used_clusters:
                        continue
                    distance = math.hypot(
                        detection[0] - cluster["x"],
                        detection[1] - cluster["y"],
                    )
                    if distance <= args.cluster_match_tol:
                        options.append((distance, cluster_index))
                if not options:
                    continue
                _, cluster_index = min(options)
                used_clusters.add(cluster_index)
                ratio = available_clusters[cluster_index]["forward_ratio"]
                if detection_index in matched_detection_indices:
                    true_positive_linked_count += 1
                    true_positive_ratios.append(ratio)
                else:
                    false_positive_linked_count += 1
                    false_positive_ratios.append(ratio)

        all_fp_count += false_positive_count
        all_tp_count += true_positive_count
        all_target_count += target_count
        all_fp_linked_count += false_positive_linked_count
        all_tp_linked_count += true_positive_linked_count
        all_fp_ratios.extend(false_positive_ratios)
        all_tp_ratios.extend(true_positive_ratios)
        print_summary(
            name,
            true_positive_count,
            true_positive_linked_count,
            true_positive_ratios,
            target_count,
            false_positive_count,
            false_positive_linked_count,
            false_positive_ratios,
        )

    print_summary(
        "overall",
        all_tp_count,
        all_tp_linked_count,
        all_tp_ratios,
        all_target_count,
        all_fp_count,
        all_fp_linked_count,
        all_fp_ratios,
    )


def print_summary(
    name,
    tp_count,
    tp_linked_count,
    tp_ratios,
    target_count,
    fp_count,
    fp_linked_count,
    fp_ratios,
):
    print(f"{name}:")
    print_distribution("TP", tp_count, tp_linked_count, tp_ratios)
    print_distribution("FP", fp_count, fp_linked_count, fp_ratios)
    if tp_ratios and fp_ratios:
        fp_removed = sum(ratio >= 0.5 for ratio in fp_ratios)
        tp_lost = sum(ratio >= 0.5 for ratio in tp_ratios)
        print(
            "  hypothetical forward_ratio>=0.50 reject on linked detections: "
            f"FP removed={fp_removed}/{len(fp_ratios)} "
            f"({100.0 * fp_removed / len(fp_ratios):.1f}%), "
            f"TP lost={tp_lost}/{len(tp_ratios)} "
            f"({100.0 * tp_lost / len(tp_ratios):.1f}%)"
        )
        filtered_tp = tp_count - tp_lost
        filtered_fp = fp_count - fp_removed
        recall = 100.0 * filtered_tp / target_count if target_count else 0.0
        false_positive_rate = (
            100.0 * filtered_fp / (filtered_tp + filtered_fp)
            if filtered_tp + filtered_fp else 0.0
        )
        print(
            "  predicted post-classification metrics "
            "(unlinked detections kept): "
            f"recall={recall:.1f}% false_positive_rate={false_positive_rate:.1f}%"
        )


def print_distribution(kind, count, linked_count, ratios):
    coverage = 100.0 * linked_count / count if count else 0.0
    print(
        f"  {kind}: count={count} linked={linked_count} "
        f"diagnostic_coverage={coverage:.1f}%"
    )
    if not ratios:
        return
    sorted_ratios = sorted(ratios)
    p75 = sorted_ratios[int(0.75 * (len(sorted_ratios) - 1))]
    counts = []
    for threshold in (0.25, 0.50, 0.75):
        threshold_count = sum(ratio >= threshold for ratio in ratios)
        counts.append(
            f">={threshold:.2f}:{threshold_count}/{len(ratios)}"
            f"({100.0 * threshold_count / len(ratios):.1f}%)"
        )
    print("    " + " ".join(counts))
    print(
        f"    ratio median={statistics.median(ratios):.3f} "
        f"p75={p75:.3f} max={max(ratios):.3f}"
    )


if __name__ == "__main__":
    main()
