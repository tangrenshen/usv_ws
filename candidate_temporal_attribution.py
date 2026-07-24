#!/usr/bin/env python3
"""Attribute Boat misses and false positives to raw candidate clusters.

The script deliberately imports the validated v2 evaluator helpers instead of
reimplementing time alignment, interpolation, coordinate transforms, frame
deduplication, or one-to-one matching.
"""

import argparse
import bisect
import json
import math
import statistics
import sys
from collections import Counter, defaultdict
from pathlib import Path


STAMP_TYPES = (
    "gt",
    "gt_buoy",
    "gt_pillar",
    "gt_block",
    "det",
    "det_buoy",
    "det_pillar",
    "det_block",
)
OTHER_GT = ("buoy", "pillar", "block")
BUCKETS = ("0-20m", "20-40m", "40-60m", "60m+")


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("logs", nargs="+", help="Tracker-off JSONL logs")
    parser.add_argument(
        "--evaluator-dir",
        required=True,
        help="Directory containing four_category_evaluator_v2.py",
    )
    parser.add_argument(
        "--epoch-offset",
        type=float,
        required=True,
        help="Validated absolute-epoch to odom-relative offset",
    )
    parser.add_argument("--match-tol", type=float, default=2.0)
    parser.add_argument(
        "--frame-tol",
        type=float,
        default=0.02,
        help="Maximum cluster/detection frame stamp difference",
    )
    parser.add_argument(
        "--json-output",
        help="Optional machine-readable aggregate report",
    )
    return parser.parse_args()


def distance_bucket(distance):
    if distance < 20:
        return "0-20m"
    if distance < 40:
        return "20-40m"
    if distance < 60:
        return "40-60m"
    return "60m+"


def percentage(part, total):
    return 100.0 * part / total if total else 0.0


def nearest_frame(frames_by_stamp, sorted_stamps, stamp, tolerance):
    index = bisect.bisect_left(sorted_stamps, stamp)
    options = []
    if index < len(sorted_stamps):
        options.append(sorted_stamps[index])
    if index:
        options.append(sorted_stamps[index - 1])
    if not options:
        return None, math.inf
    best = min(options, key=lambda value: abs(value - stamp))
    gap = abs(best - stamp)
    if gap > tolerance:
        return None, gap
    return frames_by_stamp[best], gap


def nearest_label(point, candidates, tolerance=0.10):
    if not candidates:
        return "unlinked"
    index = min(
        range(len(candidates)),
        key=lambda candidate_index: math.hypot(
            point[0] - candidates[candidate_index]["x"],
            point[1] - candidates[candidate_index]["y"],
        ),
    )
    distance = math.hypot(
        point[0] - candidates[index]["x"], point[1] - candidates[index]["y"]
    )
    return candidates[index]["label"] if distance <= tolerance else "unlinked"


def quantiles(values):
    if not values:
        return {"n": 0, "p10": None, "median": None, "p90": None}
    ordered = sorted(values)

    def get(fraction):
        return ordered[round((len(ordered) - 1) * fraction)]

    return {
        "n": len(values),
        "p10": get(0.10),
        "median": get(0.50),
        "p90": get(0.90),
    }


def transform_gt(interpolator, stamp, odom, evaluator):
    frame = interpolator(stamp)
    if not frame:
        return []
    return [evaluator.world_to_boat(pose, odom) for pose in frame["poses"]]


def align_clusters(clusters, offset, odom_mid):
    if not clusters:
        return False
    sample = clusters[len(clusters) // 2]["t"]
    if abs(sample - odom_mid) <= 1000:
        return False
    for cluster in clusters:
        cluster["t"] -= offset
    return True


def candidate_match_map(candidates, targets, tolerance, evaluator):
    points = [(item["x"], item["y"]) for item in candidates]
    pairs, _ = evaluator.maximum_cardinality_matches(points, targets, tolerance)
    return {target_index: candidates[candidate_index] for candidate_index, target_index in pairs}


def other_gt_attribution(point, gt_by_category, tolerance):
    close = []
    nearest = []
    for category in OTHER_GT:
        points = gt_by_category[category]
        distance = min(
            (math.hypot(point[0] - x, point[1] - y) for x, y in points),
            default=math.inf,
        )
        nearest.append((distance, category))
        if distance < tolerance:
            close.append(category)
    if not close:
        return "unmatched"
    if len(close) > 1:
        return "ambiguous:" + "+".join(sorted(close))
    return min(nearest)[1]


def analyze_log(path, evaluator, offset, match_tol, frame_tol):
    records = evaluator.load_records(path)
    odom = sorted(records["odom"], key=lambda item: item["stamp"])
    if not odom:
        raise ValueError(f"{path}: no odom records")
    odom_mid = odom[len(odom) // 2]["stamp"]

    for key in STAMP_TYPES:
        evaluator.align_to_odom(records[key], offset, odom_mid)
    align_clusters(records["cluster"], offset, odom_mid)

    boat_frames, duplicate_count, conflict_count = evaluator.deduplicate_frames(
        records["det"]
    )
    interpolate_odom = evaluator.make_odom_interpolator(odom)
    interpolate_gt = {
        "boat": evaluator.make_gt_interpolator(records["gt"]),
        "buoy": evaluator.make_gt_interpolator(records["gt_buoy"]),
        "pillar": evaluator.make_gt_interpolator(records["gt_pillar"]),
        "block": evaluator.make_gt_interpolator(records["gt_block"]),
    }

    cluster_frames = defaultdict(list)
    for cluster in records["cluster"]:
        cluster_frames[round(cluster["t"], 6)].append(cluster)
    cluster_stamps = sorted(cluster_frames)

    counts = Counter()
    miss_labels = Counter()
    miss_labels_by_range = defaultdict(Counter)
    hit_labels = Counter()
    fp_sources = Counter()
    fp_sources_by_range = defaultdict(Counter)
    fp_candidate_labels = Counter()
    miss_candidate_pts = defaultdict(list)
    frame_gaps = []

    for frame in boat_frames:
        stamp = frame["stamp"]
        odom_frame = interpolate_odom(stamp)
        if odom_frame is None:
            counts["skipped_no_odom"] += 1
            continue
        gt_boats = transform_gt(interpolate_gt["boat"], stamp, odom_frame, evaluator)
        if not gt_boats:
            counts["skipped_no_boat_gt"] += 1
            continue

        detections = [(pose["x"], pose["y"]) for pose in frame["poses"]]
        det_pairs, _ = evaluator.maximum_cardinality_matches(
            detections, gt_boats, match_tol
        )
        hit_targets = {target for _, target in det_pairs}
        matched_detections = {detection for detection, _ in det_pairs}

        clusters, gap = nearest_frame(
            cluster_frames, cluster_stamps, stamp, frame_tol
        )
        if clusters is None:
            clusters = []
            counts["frames_without_cluster_link"] += 1
        else:
            counts["frames_with_cluster_link"] += 1
            frame_gaps.append(gap)

        candidate_by_target = candidate_match_map(
            clusters, gt_boats, match_tol, evaluator
        )
        counts["gt_total"] += len(gt_boats)
        counts["hit"] += len(hit_targets)
        counts["miss"] += len(gt_boats) - len(hit_targets)

        for target_index, point in enumerate(gt_boats):
            candidate = candidate_by_target.get(target_index)
            label = candidate["label"] if candidate else "no_candidate"
            bucket = distance_bucket(math.hypot(*point))
            if target_index in hit_targets:
                hit_labels[label] += 1
            else:
                miss_labels[label] += 1
                miss_labels_by_range[bucket][label] += 1
                if candidate:
                    miss_candidate_pts[label].append(candidate["pts"])

        gt_by_category = {
            category: transform_gt(
                interpolate_gt[category], stamp, odom_frame, evaluator
            )
            for category in OTHER_GT
        }
        for detection_index, point in enumerate(detections):
            if detection_index in matched_detections:
                continue
            counts["false_positive"] += 1
            source = other_gt_attribution(point, gt_by_category, match_tol)
            bucket = distance_bucket(math.hypot(*point))
            fp_sources[source] += 1
            fp_sources_by_range[bucket][source] += 1
            fp_candidate_labels[nearest_label(point, clusters)] += 1

    counts["raw_det_frames"] = len(records["det"])
    counts["unique_det_frames"] = len(boat_frames)
    counts["duplicate_det_frames"] = duplicate_count
    counts["conflicting_det_stamps"] = conflict_count
    counts["raw_clusters"] = len(records["cluster"])

    recall = percentage(counts["hit"], counts["gt_total"])
    fpr = percentage(
        counts["false_positive"], counts["hit"] + counts["false_positive"]
    )
    return {
        "path": str(path),
        "counts": dict(counts),
        "recall": recall,
        "fpr": fpr,
        "miss_labels": dict(miss_labels),
        "miss_labels_by_range": {
            bucket: dict(counter) for bucket, counter in miss_labels_by_range.items()
        },
        "hit_labels": dict(hit_labels),
        "fp_sources": dict(fp_sources),
        "fp_sources_by_range": {
            bucket: dict(counter) for bucket, counter in fp_sources_by_range.items()
        },
        "fp_candidate_labels": dict(fp_candidate_labels),
        "miss_candidate_pts": {
            label: quantiles(values) for label, values in miss_candidate_pts.items()
        },
        "frame_gap": quantiles(frame_gaps),
    }


def merge_counter(reports, field):
    total = Counter()
    for report in reports:
        total.update(report[field])
    return total


def merge_nested_counter(reports, field):
    total = defaultdict(Counter)
    for report in reports:
        for bucket, values in report[field].items():
            total[bucket].update(values)
    return total


def print_counter(counter, denominator, indent="  "):
    for label, count in counter.most_common():
        print(f"{indent}{label:<36} {count:>6}  {percentage(count, denominator):>6.2f}%")


def main():
    args = parse_args()
    sys.path.insert(0, str(Path(args.evaluator_dir).resolve()))
    import four_category_evaluator_v2 as evaluator

    reports = [
        analyze_log(
            path,
            evaluator,
            args.epoch_offset,
            args.match_tol,
            args.frame_tol,
        )
        for path in args.logs
    ]

    print("Per-run cross-check (must reproduce the v2 Boat baseline)")
    for report in reports:
        counts = report["counts"]
        print(
            f"  {Path(report['path']).name}: "
            f"recall={report['recall']:.2f}% fpr={report['fpr']:.2f}% "
            f"hit={counts.get('hit', 0)} miss={counts.get('miss', 0)} "
            f"fp={counts.get('false_positive', 0)} "
            f"frames={counts.get('unique_det_frames', 0)} "
            f"cluster_link={counts.get('frames_with_cluster_link', 0)}/"
            f"{counts.get('unique_det_frames', 0)}"
        )

    print(
        "\nMedian baseline: "
        f"recall={statistics.median(r['recall'] for r in reports):.2f}% "
        f"fpr={statistics.median(r['fpr'] for r in reports):.2f}%"
    )

    miss_labels = merge_counter(reports, "miss_labels")
    miss_total = sum(miss_labels.values())
    print(f"\nMiss attribution across {len(reports)} runs (n={miss_total})")
    print_counter(miss_labels, miss_total)

    miss_by_range = merge_nested_counter(reports, "miss_labels_by_range")
    print("\nMiss attribution by GT range")
    for bucket in BUCKETS:
        counter = miss_by_range[bucket]
        total = sum(counter.values())
        if not total:
            continue
        candidate_count = total - counter["no_candidate"]
        print(
            f"  {bucket}: n={total}, raw candidate present="
            f"{candidate_count} ({percentage(candidate_count, total):.1f}%)"
        )
        print_counter(counter, total, indent="    ")

    fp_sources = merge_counter(reports, "fp_sources")
    fp_total = sum(fp_sources.values())
    print(f"\nBoat false-positive attribution (n={fp_total})")
    print_counter(fp_sources, fp_total)

    fp_by_range = merge_nested_counter(reports, "fp_sources_by_range")
    print("\nFalse positives by detection range")
    for bucket in BUCKETS:
        counter = fp_by_range[bucket]
        total = sum(counter.values())
        if not total:
            continue
        print(f"  {bucket}: n={total}")
        print_counter(counter, total, indent="    ")

    fp_candidate_labels = merge_counter(reports, "fp_candidate_labels")
    print("\nRaw labels linked to Boat false positives")
    print_counter(fp_candidate_labels, fp_total)

    aggregate = {
        "runs": reports,
        "median_recall": statistics.median(r["recall"] for r in reports),
        "median_fpr": statistics.median(r["fpr"] for r in reports),
        "miss_labels": dict(miss_labels),
        "miss_labels_by_range": {
            bucket: dict(counter) for bucket, counter in miss_by_range.items()
        },
        "fp_sources": dict(fp_sources),
        "fp_sources_by_range": {
            bucket: dict(counter) for bucket, counter in fp_by_range.items()
        },
        "fp_candidate_labels": dict(fp_candidate_labels),
    }
    if args.json_output:
        with open(args.json_output, "w", encoding="utf-8") as stream:
            json.dump(aggregate, stream, ensure_ascii=False, indent=2)
            stream.write("\n")


if __name__ == "__main__":
    main()
