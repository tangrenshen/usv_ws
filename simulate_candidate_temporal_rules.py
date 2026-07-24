#!/usr/bin/env python3
"""Offline causal temporal rules over all raw clusters.

Candidate association and evaluation use the validated publish-time frame so
the simulated baseline is directly comparable with the v2 evaluator. The raw
lidar measurement stamp is required and checked, but is not substituted for
the official evaluation time.
"""

import argparse
import bisect
import contextlib
import io
import math
import sys
from collections import Counter, defaultdict, deque


BOAT_LABELS = {"boat", "boat_fallback"}
PROMOTABLE_LABELS = {
    "buoy",
    "discarded_shape_implausible",
    "discarded_low_confidence",
}


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("log_path")
    parser.add_argument("--evaluator-dir", required=True)
    parser.add_argument("--epoch-offset", type=float, required=True)
    parser.add_argument("--gates", type=float, nargs="+", default=(0.5, 1.0, 1.5))
    parser.add_argument("--max-gap", type=float, default=0.35)
    return parser.parse_args()


def base_to_world(point, odom, evaluator):
    yaw = evaluator.quat_to_yaw(odom)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        odom["x"] + point[0] * cos_yaw - point[1] * sin_yaw,
        odom["y"] + point[0] * sin_yaw + point[1] * cos_yaw,
    )


def associate(candidates, active_tracks, gate):
    options = []
    for candidate_index, candidate in enumerate(candidates):
        wx, wy = candidate["world"]
        for track_id, track in active_tracks.items():
            distance = math.hypot(wx - track["x"], wy - track["y"])
            if distance <= gate:
                options.append((distance, candidate_index, track_id))
    matched_candidates = set()
    matched_tracks = set()
    result = {}
    for _, candidate_index, track_id in sorted(options):
        if candidate_index in matched_candidates or track_id in matched_tracks:
            continue
        matched_candidates.add(candidate_index)
        matched_tracks.add(track_id)
        result[candidate_index] = track_id
    return result


def recent_count(values, now, window):
    return sum(value >= now - window for value in values)


def rule_outputs(candidate, track, stamp):
    label = candidate["label"]
    raw_boat = label in BOAT_LABELS
    near = math.hypot(candidate["x"], candidate["y"]) <= 20.0
    promotable = near and label in PROMOTABLE_LABELS
    prior_any_05 = recent_count(track["observations"], stamp, 0.5) >= 1
    prior_boat_05 = recent_count(track["boat_times"], stamp, 0.5)
    prior_boat_10 = recent_count(track["boat_times"], stamp, 1.0)

    baseline = raw_boat
    main_boat = label == "boat"
    fallback = label == "boat_fallback"
    return {
        "baseline": baseline,
        "promote_k1_w05": baseline or (promotable and prior_boat_05 >= 1),
        "promote_k1_w10": baseline or (promotable and prior_boat_10 >= 1),
        "promote_k2_w10": baseline or (promotable and prior_boat_10 >= 2),
        "fallback_confirm_any_w05": main_boat or (fallback and prior_any_05),
        "fallback_confirm_boat_w10": main_boat or (
            fallback and prior_boat_10 >= 1
        ),
        "combined_any": (
            main_boat
            or (fallback and prior_any_05)
            or (promotable and prior_boat_10 >= 1)
        ),
        "combined_boat": (
            main_boat
            or (fallback and prior_boat_10 >= 1)
            or (promotable and prior_boat_10 >= 1)
        ),
    }


def analyze(records, evaluator, offset, gate, max_gap):
    odom = sorted(records["odom"], key=lambda item: item["stamp"])
    odom_mid = odom[len(odom) // 2]["stamp"]
    for record_type, items in records.items():
        if record_type not in {"odom", "cluster"}:
            evaluator.align_to_odom(items, offset, odom_mid)

    clusters = [dict(item) for item in records["cluster"]]
    if clusters and abs(clusters[len(clusters) // 2]["t"] - odom_mid) > 1000:
        for cluster in clusters:
            cluster["t"] -= offset
    measurement_stamp_count = sum(
        cluster.get("measurement_t") is not None for cluster in clusters
    )

    cluster_frames = defaultdict(list)
    for cluster in clusters:
        cluster_frames[round(cluster["t"], 6)].append(cluster)

    interpolate_odom = evaluator.make_odom_interpolator(odom)
    interpolate_gt = {
        "boat": evaluator.make_gt_interpolator(records["gt"]),
        "buoy": evaluator.make_gt_interpolator(records["gt_buoy"]),
    }
    published_frames, _, _ = evaluator.deduplicate_frames(records["det"])
    published_by_stamp = {
        round(frame["stamp"], 6): frame for frame in published_frames
    }

    tracks = {}
    next_track_id = 0
    metrics = defaultdict(Counter)
    changes = defaultdict(Counter)
    frame_count = 0
    cluster_baseline_mismatch = Counter()

    for stamp_key in sorted(cluster_frames):
        stamp = float(stamp_key)
        odom_frame = interpolate_odom(stamp)
        boat_gt_frame = interpolate_gt["boat"](stamp)
        buoy_gt_frame = interpolate_gt["buoy"](stamp)
        if odom_frame is None or boat_gt_frame is None or buoy_gt_frame is None:
            continue
        frame_count += 1
        boat_targets = [
            evaluator.world_to_boat(pose, odom_frame)
            for pose in boat_gt_frame["poses"]
        ]
        buoy_targets = [
            evaluator.world_to_boat(pose, odom_frame)
            for pose in buoy_gt_frame["poses"]
        ]
        candidates = cluster_frames[stamp_key]
        for candidate in candidates:
            candidate["world"] = base_to_world(
                (candidate["x"], candidate["y"]), odom_frame, evaluator
            )

        active = {
            track_id: track
            for track_id, track in tracks.items()
            if stamp - track["last_stamp"] <= max_gap
        }
        assignments = associate(candidates, active, gate)
        for candidate_index in range(len(candidates)):
            if candidate_index not in assignments:
                assignments[candidate_index] = next_track_id
                tracks[next_track_id] = {
                    "x": candidates[candidate_index]["world"][0],
                    "y": candidates[candidate_index]["world"][1],
                    "last_stamp": stamp,
                    "observations": deque(),
                    "boat_times": deque(),
                }
                next_track_id += 1

        boat_output_by_rule = defaultdict(list)
        buoy_output_by_rule = defaultdict(list)
        for candidate_index, candidate in enumerate(candidates):
            track = tracks[assignments[candidate_index]]
            while track["observations"] and track["observations"][0] < stamp - 1.0:
                track["observations"].popleft()
            while track["boat_times"] and track["boat_times"][0] < stamp - 1.0:
                track["boat_times"].popleft()

            outputs = rule_outputs(candidate, track, stamp)
            baseline = outputs["baseline"]
            for rule, selected in outputs.items():
                if selected:
                    boat_output_by_rule[rule].append(
                        (candidate["x"], candidate["y"])
                    )
                # A promoted primary-buoy candidate is a reclassification, not
                # a duplicate label. buoy_fallback is not promotable.
                if candidate["label"] in {"buoy", "buoy_fallback"} and not (
                    selected and not baseline and candidate["label"] == "buoy"
                ):
                    buoy_output_by_rule[rule].append(
                        (candidate["x"], candidate["y"])
                    )
                if selected and not baseline:
                    changes[rule]["promoted"] += 1
                    changes[rule][f"promoted:{candidate['label']}"] += 1
                if baseline and not selected:
                    changes[rule]["suppressed"] += 1
                    changes[rule][f"suppressed:{candidate['label']}"] += 1

            track["x"], track["y"] = candidate["world"]
            track["last_stamp"] = stamp
            track["observations"].append(stamp)
            if candidate["label"] in BOAT_LABELS:
                track["boat_times"].append(stamp)

        published = published_by_stamp.get(stamp_key)
        if published is not None:
            published_points = [
                (pose["x"], pose["y"]) for pose in published["poses"]
            ]
            cluster_points = boat_output_by_rule["baseline"]
            if len(published_points) != len(cluster_points):
                cluster_baseline_mismatch["frames"] += 1
                cluster_baseline_mismatch["count_delta"] += (
                    len(cluster_points) - len(published_points)
                )

        for rule, points in boat_output_by_rule.items():
            pairs, _ = evaluator.maximum_cardinality_matches(
                points, boat_targets, 2.0
            )
            hit = len(pairs)
            metrics[(rule, "boat")]["hit"] += hit
            metrics[(rule, "boat")]["miss"] += len(boat_targets) - hit
            metrics[(rule, "boat")]["fp"] += len(points) - hit
        for rule, points in buoy_output_by_rule.items():
            pairs, _ = evaluator.maximum_cardinality_matches(
                points, buoy_targets, 2.0
            )
            hit = len(pairs)
            metrics[(rule, "buoy")]["hit"] += hit
            metrics[(rule, "buoy")]["miss"] += len(buoy_targets) - hit
            metrics[(rule, "buoy")]["fp"] += len(points) - hit

    return (
        frame_count,
        metrics,
        changes,
        cluster_baseline_mismatch,
        measurement_stamp_count,
        len(clusters),
    )


def main():
    args = parse_args()
    sys.path.insert(0, args.evaluator_dir)
    import four_category_evaluator_v2 as evaluator

    for gate in args.gates:
        records = evaluator.load_records(args.log_path)
        frames, metrics, changes, mismatch, measurement_count, cluster_count = analyze(
            records, evaluator, args.epoch_offset, gate, args.max_gap
        )
        print(
            f"\nassociation_gate={gate:.2f}m max_gap={args.max_gap:.2f}s "
            f"valid_frames={frames} baseline_frame_mismatch={dict(mismatch)} "
            f"measurement_t={measurement_count}/{cluster_count}"
        )
        print(
            "rule                           boat_recall boat_fpr "
            "buoy_recall buoy_fpr promoted suppressed"
        )
        rules = [rule for rule, category in metrics if category == "boat"]
        for rule in rules:
            boat = metrics[(rule, "boat")]
            buoy = metrics[(rule, "buoy")]
            boat_recall = 100 * boat["hit"] / (boat["hit"] + boat["miss"])
            boat_fpr = 100 * boat["fp"] / (boat["hit"] + boat["fp"])
            buoy_recall = 100 * buoy["hit"] / (buoy["hit"] + buoy["miss"])
            buoy_fpr = 100 * buoy["fp"] / (buoy["hit"] + buoy["fp"])
            delta = changes[rule]
            print(
                f"{rule:<30} {boat_recall:9.2f}% {boat_fpr:7.2f}% "
                f"{buoy_recall:9.2f}% {buoy_fpr:7.2f}% "
                f"{delta['promoted']:8d} {delta['suppressed']:10d}"
            )
        print("change details")
        for rule, counter in changes.items():
            details = ", ".join(
                f"{key}={value}"
                for key, value in sorted(counter.items())
                if ":" in key
            )
            if details:
                print(f"  {rule}: {details}")


if __name__ == "__main__":
    main()
