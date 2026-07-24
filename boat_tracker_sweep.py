#!/usr/bin/env python3
"""Offline temporal tracker sweep for boat detections.

The tracker uses only detections, odometry, and timestamps. Ground truth is
used exclusively after tracking to score each parameter set.
"""

import argparse
import math
from dataclasses import dataclass

import four_category_evaluator_v2 as evaluator


@dataclass
class Track:
    track_id: int
    x: float
    y: float
    vx: float
    vy: float
    stamp: float
    first_x: float
    first_y: float
    first_stamp: float
    consecutive_hits: int = 1
    misses: int = 0
    confirmed: bool = False

    def predict_to(self, stamp):
        dt = max(0.0, stamp - self.stamp)
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.stamp = stamp

    def average_speed(self):
        duration = self.stamp - self.first_stamp
        if duration <= 1e-6:
            return 0.0
        return math.hypot(self.x - self.first_x, self.y - self.first_y) / duration


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("log_path")
    parser.add_argument("--epoch-offset", type=float, required=True)
    return parser.parse_args()


def boat_to_world(point, odom):
    angle = evaluator.quat_to_yaw(odom)
    return (
        odom["x"] + point[0] * math.cos(angle) - point[1] * math.sin(angle),
        odom["y"] + point[0] * math.sin(angle) + point[1] * math.cos(angle),
    )


def world_to_boat(point, odom):
    pose = {"x": point[0], "y": point[1]}
    return evaluator.world_to_boat(pose, odom)


def associate(tracks, measurements, gate):
    candidates = []
    for track_index, track in enumerate(tracks):
        for measurement_index, measurement in enumerate(measurements):
            distance = math.hypot(
                track.x - measurement[0], track.y - measurement[1]
            )
            if distance <= gate:
                candidates.append((distance, track_index, measurement_index))
    matched_tracks = set()
    matched_measurements = set()
    pairs = []
    for distance, track_index, measurement_index in sorted(candidates):
        if track_index in matched_tracks or measurement_index in matched_measurements:
            continue
        matched_tracks.add(track_index)
        matched_measurements.add(measurement_index)
        pairs.append((track_index, measurement_index, distance))
    return pairs, matched_tracks, matched_measurements


def run_tracker(
    frames,
    interpolate_odom,
    gate,
    min_hits,
    max_misses,
    velocity_alpha,
    min_speed,
    max_speed,
    max_frame_gap=0.5,
):
    tracks = []
    next_track_id = 1
    output = []
    previous_stamp = None

    for frame in frames:
        stamp = frame["stamp"]
        odom = interpolate_odom(stamp)
        if odom is None:
            continue
        if previous_stamp is not None and stamp - previous_stamp > max_frame_gap:
            tracks = []
        previous_stamp = stamp

        measurements = [
            boat_to_world((pose["x"], pose["y"]), odom) for pose in frame["poses"]
        ]
        previous_states = [
            (track.x, track.y, track.stamp, track.vx, track.vy) for track in tracks
        ]
        for track in tracks:
            track.predict_to(stamp)

        pairs, matched_tracks, matched_measurements = associate(
            tracks, measurements, gate
        )

        for track_index, measurement_index, _ in pairs:
            track = tracks[track_index]
            measurement = measurements[measurement_index]
            old_x, old_y, old_stamp, old_vx, old_vy = previous_states[track_index]
            dt = stamp - old_stamp
            if dt > 1e-6:
                measured_vx = (measurement[0] - old_x) / dt
                measured_vy = (measurement[1] - old_y) / dt
                track.vx = (
                    velocity_alpha * measured_vx + (1 - velocity_alpha) * old_vx
                )
                track.vy = (
                    velocity_alpha * measured_vy + (1 - velocity_alpha) * old_vy
                )
            track.x, track.y = measurement
            track.stamp = stamp
            track.consecutive_hits += 1
            track.misses = 0
            if track.consecutive_hits >= min_hits:
                track.confirmed = True

        for track_index, track in enumerate(tracks):
            if track_index not in matched_tracks:
                track.misses += 1
                if not track.confirmed:
                    track.consecutive_hits = 0

        for measurement_index, measurement in enumerate(measurements):
            if measurement_index in matched_measurements:
                continue
            track = Track(
                track_id=next_track_id,
                x=measurement[0],
                y=measurement[1],
                vx=0.0,
                vy=0.0,
                stamp=stamp,
                first_x=measurement[0],
                first_y=measurement[1],
                first_stamp=stamp,
                confirmed=min_hits <= 1,
            )
            next_track_id += 1
            tracks.append(track)

        tracks = [
            track
            for track in tracks
            if (track.confirmed and track.misses <= max_misses)
            or (not track.confirmed and track.misses == 0)
        ]

        poses = []
        for track in tracks:
            speed = track.average_speed()
            if (
                not track.confirmed
                or speed < min_speed
                or speed > max_speed
            ):
                continue
            bx, by = world_to_boat((track.x, track.y), odom)
            poses.append({"x": bx, "y": by, "z": 0.0})
        output.append({"type": "det", "stamp": stamp, "poses": poses})

    return output


def score(frames, gt_records, interpolate_odom, tolerance=2.0):
    interpolate_gt = evaluator.make_gt_interpolator(gt_records)
    hit = miss = false_positive = 0
    for frame in frames:
        odom = interpolate_odom(frame["stamp"])
        gt = interpolate_gt(frame["stamp"])
        if odom is None or gt is None:
            continue
        targets = [evaluator.world_to_boat(pose, odom) for pose in gt["poses"]]
        detections = [(pose["x"], pose["y"]) for pose in frame["poses"]]
        pairs, _ = evaluator.maximum_cardinality_matches(
            detections, targets, tolerance
        )
        frame_hits = len(pairs)
        hit += frame_hits
        miss += len(targets) - frame_hits
        false_positive += len(detections) - frame_hits
    recall = 100 * hit / (hit + miss) if hit + miss else 0.0
    precision = 100 * hit / (hit + false_positive) if hit + false_positive else 0.0
    fpr = 100 - precision
    f1 = (
        2 * recall * precision / (recall + precision)
        if recall + precision
        else 0.0
    )
    return {
        "hit": hit,
        "miss": miss,
        "fp": false_positive,
        "recall": recall,
        "precision": precision,
        "fpr": fpr,
        "f1": f1,
    }


def main():
    args = parse_args()
    records = evaluator.load_records(args.log_path)
    odom_mid = records["odom"][len(records["odom"]) // 2]["stamp"]
    for record_type, items in records.items():
        if record_type != "odom":
            evaluator.align_to_odom(items, args.epoch_offset, odom_mid)

    frames, duplicate_records, conflicts = evaluator.deduplicate_frames(
        records["det"]
    )
    interpolate_odom = evaluator.make_odom_interpolator(records["odom"])
    baseline = score(frames, records["gt"], interpolate_odom)
    print(
        f"baseline unique_frames={len(frames)} duplicate_records={duplicate_records} "
        f"conflicts={conflicts} recall={baseline['recall']:.2f}% "
        f"precision={baseline['precision']:.2f}% fpr={baseline['fpr']:.2f}% "
        f"f1={baseline['f1']:.2f}% hit={baseline['hit']} fp={baseline['fp']}"
    )

    gt_speeds = []
    gt_records = sorted(records["gt"], key=lambda item: item["stamp"])
    for previous, current in zip(gt_records, gt_records[1:]):
        dt = current["stamp"] - previous["stamp"]
        if dt <= 1e-6 or len(previous["poses"]) != len(current["poses"]):
            continue
        for left, right in zip(previous["poses"], current["poses"]):
            gt_speeds.append(
                math.hypot(right["x"] - left["x"], right["y"] - left["y"]) / dt
            )
    gt_speeds.sort()
    if gt_speeds:
        def q(fraction):
            return gt_speeds[round((len(gt_speeds) - 1) * fraction)]
        print(
            "GT boat world-speed quantiles m/s: "
            f"p05={q(.05):.3f} p10={q(.10):.3f} p50={q(.50):.3f} "
            f"p90={q(.90):.3f} p95={q(.95):.3f}"
        )

    results = []
    for gate in (0.5, 0.75, 1.0, 1.5, 2.0):
        for min_hits in (2, 3, 4):
            for max_misses in (0, 1, 2, 3, 5):
                for min_speed in (0.0, 0.10, 0.25, 0.50, 0.75, 1.0):
                    tracked = run_tracker(
                        frames,
                        interpolate_odom,
                        gate=gate,
                        min_hits=min_hits,
                        max_misses=max_misses,
                        velocity_alpha=0.35,
                        min_speed=min_speed,
                        max_speed=8.0,
                    )
                    metrics = score(tracked, records["gt"], interpolate_odom)
                    metrics.update(
                        {
                            "gate": gate,
                            "min_hits": min_hits,
                            "max_misses": max_misses,
                            "min_speed": min_speed,
                        }
                    )
                    results.append(metrics)

    print("\nTop 20 by F1")
    print("gate hits misses minspd recall precision fpr f1 hit fp")
    for result in sorted(results, key=lambda item: item["f1"], reverse=True)[:20]:
        print(
            f"{result['gate']:>4.2f} {result['min_hits']:>4} "
            f"{result['max_misses']:>6} {result['min_speed']:>6.2f} "
            f"{result['recall']:>6.2f}% "
            f"{result['precision']:>9.2f}% {result['fpr']:>6.2f}% "
            f"{result['f1']:>6.2f}% {result['hit']:>4} {result['fp']:>4}"
        )

    print("\nPareto candidates: recall >= baseline and precision > baseline")
    for result in sorted(results, key=lambda item: item["precision"], reverse=True):
        if (
            result["recall"] >= baseline["recall"]
            and result["precision"] > baseline["precision"]
        ):
            print(
                f"gate={result['gate']:.2f} min_hits={result['min_hits']} "
                f"max_misses={result['max_misses']} min_speed={result['min_speed']:.2f} "
                f"recall={result['recall']:.2f}% precision={result['precision']:.2f}% "
                f"fpr={result['fpr']:.2f}% f1={result['f1']:.2f}%"
            )


if __name__ == "__main__":
    main()
