#!/usr/bin/env python3
"""Attribute boat false positives to other ground-truth categories.

This intentionally mirrors four_category_evaluator.py:
- one canonical epoch offset derived from boat GT and odometry
- linear odometry and GT interpolation
- world-to-boat coordinate conversion
- 2 m boat matching tolerance

For boat detections that do not match boat GT, the script reports proximity to
buoy/pillar/block GT at several radii. Nearest-category attribution is also
reported at the canonical 2 m tolerance.
"""

import bisect
import json
import math
import re
import sys
from collections import Counter, defaultdict


MATCH_TOL = 2.0
MAX_TIME_GAP = 0.5
RADII = (0.5, 1.0, 1.5, 2.0)
OTHER_CATEGORIES = ("buoy", "pillar", "block")
DIAG_RE = re.compile(
    r"\[cluster_diagnostic\]\s+t=(?P<t>[-+0-9.eE]+)\s+"
    r"center=\((?P<x>[-+0-9.eE]+),(?P<y>[-+0-9.eE]+),(?P<z>[-+0-9.eE]+)\)\s+"
    r"fp_max=(?P<fp_max>[-+0-9.eE]+)\s+fp_min=(?P<fp_min>[-+0-9.eE]+)\s+"
    r"dz=(?P<dz>[-+0-9.eE]+)\s+square=(?P<square>[-+0-9.eE]+)\s+"
    r"pts=(?P<pts>\d+)\s+->\s+assigned=(?P<label>boat(?:_fallback)?)"
)


def quat_to_yaw(qx, qy, qz, qw):
    return math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


def world_to_boat(wx, wy, bx, by, yaw):
    dx, dy = wx - bx, wy - by
    return (
        dx * math.cos(-yaw) - dy * math.sin(-yaw),
        dx * math.sin(-yaw) + dy * math.cos(-yaw),
    )


def make_linear_interpolator(records, pose_fields):
    records = sorted(records, key=lambda r: r["stamp"])
    stamps = [r["stamp"] for r in records]

    def interpolate(t):
        if not records:
            return None
        idx = bisect.bisect_left(stamps, t)
        if idx == 0:
            return records[0] if t <= stamps[0] + MAX_TIME_GAP else None
        if idx >= len(records):
            return records[-1] if t >= stamps[-1] - MAX_TIME_GAP else None
        r0, r1 = records[idx - 1], records[idx]
        t0, t1 = r0["stamp"], r1["stamp"]
        if t < t0 - MAX_TIME_GAP or t > t1 + MAX_TIME_GAP:
            return None
        if abs(t1 - t0) < 1e-9:
            return r0
        alpha = (t - t0) / (t1 - t0)
        if pose_fields == "odom":
            return {
                key: r0[key] + (r1[key] - r0[key]) * alpha
                for key in ("x", "y", "qx", "qy", "qz", "qw")
            }
        poses0, poses1 = r0["poses"], r1["poses"]
        if len(poses0) != len(poses1):
            return r0 if abs(t - t0) < abs(t - t1) else r1
        return {
            "poses": [
                {
                    "x": p0["x"] + (p1["x"] - p0["x"]) * alpha,
                    "y": p0["y"] + (p1["y"] - p0["y"]) * alpha,
                }
                for p0, p1 in zip(poses0, poses1)
            ]
        }

    return interpolate


def min_distance(point, targets):
    if not targets:
        return math.inf
    x, y = point
    return min(math.hypot(x - tx, y - ty) for tx, ty in targets)


def distance_bucket(distance):
    if distance < 20:
        return "0-20m"
    if distance < 40:
        return "20-40m"
    if distance < 60:
        return "40-60m"
    return "60m+"


def parse_diagnostics(path, canonical_offset):
    by_time = defaultdict(list)
    if not path:
        return by_time
    with open(path, encoding="utf-8", errors="replace") as stream:
        for line in stream:
            match = DIAG_RE.search(line)
            if not match:
                continue
            item = {
                key: (int(value) if key == "pts" else value if key == "label" else float(value))
                for key, value in match.groupdict().items()
            }
            if item["t"] > 1000:
                item["t"] -= canonical_offset
            by_time[round(item["t"], 3)].append(item)
    return by_time


def attach_diagnostic(record, diagnostics):
    candidates = diagnostics.get(round(record["stamp"], 3), [])
    if not candidates:
        return None
    best = min(
        candidates,
        key=lambda item: math.hypot(record["x"] - item["x"], record["y"] - item["y"]),
    )
    if math.hypot(record["x"] - best["x"], record["y"] - best["y"]) > 0.05:
        return None
    return best


def quantile(values, fraction):
    if not values:
        return math.nan
    values = sorted(values)
    index = min(len(values) - 1, int(round((len(values) - 1) * fraction)))
    return values[index]


def main(path, diagnostic_path=None):
    records = defaultdict(list)
    with open(path, encoding="utf-8") as stream:
        for line in stream:
            rec = json.loads(line)
            records[rec["type"]].append(rec)

    odom = sorted(records["odom"], key=lambda r: r["stamp"])
    boat_gt = records["gt"]
    det = records["det"]
    if not odom or not boat_gt or not det:
        raise SystemExit("required odom/gt/det records are missing")

    canonical_offset = boat_gt[0]["stamp"] - odom[0]["stamp"]
    odom_mid = odom[len(odom) // 2]["stamp"]

    def align(items):
        if items and abs(items[len(items) // 2]["stamp"] - odom_mid) > 1000:
            for item in items:
                item["stamp"] -= canonical_offset

    for key in (
        "gt",
        "gt_buoy",
        "gt_pillar",
        "gt_block",
        "det",
        "det_buoy",
        "det_pillar",
        "det_block",
    ):
        align(records[key])

    interpolate_odom = make_linear_interpolator(odom, "odom")
    gt_interpolators = {
        "boat": make_linear_interpolator(records["gt"], "poses"),
        "buoy": make_linear_interpolator(records["gt_buoy"], "poses"),
        "pillar": make_linear_interpolator(records["gt_pillar"], "poses"),
        "block": make_linear_interpolator(records["gt_block"], "poses"),
    }

    boat_hits = 0
    false_positives = []
    proximity_counts = {radius: Counter() for radius in RADII}
    nearest_attribution = Counter()
    distance_buckets = Counter()
    attribution_by_distance = defaultdict(Counter)
    overlap_patterns = Counter()
    nearest_distances = defaultdict(list)
    detection_records = []

    for frame in records["det"]:
        t = frame["stamp"]
        od = interpolate_odom(t)
        if od is None:
            continue
        yaw = quat_to_yaw(od["qx"], od["qy"], od["qz"], od["qw"])
        gt_by_category = {}
        for category, interpolator in gt_interpolators.items():
            gt_frame = interpolator(t)
            poses = gt_frame["poses"] if gt_frame else []
            gt_by_category[category] = [
                world_to_boat(p["x"], p["y"], od["x"], od["y"], yaw) for p in poses
            ]

        for pose in frame["poses"]:
            point = (pose["x"], pose["y"])
            boat_distance = min_distance(point, gt_by_category["boat"])
            if boat_distance < MATCH_TOL:
                boat_hits += 1
                detection_records.append(
                    {
                        "stamp": t,
                        "x": point[0],
                        "y": point[1],
                        "z": pose.get("z", 0.0),
                        "range": math.hypot(*point),
                        "attribution": "boat_gt",
                    }
                )
                continue

            category_distances = {
                category: min_distance(point, gt_by_category[category])
                for category in OTHER_CATEGORIES
            }
            sensor_distance = math.hypot(*point)
            bucket = distance_bucket(sensor_distance)
            distance_buckets[bucket] += 1

            for radius in RADII:
                matched = [
                    category
                    for category, distance in category_distances.items()
                    if distance < radius
                ]
                for category in matched:
                    proximity_counts[radius][category] += 1
                if matched:
                    proximity_counts[radius]["any_other_gt"] += 1

            matched_2m = tuple(
                sorted(
                    category
                    for category, distance in category_distances.items()
                    if distance < MATCH_TOL
                )
            )
            overlap_patterns[matched_2m or ("none",)] += 1

            nearest_category, nearest_distance = min(
                category_distances.items(), key=lambda item: item[1]
            )
            if nearest_distance < MATCH_TOL:
                attribution = nearest_category
                nearest_distances[attribution].append(nearest_distance)
            else:
                attribution = "unmatched"
            nearest_attribution[attribution] += 1
            attribution_by_distance[bucket][attribution] += 1
            false_positives.append(
                {
                    "stamp": t,
                    "x": point[0],
                    "y": point[1],
                    "z": pose.get("z", 0.0),
                    "range": sensor_distance,
                    "attribution": attribution,
                    "nearest_distance": nearest_distance,
                }
            )
            detection_records.append(false_positives[-1])

    total_fp = len(false_positives)
    print(f"canonical_epoch_offset={canonical_offset:.6f}")
    print(f"boat detections matching boat GT={boat_hits}")
    print(f"boat false positives={total_fp}")

    print("\nSensitivity: FP positions near other-category GT")
    print("radius  buoy  pillar  block  any_other_gt")
    for radius in RADII:
        counts = proximity_counts[radius]
        print(
            f"{radius:>4.1f}m "
            f"{counts['buoy']:>6} {counts['pillar']:>7} {counts['block']:>6} "
            f"{counts['any_other_gt']:>13}"
        )

    print("\nNearest-category attribution at 2.0 m")
    for category in (*OTHER_CATEGORIES, "unmatched"):
        count = nearest_attribution[category]
        pct = 100 * count / total_fp if total_fp else 0
        distances = nearest_distances[category]
        if distances:
            distances.sort()
            median = distances[len(distances) // 2]
            detail = f", median separation={median:.3f}m"
        else:
            detail = ""
        print(f"{category:>9}: {count:>5} ({pct:>5.1f}%){detail}")

    print("\nOverlap patterns within 2.0 m")
    for pattern, count in overlap_patterns.most_common():
        print(f"{'+'.join(pattern):>20}: {count:>5}")

    print("\nAttribution by detection range")
    print("range       total  buoy  pillar  block  unmatched")
    for bucket in ("0-20m", "20-40m", "40-60m", "60m+"):
        counts = attribution_by_distance[bucket]
        total = distance_buckets[bucket]
        print(
            f"{bucket:<10} {total:>5} {counts['buoy']:>5} {counts['pillar']:>7} "
            f"{counts['block']:>6} {counts['unmatched']:>9}"
        )

    if diagnostic_path:
        diagnostics = parse_diagnostics(diagnostic_path, canonical_offset)
        linked = 0
        feature_groups = defaultdict(list)
        linked_records = []
        for record in detection_records:
            diagnostic = attach_diagnostic(record, diagnostics)
            if diagnostic is None:
                continue
            linked += 1
            feature_groups[record["attribution"]].append(diagnostic)
            linked_records.append((record, diagnostic))

        print(f"\nDiagnostic linkage: {linked}/{len(detection_records)} detections")
        print("group       n  fallback%  fp_max(p10/50/90)  dz(p10/50/90)  pts(p10/50/90)")
        for group in ("boat_gt", "buoy", "pillar", "block", "unmatched"):
            items = feature_groups[group]
            fallback = (
                100 * sum(item["label"] == "boat_fallback" for item in items) / len(items)
                if items
                else 0
            )
            fp_max = [item["fp_max"] for item in items]
            dz = [item["dz"] for item in items]
            pts = [item["pts"] for item in items]
            print(
                f"{group:<10} {len(items):>4} {fallback:>9.1f}%  "
                f"{quantile(fp_max, .1):.2f}/{quantile(fp_max, .5):.2f}/{quantile(fp_max, .9):.2f}       "
                f"{quantile(dz, .1):.2f}/{quantile(dz, .5):.2f}/{quantile(dz, .9):.2f}    "
                f"{quantile(pts, .1):.0f}/{quantile(pts, .5):.0f}/{quantile(pts, .9):.0f}"
            )

        def print_scan(title, candidates, predicate):
            print(f"\n{title}")
            print("value  total  boat_gt  buoy  pillar  block  unmatched")
            for value in candidates:
                counts = Counter(
                    record["attribution"]
                    for record, diagnostic in linked_records
                    if predicate(diagnostic, value)
                )
                total = sum(counts.values())
                print(
                    f"{value:>5} {total:>6} {counts['boat_gt']:>8} {counts['buoy']:>5} "
                    f"{counts['pillar']:>7} {counts['block']:>6} {counts['unmatched']:>9}"
                )

        def evaluate_frames(category, frames):
            hit = miss = fp = 0
            for frame in frames:
                t = frame["stamp"]
                od = interpolate_odom(t)
                gt_frame = gt_interpolators[category](t)
                if od is None:
                    continue
                if gt_frame is None or not gt_frame["poses"]:
                    fp += len(frame["poses"])
                    continue
                yaw = quat_to_yaw(od["qx"], od["qy"], od["qz"], od["qw"])
                targets = [
                    world_to_boat(p["x"], p["y"], od["x"], od["y"], yaw)
                    for p in gt_frame["poses"]
                ]
                matched = set()
                for pose in frame["poses"]:
                    point = (pose["x"], pose["y"])
                    distances = [
                        math.hypot(point[0] - target[0], point[1] - target[1])
                        for target in targets
                    ]
                    if distances and min(distances) < MATCH_TOL:
                        matched.add(min(range(len(distances)), key=distances.__getitem__))
                    else:
                        fp += 1
                hit += len(matched)
                miss += len(targets) - len(matched)
            recall = 100 * hit / (hit + miss) if hit + miss else 0
            fpr = 100 * fp / (hit + fp) if hit + fp else 0
            return hit, miss, fp, recall, fpr

        def simulate_reclassification(target_category, value, predicate):
            selected = {
                (
                    round(record["stamp"], 6),
                    round(record["x"], 4),
                    round(record["y"], 4),
                )
                for record, diagnostic in linked_records
                if predicate(diagnostic, value)
            }
            moved_by_stamp = defaultdict(list)
            filtered_boat = []
            for frame in records["det"]:
                kept = []
                for pose in frame["poses"]:
                    key = (
                        round(frame["stamp"], 6),
                        round(pose["x"], 4),
                        round(pose["y"], 4),
                    )
                    if key in selected:
                        moved_by_stamp[round(frame["stamp"], 6)].append(dict(pose))
                    else:
                        kept.append(dict(pose))
                filtered_boat.append({"stamp": frame["stamp"], "poses": kept})

            target_key = f"det_{target_category}"
            augmented_target = []
            for frame in records[target_key]:
                poses = list(map(dict, frame["poses"]))
                poses.extend(moved_by_stamp.get(round(frame["stamp"], 6), []))
                augmented_target.append({"stamp": frame["stamp"], "poses": poses})

            return (
                len(selected),
                evaluate_frames("boat", filtered_boat),
                evaluate_frames(target_category, augmented_target),
            )

        print_scan(
            "Candidate: raise pillar_max_pts (currently 3000)",
            (3500, 4000, 5000, 6000, 7500, 8000),
            lambda d, cap: (
                3000 < d["pts"] <= cap
                and d["fp_max"] <= 2.0
                and d["dz"] >= 1.5
                and d["dz"] / max(d["fp_max"], 0.05) >= 2.5
            ),
        )
        print_scan(
            "Candidate: lower block_fp_min (currently 1.7)",
            (1.65, 1.60, 1.55, 1.50),
            lambda d, minimum: (
                minimum <= d["fp_max"] < 1.7
                and d["fp_max"] <= 3.2
                and d["dz"] <= 1.5
                and d["square"] >= 0.55
                and d["pts"] >= 10
                and abs(d["z"]) <= 1.0
            ),
        )
        print_scan(
            "Candidate: raise buoy_fp_max (currently 1.0)",
            (1.05, 1.10, 1.15, 1.20),
            lambda d, maximum: (
                1.0 < d["fp_max"] <= maximum
                and d["dz"] <= 1.5
            ),
        )

        simulations = (
            (
                "pillar_max_pts",
                "pillar",
                (3500, 4000, 5000, 6000, 7500, 8000),
                lambda d, cap: (
                    3000 < d["pts"] <= cap
                    and d["fp_max"] <= 2.0
                    and d["dz"] >= 1.5
                    and d["dz"] / max(d["fp_max"], 0.05) >= 2.5
                ),
            ),
            (
                "block_fp_min",
                "block",
                (1.65, 1.60, 1.55, 1.50),
                lambda d, minimum: (
                    minimum <= d["fp_max"] < 1.7
                    and d["fp_max"] <= 3.2
                    and d["dz"] <= 1.5
                    and d["square"] >= 0.55
                    and d["pts"] >= 10
                    and abs(d["z"]) <= 1.0
                ),
            ),
            (
                "buoy_fp_max",
                "buoy",
                (1.05, 1.10, 1.15, 1.20),
                lambda d, maximum: 1.0 < d["fp_max"] <= maximum and d["dz"] <= 1.5,
            ),
        )
        print("\nEvaluator-faithful reclassification simulation (linked detections only)")
        print(
            "parameter=value moved | boat: recall/fpr/hit/fp | "
            "target: recall/fpr/hit/fp"
        )
        for name, target, values, predicate in simulations:
            for value in values:
                moved, boat_metrics, target_metrics = simulate_reclassification(
                    target, value, predicate
                )
                bh, _, bfp, br, bfpr = boat_metrics
                th, _, tfp, tr, tfpr = target_metrics
                print(
                    f"{name}={value} {moved:>4} | "
                    f"boat {br:>4.1f}%/{bfpr:>4.1f}%/{bh}/{bfp} | "
                    f"{target} {tr:>4.1f}%/{tfpr:>4.1f}%/{th}/{tfp}"
                )


if __name__ == "__main__":
    main(
        sys.argv[1] if len(sys.argv) > 1 else "/home/lyf040817/usv_ws/boat_log.jsonl",
        sys.argv[2] if len(sys.argv) > 2 else None,
    )
