#!/usr/bin/env python3
"""Four-category evaluator with deterministic time and counting semantics.

Differences from the legacy evaluator:
1. Duplicate detection messages with the same sensor stamp are counted once.
2. Epoch alignment can use an explicit validated offset or logger clock evidence.
3. Detection-to-GT matching is one-to-one, so duplicate detections become FPs.
4. Raw/unique/conflicting stamp counts are reported for every category.
"""

import argparse
import bisect
import json
import math
import statistics
from collections import defaultdict


MATCH_TOL = 2.0
MAX_TIME_GAP = 0.5
CATEGORIES = (
    ("boat", "gt", "det"),
    ("buoy", "gt_buoy", "det_buoy"),
    ("pillar", "gt_pillar", "det_pillar"),
    ("block", "gt_block", "det_block"),
)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("log_path")
    parser.add_argument(
        "--epoch-offset",
        type=float,
        help="Validated absolute-epoch to odom-relative offset.",
    )
    parser.add_argument("--match-tol", type=float, default=MATCH_TOL)
    parser.add_argument(
        "--legacy-offset-fallback",
        action="store_true",
        help="Allow gt[0]-odom[0] when no validated/clock-derived offset exists.",
    )
    parser.add_argument(
        "--sensor-clock-gap",
        type=float,
        default=0.0,
        help=(
            "Additional seconds subtracted from det/det_buoy/det_pillar/det_block "
            "stamps only, on top of --epoch-offset. Corrects a separate clock "
            "domain gap between ros2 bag play's live-synthesized /clock (which "
            "the node's now()-based publish stamp tracks) and the bag's own "
            "recorded sensor/GT/odom clock domain (confirmed ~9.0s for "
            "bag_09_03_17 via a full-population dt scan, 2026-08-25). Does not "
            "touch GT/odom, which are already correctly aligned by --epoch-offset "
            "alone - this is deliberately a separate, opt-in correction so the "
            "already-validated recall/FP pipeline is unaffected unless explicitly "
            "requested."
        ),
    )
    return parser.parse_args()


def load_records(path):
    records = defaultdict(list)
    with open(path, encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            try:
                record = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(f"invalid JSON at line {line_number}: {exc}") from exc
            records[record["type"]].append(record)
    return records


def pose_signature(frame):
    return tuple(
        (
            round(pose["x"], 5),
            round(pose["y"], 5),
            round(pose.get("z", 0.0), 5),
        )
        for pose in frame.get("poses", ())
    )


def deduplicate_frames(frames):
    groups = defaultdict(list)
    for frame in frames:
        groups[round(frame["stamp"], 6)].append(frame)

    unique = []
    duplicate_records = 0
    conflicting_stamps = 0
    for stamp in sorted(groups):
        copies = groups[stamp]
        duplicate_records += len(copies) - 1
        signatures = {pose_signature(copy) for copy in copies}
        if len(signatures) > 1:
            conflicting_stamps += 1
            # A later callback is the best representation of the final published state.
            unique.append(copies[-1])
        else:
            unique.append(copies[0])
    return unique, duplicate_records, conflicting_stamps


def resolve_epoch_offset(records, explicit, allow_legacy):
    if explicit is not None:
        return explicit, "explicit validated offset"

    clock_offsets = [
        record["clock_stamp"] - record["stamp"]
        for record in records["odom"]
        if "clock_stamp" in record
    ]
    if clock_offsets:
        return statistics.median(clock_offsets), "median(clock_stamp - odom_stamp)"

    if allow_legacy and records["gt"] and records["odom"]:
        offset = records["gt"][0]["stamp"] - records["odom"][0]["stamp"]
        return offset, "LEGACY gt[0]-odom[0] fallback (not robust)"

    raise SystemExit(
        "No robust epoch evidence in this log. Pass --epoch-offset with the "
        "validated value, or record clock_stamp in the logger."
    )


def align_to_odom(items, offset, odom_mid):
    if not items or "stamp" not in items[0]:
        return False
    sample = items[len(items) // 2]["stamp"]
    if abs(sample - odom_mid) <= 1000:
        return False
    for item in items:
        item["stamp"] -= offset
    return True


def make_odom_interpolator(records):
    # 越界钳制到首/末已知位姿是有害的静默行为（2026-07-24发现odom比gt早停约9秒，
    # 钳制会让这9秒里的每一帧都用同一个冻结位姿做world_to_boat变换，产生系统性
    # 错误坐标，四个类别的评估全部被污染，且不报错、不跳过，很难被发现）。
    # 边界处理和gt插值器保持一致：超出MAX_TIME_GAP容差直接返回None。
    records = sorted(records, key=lambda item: item["stamp"])
    stamps = [item["stamp"] for item in records]

    def interpolate(t):
        if not records:
            return None
        index = bisect.bisect_left(stamps, t)
        if index == 0:
            return records[0] if t >= stamps[0] - MAX_TIME_GAP else None
        if index >= len(records):
            return records[-1] if t <= stamps[-1] + MAX_TIME_GAP else None
        left, right = records[index - 1], records[index]
        dt = right["stamp"] - left["stamp"]
        if abs(dt) < 1e-9:
            return left
        alpha = (t - left["stamp"]) / dt
        return {
            key: left[key] + (right[key] - left[key]) * alpha
            for key in ("x", "y", "qx", "qy", "qz", "qw")
        }

    return interpolate


def make_gt_interpolator(records):
    records = sorted(records, key=lambda item: item["stamp"])
    stamps = [item["stamp"] for item in records]

    def interpolate(t):
        if not records:
            return None
        index = bisect.bisect_left(stamps, t)
        if index == 0:
            return records[0] if t >= stamps[0] - MAX_TIME_GAP else None
        if index >= len(records):
            return records[-1] if t <= stamps[-1] + MAX_TIME_GAP else None
        left, right = records[index - 1], records[index]
        if t < left["stamp"] - MAX_TIME_GAP or t > right["stamp"] + MAX_TIME_GAP:
            return None
        left_poses, right_poses = left["poses"], right["poses"]
        if len(left_poses) != len(right_poses):
            return left if abs(t - left["stamp"]) < abs(t - right["stamp"]) else right
        dt = right["stamp"] - left["stamp"]
        if abs(dt) < 1e-9:
            return left
        alpha = (t - left["stamp"]) / dt
        return {
            "poses": [
                {
                    "x": p0["x"] + (p1["x"] - p0["x"]) * alpha,
                    "y": p0["y"] + (p1["y"] - p0["y"]) * alpha,
                }
                for p0, p1 in zip(left_poses, right_poses)
            ]
        }

    return interpolate


def quat_to_yaw(odom):
    return math.atan2(
        2 * (odom["qw"] * odom["qz"] + odom["qx"] * odom["qy"]),
        1 - 2 * (odom["qy"] ** 2 + odom["qz"] ** 2),
    )


def world_to_boat(pose, odom):
    dx, dy = pose["x"] - odom["x"], pose["y"] - odom["y"]
    angle = -quat_to_yaw(odom)
    return (
        dx * math.cos(angle) - dy * math.sin(angle),
        dx * math.sin(angle) + dy * math.cos(angle),
    )


def maximum_cardinality_matches(detections, targets, tolerance):
    adjacency = []
    distances = {}
    for detection_index, detection in enumerate(detections):
        options = []
        for target_index, target in enumerate(targets):
            distance = math.hypot(
                detection[0] - target[0], detection[1] - target[1]
            )
            if distance < tolerance:
                options.append((distance, target_index))
                distances[(detection_index, target_index)] = distance
        adjacency.append([target for _, target in sorted(options)])

    target_to_detection = [-1] * len(targets)

    def augment(detection_index, visited):
        for target_index in adjacency[detection_index]:
            if visited[target_index]:
                continue
            visited[target_index] = True
            previous = target_to_detection[target_index]
            if previous == -1 or augment(previous, visited):
                target_to_detection[target_index] = detection_index
                return True
        return False

    order = sorted(range(len(detections)), key=lambda index: len(adjacency[index]))
    for detection_index in order:
        augment(detection_index, [False] * len(targets))

    pairs = [
        (detection_index, target_index)
        for target_index, detection_index in enumerate(target_to_detection)
        if detection_index != -1
    ]
    return pairs, [distances[pair] for pair in pairs]


def evaluate_category(
    name,
    gt_records,
    raw_detection_records,
    interpolate_odom,
    match_tolerance,
):
    detections, duplicate_records, conflicting_stamps = deduplicate_frames(
        raw_detection_records
    )
    interpolate_gt = make_gt_interpolator(gt_records)
    hit = miss = false_positive = 0
    errors = []
    skipped_gt = 0

    for frame in detections:
        t = frame["stamp"]
        odom = interpolate_odom(t)
        gt = interpolate_gt(t)
        if odom is None or gt is None:
            skipped_gt += 1
            continue

        targets = [world_to_boat(pose, odom) for pose in gt["poses"]]
        points = [(pose["x"], pose["y"]) for pose in frame["poses"]]
        pairs, pair_errors = maximum_cardinality_matches(
            points, targets, match_tolerance
        )
        frame_hits = len(pairs)
        hit += frame_hits
        miss += len(targets) - frame_hits
        false_positive += len(points) - frame_hits
        errors.extend(pair_errors)

    recall = 100 * hit / (hit + miss) if hit + miss else 0.0
    fpr = 100 * false_positive / (hit + false_positive) if hit + false_positive else 0.0
    mean_error = statistics.fmean(errors) if errors else 0.0
    covered_frames = len(detections) - skipped_gt
    gt_total = hit + miss
    result = {
        "name": name,
        "hit": hit,
        "miss": miss,
        "fp": false_positive,
        "recall": recall,
        "fpr": fpr,
        "det_count": len(detections),
        "covered_frames": covered_frames,
        "gt_total": gt_total,
    }
    print(f"\n=== [{name}] v2 ===")
    print(
        f"检测消息 raw={len(raw_detection_records)} unique={len(detections)} "
        f"duplicate_records={duplicate_records} "
        f"conflicting_duplicate_stamps={conflicting_stamps} skipped={skipped_gt}"
    )
    print(f"[口径] 有效帧数(odom+gt均覆盖)={covered_frames}  真值总数(命中+遗漏)={gt_total}")
    print(
        f"命中={hit} 遗漏={miss} 误检={false_positive} "
        f"召回率={recall:.1f}% 误检率={fpr:.1f}% "
        f"位置误差均值={mean_error:.2f}m"
    )
    return result


def main():
    args = parse_args()
    records = load_records(args.log_path)
    if not records["odom"]:
        raise SystemExit("No odom records")
    offset, offset_source = resolve_epoch_offset(
        records, args.epoch_offset, args.legacy_offset_fallback
    )
    odom_mid = records["odom"][len(records["odom"]) // 2]["stamp"]
    for record_type, items in records.items():
        if record_type != "odom":
            align_to_odom(items, offset, odom_mid)

    if args.sensor_clock_gap:
        det_keys = {key for _, _, key in CATEGORIES}
        for record_type in det_keys:
            for item in records[record_type]:
                item["stamp"] -= args.sensor_clock_gap

    print(f"epoch_offset={offset:.6f} source={offset_source} sensor_clock_gap={args.sensor_clock_gap:.3f}")
    print("matching=one-to-one maximum-cardinality; duplicate stamp=one frame")
    interpolate_odom = make_odom_interpolator(records["odom"])

    results = []
    for name, gt_key, detection_key in CATEGORIES:
        if not records[gt_key] or not records[detection_key]:
            print(f"\n=== [{name}] === missing data")
            continue
        results.append(
            evaluate_category(
                name,
                records[gt_key],
                records[detection_key],
                interpolate_odom,
                args.match_tol,
            )
        )

    total_hit = sum(result["hit"] for result in results)
    total_miss = sum(result["miss"] for result in results)
    total_fp = sum(result["fp"] for result in results)
    overall_recall = 100 * total_hit / (total_hit + total_miss)
    overall_fpr = 100 * total_fp / (total_hit + total_fp)
    print("\n=== 四类汇总 v2 ===")
    print(
        f"命中={total_hit} 遗漏={total_miss} 误检={total_fp} "
        f"总召回率={overall_recall:.1f}% 总误检率={overall_fpr:.1f}%"
    )

    det_count = results[0]["det_count"] if results else 0
    covered = results[0]["covered_frames"] if results else 0
    gt_total = sum(result["gt_total"] for result in results)
    print(f"\n[口径基准] det消息总数={det_count} 有效帧数(odom+gt均覆盖)={covered} 四类真值总数合计={gt_total}")
    print("[口径警告] 和其他跑测对比总召回率/总误检率之前，先比较这三个数字——")
    print("           如果det消息总数或有效帧数差异超过5%，说明两次统计的是不同的帧集合，")
    print("           总账不能直接比较涨跌，需要先统一时间窗口或帧集合再对比。")


if __name__ == "__main__":
    main()
