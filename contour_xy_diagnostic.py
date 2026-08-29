#!/usr/bin/env python3
"""
轮廓/尺寸精度诊断：本轮范围收紧为 pillar XY(主基准) + block XY(旋转修正验证)。

- pillar: 圆柱体半径固定0.8m，XY footprint在任意朝向下都是1.6x1.6，旋转不变，
  不需要任何朝向修正——它的XY偏差就是纯粹的检测收缩(近端面效应等)，是本诊断
  唯一无需假设、直接可读的干净基准。
- block: 2x2m方形浮块，AABB会随本船-方块相对朝向变化(方形转45度时AABB膨胀最多
  sqrt(2)倍)，必须用GT朝向减去本船朝向算出相对yaw，把canonical边长旋转到
  base_link系再和检测值比较，否则旋转膨胀和检测收缩两个方向相反的效应会互相
  抵消，测出"尺寸挺准"的假象。每条记录同时算修正前(uncorrected, 相当于假设
  始终对齐)和修正后(corrected)两个期望值——如果这批数据里block大多接近轴对齐，
  两者应该很接近；如果差异很大，修正后的偏差应该比未修正的更接近pillar这个
  纯收缩基准，否则说明朝向对齐或坐标变换本身有错，不能倒过来解读。
- buoy本轮不做：round(球r=0.25→直径0.5)和marker(套环r=0.325→直径0.65)两型
  直径差30%，GT话题和检测输出都不带类型字段，无法确定具体某个实例是哪型，
  混着比会把量级相当的未知类型混淆量塞进本来要精确测量的近端面效应里，测出
  的数字不能证实也不能证伪任何结论。boat/z轴同样推后(z受水面滤波影响，boat
  是唯一动态类别，历史上sensor-clock-gap类修正对它有害，需要单独处理)。

复用 four_category_evaluator_v2.py 里已验证过的：epoch对齐(resolve_epoch_offset/
align_to_odom)、odom插值(make_odom_interpolator)、world_to_boat坐标变换、
最大基数二分匹配(maximum_cardinality_matches)——这些是这条线上历史上最容易出
错的部分(第四节的时钟域bug)，不重新实现。

数据来源: boat_logger.py（已加orientation捕获给gt_block、clock_stamp给odom）
录制的JSONL日志，需要重新采集(历史日志没有这两个新字段)。

用法:
    python3 contour_xy_diagnostic.py boat_log.jsonl [--match-tol 2.0] [--voxel-pad 0.20]
"""
import argparse
import bisect
import math
import statistics
from collections import defaultdict

import four_category_evaluator_v2 as fce

PILLAR_DIAMETER = 1.6  # 2 * radius(0.8m)
BLOCK_SIDE = 2.0

DIST_BUCKETS = [(0.0, 20.0), (20.0, 40.0), (40.0, math.inf)]


def bucket_label(d):
    for lo, hi in DIST_BUCKETS:
        if lo <= d < hi:
            return f"{lo:g}-{hi:g}m" if hi != math.inf else f"{lo:g}m+"
    return "?"


def quat_yaw(qx, qy, qz, qw):
    return math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy ** 2 + qz ** 2))


def make_gt_interpolator_oriented(records):
    """同fce.make_gt_interpolator的防御逻辑(pose数量不一致时退化到更近的单帧)，
    额外带朝向四元数。朝向不做slerp——直接取时间上更近一帧的朝向，浮块朝向变化
    慢，没必要为此单独实现四元数插值。"""
    records = sorted(records, key=lambda item: item["stamp"])
    stamps = [item["stamp"] for item in records]

    def interpolate(t):
        if not records:
            return None
        index = bisect.bisect_left(stamps, t)
        if index == 0:
            return records[0] if t >= stamps[0] - fce.MAX_TIME_GAP else None
        if index >= len(records):
            return records[-1] if t <= stamps[-1] + fce.MAX_TIME_GAP else None
        left, right = records[index - 1], records[index]
        if t < left["stamp"] - fce.MAX_TIME_GAP or t > right["stamp"] + fce.MAX_TIME_GAP:
            return None
        nearer = left if abs(t - left["stamp"]) <= abs(t - right["stamp"]) else right
        left_poses, right_poses = left["poses"], right["poses"]
        if len(left_poses) != len(right_poses):
            return nearer
        dt = right["stamp"] - left["stamp"]
        if abs(dt) < 1e-9:
            return left
        alpha = (t - left["stamp"]) / dt
        poses = [
            {
                "x": p0["x"] + (p1["x"] - p0["x"]) * alpha,
                "y": p0["y"] + (p1["y"] - p0["y"]) * alpha,
                "qx": pn["qx"], "qy": pn["qy"], "qz": pn["qz"], "qw": pn["qw"],
            }
            for p0, p1, pn in zip(left_poses, right_poses, nearer["poses"])
        ]
        return {"poses": poses}

    return interpolate


def group_cluster_frames(cluster_records, label):
    """cluster_diagnostic按单条记录写入(boat_logger.py解析main.cpp的
    [cluster_diagnostic]日志行)，不像det_*那样已经打包成一帧一个MarkerArray——
    这里按t分组、只保留label匹配的条目，伪装成和det_pillar同样的{stamp,poses}
    结构，直接复用现成的匹配管线。cluster的't'字段来自main.cpp的publish_stamp_sec
    (now()时钟)，和det_pillar同一个域，用同一套epoch shift即可，不需要单独对齐。"""
    frames = defaultdict(list)
    for r in cluster_records:
        if r.get("label") != label:
            continue
        frames[r["t"]].append(r)
    return [
        {"stamp": t, "poses": [{"x": c["x"], "y": c["y"], "pts": c["pts"]} for c in items]}
        for t, items in sorted(frames.items())
    ]


def analyze_pillar_cluster_pts(gt_records, cluster_frames, interpolate_odom, match_tol):
    """回答"40m+的收缩是几何遮挡还是点云稀疏"：按距离桶统计匹配到的pillar
    cluster点数分布，而不是尺寸——点数本身不受voxel_pad/旋转修正等假设影响，
    是本诊断里最少假设的一个量。"""
    interpolate_gt = fce.make_gt_interpolator(gt_records)
    samples = []
    skipped = 0
    for frame in cluster_frames:
        t = frame["stamp"]
        odom = interpolate_odom(t)
        gt = interpolate_gt(t)
        if odom is None or gt is None:
            skipped += 1
            continue
        targets = [fce.world_to_boat(pose, odom) for pose in gt["poses"]]
        points = [(pose["x"], pose["y"]) for pose in frame["poses"]]
        pairs, _ = fce.maximum_cardinality_matches(points, targets, match_tol)
        for det_idx, tgt_idx in pairs:
            tx, ty = targets[tgt_idx]
            dist = math.hypot(tx, ty)
            samples.append({"dist": dist, "pts": frame["poses"][det_idx]["pts"]})
    return samples, skipped, len(cluster_frames)


def summarize_pts(samples):
    print("\n=== [pillar] 匹配cluster点数 vs 距离(区分几何遮挡 vs 采样稀疏) ===")
    if not samples:
        print("(无样本 —— 检查CLUSTER_LOG_PATH是否指向了node实际写日志的文件)")
        return
    buckets = defaultdict(list)
    for s in samples:
        buckets[bucket_label(s["dist"])].append(s["pts"])
    for lo, hi in DIST_BUCKETS:
        label = bucket_label(lo)
        pts = buckets.get(label, [])
        if not pts:
            print(f"  {label:>8}: (无样本)")
            continue
        pts_sorted = sorted(pts)
        p10 = pts_sorted[int(len(pts_sorted) * 0.1)]
        p90 = pts_sorted[min(len(pts_sorted) - 1, int(len(pts_sorted) * 0.9))]
        print(
            f"  {label:>8}: n={len(pts):>4}  "
            f"pts 中位={statistics.median(pts):.1f}  均值={statistics.fmean(pts):.1f}  "
            f"p10={p10}  p90={p90}  min={min(pts)}  max={max(pts)}"
        )


def analyze_pillar(gt_records, det_records, interpolate_odom, match_tol, voxel_pad):
    """圆柱体，XY旋转不变——不需要本船/目标朝向，纯位置匹配+尺寸比较。"""
    detections, _, _ = fce.deduplicate_frames(det_records)
    interpolate_gt = fce.make_gt_interpolator(gt_records)
    expected = PILLAR_DIAMETER + voxel_pad
    samples = []
    skipped = 0
    all_detections = []

    for frame in detections:
        t = frame["stamp"]
        odom = interpolate_odom(t)
        gt = interpolate_gt(t)
        if odom is None or gt is None:
            skipped += 1
            continue
        targets = [fce.world_to_boat(pose, odom) for pose in gt["poses"]]
        points = [(pose["x"], pose["y"]) for pose in frame["poses"]]
        pairs, _ = fce.maximum_cardinality_matches(points, targets, match_tol)
        matched_det = {d for d, _ in pairs}
        for det_idx, tgt_idx in pairs:
            tx, ty = targets[tgt_idx]
            dist = math.hypot(tx, ty)
            det_dx = frame["poses"][det_idx]["dx"]
            det_dy = frame["poses"][det_idx]["dy"]
            gt_map_x = gt["poses"][tgt_idx]["x"]
            gt_map_y = gt["poses"][tgt_idx]["y"]
            samples.append({
                "dist": dist, "det_dx": det_dx, "det_dy": det_dy,
                "expected_dx": expected, "expected_dy": expected,
                "tgt_idx": tgt_idx, "gt_map_x": gt_map_x, "gt_map_y": gt_map_y,
            })
        # 未匹配的检测：用检测自身在base_link下的位置算距离(base_link原点=本船)，
        # 用来算每桶的匹配率——分母是"该桶里到底有多少个检测"，不是"匹配上了
        # 多少个"，这两个此前一直没有区分开。
        for det_idx, (px, py) in enumerate(points):
            if det_idx not in matched_det:
                all_detections.append({"dist": math.hypot(px, py), "matched": False})
        for det_idx, tgt_idx in pairs:
            px, py = points[det_idx]
            all_detections.append({"dist": math.hypot(px, py), "matched": True})
    return samples, skipped, len(detections), all_detections


def analyze_block(gt_records, det_records, interpolate_odom, match_tol, voxel_pad):
    """方形，AABB随相对朝向变化——每个匹配同时算corrected(带旋转修正)和
    uncorrected(假设始终轴对齐, θ=0)两个期望值。"""
    detections, _, _ = fce.deduplicate_frames(det_records)
    interpolate_gt = make_gt_interpolator_oriented(gt_records)
    uncorrected_expected = BLOCK_SIDE + voxel_pad
    samples = []
    skipped = 0

    for frame in detections:
        t = frame["stamp"]
        odom = interpolate_odom(t)
        gt = interpolate_gt(t)
        if odom is None or gt is None:
            skipped += 1
            continue
        targets_xy = [fce.world_to_boat(pose, odom) for pose in gt["poses"]]
        points = [(pose["x"], pose["y"]) for pose in frame["poses"]]
        pairs, _ = fce.maximum_cardinality_matches(points, targets_xy, match_tol)
        ship_yaw = fce.quat_to_yaw(odom)
        for det_idx, tgt_idx in pairs:
            tx, ty = targets_xy[tgt_idx]
            dist = math.hypot(tx, ty)
            gt_pose = gt["poses"][tgt_idx]
            obj_yaw = quat_yaw(gt_pose["qx"], gt_pose["qy"], gt_pose["qz"], gt_pose["qw"])
            rel_yaw = obj_yaw - ship_yaw
            c, s = abs(math.cos(rel_yaw)), abs(math.sin(rel_yaw))
            corrected = BLOCK_SIDE * c + BLOCK_SIDE * s + voxel_pad
            det_dx = frame["poses"][det_idx]["dx"]
            det_dy = frame["poses"][det_idx]["dy"]
            samples.append({
                "dist": dist, "det_dx": det_dx, "det_dy": det_dy,
                "expected_dx": corrected, "expected_dy": corrected,
                "expected_dx_uncorrected": uncorrected_expected,
                "expected_dy_uncorrected": uncorrected_expected,
                "rel_yaw_deg": math.degrees(rel_yaw),
            })
    return samples, skipped, len(detections)


# 第八节pillar误检调查已定位的未标注结构坐标区域(map系)——检查是否有真实
# GT立柱恰好落在附近，检测可能混入了这块区域里未标注结构的点。
UNLABELED_REGION_X = (-720.0, -580.0)
UNLABELED_REGION_Y = (290.0, 450.0)


def in_unlabeled_region(x, y):
    return UNLABELED_REGION_X[0] <= x <= UNLABELED_REGION_X[1] and \
        UNLABELED_REGION_Y[0] <= y <= UNLABELED_REGION_Y[1]


def summarize_pillar_integrity(samples, all_detections):
    """样本独立性/匹配质量自查：n是"帧×目标"的乘积，不是独立样本数；匹配率低
    说明大量检测没有对应真值；未标注结构区域内的GT若被匹配上，其检测大概率
    混入了旁边未标注结构的点，会污染尺寸统计而不被发现。"""
    print("\n=== [pillar] 匹配质量自查(样本独立性/匹配率/未标注区域重叠) ===")
    for lo, hi in DIST_BUCKETS:
        label = bucket_label(lo)
        bucket_samples = [s for s in samples if lo <= s["dist"] < hi]
        bucket_dets = [d for d in all_detections if lo <= d["dist"] < hi]
        n_matched = sum(1 for d in bucket_dets if d["matched"])
        n_total = len(bucket_dets)
        match_rate = 100 * n_matched / n_total if n_total else 0.0
        distinct_targets = len({s["tgt_idx"] for s in bucket_samples})
        in_region = sum(
            1 for s in bucket_samples
            if in_unlabeled_region(s["gt_map_x"], s["gt_map_y"])
        )
        distinct_in_region = len({
            s["tgt_idx"] for s in bucket_samples
            if in_unlabeled_region(s["gt_map_x"], s["gt_map_y"])
        })
        print(
            f"  {label:>8}: 样本数(帧x目标)={len(bucket_samples):>6}  "
            f"检测总数={n_total:>6}  匹配上={n_matched:>6}  匹配率={match_rate:.1f}%  "
            f"不同GT立柱实例数={distinct_targets:>3}  "
            f"落在未标注区域内的样本={in_region}(对应{distinct_in_region}个实例)"
        )


def summarize(name, samples, show_uncorrected=False):
    print(f"\n=== [{name}] XY尺寸诊断 ===")
    print(f"匹配样本数={len(samples)}")
    if not samples:
        print("(无匹配样本)")
        return

    buckets = defaultdict(list)
    for s in samples:
        buckets[bucket_label(s["dist"])].append(s)

    for lo, hi in DIST_BUCKETS:
        label = bucket_label(lo)
        rows = buckets.get(label, [])
        if not rows:
            print(f"  {label:>8}: (无样本)")
            continue
        err_x = [r["det_dx"] - r["expected_dx"] for r in rows]
        err_y = [r["det_dy"] - r["expected_dy"] for r in rows]
        rel_x = [(r["det_dx"] - r["expected_dx"]) / r["expected_dx"] for r in rows]
        rel_y = [(r["det_dy"] - r["expected_dy"]) / r["expected_dy"] for r in rows]
        print(
            f"  {label:>8}: n={len(rows):>4}  "
            f"dx误差 中位={statistics.median(err_x):+.3f}m({statistics.median(rel_x)*100:+.1f}%)  "
            f"dy误差 中位={statistics.median(err_y):+.3f}m({statistics.median(rel_y)*100:+.1f}%)  "
            f"det_dx中位={statistics.median(r['det_dx'] for r in rows):.3f}m  "
            f"expected_dx中位={statistics.median(r['expected_dx'] for r in rows):.3f}m"
        )
        if show_uncorrected and "expected_dx_uncorrected" in rows[0]:
            err_x_unc = [r["det_dx"] - r["expected_dx_uncorrected"] for r in rows]
            rel_x_unc = [(r["det_dx"] - r["expected_dx_uncorrected"]) / r["expected_dx_uncorrected"] for r in rows]
            avg_abs_rel_yaw = statistics.median(abs(r["rel_yaw_deg"]) for r in rows)
            print(
                f"           (未修正对照) dx误差 中位={statistics.median(err_x_unc):+.3f}m"
                f"({statistics.median(rel_x_unc)*100:+.1f}%)  "
                f"相对yaw绝对值中位={avg_abs_rel_yaw:.1f}°"
            )

    all_err_x = [r["det_dx"] - r["expected_dx"] for r in samples]
    all_rel_x = [(r["det_dx"] - r["expected_dx"]) / r["expected_dx"] for r in samples]
    print(
        f"  {'全距离':>8}: n={len(samples):>4}  "
        f"dx误差 中位={statistics.median(all_err_x):+.3f}m({statistics.median(all_rel_x)*100:+.1f}%)  "
        f"均值={statistics.fmean(all_err_x):+.3f}m  "
        f"stdev={statistics.stdev(all_err_x) if len(all_err_x) > 1 else 0:.3f}m"
    )
    if show_uncorrected and "expected_dx_uncorrected" in samples[0]:
        all_err_x_unc = [r["det_dx"] - r["expected_dx_uncorrected"] for r in samples]
        print(
            f"           (未修正对照全距离) dx误差 中位="
            f"{statistics.median(all_err_x_unc):+.3f}m  均值={statistics.fmean(all_err_x_unc):+.3f}m"
        )
        # rel_yaw分布自查：|cos|+|sin|对符号和±90°不敏感，公式算错了也不一定能从
        # 误差数字本身看出来。场景里block朝向若接近随机，折算到0-45°后应大致均匀；
        # 挤在窄区间大概率是yaw定义/旋转方向和world_to_boat没对齐，而不是场景真这样。
        folded = []
        for r in samples:
            a = abs(r["rel_yaw_deg"]) % 90.0
            folded.append(a if a <= 45.0 else 90.0 - a)
        edges = [(0, 15), (15, 30), (30, 45)]
        print("           rel_yaw折算到0-45°分布(自查用，场景随机朝向下应大致均匀):")
        for lo, hi in edges:
            n = sum(1 for a in folded if lo <= a < hi or (hi == 45 and a == 45))
            print(f"             {lo:>2}-{hi:>2}°: {n:>4} ({100*n/len(folded):.0f}%)")


def align_category(records, odom0, gt_key, det_key):
    """gt/det/odom三个时钟域各自独立(gt是原始录制的wall-clock epoch，det是本次
    回放会话的wall-clock epoch，odom是从0开始的sim-time)，fce.resolve_epoch_offset
    的clock_stamp方法对不上——那个方法对齐的是/clock与odom(两者本就都是sim-time，
    算出offset=0，是另一个问题的正确答案，不是这个)。也不用--legacy-offset-fallback
    (未经本session验证的历史数字)。这里改为直接从当前这份log用每个类别自己的首帧
    锚点计算：同一类别内部锚点用多个独立GT话题(gt/gt_buoy/gt_block)交叉验证过，
    互相精确吻合(差异<1ms)，只有gt_pillar因为自己是1Hz(其余是3Hz)有约0.667秒的
    独立发布相位偏移——这是话题自身启动时序的正常现象，不是跨域漂移，所以按类别
    独立对齐，不强求所有类别共用同一个锚点。
    早前一版曾用"首/中/尾索引"直接比较gt_pillar和odom，误判成"8.4秒漂移"——
    实际原因是odom这段录制比gt早结束约9秒，索引位置不对应同一时刻，不是真漂移，
    已废弃那种比法。"""
    gt_records = records[gt_key]
    det_records = records[det_key]
    if not gt_records or not det_records:
        return None
    gt_anchor = gt_records[0]["stamp"] - odom0
    det_gap = det_records[0]["stamp"] - gt_records[0]["stamp"]
    for item in gt_records:
        item["stamp"] -= gt_anchor
    for item in det_records:
        item["stamp"] -= (gt_anchor + det_gap)
    print(f"[{gt_key}/{det_key}] gt_anchor(vs odom[0])={gt_anchor:.3f}s  "
          f"det_gap(vs {gt_key}[0])={det_gap:.3f}s  (均为本次log现测，非历史值)")
    return gt_anchor, det_gap


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("log_path")
    parser.add_argument("--match-tol", type=float, default=2.0)
    parser.add_argument("--voxel-pad", type=float, default=0.20,
                         help="ObstacleDetector cfg_.voxel默认值，需要和C++侧实际配置保持一致")
    args = parser.parse_args()

    records = fce.load_records(args.log_path)
    if not records["odom"]:
        raise SystemExit("No odom records")

    odom0 = records["odom"][0]["stamp"]
    has_pillar = align_category(records, odom0, "gt_pillar", "det_pillar")
    has_block = align_category(records, odom0, "gt_block", "det_block")

    interpolate_odom = fce.make_odom_interpolator(records["odom"])

    if has_pillar:
        samples, skipped, det_n, all_detections = analyze_pillar(
            records["gt_pillar"], records["det_pillar"], interpolate_odom,
            args.match_tol, args.voxel_pad,
        )
        print(f"\n[pillar] 检测帧数={det_n} 跳过(odom/gt缺失)={skipped}")
        summarize("pillar (主基准，无需修正)", samples)
        summarize_pillar_integrity(samples, all_detections)

        if records["cluster"]:
            # cluster_diagnostic的't'和det_pillar同域(都是main.cpp的publish_stamp_sec/
            # now())，用pillar同一组(gt_anchor,det_gap)平移，不用重新求一遍。
            gt_anchor, det_gap = has_pillar
            cluster_records = [dict(r) for r in records["cluster"]]
            for item in cluster_records:
                item["t"] -= (gt_anchor + det_gap)
            cluster_frames = group_cluster_frames(cluster_records, "pillar")
            pts_samples, pts_skipped, pts_frames_n = analyze_pillar_cluster_pts(
                records["gt_pillar"], cluster_frames, interpolate_odom, args.match_tol
            )
            print(f"\n[pillar cluster pts] cluster帧数={pts_frames_n} 跳过={pts_skipped}")
            summarize_pts(pts_samples)
        else:
            print("\n[pillar cluster pts] 没有cluster记录 —— 需要CLUSTER_LOG_PATH"
                  "指向node实际的stdout日志文件，重新采集一次")
    else:
        print("\n[pillar] 缺少gt_pillar或det_pillar记录，跳过")

    if has_block:
        first_nonempty = next(
            (r for r in records["gt_block"] if r["poses"]), None
        )
        if first_nonempty is None or "qx" not in first_nonempty["poses"][0]:
            print("\n[block] gt_block记录里没有朝向字段——用的是旧版boat_logger.py录制的日志，"
                  "需要用更新后的boat_logger.py重新录制")
        else:
            samples, skipped, det_n = analyze_block(
                records["gt_block"], records["det_block"], interpolate_odom,
                args.match_tol, args.voxel_pad,
            )
            print(f"\n[block] 检测帧数={det_n} 跳过(odom/gt缺失)={skipped}")
            summarize("block (旋转修正 vs 未修正对照)", samples, show_uncorrected=True)
    else:
        print("\n[block] 缺少gt_block或det_block记录，跳过")


if __name__ == "__main__":
    main()
