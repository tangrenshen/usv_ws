#!/usr/bin/env python3
"""
分析各距离桶的召回率/误检情况：
1. 分距离桶（0-20/20-40/40-60/60+）统计真值船数量与命中率
2. 20-40m范围内真值船详细统计
3. 对误检candidate做距离归因分析

时间对齐方式与boat_evaluator.py保持完全一致（interpolate_odom/interpolate_gt线性插值
+_align_to_odom偏移校正），不用find_nearest+硬容差——后者会在odom录制提前结束的
尾段（这份bag里odom比gt/det早停约9秒）把对应的det帧直接丢弃，systematic地漏掉那段
数据，曾经导致这个脚本和boat_evaluator.py在同一份数据上算出的真值总数对不上
（8580 vs 6890）。
"""
import json
import math
import bisect

BOAT_LOG_PATH = '/home/lyf040817/usv_ws/boat_log.jsonl'
MATCH_TOL = 2.0
MAX_TIME_GAP = 0.5

gt_records = []
odom_records = []
det_records = []

with open(BOAT_LOG_PATH) as f:
    for line in f:
        rec = json.loads(line)
        if rec['type'] == 'gt':
            gt_records.append(rec)
        elif rec['type'] == 'odom':
            odom_records.append(rec)
        elif rec['type'] == 'det':
            det_records.append(rec)

print(f"boat_log: gt={len(gt_records)} odom={len(odom_records)} det={len(det_records)}")

_epoch_offset = 0.0
if gt_records and odom_records:
    _epoch_offset = gt_records[0]['stamp'] - odom_records[0]['stamp']

def _align_to_odom(records, odom_records, label, epoch_offset):
    """det的时钟域取决于main.cpp具体版本（now()绝对纪元 vs 传感器header.stamp
    的bag相对时间），不能假设它和gt同一个偏移量。偏移量只能用gt估计（gt几乎从
    t=0就有记录，起始延迟可忽略），det只用中位数样本判断是否需要套用这个偏移。
    与boat_evaluator.py的_align_to_odom保持同一套逻辑，避免两处各自实现、
    各自出错。"""
    if not records or not odom_records:
        return
    sample = records[len(records) // 2]['stamp']
    odom_mid = odom_records[len(odom_records) // 2]['stamp']
    if abs(sample - odom_mid) > 1000:
        for r in records:
            r['stamp'] -= epoch_offset
        print(f"[时间基准校正] {label}时间戳偏移量估计: {epoch_offset:.4f}s，已统一到odom基准")

_align_to_odom(gt_records, odom_records, "gt", _epoch_offset)
_align_to_odom(det_records, odom_records, "det", _epoch_offset)

gt_records.sort(key=lambda r: r['stamp'])
odom_records.sort(key=lambda r: r['stamp'])
gt_stamps = [r['stamp'] for r in gt_records]
odom_stamps = [r['stamp'] for r in odom_records]

def interpolate_odom(t):
    idx = bisect.bisect_left(odom_stamps, t)
    if idx == 0:
        return odom_records[0] if odom_records else None
    if idx >= len(odom_records):
        return odom_records[-1] if odom_records else None

    r0 = odom_records[idx-1]
    r1 = odom_records[idx]
    t0, t1 = r0['stamp'], r1['stamp']

    if abs(t1 - t0) < 1e-9:
        return r0

    alpha = (t - t0) / (t1 - t0)

    def lerp(a, b):
        return a + (b - a) * alpha

    return {
        'stamp': t,
        'x': lerp(r0['x'], r1['x']),
        'y': lerp(r0['y'], r1['y']),
        'z': lerp(r0['z'], r1['z']),
        'qx': lerp(r0['qx'], r1['qx']),
        'qy': lerp(r0['qy'], r1['qy']),
        'qz': lerp(r0['qz'], r1['qz']),
        'qw': lerp(r0['qw'], r1['qw']),
    }

def interpolate_gt(t):
    idx = bisect.bisect_left(gt_stamps, t)
    if idx == 0:
        if t <= gt_stamps[0] + MAX_TIME_GAP:
            return gt_records[0]
        return None
    if idx >= len(gt_records):
        if t >= gt_stamps[-1] - MAX_TIME_GAP:
            return gt_records[-1]
        return None

    r0 = gt_records[idx-1]
    r1 = gt_records[idx]
    t0, t1 = r0['stamp'], r1['stamp']

    if t < t0 - MAX_TIME_GAP or t > t1 + MAX_TIME_GAP:
        return None

    poses0 = r0['poses']
    poses1 = r1['poses']

    if len(poses0) != len(poses1):
        if abs(t - t0) < abs(t - t1):
            return r0
        else:
            return r1

    alpha = (t - t0) / (t1 - t0)

    def lerp(a, b):
        return a + (b - a) * alpha

    interpolated_poses = []
    for p0, p1 in zip(poses0, poses1):
        interpolated_poses.append({
            'x': lerp(p0['x'], p1['x']),
            'y': lerp(p0['y'], p1['y']),
            'z': lerp(p0['z'], p1['z']),
        })

    return {
        'stamp': t,
        'poses': interpolated_poses,
    }

def world_to_boat(wx, wy, bx, by, yaw):
    dx, dy = wx - bx, wy - by
    rx = dx*math.cos(-yaw) - dy*math.sin(-yaw)
    ry = dx*math.sin(-yaw) + dy*math.cos(-yaw)
    return rx, ry

def quat_to_yaw(qx, qy, qz, qw):
    return math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))

print(f"\n=== 任务3: 20-40m范围内真值船统计 ===")

gt_boats_in_range = []
fp_candidates = []
dist_buckets = [(0, 20), (20, 40), (40, 60), (60, 1e9)]
bucket_totals = {rng: 0 for rng in dist_buckets}
bucket_hits = {rng: 0 for rng in dist_buckets}

def bucket_of(dist):
    for rng in dist_buckets:
        if rng[0] <= dist < rng[1]:
            return rng
    return None

skipped_odom = 0
gt_not_covered = 0

for det in det_records:
    t = det['stamp']
    odom = interpolate_odom(t)
    gt = interpolate_gt(t)

    if odom is None:
        skipped_odom += 1
        continue
    if gt is None or len(gt['poses']) == 0:
        gt_not_covered += 1
        for p in det['poses']:
            fp_candidates.append({'t': t, 'x': p['x'], 'y': p['y'], 'dist': math.hypot(p['x'], p['y'])})
        continue

    yaw = quat_to_yaw(odom['qx'], odom['qy'], odom['qz'], odom['qw'])
    gt_boat_frame = [world_to_boat(p['x'], p['y'], odom['x'], odom['y'], yaw) for p in gt['poses']]
    det_pts = [(p['x'], p['y']) for p in det['poses']]

    matched_gt = set()
    for dx, dy in det_pts:
        best_d = 1e9
        best_i = -1
        for i, (gx, gy) in enumerate(gt_boat_frame):
            d = math.hypot(dx-gx, dy-gy)
            if d < best_d:
                best_d = d
                best_i = i
        if best_d < MATCH_TOL:
            matched_gt.add(best_i)
        else:
            fp_candidates.append({'t': t, 'x': dx, 'y': dy, 'dist': math.hypot(dx, dy)})

    for i, (gx, gy) in enumerate(gt_boat_frame):
        dist = math.hypot(gx, gy)
        if 20 <= dist < 40:
            gt_boats_in_range.append({'t': t, 'x': gx, 'y': gy, 'dist': dist})
        rng = bucket_of(dist)
        if rng is not None:
            bucket_totals[rng] += 1
            if i in matched_gt:
                bucket_hits[rng] += 1

print(f"跳过(odom不可用): {skipped_odom}  跳过(真值不覆盖): {gt_not_covered}")
print(f"20-40m范围内真值船数量: {len(gt_boats_in_range)}")
if gt_boats_in_range:
    print(f"距离分布: min={min(b['dist'] for b in gt_boats_in_range):.1f}m, max={max(b['dist'] for b in gt_boats_in_range):.1f}m, avg={sum(b['dist'] for b in gt_boats_in_range)/len(gt_boats_in_range):.1f}m")

print(f"\n=== 任务4: 误检candidate归因分析 ===")
print(f"误检candidate总数: {len(fp_candidates)}")

if fp_candidates:
    fp_dist_bins = [(0, 20), (20, 40), (40, 60), (60, 100)]
    fp_dist_counts = {rng: 0 for rng in fp_dist_bins}
    for fp in fp_candidates:
        for rng in fp_dist_bins:
            if rng[0] <= fp['dist'] < rng[1]:
                fp_dist_counts[rng] += 1
                break

    print(f"\n误检距离分布:")
    for rng in fp_dist_bins:
        pct = fp_dist_counts[rng] / len(fp_candidates) * 100
        print(f"  [{rng[0]:>3}, {rng[1]:>3}m): {fp_dist_counts[rng]} ({pct:.1f}%)")

    print(f"\n误检距离统计:")
    print(f"  最小距离: {min(fp['dist'] for fp in fp_candidates):.1f}m")
    print(f"  最大距离: {max(fp['dist'] for fp in fp_candidates):.1f}m")
    print(f"  平均距离: {sum(fp['dist'] for fp in fp_candidates)/len(fp_candidates):.1f}m")

print(f"\n=== 任务5: 分距离桶召回率 ===")
print(f"{'距离范围':<12} {'真值总数':>8} {'命中':>8} {'召回率':>8}")
for rng in dist_buckets:
    total = bucket_totals[rng]
    hits = bucket_hits[rng]
    recall = hits / total * 100 if total > 0 else 0
    label = f"[{rng[0]},{rng[1]:.0f}m)" if rng[1] < 1e8 else f"[{rng[0]}m+)"
    print(f"{label:<12} {total:>8} {hits:>8} {recall:>7.1f}%")

total_gt = sum(bucket_totals.values())
total_hit = sum(bucket_hits.values())
print(f"\n合计: 真值总数={total_gt} 命中={total_hit} 召回率={total_hit/total_gt*100:.1f}%（应与boat_evaluator.py的整体召回率一致，用于交叉验证）")
