#!/usr/bin/env python3
"""
分析20-40m桶的关键问题：
1. 统计20-40m范围内有多少艘真值船
2. 分析每艘真值船周围有多少检测候选
3. 对误检candidate做归因分析
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

if gt_records and odom_records:
    _offset = gt_records[0]['stamp'] - odom_records[0]['stamp']
    for r in gt_records:
        r['stamp'] -= _offset
    print(f"[时间基准校正] gt时间戳偏移量估计: {_offset:.4f}s")

gt_records.sort(key=lambda r: r['stamp'])
odom_records.sort(key=lambda r: r['stamp'])
gt_stamps = [r['stamp'] for r in gt_records]
odom_stamps = [r['stamp'] for r in odom_records]

def find_nearest(stamps, records, t):
    idx = bisect.bisect_left(stamps, t)
    candidates = []
    if idx < len(records): candidates.append(records[idx])
    if idx > 0: candidates.append(records[idx-1])
    if not candidates: return None, None
    best = min(candidates, key=lambda r: abs(r['stamp'] - t))
    return best, abs(best['stamp'] - t)

def world_to_boat(wx, wy, bx, by, yaw):
    dx, dy = wx - bx, wy - by
    rx = dx*math.cos(-yaw) - dy*math.sin(-yaw)
    ry = dx*math.sin(-yaw) + dy*math.cos(-yaw)
    return rx, ry

def quat_to_yaw(qx, qy, qz, qw):
    return math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))

print(f"\n=== 任务3: 20-40m范围内真值船统计 ===")

gt_boats_in_range = []
for det in det_records:
    t = det['stamp']
    odom, odom_gap = find_nearest(odom_stamps, odom_records, t)
    gt, gt_gap = find_nearest(gt_stamps, gt_records, t)
    if odom is None or gt is None:
        continue
    if odom_gap > MAX_TIME_GAP or gt_gap > MAX_TIME_GAP:
        continue
    
    yaw = quat_to_yaw(odom['qx'], odom['qy'], odom['qz'], odom['qw'])
    gt_boat_frame = [world_to_boat(p['x'], p['y'], odom['x'], odom['y'], yaw) for p in gt['poses']]
    
    for gx, gy in gt_boat_frame:
        dist = math.hypot(gx, gy)
        if 20 <= dist < 40:
            gt_boats_in_range.append({
                't': t,
                'x': gx,
                'y': gy,
                'dist': dist
            })

print(f"20-40m范围内真值船数量: {len(gt_boats_in_range)}")
if gt_boats_in_range:
    print(f"距离分布: min={min(b['dist'] for b in gt_boats_in_range):.1f}m, max={max(b['dist'] for b in gt_boats_in_range):.1f}m, avg={sum(b['dist'] for b in gt_boats_in_range)/len(gt_boats_in_range):.1f}m")

print(f"\n=== 任务4: 误检candidate归因分析 ===")

fp_candidates = []
for det in det_records:
    t = det['stamp']
    odom, odom_gap = find_nearest(odom_stamps, odom_records, t)
    gt, gt_gap = find_nearest(gt_stamps, gt_records, t)
    if odom is None or gt is None:
        continue
    if odom_gap > MAX_TIME_GAP or gt_gap > MAX_TIME_GAP:
        continue
    
    yaw = quat_to_yaw(odom['qx'], odom['qy'], odom['qz'], odom['qw'])
    gt_boat_frame = [world_to_boat(p['x'], p['y'], odom['x'], odom['y'], yaw) for p in gt['poses']]
    det_pts = [(p['x'], p['y']) for p in det['poses']]
    
    for dx, dy in det_pts:
        is_fp = True
        for gx, gy in gt_boat_frame:
            if math.hypot(dx-gx, dy-gy) < MATCH_TOL:
                is_fp = False
                break
        if is_fp:
            fp_candidates.append({
                't': t,
                'x': dx,
                'y': dy,
                'dist': math.hypot(dx, dy)
            })

print(f"误检candidate总数: {len(fp_candidates)}")

if fp_candidates:
    dist_bins = [(0, 20), (20, 40), (40, 60), (60, 100)]
    dist_counts = {rng: 0 for rng in dist_bins}
    for fp in fp_candidates:
        for rng in dist_bins:
            if rng[0] <= fp['dist'] < rng[1]:
                dist_counts[rng] += 1
                break
    
    print(f"\n误检距离分布:")
    for rng in dist_bins:
        pct = dist_counts[rng] / len(fp_candidates) * 100
        print(f"  [{rng[0]:>3}, {rng[1]:>3}m): {dist_counts[rng]} ({pct:.1f}%)")
    
    print(f"\n误检距离统计:")
    print(f"  最小距离: {min(fp['dist'] for fp in fp_candidates):.1f}m")
    print(f"  最大距离: {max(fp['dist'] for fp in fp_candidates):.1f}m")
    print(f"  平均距离: {sum(fp['dist'] for fp in fp_candidates)/len(fp_candidates):.1f}m")

print(f"\n=== 综合分析 ===")
print(f"20-40m原始点: 2227 (来自裁剪前染色统计)")
print(f"20-40m真值船: {len(gt_boats_in_range)}")
if gt_boats_in_range:
    avg_pts_per_boat = 2227 / len(gt_boats_in_range)
    print(f"平均每艘真值船摊到的原始点数: {avg_pts_per_boat:.1f}")
    print(f"结论: {'调参可救(每艘>30点)' if avg_pts_per_boat > 30 else '点太稀(需要更根本手段)' if avg_pts_per_boat < 10 else '临界(需要看具体分布)'}")