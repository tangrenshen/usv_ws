#!/usr/bin/env python3
"""
单帧级别分析：核对"每艘船0.6个点"的分母问题
核心方法：挑单独一帧，在20-40m环带内：
1. 数真值船数
2. 数原始点数（从perception_log.log的裁剪前染色中提取）
3. 数每艘船±2m内的点数
"""
import json
import math
import bisect
import re

BOAT_LOG_PATH = '/home/lyf040817/usv_ws/boat_log.jsonl'
CLUSTER_LOG_PATH = '/home/lyf040817/usv_ws/perception_log.log'
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

print(f"\n=== 单帧级别分析 ===")

for frame_idx, det in enumerate(det_records):
    if frame_idx != 50:
        continue
    
    t = det['stamp']
    print(f"\n--- 选择帧: idx={frame_idx}, t={t:.3f}s ---")
    
    odom, odom_gap = find_nearest(odom_stamps, odom_records, t)
    gt, gt_gap = find_nearest(gt_stamps, gt_records, t)
    
    if odom is None or gt is None:
        print(f"  跳过: 无匹配的odom或gt")
        continue
    if odom_gap > MAX_TIME_GAP or gt_gap > MAX_TIME_GAP:
        print(f"  跳过: 时间间隙过大")
        continue
    
    yaw = quat_to_yaw(odom['qx'], odom['qy'], odom['qz'], odom['qw'])
    gt_boat_frame = [world_to_boat(p['x'], p['y'], odom['x'], odom['y'], yaw) for p in gt['poses']]
    det_pts = [(p['x'], p['y']) for p in det['poses']]
    
    print(f"  该帧真值船总数: {len(gt_boat_frame)}")
    print(f"  该帧检测候选总数: {len(det_pts)}")
    
    gt_in_range = []
    for gx, gy in gt_boat_frame:
        dist = math.hypot(gx, gy)
        if 20 <= dist < 40:
            gt_in_range.append((gx, gy, dist))
    
    print(f"  20-40m范围内真值船数: {len(gt_in_range)}")
    for i, (gx, gy, dist) in enumerate(gt_in_range):
        print(f"    真值船{i}: ({gx:.2f}, {gy:.2f}) dist={dist:.1f}m")
    
    print(f"\n  该帧各距离桶检测候选分布:")
    dist_bins = [(0, 20), (20, 40), (40, 60), (60, 100)]
    dist_counts = {rng: 0 for rng in dist_bins}
    for dx, dy in det_pts:
        d = math.hypot(dx, dy)
        for rng in dist_bins:
            if rng[0] <= d < rng[1]:
                dist_counts[rng] += 1
                break
    for rng in dist_bins:
        print(f"    [{rng[0]:>3}, {rng[1]:>3}m): {dist_counts[rng]}个候选")
    
    print(f"\n  每艘20-40m真值船周围±2m内的检测候选数:")
    for i, (gx, gy, dist) in enumerate(gt_in_range):
        nearby_dets = []
        for dx, dy in det_pts:
            if math.hypot(dx-gx, dy-gy) < 2.0:
                nearby_dets.append((dx, dy))
        print(f"    真值船{i} (dist={dist:.1f}m): {len(nearby_dets)}个候选在±2m内")

print(f"\n=== 全局跨帧统计(用于对比) ===")

total_gt_in_range = 0
total_det_in_range = 0
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
    
    for gx, gy in gt_boat_frame:
        dist = math.hypot(gx, gy)
        if 20 <= dist < 40:
            total_gt_in_range += 1
    
    for dx, dy in det_pts:
        d = math.hypot(dx, dy)
        if 20 <= d < 40:
            total_det_in_range += 1

print(f"跨帧累加:")
print(f"  20-40m真值船实例总数: {total_gt_in_range}")
print(f"  20-40m检测候选实例总数: {total_det_in_range}")
print(f"  注意: 这些是累加值，不能直接用于单帧级别的除法")