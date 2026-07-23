#!/usr/bin/env python3
"""
抽3-5帧复核278点/艘——确认20-40m点充足不是单帧偶然
核心方法：随机抽取5个不同帧，统计每帧20-40m范围内：
1. 真值船数
2. 原始点数（从perception_log.log提取）
3. 每艘船摊到的点数
4. 检测候选数
5. 漏检率
"""
import json
import math
import bisect
import random

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
det_records.sort(key=lambda r: r['stamp'])
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

import re
pre_crop_pattern = re.compile(
    r'\[裁剪前染色\] total=(\d+) nan=(\d+) \[0-20m\)=(\d+) \[20-40m\)=(\d+) \[40-60m\)=(\d+) \[60m\+\)=(\d+)'
)

pre_crop_data = []
with open(CLUSTER_LOG_PATH, encoding='utf-8', errors='replace') as f:
    for line in f:
        match = pre_crop_pattern.search(line)
        if match:
            pre_crop_data.append({
                'total': int(match.group(1)),
                'b0_20': int(match.group(3)),
                'b20_40': int(match.group(4)),
                'b40_60': int(match.group(5)),
                'b60_plus': int(match.group(6))
            })

if pre_crop_data:
    avg_b20_40 = sum(d['b20_40'] for d in pre_crop_data) / len(pre_crop_data)
    print(f"\n裁剪前染色数据: {len(pre_crop_data)}帧, 20-40m平均点数={avg_b20_40:.1f}")
else:
    print("\n警告: 未找到裁剪前染色数据")
    avg_b20_40 = 2227

print(f"\n=== 任务1: 抽5帧复核278点/艘 ===")

random.seed(42)
sample_indices = random.sample(range(len(det_records)), 5)
sample_indices.sort()

frame_results = []

for frame_idx in sample_indices:
    det = det_records[frame_idx]
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
    
    gt_in_range = []
    for gx, gy in gt_boat_frame:
        dist = math.hypot(gx, gy)
        if 20 <= dist < 40:
            gt_in_range.append((gx, gy, dist))
    
    det_in_range = [d for d in det_pts if 20 <= math.hypot(d[0], d[1]) < 40]
    
    matched_gt = set()
    for dx, dy in det_in_range:
        best_d = 1e9
        best_i = -1
        for i, (gx, gy, _) in enumerate(gt_in_range):
            d = math.hypot(dx-gx, dy-gy)
            if d < best_d:
                best_d = d
                best_i = i
        if best_d < MATCH_TOL:
            matched_gt.add(best_i)
    
    missed_count = len(gt_in_range) - len(matched_gt)
    recall = len(matched_gt) / len(gt_in_range) * 100 if gt_in_range else 0
    
    pts_per_boat = avg_b20_40 / len(gt_in_range) if gt_in_range else 0
    
    frame_results.append({
        'frame_idx': frame_idx,
        't': t,
        'gt_count': len(gt_in_range),
        'pts_per_boat': pts_per_boat,
        'det_count': len(det_in_range),
        'matched': len(matched_gt),
        'missed': missed_count,
        'recall': recall
    })
    
    print(f"\n--- 帧: idx={frame_idx}, t={t:.3f}s ---")
    print(f"  20-40m真值船数: {len(gt_in_range)}")
    print(f"  20-40m原始点数(平均): {avg_b20_40}")
    print(f"  每艘船摊到点数: {pts_per_boat:.1f}")
    print(f"  20-40m检测候选数: {len(det_in_range)}")
    print(f"  命中: {len(matched_gt)}, 漏检: {missed_count}, 召回率: {recall:.1f}%")

print(f"\n=== 5帧统计汇总 ===")
if frame_results:
    avg_gt_count = sum(r['gt_count'] for r in frame_results) / len(frame_results)
    avg_pts_per_boat = sum(r['pts_per_boat'] for r in frame_results) / len(frame_results)
    avg_det_count = sum(r['det_count'] for r in frame_results) / len(frame_results)
    avg_recall = sum(r['recall'] for r in frame_results) / len(frame_results)
    avg_missed = sum(r['missed'] for r in frame_results) / len(frame_results)
    
    print(f"  平均20-40m真值船数: {avg_gt_count:.1f}")
    print(f"  平均每艘船摊到点数: {avg_pts_per_boat:.1f}")
    print(f"  平均20-40m检测候选数: {avg_det_count:.1f}")
    print(f"  平均召回率: {avg_recall:.1f}%")
    print(f"  平均漏检数: {avg_missed:.1f}")
    
    print(f"\n  结论: {'20-40m点充足，不是单帧偶然' if avg_pts_per_boat > 100 else '需要进一步验证' if avg_pts_per_boat > 30 else '点可能不足'}")