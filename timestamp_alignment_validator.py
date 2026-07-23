#!/usr/bin/env python3
"""
验证cluster记录和det/gt的时间戳配对是否对齐
核心方法：
1. 从perception_log.log读取cluster_diagnostic的时间戳
2. 从boat_log.jsonl读取det/gt的时间戳
3. 检查两者是否能在时间窗口内配对
"""
import json
import bisect
import re
import math

BOAT_LOG_PATH = '/home/lyf040817/usv_ws/boat_log.jsonl'
CLUSTER_LOG_PATH = '/home/lyf040817/usv_ws/perception_log.log'
FRAME_MATCH_TOL = 0.05

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

det_stamps = [r['stamp'] for r in det_records]

pending_pattern = re.compile(
    r'\[cluster_diagnostic\] t=([\d.]+) center=\(([\d.-]+),([\d.-]+),([\d.-]+)\)'
    r' size=\(([\d.]+),([\d.]+),([\d.]+)\)'
    r' fp_max=([\d.]+) fp_min=([\d.]+) square=([\d.]+) pts=(\d+) -> classification pending'
)

cluster_frames = {}
with open(CLUSTER_LOG_PATH, encoding='utf-8', errors='replace') as f:
    current_cluster = None
    for line in f:
        line = line.strip()
        pending_match = pending_pattern.match(line)
        if pending_match:
            if current_cluster:
                cluster_frames.setdefault(current_cluster['t'], []).append(current_cluster)
            try:
                t = float(pending_match.group(1))
                current_cluster = {
                    't': t,
                    'x': float(pending_match.group(2)),
                    'y': float(pending_match.group(3)),
                    'z': float(pending_match.group(4)),
                    'dx': float(pending_match.group(5)),
                    'dy': float(pending_match.group(6)),
                    'dz': float(pending_match.group(7)),
                    'fp_max': float(pending_match.group(8)),
                    'fp_min': float(pending_match.group(9)),
                    'square': float(pending_match.group(10)),
                    'pts': int(pending_match.group(11)),
                    'label': None
                }
            except (ValueError, IndexError):
                current_cluster = None
                continue
        else:
            assigned_match = re.match(r'\[cluster_diagnostic\] assigned=(\w+)', line)
            if assigned_match and current_cluster:
                current_cluster['label'] = assigned_match.group(1)
                cluster_frames.setdefault(current_cluster['t'], []).append(current_cluster)
                current_cluster = None

cluster_stamps = sorted(cluster_frames.keys())
print(f"\ncluster_diagnostic: {len(cluster_frames)}帧, {sum(len(c) for c in cluster_frames.values())}个候选簇")

print(f"\n=== 任务2: 时间戳配对验证 ===")

matched_count = 0
unmatched_det = 0
max_gap = 0
min_gap = 1e9

for det in det_records:
    t = det['stamp']
    idx = bisect.bisect_left(cluster_stamps, t)
    matched = False
    
    candidate_times = []
    if idx < len(cluster_stamps): candidate_times.append(cluster_stamps[idx])
    if idx > 0: candidate_times.append(cluster_stamps[idx-1])
    
    for ct in candidate_times:
        gap = abs(ct - t)
        if gap <= FRAME_MATCH_TOL:
            matched = True
            matched_count += 1
            max_gap = max(max_gap, gap)
            min_gap = min(min_gap, gap)
            break
    
    if not matched:
        unmatched_det += 1

print(f"\n检测消息与cluster_diagnostic配对结果:")
print(f"  可配对的检测消息: {matched_count}")
print(f"  不可配对的检测消息: {unmatched_det}")
print(f"  配对率: {matched_count / len(det_records) * 100:.1f}%")
print(f"  时间间隙范围: [{min_gap*1000:.1f}ms, {max_gap*1000:.1f}ms]")

print(f"\n=== 第50帧详细配对验证 ===")

frame_idx = 50
det = det_records[frame_idx]
t = det['stamp']
print(f"\n检测消息: idx={frame_idx}, t={t:.6f}s")

idx = bisect.bisect_left(cluster_stamps, t)
candidate_times = []
if idx < len(cluster_stamps): candidate_times.append(cluster_stamps[idx])
if idx > 0: candidate_times.append(cluster_stamps[idx-1])

for ct in candidate_times:
    gap = abs(ct - t)
    print(f"  cluster_diagnostic帧: t={ct:.6f}s, 时间差={gap*1000:.2f}ms, {'可配对' if gap <= FRAME_MATCH_TOL else '不可配对'}")
    
    if gap <= FRAME_MATCH_TOL:
        clusters = cluster_frames[ct]
        print(f"    该帧候选簇数: {len(clusters)}")
        for i, cluster in enumerate(clusters[:5]):
            dist = math.hypot(cluster['x'], cluster['y'])
            print(f"      簇{i}: ({cluster['x']:.2f}, {cluster['y']:.2f}) dist={dist:.1f}m label={cluster['label']}")
        if len(clusters) > 5:
            print(f"      ... 还有 {len(clusters) - 5} 个簇")

print(f"\n=== 结论 ===")
if matched_count / len(det_records) > 0.95:
    print(f"  ✓ 时间戳配对验证通过: {matched_count/len(det_records)*100:.1f}%的检测消息可与cluster_diagnostic配对")
    print(f"  ✓ 时间间隙在{FRAME_MATCH_TOL*1000}ms窗口内，可安全用于漏检归因分析")
else:
    print(f"  ✗ 时间戳配对验证失败: 仅{matched_count/len(det_records)*100:.1f}%的检测消息可配对")
    print(f"  ✗ 需要进一步检查时间基准")

import math