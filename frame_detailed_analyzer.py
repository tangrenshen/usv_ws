#!/usr/bin/env python3
"""
详细分析20-40m误检-漏检并存的矛盾：
1. 单帧20-40m有2227个点，8艘船，每艘278个点——点充足
2. 但8艘船全部漏检，同时该区间有4个检测候选——这些候选是什么？
3. 分析检测候选的性质：是否是船被错分类，还是水面噪声？
"""
import json
import math
import bisect
import re

BOAT_LOG_PATH = '/home/lyf040817/usv_ws/boat_log.jsonl'
CLUSTER_LOG_PATH = '/home/lyf040817/usv_ws/perception_log.log'
MATCH_TOL = 2.0
MAX_TIME_GAP = 0.5
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

gt_records.sort(key=lambda r: r['stamp'])
odom_records.sort(key=lambda r: r['stamp'])
det_records.sort(key=lambda r: r['stamp'])
gt_stamps = [r['stamp'] for r in gt_records]
odom_stamps = [r['stamp'] for r in odom_records]
det_stamps = [r['stamp'] for r in det_records]

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

pending_pattern = re.compile(
    r'\[cluster_diagnostic\] t=([\d.]+) center=\(([\d.-]+),([\d.-]+),([\d.-]+)\)'
    r' size=\(([\d.]+),([\d.]+),([\d.]+)\)'
    r' fp_max=([\d.]+) fp_min=([\d.]+) square=([\d.]+) pts=(\d+) -> classification pending'
)
assigned_pattern = re.compile(
    r'\[cluster_diagnostic\] assigned=(\w+)'
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
            assigned_match = assigned_pattern.match(line)
            if assigned_match and current_cluster:
                current_cluster['label'] = assigned_match.group(1)
                cluster_frames.setdefault(current_cluster['t'], []).append(current_cluster)
                current_cluster = None

cluster_stamps = sorted(cluster_frames.keys())
print(f"\n解析到 {len(cluster_frames)} 帧的cluster_diagnostic数据")

print(f"\n=== 单帧详细分析(第50帧) ===")

frame_idx = 50
det = det_records[frame_idx]
t = det['stamp']
print(f"\n--- 帧: idx={frame_idx}, t={t:.3f}s ---")

odom, odom_gap = find_nearest(odom_stamps, odom_records, t)
gt, gt_gap = find_nearest(gt_stamps, gt_records, t)

yaw = quat_to_yaw(odom['qx'], odom['qy'], odom['qz'], odom['qw'])
gt_boat_frame = [world_to_boat(p['x'], p['y'], odom['x'], odom['y'], yaw) for p in gt['poses']]
det_pts = [(p['x'], p['y']) for p in det['poses']]

gt_in_range = []
for gx, gy in gt_boat_frame:
    dist = math.hypot(gx, gy)
    if 20 <= dist < 40:
        gt_in_range.append((gx, gy, dist))

print(f"20-40m真值船数: {len(gt_in_range)}")
print(f"20-40m检测候选数: {len([d for d in det_pts if 20 <= math.hypot(d[0], d[1]) < 40])}")

print(f"\n真值船位置:")
for i, (gx, gy, dist) in enumerate(gt_in_range):
    print(f"  船{i}: ({gx:.2f}, {gy:.2f}) dist={dist:.1f}m")

print(f"\n20-40m检测候选位置(这些是误检):")
for i, (dx, dy) in enumerate(det_pts):
    d = math.hypot(dx, dy)
    if 20 <= d < 40:
        print(f"  候选{i}: ({dx:.2f}, {dy:.2f}) dist={d:.1f}m")

print(f"\n=== 查找该帧的cluster_diagnostic数据 ===")
idx = bisect.bisect_left(cluster_stamps, t)
candidate_times = []
if idx < len(cluster_stamps): candidate_times.append(cluster_stamps[idx])
if idx > 0: candidate_times.append(cluster_stamps[idx-1])

for ct in candidate_times:
    if abs(ct - t) > FRAME_MATCH_TOL:
        continue
    print(f"\n匹配到cluster_diagnostic帧: t={ct:.3f}s")
    clusters = cluster_frames[ct]
    
    print(f"\n该帧所有候选簇:")
    for i, cluster in enumerate(clusters):
        cx, cy = cluster['x'], cluster['y']
        dist = math.hypot(cx, cy)
        in_range = 20 <= dist < 40
        print(f"  簇{i}: center=({cx:.2f}, {cy:.2f}) dist={dist:.1f}m {'[20-40m]' if in_range else ''}")
        print(f"        size=({cluster['dx']:.2f}, {cluster['dy']:.2f}, {cluster['dz']:.2f})")
        print(f"        fp_max={cluster['fp_max']:.2f} fp_min={cluster['fp_min']:.2f}")
        print(f"        square={cluster['square']:.2f} pts={cluster['pts']}")
        print(f"        label={cluster['label']}")
    
    print(f"\n20-40m范围内的候选簇(分析误检来源):")
    in_range_clusters = [c for c in clusters if 20 <= math.hypot(c['x'], c['y']) < 40]
    for i, cluster in enumerate(in_range_clusters):
        print(f"  簇{i}: label={cluster['label']} pts={cluster['pts']} fp_max={cluster['fp_max']:.2f} dz={cluster['dz']:.2f}")
    
    print(f"\n20-40m范围内真值船附近的簇:")
    for i, (gx, gy, dist) in enumerate(gt_in_range):
        closest_cluster = None
        closest_dist = 1e9
        for cluster in clusters:
            cd = math.hypot(cluster['x'] - gx, cluster['y'] - gy)
            if cd < closest_dist:
                closest_dist = cd
                closest_cluster = cluster
        if closest_cluster:
            print(f"  真值船{i} (dist={dist:.1f}m): 最近簇距离={closest_dist:.2f}m, label={closest_cluster['label']}, pts={closest_cluster['pts']}, fp_max={closest_cluster['fp_max']:.2f}")
        else:
            print(f"  真值船{i} (dist={dist:.1f}m): 无附近簇")

print(f"\n=== 全局分析: 20-40m检测候选的label分布 ===")
label_dist = {}
for ct, clusters in cluster_frames.items():
    for cluster in clusters:
        dist = math.hypot(cluster['x'], cluster['y'])
        if 20 <= dist < 40:
            label = cluster['label'] or 'unknown'
            label_dist[label] = label_dist.get(label, 0) + 1

total_in_range = sum(label_dist.values())
print(f"20-40m范围内候选簇总数: {total_in_range}")
for label, count in sorted(label_dist.items(), key=lambda x: -x[1]):
    pct = count / total_in_range * 100
    print(f"  {label}: {count} ({pct:.1f}%)")