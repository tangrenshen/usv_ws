#!/usr/bin/env python3
"""
过滤邻居簇——三角验证：簇→反查最近真值，离浮标/其他物体更近则排除
核心方法：
1. 对每个漏检船，找到它附近的簇
2. 对这个簇，反查最近的真值（所有类型：船、浮标等）
3. 如果簇离其他真值比离当前漏检船更近，排除这个簇
4. 重算"真·有簇"比例，分距离带确认聚类vs分类各自背多少锅
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
SEARCH_RADIUS = 10.0

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

print(f"\n=== 任务1: 过滤邻居簇——三角验证 ===")

results_by_bin = {
    '0-20m': {'total': 0, 'has_cluster_raw': 0, 'has_cluster_filtered': 0},
    '20-40m': {'total': 0, 'has_cluster_raw': 0, 'has_cluster_filtered': 0},
    '40-60m': {'total': 0, 'has_cluster_raw': 0, 'has_cluster_filtered': 0},
    '60m+': {'total': 0, 'has_cluster_raw': 0, 'has_cluster_filtered': 0}
}

cluster_to_nearest_gt = {}

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
    
    idx = bisect.bisect_left(cluster_stamps, t)
    candidate_times = []
    if idx < len(cluster_stamps): candidate_times.append(cluster_stamps[idx])
    if idx > 0: candidate_times.append(cluster_stamps[idx-1])
    
    clusters_in_frame = []
    for ct in candidate_times:
        if abs(ct - t) <= FRAME_MATCH_TOL:
            clusters_in_frame.extend(cluster_frames[ct])
    
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
    
    for i, (gx, gy) in enumerate(gt_boat_frame):
        if i in matched_gt:
            continue
        
        dist = math.hypot(gx, gy)
        dist_bin = '0-20m' if dist < 20 else '20-40m' if dist < 40 else '40-60m' if dist < 60 else '60m+'
        results_by_bin[dist_bin]['total'] += 1
        
        min_cd = 1e9
        min_cluster = None
        for cluster in clusters_in_frame:
            cd = math.hypot(cluster['x'] - gx, cluster['y'] - gy)
            if cd < min_cd:
                min_cd = cd
                min_cluster = cluster
        
        if min_cd <= SEARCH_RADIUS:
            results_by_bin[dist_bin]['has_cluster_raw'] += 1
            
            min_cluster_key = f"{cluster['x']:.2f}_{cluster['y']:.2f}_{cluster['t']:.3f}"
            if min_cluster_key not in cluster_to_nearest_gt:
                min_gt_dist = 1e9
                min_gt_idx = -1
                for j, (gx2, gy2) in enumerate(gt_boat_frame):
                    gd = math.hypot(cluster['x'] - gx2, cluster['y'] - gy2)
                    if gd < min_gt_dist:
                        min_gt_dist = gd
                        min_gt_idx = j
                
                cluster_to_nearest_gt[min_cluster_key] = {
                    'cluster': cluster,
                    'nearest_gt_idx': min_gt_idx,
                    'nearest_gt_dist': min_gt_dist
                }
            
            info = cluster_to_nearest_gt[min_cluster_key]
            if info['nearest_gt_idx'] == i:
                results_by_bin[dist_bin]['has_cluster_filtered'] += 1

print(f"\n=== 三角验证结果：过滤邻居簇前后对比 ===")
print(f"\n搜索半径: {SEARCH_RADIUS}m")
print(f"过滤规则: 簇必须离当前漏检船最近（排除离其他真值更近的邻居簇）")

for dist_bin in ['0-20m', '20-40m', '40-60m', '60m+']:
    r = results_by_bin[dist_bin]
    if r['total'] == 0:
        continue
    
    raw_pct = r['has_cluster_raw'] / r['total'] * 100
    filtered_pct = r['has_cluster_filtered'] / r['total'] * 100
    reduction_pct = (r['has_cluster_raw'] - r['has_cluster_filtered']) / r['has_cluster_raw'] * 100 if r['has_cluster_raw'] > 0 else 0
    
    print(f"\n{dist_bin}:")
    print(f"  漏检总数: {r['total']}")
    print(f"  原始有簇(raw): {r['has_cluster_raw']} ({raw_pct:.1f}%)")
    print(f"  过滤后有簇(filtered): {r['has_cluster_filtered']} ({filtered_pct:.1f}%)")
    print(f"  被邻居簇污染比例: {reduction_pct:.1f}%")
    
    if filtered_pct < 10:
        print(f"  → 大部分漏检船没有自己的簇，聚类问题为主")
    elif filtered_pct < 30:
        print(f"  → 约三成漏检船有自己的簇，聚类和分类问题并存")
    else:
        print(f"  → 超过三成漏检船有自己的簇，分类问题为主")

print(f"\n=== 关键发现 ===")
print(f"1. 如果过滤后'真·有簇'比例接近原始2m搜索半径的3.8%，说明10m内大部分是邻居簇")
print(f"2. 如果过滤后比例仍显著高于3.8%，说明确实有很多船聚出了自己的簇但被判错")
print(f"3. 这个结果决定下一步是修聚类还是修分类")

print(f"\n=== 第50帧详细示例 ===")

frame_idx = 50
det = det_records[frame_idx]
t = det['stamp']
odom, _ = find_nearest(odom_stamps, odom_records, t)
gt, _ = find_nearest(gt_stamps, gt_records, t)

yaw = quat_to_yaw(odom['qx'], odom['qy'], odom['qz'], odom['qw'])
gt_boat_frame = [world_to_boat(p['x'], p['y'], odom['x'], odom['y'], yaw) for p in gt['poses']]

idx = bisect.bisect_left(cluster_stamps, t)
candidate_times = []
if idx < len(cluster_stamps): candidate_times.append(cluster_stamps[idx])
if idx > 0: candidate_times.append(cluster_stamps[idx-1])

clusters_in_frame = []
for ct in candidate_times:
    if abs(ct - t) <= FRAME_MATCH_TOL:
        clusters_in_frame.extend(cluster_frames[ct])

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

print(f"\n帧idx={frame_idx}, t={t:.3f}s")
print(f"20-40m真值船:")
for i, (gx, gy) in enumerate(gt_boat_frame):
    dist = math.hypot(gx, gy)
    if not (20 <= dist < 40):
        continue
    
    is_matched = i in matched_gt
    
    min_cd = 1e9
    min_cluster = None
    for cluster in clusters_in_frame:
        cd = math.hypot(cluster['x'] - gx, cluster['y'] - gy)
        if cd < min_cd:
            min_cd = cd
            min_cluster = cluster
    
    if min_cluster:
        min_gt_dist = 1e9
        min_gt_idx = -1
        for j, (gx2, gy2) in enumerate(gt_boat_frame):
            gd = math.hypot(min_cluster['x'] - gx2, min_cluster['y'] - gy2)
            if gd < min_gt_dist:
                min_gt_dist = gd
                min_gt_idx = j
        
        is_own_cluster = (min_gt_idx == i)
        
        print(f"  真值{i}: ({gx:.2f}, {gy:.2f}) dist={dist:.1f}m {'命中' if is_matched else '漏检'}")
        print(f"    最近簇: ({min_cluster['x']:.2f}, {min_cluster['y']:.2f}) dist={min_cd:.2f}m label={min_cluster['label']} pts={min_cluster['pts']}")
        print(f"    该簇最近真值: 真值{min_gt_idx} dist={min_gt_dist:.2f}m")
        print(f"    {'→ 是自己的簇' if is_own_cluster else '→ 是邻居的簇(污染)'}")
    else:
        print(f"  真值{i}: ({gx:.2f}, {gy:.2f}) dist={dist:.1f}m {'命中' if is_matched else '漏检'}")
        print(f"    10m内无候选簇")