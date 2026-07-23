#!/usr/bin/env python3
"""
追踪单艘20-40m真值船的完整链路——裁决587点 vs 96.2%无簇的矛盾
核心方法：对单帧的一艘20-40m真值船，走完整链路：
1. 在真值船±2m球内，找cluster_diagnostic中的候选簇（搜索半径先放5m）
2. 看这些簇的点数——如果簇存在且点数充足，说明聚类没问题，是分类问题
3. 如果簇不存在或点数极少，说明确实聚不出来，需要查降采样
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
SEARCH_RADIUS = 5.0

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

print(f"\n=== 任务1: 追踪单艘20-40m真值船的完整链路 ===")

frame_idx = 50
det = det_records[frame_idx]
t = det['stamp']
print(f"\n--- 选择帧: idx={frame_idx}, t={t:.3f}s ---")

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

idx = bisect.bisect_left(cluster_stamps, t)
candidate_times = []
if idx < len(cluster_stamps): candidate_times.append(cluster_stamps[idx])
if idx > 0: candidate_times.append(cluster_stamps[idx-1])

clusters_in_frame = []
for ct in candidate_times:
    if abs(ct - t) <= FRAME_MATCH_TOL:
        clusters_in_frame.extend(cluster_frames[ct])

print(f"该帧候选簇总数: {len(clusters_in_frame)}")

target_boat_idx = 0
gx, gy, dist = gt_in_range[target_boat_idx]
print(f"\n--- 追踪真值船{target_boat_idx}: ({gx:.2f}, {gy:.2f}) dist={dist:.1f}m ---")

nearby_clusters = []
for cluster in clusters_in_frame:
    cd = math.hypot(cluster['x'] - gx, cluster['y'] - gy)
    if cd <= SEARCH_RADIUS:
        nearby_clusters.append((cd, cluster))

nearby_clusters.sort(key=lambda x: x[0])

print(f"\n在真值船{SEARCH_RADIUS}m范围内的候选簇:")
if nearby_clusters:
    for i, (cd, cluster) in enumerate(nearby_clusters):
        print(f"  簇{i}: 距离={cd:.2f}m, label={cluster['label']}, pts={cluster['pts']}, fp_max={cluster['fp_max']:.2f}, fp_min={cluster['fp_min']:.2f}, dz={cluster['dz']:.2f}")
    
    best_cluster = nearby_clusters[0][1]
    print(f"\n  最近簇: 距离={nearby_clusters[0][0]:.2f}m, label={best_cluster['label']}, pts={best_cluster['pts']}")
    
    if best_cluster['pts'] >= 4:
        print(f"  ✓ 最近簇点数={best_cluster['pts']} >= min_cluster_pts=4, 聚类没问题")
    else:
        print(f"  ✗ 最近簇点数={best_cluster['pts']} < min_cluster_pts=4, 聚类被打散")
    
    if best_cluster['label'] == 'boat':
        print(f"  ✓ 最近簇被正确分类为boat")
    else:
        print(f"  ✗ 最近簇被分类为{best_cluster['label']}, 是分类问题")
else:
    print(f"  ✗ {SEARCH_RADIUS}m范围内无候选簇")
    
    print(f"\n  搜索整个帧中最近的簇:")
    min_dist = 1e9
    min_cluster = None
    for cluster in clusters_in_frame:
        cd = math.hypot(cluster['x'] - gx, cluster['y'] - gy)
        if cd < min_dist:
            min_dist = cd
            min_cluster = cluster
    
    if min_cluster:
        print(f"    最近簇距离={min_dist:.2f}m, label={min_cluster['label']}, pts={min_cluster['pts']}")
        print(f"    {'可能是标定残差导致偏移' if 2 < min_dist < 6 else '距离太远，可能是另一艘船'}")

print(f"\n=== 任务2: 统计20-40m所有漏检船附近的簇分布(搜索半径5m) ===")

total_missed = 0
has_cluster_2m = 0
has_cluster_5m = 0
has_cluster_10m = 0
closest_distances = []

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
    
    for i, (gx, gy) in enumerate(gt_boat_frame):
        if i in matched_gt:
            continue
        
        dist = math.hypot(gx, gy)
        if not (20 <= dist < 40):
            continue
        
        total_missed += 1
        
        min_cd = 1e9
        for cluster in clusters_in_frame:
            cd = math.hypot(cluster['x'] - gx, cluster['y'] - gy)
            if cd < min_cd:
                min_cd = cd
        
        closest_distances.append(min_cd)
        
        if min_cd <= 2.0:
            has_cluster_2m += 1
        if min_cd <= 5.0:
            has_cluster_5m += 1
        if min_cd <= 10.0:
            has_cluster_10m += 1

print(f"\n20-40m漏检船总数: {total_missed}")
print(f"附近2m内有簇: {has_cluster_2m} ({has_cluster_2m/total_missed*100:.1f}%)")
print(f"附近5m内有簇: {has_cluster_5m} ({has_cluster_5m/total_missed*100:.1f}%)")
print(f"附近10m内有簇: {has_cluster_10m} ({has_cluster_10m/total_missed*100:.1f}%)")

if closest_distances:
    dist_bins = [(0, 2), (2, 3), (3, 4), (4, 5), (5, 6), (6, 10), (10, 20), (20, 100)]
    dist_counts = {rng: 0 for rng in dist_bins}
    for d in closest_distances:
        for rng in dist_bins:
            if rng[0] <= d < rng[1]:
                dist_counts[rng] += 1
                break
    
    print(f"\n最近簇距离分布:")
    for rng in dist_bins:
        pct = dist_counts[rng] / len(closest_distances) * 100
        print(f"  [{rng[0]:>3}, {rng[1]:>3}m): {dist_counts[rng]} ({pct:.1f}%)")