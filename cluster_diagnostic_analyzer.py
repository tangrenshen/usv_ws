#!/usr/bin/env python3
"""
漏检真船二分归因分析器：找出漏检的真船在聚类阶段是否有候选簇。

核心方法：复用boat_evaluator.py一模一样的加载/时间校正/匹配逻辑找出每一帧的"漏检真船"。
同时解析ObstacleDetector.cpp里[cluster_diagnostic]的两行一组日志，按时间戳配对到同一帧，
再看这一帧里离漏检真船最近的候选簇有多远、被分成了什么。
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
CANDIDATE_SEARCH_RADIUS = 3.0  

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
    print(f"[时间基准校正] gt时间戳偏移量估计: {_offset:.4f}s，已统一到odom/det基准")

gt_records.sort(key=lambda r: r['stamp'])
odom_records.sort(key=lambda r: r['stamp'])
gt_stamps = [r['stamp'] for r in gt_records]
odom_stamps = [r['stamp'] for r in odom_records]

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
parse_success = 0
parse_fail = 0

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
                parse_success += 1
            except (ValueError, IndexError):
                parse_fail += 1
                current_cluster = None
                continue
        else:
            assigned_match = assigned_pattern.match(line)
            if assigned_match and current_cluster:
                current_cluster['label'] = assigned_match.group(1)
                cluster_frames.setdefault(current_cluster['t'], []).append(current_cluster)
                current_cluster = None

if current_cluster:
    cluster_frames.setdefault(current_cluster['t'], []).append(current_cluster)

cluster_stamps = sorted(cluster_frames.keys())
print(f"[cluster_diagnostic] 共解析到 {len(cluster_frames)} 帧，{sum(len(c) for c in cluster_frames.values())} 个候选簇")
print(f"[解析统计] 匹配成功: {parse_success}, 解析失败: {parse_fail}")

missed_gt_details = []
total_hit = 0
total_miss = 0
total_fp = 0

gt_by_dist_bin = {}
for bin_idx in range(4):
    gt_by_dist_bin[bin_idx] = {'total': 0, 'hit': 0}

for det in det_records:
    t = det['stamp']
    odom = interpolate_odom(t)
    gt = interpolate_gt(t)
    if odom is None:
        continue
    if gt is None or len(gt['poses']) == 0:
        for p in det['poses']:
            total_fp += 1
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
            total_fp += 1

    total_hit += len(matched_gt)

    for i, (gx, gy) in enumerate(gt_boat_frame):
        dist = math.hypot(gx, gy)
        bin_idx = min(int(dist // 20), 3)
        gt_by_dist_bin[bin_idx]['total'] += 1
        if i in matched_gt:
            gt_by_dist_bin[bin_idx]['hit'] += 1

    for i, (gx, gy) in enumerate(gt_boat_frame):
        if i not in matched_gt:
            total_miss += 1
            matched_det = None
            matched_det_d = 1e9
            for dx, dy in det_pts:
                d = math.hypot(dx-gx, dy-gy)
                if d < matched_det_d:
                    matched_det_d = d
                    matched_det = (dx, dy)

            best_cluster = None
            best_cluster_d = 1e9
            best_cluster_label = None
            best_cluster_fp_max = None
            best_cluster_dz = None
            best_cluster_pts = None

            idx = bisect.bisect_left(cluster_stamps, t)
            candidate_times = []
            if idx < len(cluster_stamps): candidate_times.append(cluster_stamps[idx])
            if idx > 0: candidate_times.append(cluster_stamps[idx-1])

            for ct in candidate_times:
                if abs(ct - t) > FRAME_MATCH_TOL:
                    continue
                for cluster in cluster_frames[ct]:
                    cx, cy = cluster['x'], cluster['y']
                    d = math.hypot(cx - gx, cy - gy)
                    if d < best_cluster_d:
                        best_cluster_d = d
                        best_cluster = (cx, cy)
                        best_cluster_label = cluster['label']
                        best_cluster_fp_max = cluster['fp_max']
                        best_cluster_dz = cluster['dz']
                        best_cluster_pts = cluster['pts']

            missed_gt_details.append({
                't': t,
                'gt_x': gx,
                'gt_y': gy,
                'closest_det_d': matched_det_d if matched_det else None,
                'closest_cluster_d': best_cluster_d if best_cluster else None,
                'closest_cluster_label': best_cluster_label,
                'closest_cluster_fp_max': best_cluster_fp_max,
                'closest_cluster_dz': best_cluster_dz,
                'closest_cluster_pts': best_cluster_pts,
                'has_cluster_candidate': best_cluster_d <= CANDIDATE_SEARCH_RADIUS if best_cluster else False
            })

print(f"\n=== 检测消息总数: {len(det_records)} ===")
print(f"真值命中(去重前累加): {total_hit}")
print(f"真值遗漏: {total_miss}")
print(f"误检: {total_fp}")
if total_hit + total_miss > 0:
    print(f"平均召回率: {total_hit/(total_hit+total_miss)*100:.1f}%")

cluster_phase_issue = 0
classification_phase_issue = 0
label_distribution = {}

for miss in missed_gt_details:
    if miss['has_cluster_candidate']:
        classification_phase_issue += 1
        label = miss['closest_cluster_label'] or 'unknown'
        label_distribution[label] = label_distribution.get(label, 0) + 1
    else:
        cluster_phase_issue += 1

print(f"\n=== 漏检归因分析 ===")
print(f"漏检总数: {len(missed_gt_details)}")
print(f"聚类阶段问题(无候选簇在{CANDIDATE_SEARCH_RADIUS}m范围内): {cluster_phase_issue} ({cluster_phase_issue/len(missed_gt_details)*100:.1f}%)")
print(f"分类阶段问题(有候选簇但未判定为boat): {classification_phase_issue} ({classification_phase_issue/len(missed_gt_details)*100:.1f}%)")

print(f"\n=== 分类阶段问题 - label分布 ===")
for label, count in sorted(label_distribution.items(), key=lambda x: -x[1]):
    percent = count / classification_phase_issue * 100 if classification_phase_issue > 0 else 0
    print(f"  {label}: {count} ({percent:.1f}%)")

print(f"\n=== 分类阶段问题 - discarded_shape_implausible样本明细(fp_max在2.0~2.5区间) ===")
implausible_samples = [m for m in missed_gt_details if m['has_cluster_candidate'] and m['closest_cluster_label'] == 'discarded_shape_implausible']
implausible_in_range = [m for m in implausible_samples if 2.0 <= m['closest_cluster_fp_max'] <= 2.5]
for m in implausible_in_range[:20]:
    print(f"  t={m['t']:.3f}s | gt=({m['gt_x']:.2f},{m['gt_y']:.2f}) | cluster_d={m['closest_cluster_d']:.2f}m | fp_max={m['closest_cluster_fp_max']:.2f} | pts={m['closest_cluster_pts']}")
if len(implausible_in_range) > 20:
    print(f"  ... 还有 {len(implausible_in_range) - 20} 个样本")

print(f"\n=== 聚类阶段问题 - 最近候选距离分布 ===")
cluster_phase_dists = [m['closest_cluster_d'] for m in missed_gt_details if not m['has_cluster_candidate'] and m['closest_cluster_d'] is not None]
if cluster_phase_dists:
    print(f"  最小距离: {min(cluster_phase_dists):.2f}m")
    print(f"  最大距离: {max(cluster_phase_dists):.2f}m")
    print(f"  平均距离: {sum(cluster_phase_dists)/len(cluster_phase_dists):.2f}m")
    for threshold in [4.0, 5.0, 6.0, 8.0, 10.0]:
        count = sum(1 for d in cluster_phase_dists if d <= threshold)
        print(f"  在{threshold}m范围内有候选的比例: {count}/{len(cluster_phase_dists)} ({count/len(cluster_phase_dists)*100:.1f}%)")
else:
    print(f"  无数据(所有漏检都有候选簇)")

print(f"\n=== 漏检真船方位角分桶 ===")
all_gt_in_boat_frame = []
for det in det_records:
    t = det['stamp']
    odom = interpolate_odom(t)
    gt = interpolate_gt(t)
    if odom is None:
        continue
    if gt is None or len(gt['poses']) == 0:
        for p in det['poses']:
            total_fp += 1
        continue
    yaw = quat_to_yaw(odom['qx'], odom['qy'], odom['qz'], odom['qw'])
    gt_boat_frame = [world_to_boat(p['x'], p['y'], odom['x'], odom['y'], yaw) for p in gt['poses']]
    for gx, gy in gt_boat_frame:
        all_gt_in_boat_frame.append((gx, gy))

def get_azimuth(x, y):
    if x == 0 and y == 0:
        return 'center'
    angle = math.atan2(y, x)
    angle_deg = math.degrees(angle)
    if -45 <= angle_deg < 45:
        return 'front'
    elif 45 <= angle_deg < 135:
        return 'right'
    elif angle_deg >= 135 or angle_deg < -135:
        return 'back'
    else:
        return 'left'

all_azimuth_counts = {}
for gx, gy in all_gt_in_boat_frame:
    az = get_azimuth(gx, gy)
    all_azimuth_counts[az] = all_azimuth_counts.get(az, 0) + 1

cluster_phase_azimuth_counts = {}
classification_phase_azimuth_counts = {}
for miss in missed_gt_details:
    az = get_azimuth(miss['gt_x'], miss['gt_y'])
    if miss['has_cluster_candidate']:
        classification_phase_azimuth_counts[az] = classification_phase_azimuth_counts.get(az, 0) + 1
    else:
        cluster_phase_azimuth_counts[az] = cluster_phase_azimuth_counts.get(az, 0) + 1

print(f"  真值分布(所有真船):")
for az in ['front', 'right', 'back', 'left']:
    count = all_azimuth_counts.get(az, 0)
    percent = count / len(all_gt_in_boat_frame) * 100 if all_gt_in_boat_frame else 0
    print(f"    {az}: {count} ({percent:.1f}%)")

print(f"  聚类阶段问题漏检分布:")
for az in ['front', 'right', 'back', 'left']:
    count = cluster_phase_azimuth_counts.get(az, 0)
    percent = count / cluster_phase_issue * 100 if cluster_phase_issue > 0 else 0
    print(f"    {az}: {count} ({percent:.1f}%)")

print(f"  分类阶段问题漏检分布:")
for az in ['front', 'right', 'back', 'left']:
    count = classification_phase_azimuth_counts.get(az, 0)
    percent = count / classification_phase_issue * 100 if classification_phase_issue > 0 else 0
    print(f"    {az}: {count} ({percent:.1f}%)")

print(f"\n=== buoy误判候选簇特征分布 ===")
buoy_misses = [m for m in missed_gt_details if m['has_cluster_candidate'] and m['closest_cluster_label'] == 'buoy']
if buoy_misses:
    fp_max_vals = [m['closest_cluster_fp_max'] for m in buoy_misses if m['closest_cluster_fp_max'] is not None]
    dz_vals = [m['closest_cluster_dz'] for m in buoy_misses if m['closest_cluster_dz'] is not None]
    pts_vals = [m['closest_cluster_pts'] for m in buoy_misses if m['closest_cluster_pts'] is not None]
    
    print(f"  样本数: {len(buoy_misses)}")
    print(f"  fp_max分布:")
    print(f"    最小: {min(fp_max_vals):.2f}m, 最大: {max(fp_max_vals):.2f}m, 平均: {sum(fp_max_vals)/len(fp_max_vals):.2f}m")
    for rng in [(0.0, 0.5), (0.5, 0.8), (0.8, 1.0), (1.0, 1.5), (1.5, 2.0)]:
        cnt = sum(1 for v in fp_max_vals if rng[0] <= v < rng[1])
        pct = cnt / len(fp_max_vals) * 100 if fp_max_vals else 0
        print(f"    [{rng[0]:.1f}, {rng[1]:.1f}): {cnt} ({pct:.1f}%)")
    
    print(f"  dz分布:")
    print(f"    最小: {min(dz_vals):.2f}m, 最大: {max(dz_vals):.2f}m, 平均: {sum(dz_vals)/len(dz_vals):.2f}m")
    for rng in [(0.0, 0.5), (0.5, 1.0), (1.0, 1.5), (1.5, 2.0), (2.0, 3.0)]:
        cnt = sum(1 for v in dz_vals if rng[0] <= v < rng[1])
        pct = cnt / len(dz_vals) * 100 if dz_vals else 0
        print(f"    [{rng[0]:.1f}, {rng[1]:.1f}): {cnt} ({pct:.1f}%)")
    
    print(f"  pts分布:")
    print(f"    最小: {min(pts_vals)}, 最大: {max(pts_vals)}, 平均: {sum(pts_vals)/len(pts_vals):.1f}")
    for rng in [(0, 20), (20, 50), (50, 100), (100, 200), (200, 500)]:
        cnt = sum(1 for v in pts_vals if rng[0] <= v < rng[1])
        pct = cnt / len(pts_vals) * 100 if pts_vals else 0
        print(f"    [{rng[0]}, {rng[1]}): {cnt} ({pct:.1f}%)")
else:
    print(f"  无数据(没有buoy误判)")

print(f"\n=== 分桶召回率（每桶命中/真值总数）===")
dist_bins = [(0, 20), (20, 40), (40, 60), (60, 100)]
dist_counts = {}
for miss in missed_gt_details:
    dist = math.hypot(miss['gt_x'], miss['gt_y'])
    for rng in dist_bins:
        if rng[0] <= dist < rng[1]:
            dist_counts[rng] = dist_counts.get(rng, 0) + 1
            break

for bin_idx, rng in enumerate(dist_bins):
    miss_count = dist_counts.get(rng, 0)
    gt_total = gt_by_dist_bin[bin_idx]['total']
    gt_hit = gt_by_dist_bin[bin_idx]['hit']
    recall = gt_hit / gt_total * 100 if gt_total > 0 else 0
    miss_pct_of_total = miss_count / len(missed_gt_details) * 100 if missed_gt_details else 0
    print(f"  [{rng[0]:>3}, {rng[1]:>3}m): 真值={gt_total:>5} 命中={gt_hit:>4} 漏检={miss_count:>5} 召回率={recall:>5.1f}% (漏检占{miss_pct_of_total:.1f}%)")

print(f"\n=== boat vs pillar同距离点数对比 ===")
all_cluster_by_label = {}
for ct, clusters in cluster_frames.items():
    for cluster in clusters:
        label = cluster['label'] or 'unknown'
        if label not in all_cluster_by_label:
            all_cluster_by_label[label] = []
        dist = math.hypot(cluster['x'], cluster['y'])
        all_cluster_by_label[label].append((dist, cluster['pts']))

for label in ['boat', 'pillar', 'buoy', 'block']:
    if label not in all_cluster_by_label or len(all_cluster_by_label[label]) == 0:
        print(f"  {label}: 无数据")
        continue
    pts_by_dist = {}
    for dist, pts in all_cluster_by_label[label]:
        bin_idx = min(int(dist // 20), 3)
        if bin_idx not in pts_by_dist:
            pts_by_dist[bin_idx] = []
        pts_by_dist[bin_idx].append(pts)
    print(f"  {label}:")
    for bin_idx in [0, 1, 2, 3]:
        if bin_idx not in pts_by_dist or len(pts_by_dist[bin_idx]) == 0:
            continue
        bin_name = f"[{bin_idx*20}, {(bin_idx+1)*20}m)" if bin_idx < 3 else "[60m+)"
        avg_pts = sum(pts_by_dist[bin_idx]) / len(pts_by_dist[bin_idx])
        print(f"    {bin_name}: avg_pts={avg_pts:.1f} (n={len(pts_by_dist[bin_idx])})")

print(f"\n=== 近距(0-20m)漏检样本解剖（前10个）===")
near_misses = []
for miss in missed_gt_details:
    dist = math.hypot(miss['gt_x'], miss['gt_y'])
    if dist < 20:
        near_misses.append(miss)

print(f"  0-20m漏检总数: {len(near_misses)}")
for i, m in enumerate(near_misses[:10]):
    dist = math.hypot(m['gt_x'], m['gt_y'])
    label = m['closest_cluster_label'] or '无候选'
    cluster_d = m['closest_cluster_d']
    fp_max = m['closest_cluster_fp_max']
    pts = m['closest_cluster_pts']
    det_d = m['closest_det_d']
    print(f"  [{i+1}] t={m['t']:.3f}s gt=({m['gt_x']:.2f},{m['gt_y']:.2f}) d={dist:.1f}m")
    print(f"      最近检测距离={det_d:.2f}m | 最近簇距离={cluster_d:.2f}m 簇label={label} fp_max={fp_max} pts={pts}")
