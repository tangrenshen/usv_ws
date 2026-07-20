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

if current_cluster:
    cluster_frames.setdefault(current_cluster['t'], []).append(current_cluster)

cluster_stamps = sorted(cluster_frames.keys())
print(f"[cluster_diagnostic] 共解析到 {len(cluster_frames)} 帧，{sum(len(c) for c in cluster_frames.values())} 个候选簇")

missed_gt_details = []
total_hit = 0
total_miss = 0
total_fp = 0

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
                        best_cluster_pts = cluster['pts']

            missed_gt_details.append({
                't': t,
                'gt_x': gx,
                'gt_y': gy,
                'closest_det_d': matched_det_d if matched_det else None,
                'closest_cluster_d': best_cluster_d if best_cluster else None,
                'closest_cluster_label': best_cluster_label,
                'closest_cluster_fp_max': best_cluster_fp_max,
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
