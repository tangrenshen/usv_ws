#!/usr/bin/env python3
"""
漏检候选label归因分析——产出改判据的施工图
核心方法：
1. 找到所有漏检的真值船（近距0-20m和远距20-40m分开统计）
2. 对每艘漏检船，找到它±2m内的候选簇
3. 统计这些候选簇的label分布和特征（fp_max、pts、dz等）
4. 产出"漏检船到底死在哪条判据"的施工图
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
CANDIDATE_SEARCH_RADIUS = 2.0

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

NUM = r'[-+]?[\d.]+(?:[eE][-+]?\d+)?'
pending_pattern = re.compile(
    r'\[cluster_diagnostic\] t=(' + NUM + r') center=\((' + NUM + r'),(' + NUM + r'),(' + NUM + r')\)'
    r' size=\((' + NUM + r'),(' + NUM + r'),(' + NUM + r')\)'
    r' fp_max=(' + NUM + r') fp_min=(' + NUM + r') square=(' + NUM + r') pts=(\d+) -> classification pending'
)

# cluster_diagnostic的t=来自和det相同的sensor_stamp_sec/now()来源，可能和det一样
# 处于绝对纪元域（需要套epoch_offset）也可能已经在odom相对域（main.cpp换成用传感器
# header.stamp的版本），不能硬编码"总是减offset"——第一遍只解析原始值，用中位数样本
# 判断这批时间戳整体上是否需要平移，第二遍再real构建cluster_frames，顺带滤掉明显损坏
# 的离群值（多线程并发写std::cout导致的字符级数据损坏，比如"61782954238"这种被多插入
# 一位数字的值）。
_raw_clusters = []
with open(CLUSTER_LOG_PATH, encoding='utf-8', errors='replace') as f:
    current_cluster = None
    for line in f:
        line = line.strip()
        pending_match = pending_pattern.match(line)
        if pending_match:
            if current_cluster:
                _raw_clusters.append(current_cluster)
            try:
                current_cluster = {
                    't': float(pending_match.group(1)),
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
    if current_cluster:
        _raw_clusters.append(current_cluster)

_cluster_sample = _raw_clusters[len(_raw_clusters) // 2]['t'] if _raw_clusters else 0.0
_odom_mid = odom_stamps[len(odom_stamps) // 2]
_needs_shift = abs(_cluster_sample - _odom_mid) > 1000
if _needs_shift:
    print(f"[时间基准校正] cluster_diagnostic时间戳偏移量估计: {_epoch_offset:.4f}s，已统一到odom基准")

_expected_lo = odom_stamps[0] - 200
_expected_hi = odom_stamps[-1] + 200

cluster_frames = {}
_dropped_outliers = 0
for c in _raw_clusters:
    t = (c['t'] - _epoch_offset) if _needs_shift else c['t']
    if not (_expected_lo <= t <= _expected_hi):
        _dropped_outliers += 1
        continue
    c['t'] = t
    cluster_frames.setdefault(t, []).append(c)
if _dropped_outliers:
    print(f"[数据质量] 丢弃{_dropped_outliers}条明显损坏的cluster_diagnostic离群值（并发写std::cout导致的字符级数据损坏）")

cluster_stamps = sorted(cluster_frames.keys())
print(f"\ncluster_diagnostic: {len(cluster_frames)}帧, {sum(len(c) for c in cluster_frames.values())}个候选簇")

print(f"\n=== 任务3: 漏检候选label归因分析 ===")

missed_details = []

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
    
    idx = bisect.bisect_left(cluster_stamps, t)
    candidate_times = []
    if idx < len(cluster_stamps): candidate_times.append(cluster_stamps[idx])
    if idx > 0: candidate_times.append(cluster_stamps[idx-1])
    
    clusters_in_frame = []
    for ct in candidate_times:
        if abs(ct - t) <= FRAME_MATCH_TOL:
            clusters_in_frame.extend(cluster_frames[ct])
    
    for i, (gx, gy) in enumerate(gt_boat_frame):
        if i in matched_gt:
            continue
        
        dist = math.hypot(gx, gy)
        dist_bin = '0-20m' if dist < 20 else '20-40m' if dist < 40 else '40-60m' if dist < 60 else '60m+'
        
        best_cluster = None
        best_cluster_d = 1e9
        for cluster in clusters_in_frame:
            cd = math.hypot(cluster['x'] - gx, cluster['y'] - gy)
            if cd < best_cluster_d:
                best_cluster_d = cd
                best_cluster = cluster
        
        has_cluster = best_cluster_d <= CANDIDATE_SEARCH_RADIUS
        
        missed_details.append({
            'dist': dist,
            'dist_bin': dist_bin,
            'has_cluster': has_cluster,
            'cluster_dist': best_cluster_d if best_cluster else None,
            'label': best_cluster['label'] if best_cluster else None,
            'fp_max': best_cluster['fp_max'] if best_cluster else None,
            'fp_min': best_cluster['fp_min'] if best_cluster else None,
            'pts': best_cluster['pts'] if best_cluster else None,
            'dz': best_cluster['dz'] if best_cluster else None,
            'square': best_cluster['square'] if best_cluster else None
        })

print(f"\n=== 全局漏检统计 ===")
print(f"漏检总数: {len(missed_details)}")

for dist_bin in ['0-20m', '20-40m', '40-60m', '60m+']:
    bin_missed = [m for m in missed_details if m['dist_bin'] == dist_bin]
    has_cluster = sum(1 for m in bin_missed if m['has_cluster'])
    no_cluster = len(bin_missed) - has_cluster
    
    print(f"\n{dist_bin}:")
    print(f"  漏检数: {len(bin_missed)}")
    if len(bin_missed) > 0:
        print(f"  附近有候选簇: {has_cluster} ({has_cluster/len(bin_missed)*100:.1f}%)")
        print(f"  附近无候选簇: {no_cluster} ({no_cluster/len(bin_missed)*100:.1f}%)")
    else:
        print(f"  无漏检数据")
    
    if has_cluster > 0:
        label_dist = {}
        fp_max_bins = {}
        pts_bins = {}
        
        for m in bin_missed:
            if not m['has_cluster']:
                continue
            
            label = m['label'] or 'unknown'
            label_dist[label] = label_dist.get(label, 0) + 1
            
            fp_max = m['fp_max']
            fp_bin = '<0.5' if fp_max < 0.5 else '0.5-1.0' if fp_max <= 1.0 else '1.0-1.5' if fp_max <= 1.5 else '1.5-2.0' if fp_max <= 2.0 else '2.0-2.5' if fp_max <= 2.5 else '>2.5'
            fp_max_bins[fp_bin] = fp_max_bins.get(fp_bin, 0) + 1
            
            pts = m['pts']
            pts_bin = '<20' if pts < 20 else '20-50' if pts <= 50 else '50-100' if pts <= 100 else '100-200' if pts <= 200 else '>200'
            pts_bins[pts_bin] = pts_bins.get(pts_bin, 0) + 1
        
        print(f"\n  漏检船附近候选簇的label分布(施工图):")
        total = sum(label_dist.values())
        for label, count in sorted(label_dist.items(), key=lambda x: -x[1]):
            pct = count / total * 100
            print(f"    {label}: {count} ({pct:.1f}%)")
        
        print(f"\n  漏检船附近候选簇的fp_max分布:")
        for fp_bin, count in sorted(fp_max_bins.items()):
            pct = count / total * 100
            print(f"    {fp_bin}: {count} ({pct:.1f}%)")
        
        print(f"\n  漏检船附近候选簇的pts分布:")
        for pts_bin, count in sorted(pts_bins.items()):
            pct = count / total * 100
            print(f"    {pts_bin}: {count} ({pct:.1f}%)")

print(f"\n=== 与全局label分布对比 ===")
print(f"注意: 以下是20-40m范围内所有候选簇的label分布(非漏检专属):")
print(f"  buoy: 6008 (37.7%)")
print(f"  discarded_shape_implausible: 3714 (23.3%)")
print(f"  pillar: 1727 (10.8%)")
print(f"  block: 1607 (10.1%)")
print(f"  boat_fallback: 1469 (9.2%)")
print(f"  boat: 948 (6.0%)")
print(f"\n施工图只看'漏检船附近的候选簇',排除了'判对的真浮标/立柱',更精准")