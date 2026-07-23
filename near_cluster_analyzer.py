#!/usr/bin/env python3
"""
近距簇分析脚本：分析0-20m范围内簇点数少的原因。

分析维度：
1. 按距离分桶统计簇的特征分布（fp_max、dz、pts）
2. 对比近距和远距簇的特征差异
3. 分析近距簇被分类为什么类型
4. 检查近距簇是否被水面去除或体素降采样影响
"""
import json
import math
import re
import bisect

BOAT_LOG_PATH = '/home/lyf040817/usv_ws/boat_log.jsonl'
CLUSTER_LOG_PATH = '/home/lyf040817/usv_ws/perception_log.log'

gt_records = []
odom_records = []
with open(BOAT_LOG_PATH) as f:
    for line in f:
        rec = json.loads(line)
        if rec['type'] == 'gt':
            gt_records.append(rec)
        elif rec['type'] == 'odom':
            odom_records.append(rec)

if gt_records and odom_records:
    _offset = gt_records[0]['stamp'] - odom_records[0]['stamp']
    for r in gt_records:
        r['stamp'] -= _offset

odom_records.sort(key=lambda r: r['stamp'])
odom_stamps = [o['stamp'] for o in odom_records]

def interpolate_odom(t):
    idx = bisect.bisect_left(odom_stamps, t)
    if idx == 0:
        return odom_records[0] if odom_records else None
    if idx >= len(odom_records):
        return odom_records[-1] if odom_records else None
    r0, r1 = odom_records[idx-1], odom_records[idx]
    t0, t1 = r0['stamp'], r1['stamp']
    if abs(t1 - t0) < 1e-9:
        return r0
    alpha = (t - t0) / (t1 - t0)
    return {'x': r0['x'] + (r1['x'] - r0['x']) * alpha, 'y': r0['y'] + (r1['y'] - r0['y']) * alpha, 'qx': r0['qx'], 'qy': r0['qy'], 'qz': r0['qz'], 'qw': r0['qw']}

def quat_to_yaw(qx, qy, qz, qw):
    return math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))

def world_to_boat(wx, wy, bx, by, yaw):
    dx, dy = wx - bx, wy - by
    rx = dx*math.cos(-yaw) - dy*math.sin(-yaw)
    ry = dx*math.sin(-yaw) + dy*math.cos(-yaw)
    return rx, ry

NUM = r'[-+]?[\d.]+(?:[eE][-+]?\d+)?'
pending_pattern = re.compile(
    r'\[cluster_diagnostic\] t=(' + NUM + r') center=\((' + NUM + r'),(' + NUM + r'),(' + NUM + r')\)'
    r' size=\((' + NUM + r'),(' + NUM + r'),(' + NUM + r')\)'
    r' fp_max=(' + NUM + r') fp_min=(' + NUM + r') square=(' + NUM + r') pts=(\d+) -> classification pending'
)
# 成功分类(block/buoy/pillar/boat/*_fallback)打印格式是"t=... -> assigned=X"，assigned=
# 不在行首；只有discarded_*两条丢弃路径是行首简短格式。不能锚定行首(.match)，否则所有
# 成功分类的label全部丢失、被误记成None。
assigned_pattern = re.compile(r'assigned=(\w+)')

all_clusters = []
with open(CLUSTER_LOG_PATH, encoding='utf-8', errors='replace') as f:
    current_cluster = None
    for line in f:
        line = line.strip()
        pending_match = pending_pattern.match(line)
        if pending_match:
            if current_cluster:
                all_clusters.append(current_cluster)
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
            assigned_match = assigned_pattern.search(line)
            if assigned_match and current_cluster:
                current_cluster['label'] = assigned_match.group(1)
                all_clusters.append(current_cluster)
                current_cluster = None

if current_cluster:
    all_clusters.append(current_cluster)

print(f"总候选簇数: {len(all_clusters)}")

# cluster_diagnostic的center x,y本身已经是船体系(base_link)坐标(ObstacleDetector.hpp文档:
# "输入：船体坐标系(base_link)下的融合点云"，detect()形参名cloud_boat)，不需要再套
# world_to_boat——那样等于把已经是船体系的小量级坐标当世界坐标去减odom的大量级世界坐标，
# 凭空制造出几百米的偏移(cost_evaluation.py踩过同一个坑，45m被误算成763m)。
# t=的时钟域判断也需要和boat_evaluator.py的_align_to_odom一致，不能硬编码固定offset。
_cluster_sample = all_clusters[len(all_clusters)//2]['t'] if all_clusters else 0.0
_odom_mid = odom_stamps[len(odom_stamps)//2]
_needs_shift = abs(_cluster_sample - _odom_mid) > 1000
_expected_lo, _expected_hi = odom_stamps[0] - 200, odom_stamps[-1] + 200
_dropped_outliers = 0

clusters_with_dist = []
for c in all_clusters:
    t_relative = (c['t'] - _offset) if _needs_shift else c['t']
    if not (_expected_lo <= t_relative <= _expected_hi):
        _dropped_outliers += 1
        continue
    odom = interpolate_odom(t_relative)
    if odom is None:
        continue
    dist = math.hypot(c['x'], c['y'])
    clusters_with_dist.append({**c, 'dist': dist})

if _dropped_outliers:
    print(f"[数据质量] 丢弃{_dropped_outliers}条明显损坏的cluster_diagnostic离群值")

print(f"坐标转换成功的簇数: {len(clusters_with_dist)}")

distance_bins = [
    (0, 20, '0-20m'),
    (20, 40, '20-40m'),
    (40, 60, '40-60m'),
    (60, 100, '60-100m'),
    (100, 500, '100-500m'),
    (500, float('inf'), '500m+')
]

print("\n" + "="*80)
print("=== [距离分桶簇特征统计] ===")
print("="*80)

all_labels = {}

for min_d, max_d, name in distance_bins:
    bucket = [c for c in clusters_with_dist if min_d <= c['dist'] < max_d]
    if not bucket:
        continue
    
    fp_max_vals = [c['fp_max'] for c in bucket]
    dz_vals = [c['dz'] for c in bucket]
    pts_vals = [c['pts'] for c in bucket]
    
    print(f"\n--- {name} (共 {len(bucket)} 个簇) ---")
    print(f"  fp_max: min={min(fp_max_vals):.2f}, max={max(fp_max_vals):.2f}, avg={sum(fp_max_vals)/len(fp_max_vals):.2f}")
    print(f"  dz:     min={min(dz_vals):.2f}, max={max(dz_vals):.2f}, avg={sum(dz_vals)/len(dz_vals):.2f}")
    print(f"  pts:    min={min(pts_vals)}, max={max(pts_vals)}, avg={sum(pts_vals)/len(pts_vals):.1f}")
    
    pts_dist = {}
    for c in bucket:
        if c['pts'] < 10:
            cat = 'pts<10'
        elif c['pts'] < 40:
            cat = '10<=pts<40'
        elif c['pts'] < 100:
            cat = '40<=pts<100'
        else:
            cat = 'pts>=100'
        pts_dist[cat] = pts_dist.get(cat, 0) + 1
    
    print(f"  点数分布: {pts_dist}")
    
    label_dist = {}
    for c in bucket:
        lbl = c['label'] or 'unknown'
        label_dist[lbl] = label_dist.get(lbl, 0) + 1
    
    print(f"  分类标签: {label_dist}")
    
    for lbl, cnt in label_dist.items():
        if lbl not in all_labels:
            all_labels[lbl] = {}
        all_labels[lbl][name] = cnt

print("\n" + "="*80)
print("=== [按分类标签看距离分布] ===")
print("="*80)

for lbl, dist_dict in sorted(all_labels.items()):
    total = sum(dist_dict.values())
    print(f"\n--- {lbl} (共 {total} 个) ---")
    for min_d, max_d, name in distance_bins:
        cnt = dist_dict.get(name, 0)
        if cnt > 0:
            print(f"  {name}: {cnt} ({cnt/total*100:.1f}%)")

print("\n" + "="*80)
print("=== [0-20m近距簇深度分析] ===")
print("="*80)

near_clusters = [c for c in clusters_with_dist if c['dist'] < 20]
print(f"\n0-20m范围内总簇数: {len(near_clusters)}")

if near_clusters:
    near_clusters_sorted = sorted(near_clusters, key=lambda c: c['dist'])
    
    print("\n--- 近距簇按距离排序(前20个) ---")
    for c in near_clusters_sorted[:20]:
        print(f"  dist={c['dist']:.1f}m | fp_max={c['fp_max']:.2f} | dz={c['dz']:.2f} | pts={c['pts']} | label={c['label']}")
    
    print("\n--- 近距簇按点数排序(前20个) ---")
    near_clusters_by_pts = sorted(near_clusters, key=lambda c: c['pts'], reverse=True)
    for c in near_clusters_by_pts[:20]:
        print(f"  pts={c['pts']} | dist={c['dist']:.1f}m | fp_max={c['fp_max']:.2f} | dz={c['dz']:.2f} | label={c['label']}")
    
    print("\n--- 近距簇中pts>=40的候选boat ---")
    near_boat_candidates = [c for c in near_clusters if c['pts'] >= 40]
    print(f"pts>=40的近距簇数: {len(near_boat_candidates)}")
    for c in sorted(near_boat_candidates, key=lambda c: c['dist']):
        print(f"  dist={c['dist']:.1f}m | fp_max={c['fp_max']:.2f} | dz={c['dz']:.2f} | pts={c['pts']} | label={c['label']}")
    
    print("\n--- 近距簇特征分布直方图 ---")
    fp_bins = [0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 5.0, 10.0, float('inf')]
    pts_bins = [0, 10, 20, 40, 80, 160, float('inf')]
    dz_bins = [0, 0.3, 0.5, 0.7, 1.0, 1.5, 2.0, float('inf')]
    
    def print_hist(data, bins, name):
        counts = [0] * (len(bins)-1)
        for v in data:
            for i in range(len(bins)-1):
                if bins[i] <= v < bins[i+1]:
                    counts[i] += 1
                    break
        print(f"  {name}:")
        for i in range(len(bins)-1):
            if counts[i] > 0:
                print(f"    [{bins[i]:.1f}, {bins[i+1]:.1f}): {counts[i]}")
    
    print_hist([c['fp_max'] for c in near_clusters], fp_bins, 'fp_max')
    print_hist([c['pts'] for c in near_clusters], pts_bins, 'pts')
    print_hist([c['dz'] for c in near_clusters], dz_bins, 'dz')

print("\n" + "="*80)
print("=== [关键发现] ===")
print("="*80)

near_pts_avg = sum(c['pts'] for c in near_clusters) / max(len(near_clusters), 1) if near_clusters else 0
far_clusters = [c for c in clusters_with_dist if c['dist'] >= 20]
far_pts_avg = sum(c['pts'] for c in far_clusters) / max(len(far_clusters), 1) if far_clusters else 0

print(f"\n1. 近距(0-20m)簇平均点数: {near_pts_avg:.1f}")
print(f"   远距(20m+)簇平均点数: {far_pts_avg:.1f}")

near_fp_max_avg = sum(c['fp_max'] for c in near_clusters) / max(len(near_clusters), 1) if near_clusters else 0
far_fp_max_avg = sum(c['fp_max'] for c in far_clusters) / max(len(far_clusters), 1) if far_clusters else 0

print(f"\n2. 近距簇平均fp_max: {near_fp_max_avg:.2f}")
print(f"   远距簇平均fp_max: {far_fp_max_avg:.2f}")

near_boat_qualified = [c for c in near_clusters if c['pts'] >= 40]
print(f"\n3. 近距簇中pts>=40的数量: {len(near_boat_qualified)}")

if near_clusters:
    near_pts_below_40 = sum(1 for c in near_clusters if c['pts'] < 40)
    print(f"   近距簇中pts<40的比例: {near_pts_below_40}/{len(near_clusters)} ({near_pts_below_40/len(near_clusters)*100:.1f}%)")

if far_clusters:
    far_pts_below_40 = sum(1 for c in far_clusters if c['pts'] < 40)
    print(f"   远距簇中pts<40的比例: {far_pts_below_40}/{len(far_clusters)} ({far_pts_below_40/len(far_clusters)*100:.1f}%)")

near_cluster_size = [c['fp_max'] * c['dz'] * c['pts'] for c in near_clusters] if near_clusters else []
far_cluster_size = [c['fp_max'] * c['dz'] * c['pts'] for c in far_clusters] if far_clusters else []
near_size_avg = sum(near_cluster_size) / max(len(near_cluster_size), 1) if near_cluster_size else 0
far_size_avg = sum(far_cluster_size) / max(len(far_cluster_size), 1) if far_cluster_size else 0

print(f"\n4. 近距簇平均'体积×点数'特征值: {near_size_avg:.1f}")
print(f"   远距簇平均'体积×点数'特征值: {far_size_avg:.1f}")