#!/usr/bin/env python3
"""
分析cluster_diagnostic输出：
1. 提取所有 assigned=boat 和 assigned=boat_fallback 的簇
2. 统计它们的特征（size、pts、fp_max等）
3. 把簇center转回世界系，对比gt船只位置
"""
import re
import math
import bisect
import json

LOG_PATH = '/home/lyf040817/usv_ws/perception_log.log'
BOAT_LOG_PATH = '/home/lyf040817/usv_ws/boat_log.jsonl'

# 解析cluster_diagnostic行
pattern = re.compile(
    r'\[cluster_diagnostic\] t=([-\d.e]+) center=\(([-\d.e]+),([-\d.e]+),([-\d.e]+)\) '
    r'size=\(([-\d.e]+),([-\d.e]+),([-\d.e]+)\) fp_max=([-\d.e]+) fp_min=([-\d.e]+) '
    r'square=([-\d.e]+) pts=(\d+) -> classification pending'
)

# 提取所有簇信息：[(t, cx, cy, cz, sx, sy, sz, fp_max, fp_min, sq, pts, label), ...]
clusters = []
current_cluster = None

with open(LOG_PATH) as f:
    for line in f:
        m = pattern.search(line)
        if m:
            current_cluster = {
                't': float(m.group(1)),
                'cx': float(m.group(2)),
                'cy': float(m.group(3)),
                'cz': float(m.group(4)),
                'sx': float(m.group(5)),
                'sy': float(m.group(6)),
                'sz': float(m.group(7)),
                'fp_max': float(m.group(8)),
                'fp_min': float(m.group(9)),
                'sq': float(m.group(10)),
                'pts': int(m.group(11)),
                'label': None
            }
        elif 'assigned=' in line and current_cluster is not None:
            label = line.strip().split('assigned=')[1]
            current_cluster['label'] = label
            clusters.append(current_cluster)
            current_cluster = None

print(f"总簇数: {len(clusters)}")

# 标签分布
from collections import Counter
labels = Counter(c['label'] for c in clusters)
print("\n=== 簇分类标签分布 ===")
for label, count in labels.most_common():
    print(f"  {label:35s}: {count}")

# 取所有 boat 和 boat_fallback 簇
boat_clusters = [c for c in clusters if c['label'] in ('boat', 'boat_fallback')]
print(f"\n=== boat + boat_fallback 簇数: {len(boat_clusters)} ===")

# 特征统计
print("\n=== boat/boat_fallback 簇特征统计 ===")
print(f"{'指标':15s} {'最小':>10s} {'最大':>10s} {'均值':>10s} {'中位数':>10s}")
for key, name in [('sx','size_x'),('sy','size_y'),('sz','size_z'),
                  ('fp_max','fp_max'),('fp_min','fp_min'),('sq','square'),('pts','pts')]:
    vals = sorted([c[key] for c in boat_clusters])
    n = len(vals)
    if n == 0: continue
    mean = sum(vals)/n
    median = vals[n//2] if n%2==1 else (vals[n//2-1]+vals[n//2])/2
    print(f"{name:15s} {min(vals):10.3f} {max(vals):10.3f} {mean:10.3f} {median:10.3f}")

# pts分布直方图
print("\n=== boat/boat_fallback 簇 pts 分布 ===")
pts_bins = [(0,50),(50,100),(100,300),(300,1000),(1000,3000),(3000,10000)]
for lo, hi in pts_bins:
    cnt = sum(1 for c in boat_clusters if lo <= c['pts'] < hi)
    print(f"  pts ∈ [{lo:5d}, {hi:5d}): {cnt}")

# size_z 分布（拖影假说预测：size_z偏大说明被涂抹拉长）
print("\n=== boat/boat_fallback 簇 size_z 分布 ===")
sz_bins = [(0,1.5),(1.5,3),(3,5),(5,7),(7,10),(10,100)]
for lo, hi in sz_bins:
    cnt = sum(1 for c in boat_clusters if lo <= c['sz'] < hi)
    print(f"  size_z ∈ [{lo:.1f}, {hi:.1f}): {cnt}")

# 加载gt和odom
records_by_type = {'gt': [], 'gt_pillar': [], 'gt_buoy': [], 'gt_block': [], 'odom': []}
with open(BOAT_LOG_PATH) as f:
    for line in f:
        rec = json.loads(line)
        if rec['type'] in records_by_type:
            records_by_type[rec['type']].append(rec)

gt_records = records_by_type['gt']
odom_records = records_by_type['odom']
if gt_records and odom_records:
    _offset = gt_records[0]['stamp'] - odom_records[0]['stamp']
    for t in ['gt', 'gt_pillar', 'gt_buoy', 'gt_block']:
        for r in records_by_type[t]:
            r['stamp'] -= _offset

for t in records_by_type:
    records_by_type[t].sort(key=lambda r: r['stamp'])
odom_stamps = [r['stamp'] for r in odom_records]

def find_nearest(stamps, records, t):
    idx = bisect.bisect_left(stamps, t)
    candidates = []
    if idx < len(records): candidates.append(records[idx])
    if idx > 0: candidates.append(records[idx-1])
    if not candidates: return None, None
    best = min(candidates, key=lambda r: abs(r['stamp'] - t))
    return best, abs(best['stamp'] - t)

def quat_to_yaw(qx, qy, qz, qw):
    return math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))

# 把每个boat簇的center转回世界系，对比gt船只位置
print("\n=== boat簇center对应的世界系位置 vs gt船只 ===")
matches = {'boat': 0, 'gt_pillar': 0, 'gt_buoy': 0, 'gt_block': 0, 'none': 0}
min_dists_to_boat = []
min_dists_to_pillar = []
min_dists_to_buoy = []

for c in boat_clusters:
    t = c['t']
    odom, gap = find_nearest(odom_stamps, odom_records, t)
    if odom is None or gap > 0.5:
        continue
    yaw = quat_to_yaw(odom['qx'], odom['qy'], odom['qz'], odom['qw'])
    bx, by = odom['x'], odom['y']
    # 船体系 -> 世界系
    wx = bx + c['cx']*math.cos(yaw) - c['cy']*math.sin(yaw)
    wy = by + c['cx']*math.sin(yaw) + c['cy']*math.cos(yaw)

    # 查找各类gt
    best_type = 'none'
    best_d = 1e9
    for gt_type in ['gt', 'gt_pillar', 'gt_buoy', 'gt_block']:
        gt_list, ggap = find_nearest([r['stamp'] for r in records_by_type[gt_type]], records_by_type[gt_type], t)
        if gt_list is None or ggap > 0.5:
            continue
        for gp in gt_list['poses']:
            d = math.hypot(gp['x']-wx, gp['y']-wy)
            if d < best_d:
                best_d = d
                best_type = gt_type
    matches[best_type] = matches.get(best_type, 0) + 1
    if best_type == 'gt':
        min_dists_to_boat.append(best_d)
    elif best_type == 'gt_pillar':
        min_dists_to_pillar.append(best_d)
    elif best_type == 'gt_buoy':
        min_dists_to_buoy.append(best_d)

print(f"\nboat/boat_fallback簇的世界系位置对应最近gt类型:")
for t, c in sorted(matches.items(), key=lambda x: -x[1]):
    pct = c/len(boat_clusters)*100 if boat_clusters else 0
    print(f"  {t:12s}: {c:4d} ({pct:.1f}%)")

if min_dists_to_boat:
    print(f"\n匹配到gt船只的boat簇到船只距离:")
    print(f"  数量={len(min_dists_to_boat)}  均值={sum(min_dists_to_boat)/len(min_dists_to_boat):.2f}m  最小={min(min_dists_to_boat):.2f}m  最大={max(min_dists_to_boat):.2f}m")

# 单独看前几个boat簇，详细打印
print("\n=== 前5个boat簇的详细信息（含世界系位置和gt对比） ===")
shown = 0
for c in boat_clusters:
    if shown >= 5: break
    t = c['t']
    odom, gap = find_nearest(odom_stamps, odom_records, t)
    if odom is None or gap > 0.5: continue
    yaw = quat_to_yaw(odom['qx'], odom['qy'], odom['qz'], odom['qw'])
    bx, by = odom['x'], odom['y']
    wx = bx + c['cx']*math.cos(yaw) - c['cy']*math.sin(yaw)
    wy = by + c['cx']*math.sin(yaw) + c['cy']*math.cos(yaw)

    print(f"\n  #{shown+1} t={t:.3f}s label={c['label']}")
    print(f"    船体系 center=({c['cx']:.2f}, {c['cy']:.2f}, {c['cz']:.2f})")
    print(f"    size=({c['sx']:.2f}, {c['sy']:.2f}, {c['sz']:.2f})  fp_max={c['fp_max']:.2f}  pts={c['pts']}")
    print(f"    本船 ({bx:.2f}, {by:.2f}) yaw={math.degrees(yaw):.2f}°")
    print(f"    世界系 center=({wx:.2f}, {wy:.2f})")
    print(f"    各类gt最近障碍物:")
    for gt_type in ['gt', 'gt_pillar', 'gt_buoy', 'gt_block']:
        gt_list, ggap = find_nearest([r['stamp'] for r in records_by_type[gt_type]], records_by_type[gt_type], t)
        if gt_list is None or ggap > 0.5: continue
        best_d = 1e9
        best_pose = None
        for gp in gt_list['poses']:
            d = math.hypot(gp['x']-wx, gp['y']-wy)
            if d < best_d:
                best_d = d
                best_pose = gp
        if best_pose:
            print(f"      {gt_type:12s}: 距离={best_d:.2f}m  位置=({best_pose['x']:.2f}, {best_pose['y']:.2f})")
    shown += 1
