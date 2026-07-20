#!/usr/bin/env python3
"""
专门分析boat_fallback分支的候选：把特征行(t=..., size=..., pts=...)和紧跟着的
assigned=标签行配对，只筛出label=boat_fallback的候选，跟真值做位置匹配，
看这条路径下"真实船只"和"误判pillar"各自的dz分布，为兜底分支单独定高度阈值。
"""
import re
import json
import math
import bisect

LOG_PATH = '/home/lyf040817/usv_ws/ml_train_log.txt'
JSONL_PATH = '/home/lyf040817/usv_ws/boat_log.jsonl'
MATCH_TOL = 2.0
MAX_TIME_GAP = 0.5

feature_pattern = re.compile(
    r'\[cluster_diagnostic\] t=([\d.]+) center=\(([-\d.]+),([-\d.]+),([-\d.]+)\) '
    r'size=\(([-\d.]+),([-\d.]+),([-\d.]+)\) fp_max=([-\d.]+) fp_min=([-\d.]+) '
    r'square=([-\d.]+) pts=(\d+)'
)
assigned_pattern = re.compile(r'\[cluster_diagnostic\] assigned=(\S+)')

records = []
pending = None
with open(LOG_PATH) as f:
    for line in f:
        m1 = feature_pattern.search(line)
        if m1:
            pending = {
                'stamp': float(m1.group(1)),
                'x': float(m1.group(2)), 'y': float(m1.group(3)), 'z': float(m1.group(4)),
                'dx': float(m1.group(5)), 'dy': float(m1.group(6)), 'dz': float(m1.group(7)),
                'fp_max': float(m1.group(8)), 'fp_min': float(m1.group(9)),
                'square': float(m1.group(10)), 'pts': int(m1.group(11)),
            }
            continue
        m2 = assigned_pattern.search(line)
        if m2 and pending is not None:
            pending['label'] = m2.group(1)
            records.append(pending)
            pending = None

fallback_records = [r for r in records if r['label'] == 'boat_fallback']
print(f"总候选记录: {len(records)}")
print(f"其中label=boat_fallback: {len(fallback_records)}")

# 加载真值
gt = {'gt': [], 'gt_pillar': [], 'gt_buoy': [], 'gt_block': [], 'odom': []}
with open(JSONL_PATH) as f:
    for line in f:
        rec = json.loads(line)
        if rec['type'] in gt:
            gt[rec['type']].append(rec)

if gt['gt'] and gt['odom']:
    _offset = gt['gt'][0]['stamp'] - gt['odom'][0]['stamp']
    for key in ['gt', 'gt_pillar', 'gt_buoy', 'gt_block']:
        for r in gt[key]:
            r['stamp'] -= _offset

for key in gt:
    gt[key].sort(key=lambda r: r['stamp'])
stamps = {key: [r['stamp'] for r in gt[key]] for key in gt}

def find_nearest(key, t):
    stamp_list = stamps[key]; rec_list = gt[key]
    idx = bisect.bisect_left(stamp_list, t)
    candidates = []
    if idx < len(rec_list): candidates.append(rec_list[idx])
    if idx > 0: candidates.append(rec_list[idx-1])
    if not candidates: return None, None
    best = min(candidates, key=lambda r: abs(r['stamp'] - t))
    return best, abs(best['stamp'] - t)

def world_to_boat(wx, wy, bx, by, yaw):
    dx, dy = wx - bx, wy - by
    return (dx*math.cos(-yaw) - dy*math.sin(-yaw), dx*math.sin(-yaw) + dy*math.cos(-yaw))

def quat_to_yaw(qx, qy, qz, qw):
    return math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))

by_cat = {'boat': [], 'pillar': [], 'buoy': [], 'block': [], 'none': []}
for r in fallback_records:
    t = r['stamp']
    odom, odom_gap = find_nearest('odom', t)
    if odom is None or odom_gap > MAX_TIME_GAP: continue

    yaw = quat_to_yaw(odom['qx'], odom['qy'], odom['qz'], odom['qw'])
    d = {}
    ok = True
    for key, label in [('gt', 'boat'), ('gt_pillar', 'pillar'), ('gt_buoy', 'buoy'), ('gt_block', 'block')]:
        rec, gap = find_nearest(key, t)
        if rec is None or gap > MAX_TIME_GAP:
            ok = False; break
        best_d = min((math.hypot(r['x']-bx, r['y']-by) for bx,by in
                      [world_to_boat(p['x'], p['y'], odom['x'], odom['y'], yaw) for p in rec['poses']]), default=1e9)
        d[label] = best_d
    if not ok: continue

    best_label = min(d, key=d.get)
    cat = best_label if d[best_label] < MATCH_TOL else 'none'
    by_cat[cat].append(r)

print(f"\n=== boat_fallback候选，按真值匹配结果分类 ===")
for cat, items in by_cat.items():
    if not items: continue
    dzs = [it['dz'] for it in items]
    print(f"\n{cat}: {len(items)}个")
    print(f"  dz范围: [{min(dzs):.2f}, {max(dzs):.2f}]  均值: {sum(dzs)/len(dzs):.2f}")

# dz阈值扫描
print(f"\n=== dz阈值扫描(仅限boat_fallback候选) ===")
dz_by_cat = {cat: [it['dz'] for it in items] for cat, items in by_cat.items() if items}
for threshold in [1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0]:
    print(f"\n阈值 dz <= {threshold}:")
    for cat in ['boat', 'pillar', 'block', 'buoy']:
        if cat not in dz_by_cat: continue
        vals = dz_by_cat[cat]
        kept = sum(1 for v in vals if v <= threshold)
        print(f"  {cat}: 保留 {kept}/{len(vals)} ({kept/len(vals)*100:.0f}%)")
