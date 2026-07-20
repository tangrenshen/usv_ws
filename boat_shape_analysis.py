#!/usr/bin/env python3
"""
把尺寸日志(带精确时间戳)跟真值匹配结果关联起来，
看"被误判成boat、实际匹配上block/pillar真值"的候选，真实尺寸/点数是什么样。
"""
import json
import math
import bisect
import re

LOG_PATH = '/tmp/boat_log.jsonl'
SHAPE_LOG = '/tmp/boat_shape_log.txt'
MATCH_TOL = 2.0
MAX_TIME_GAP = 0.5

# 解析尺寸日志: t=时间戳 船体系=(x,y,z) size=(dx,dy,dz) 点数=N
shape_records = []
pattern = re.compile(r't=([\d.]+) 船体系=\(([-\d.]+),([-\d.]+),([-\d.]+)\) size=\(([-\d.]+),([-\d.]+),([-\d.]+)\) 点数=(\d+)')
with open(SHAPE_LOG) as f:
    for line in f:
        m = pattern.search(line)
        if m:
            shape_records.append({
                'stamp': float(m.group(1)),
                'x': float(m.group(2)), 'y': float(m.group(3)), 'z': float(m.group(4)),
                'dx': float(m.group(5)), 'dy': float(m.group(6)), 'dz': float(m.group(7)),
                'pts': int(m.group(8)),
            })
print(f"解析到尺寸记录: {len(shape_records)}")

records = {'gt': [], 'gt_pillar': [], 'gt_buoy': [], 'gt_block': [], 'odom': []}
with open(LOG_PATH) as f:
    for line in f:
        rec = json.loads(line)
        if rec['type'] in records:
            records[rec['type']].append(rec)

if records['gt'] and records['odom']:
    _offset = records['gt'][0]['stamp'] - records['odom'][0]['stamp']
    for key in ['gt', 'gt_pillar', 'gt_buoy', 'gt_block']:
        for r in records[key]:
            r['stamp'] -= _offset

for key in records:
    records[key].sort(key=lambda r: r['stamp'])
stamps = {key: [r['stamp'] for r in records[key]] for key in records}

def find_nearest(key, t):
    stamp_list = stamps[key]
    rec_list = records[key]
    idx = bisect.bisect_left(stamp_list, t)
    candidates = []
    if idx < len(rec_list): candidates.append(rec_list[idx])
    if idx > 0: candidates.append(rec_list[idx-1])
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

by_category = {'boat': [], 'pillar': [], 'buoy': [], 'block': [], 'none': []}

for sr in shape_records:
    t = sr['stamp']
    odom, odom_gap = find_nearest('odom', t)
    if odom is None or odom_gap > MAX_TIME_GAP: continue

    gt_frames = {}
    ok = True
    for key in ['gt', 'gt_pillar', 'gt_buoy', 'gt_block']:
        rec, gap = find_nearest(key, t)
        if rec is None or gap > MAX_TIME_GAP:
            ok = False; break
        gt_frames[key] = rec
    if not ok: continue

    yaw = quat_to_yaw(odom['qx'], odom['qy'], odom['qz'], odom['qw'])
    frames_boat = {k: [world_to_boat(p['x'], p['y'], odom['x'], odom['y'], yaw) for p in gt_frames[k]['poses']]
                   for k in gt_frames}

    dx_, dy_ = sr['x'], sr['y']
    d = {}
    d['boat'] = min((math.hypot(dx_-gx,dy_-gy) for gx,gy in frames_boat['gt']), default=1e9)
    d['pillar'] = min((math.hypot(dx_-gx,dy_-gy) for gx,gy in frames_boat['gt_pillar']), default=1e9)
    d['buoy'] = min((math.hypot(dx_-gx,dy_-gy) for gx,gy in frames_boat['gt_buoy']), default=1e9)
    d['block'] = min((math.hypot(dx_-gx,dy_-gy) for gx,gy in frames_boat['gt_block']), default=1e9)

    best_cat = min(d, key=d.get)
    cat = best_cat if d[best_cat] < MATCH_TOL else 'none'
    by_category[cat].append(sr)

for cat, items in by_category.items():
    if not items: continue
    dxs = [it['dx'] for it in items]
    dys = [it['dy'] for it in items]
    dzs = [it['dz'] for it in items]
    pts = [it['pts'] for it in items]
    print(f"\n=== 匹配到{cat}真值的候选 (共{len(items)}个) ===")
    print(f"  dx: 均值={sum(dxs)/len(dxs):.2f} 范围=[{min(dxs):.2f},{max(dxs):.2f}]")
    print(f"  dy: 均值={sum(dys)/len(dys):.2f} 范围=[{min(dys):.2f},{max(dys):.2f}]")
    print(f"  dz: 均值={sum(dzs)/len(dzs):.2f} 范围=[{min(dzs):.2f},{max(dzs):.2f}]")
    print(f"  点数: 均值={sum(pts)/len(pts):.0f} 范围=[{min(pts)},{max(pts)}]")

print("\n\n=== fp_max(=max(dx,dy)) 阈值扫描：不同阈值下各类别的保留/排除比例 ===")
fp_max_by_cat = {cat: [max(it['dx'], it['dy']) for it in items] for cat, items in by_category.items() if items}
for threshold in [1.5, 2.0, 2.5, 2.8, 3.0, 3.5, 4.0]:
    print(f"\n阈值 fp_max <= {threshold}:")
    for cat in ['boat', 'block', 'pillar', 'buoy']:
        if cat not in fp_max_by_cat: continue
        vals = fp_max_by_cat[cat]
        kept = sum(1 for v in vals if v <= threshold)
        print(f"  {cat}: 保留 {kept}/{len(vals)} ({kept/len(vals)*100:.0f}%)")

print("\n\n=== dz(高度) 阈值扫描：不同上限下各类别的保留/排除比例 ===")
dz_by_cat = {cat: [it['dz'] for it in items] for cat, items in by_category.items() if items}
for threshold in [1.0, 1.5, 1.8, 2.0, 2.2, 2.5, 3.0]:
    print(f"\n阈值 dz <= {threshold}:")
    for cat in ['boat', 'block', 'pillar', 'buoy']:
        if cat not in dz_by_cat: continue
        vals = dz_by_cat[cat]
        kept = sum(1 for v in vals if v <= threshold)
        print(f"  {cat}: 保留 {kept}/{len(vals)} ({kept/len(vals)*100:.0f}%)")

print("\n\n=== 点数 阈值扫描：不同上限下各类别的保留/排除比例 ===")
pts_by_cat = {cat: [it['pts'] for it in items] for cat, items in by_category.items() if items}
for threshold in [100, 150, 200, 300, 400, 500, 700]:
    print(f"\n阈值 点数 <= {threshold}:")
    for cat in ['boat', 'block', 'pillar', 'buoy']:
        if cat not in pts_by_cat: continue
        vals = pts_by_cat[cat]
        kept = sum(1 for v in vals if v <= threshold)
        print(f"  {cat}: 保留 {kept}/{len(vals)} ({kept/len(vals)*100:.0f}%)")
