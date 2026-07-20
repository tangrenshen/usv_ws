#!/usr/bin/env python3
"""
把boat检测候选依次跟boat/pillar/buoy/block四类真值做匹配，
找出"两者都不匹配"的候选，到底更接近哪个类别，缩小误检来源范围。
"""
import json
import math
import bisect

LOG_PATH = '/home/lyf040817/usv_ws/boat_log.jsonl'
MATCH_TOL = 2.0
MAX_TIME_GAP = 0.5

records = {'gt': [], 'gt_pillar': [], 'gt_buoy': [], 'gt_block': [], 'odom': [], 'det': []}
with open(LOG_PATH) as f:
    for line in f:
        rec = json.loads(line)
        records[rec['type']].append(rec)

if records['gt'] and records['odom']:
    _offset = records['gt'][0]['stamp'] - records['odom'][0]['stamp']
    for key in ['gt', 'gt_pillar', 'gt_buoy', 'gt_block']:
        for r in records[key]:
            r['stamp'] -= _offset
    print(f"[时间基准校正] 偏移量: {_offset:.4f}s")

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

counts = {'boat': 0, 'pillar': 0, 'buoy': 0, 'block': 0, 'none': 0}
none_samples = []

for det in records['det']:
    t = det['stamp']
    odom, odom_gap = find_nearest('odom', t)
    if odom is None or odom_gap > MAX_TIME_GAP: continue

    gt_frames = {}
    ok = True
    for key in ['gt', 'gt_pillar', 'gt_buoy', 'gt_block']:
        rec, gap = find_nearest(key, t)
        if rec is None or gap > MAX_TIME_GAP:
            ok = False
            break
        gt_frames[key] = rec
    if not ok: continue

    yaw = quat_to_yaw(odom['qx'], odom['qy'], odom['qz'], odom['qw'])
    frames_boat = {k: [world_to_boat(p['x'], p['y'], odom['x'], odom['y'], yaw) for p in gt_frames[k]['poses']]
                   for k in gt_frames}

    for p in det['poses']:
        dx, dy = p['x'], p['y']
        d = {}
        d['boat'] = min((math.hypot(dx-gx,dy-gy) for gx,gy in frames_boat['gt']), default=1e9)
        d['pillar'] = min((math.hypot(dx-gx,dy-gy) for gx,gy in frames_boat['gt_pillar']), default=1e9)
        d['buoy'] = min((math.hypot(dx-gx,dy-gy) for gx,gy in frames_boat['gt_buoy']), default=1e9)
        d['block'] = min((math.hypot(dx-gx,dy-gy) for gx,gy in frames_boat['gt_block']), default=1e9)

        best_cat = min(d, key=d.get)
        if d[best_cat] < MATCH_TOL:
            counts[best_cat] += 1
        else:
            counts['none'] += 1
            if len(none_samples) < 15:
                none_samples.append((dx, dy, {k: round(v,1) for k,v in d.items()}))

total = sum(counts.values())
print(f"\n检测候选总数: {total}")
for k, v in counts.items():
    print(f"  {k}: {v} ({v/total*100:.1f}%)")

print(f"\n=== 完全无法匹配的候选样本(最近15个，附各类别最近距离) ===")
for dx, dy, dists in none_samples:
    print(f"  ({dx:.2f},{dy:.2f}) {dists}")
