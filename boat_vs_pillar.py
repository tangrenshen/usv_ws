#!/usr/bin/env python3
"""
验证"游船误检候选是否实为立柱"的假说：
把boat候选同时跟boat真值和pillar真值做匹配，看有多少boat候选其实更接近某根立柱。
"""
import json
import math
import bisect

LOG_PATH = '/tmp/boat_log.jsonl'
MATCH_TOL = 2.0
MAX_TIME_GAP = 0.5

gt_boat_records, gt_pillar_records, odom_records, det_records = [], [], [], []
with open(LOG_PATH) as f:
    for line in f:
        rec = json.loads(line)
        if rec['type'] == 'gt': gt_boat_records.append(rec)
        elif rec['type'] == 'gt_pillar': gt_pillar_records.append(rec)
        elif rec['type'] == 'odom': odom_records.append(rec)
        elif rec['type'] == 'det': det_records.append(rec)

# 时间基准校正(两类gt话题用同一个偏移量估计，因为都来自bag录制)
if gt_boat_records and odom_records:
    _offset = gt_boat_records[0]['stamp'] - odom_records[0]['stamp']
    for r in gt_boat_records: r['stamp'] -= _offset
    for r in gt_pillar_records: r['stamp'] -= _offset
    print(f"[时间基准校正] 偏移量: {_offset:.4f}s")

gt_boat_records.sort(key=lambda r: r['stamp'])
gt_pillar_records.sort(key=lambda r: r['stamp'])
odom_records.sort(key=lambda r: r['stamp'])
gt_boat_stamps = [r['stamp'] for r in gt_boat_records]
gt_pillar_stamps = [r['stamp'] for r in gt_pillar_records]
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

total_det = 0
matched_boat = 0
matched_pillar = 0
matched_neither = 0

for det in det_records:
    t = det['stamp']
    odom, odom_gap = find_nearest(odom_stamps, odom_records, t)
    gtb, gtb_gap = find_nearest(gt_boat_stamps, gt_boat_records, t)
    gtp, gtp_gap = find_nearest(gt_pillar_stamps, gt_pillar_records, t)
    if odom is None or gtb is None or gtp is None: continue
    if odom_gap > MAX_TIME_GAP or gtb_gap > MAX_TIME_GAP or gtp_gap > MAX_TIME_GAP: continue

    yaw = quat_to_yaw(odom['qx'], odom['qy'], odom['qz'], odom['qw'])
    gtb_boat_frame = [world_to_boat(p['x'], p['y'], odom['x'], odom['y'], yaw) for p in gtb['poses']]
    gtp_boat_frame = [world_to_boat(p['x'], p['y'], odom['x'], odom['y'], yaw) for p in gtp['poses']]

    for p in det['poses']:
        dx, dy = p['x'], p['y']
        total_det += 1
        d_boat = min((math.hypot(dx-gx,dy-gy) for gx,gy in gtb_boat_frame), default=1e9)
        d_pillar = min((math.hypot(dx-gx,dy-gy) for gx,gy in gtp_boat_frame), default=1e9)
        if d_boat < MATCH_TOL:
            matched_boat += 1
        elif d_pillar < MATCH_TOL:
            matched_pillar += 1
        else:
            matched_neither += 1

print(f"\n检测候选总数(累加所有帧): {total_det}")
print(f"匹配到真实boat: {matched_boat} ({matched_boat/total_det*100:.1f}%)")
print(f"匹配到真实pillar(疑似误分类): {matched_pillar} ({matched_pillar/total_det*100:.1f}%)")
print(f"两者都不匹配(真正的误检/未知来源): {matched_neither} ({matched_neither/total_det*100:.1f}%)")
