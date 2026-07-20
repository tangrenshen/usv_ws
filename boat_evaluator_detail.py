#!/usr/bin/env python3
"""
详细版离线裁判：跟boat_evaluator.py逻辑一致，但额外输出逐帧的真值/候选匹配明细，
用于诊断"误检"候选具体是什么、是否跟真值有系统性偏移。
"""
import json
import math
import bisect

LOG_PATH = '/tmp/boat_log.jsonl'
MATCH_TOL = 2.0
MAX_TIME_GAP = 0.5

gt_records, odom_records, det_records = [], [], []
with open(LOG_PATH) as f:
    for line in f:
        rec = json.loads(line)
        if rec['type'] == 'gt': gt_records.append(rec)
        elif rec['type'] == 'odom': odom_records.append(rec)
        elif rec['type'] == 'det': det_records.append(rec)

if gt_records and odom_records:
    _offset = gt_records[0]['stamp'] - odom_records[0]['stamp']
    for r in gt_records:
        r['stamp'] -= _offset
    print(f"[时间基准校正] gt偏移量: {_offset:.4f}s")

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

# 只详细打印前3个有效帧
printed = 0
for det in det_records:
    if printed >= 3: break
    t = det['stamp']
    odom, odom_gap = find_nearest(odom_stamps, odom_records, t)
    gt, gt_gap = find_nearest(gt_stamps, gt_records, t)
    if odom is None or gt is None or odom_gap > MAX_TIME_GAP or gt_gap > MAX_TIME_GAP:
        continue

    yaw = quat_to_yaw(odom['qx'], odom['qy'], odom['qz'], odom['qw'])
    gt_boat_frame = [world_to_boat(p['x'], p['y'], odom['x'], odom['y'], yaw) for p in gt['poses']]
    det_pts = [(p['x'], p['y']) for p in det['poses']]

    print(f"\n=== 帧 t={t:.2f} (odom_gap={odom_gap:.3f}s gt_gap={gt_gap:.3f}s) ===")
    print(f"真值船只数: {len(gt_boat_frame)}  检测候选数: {len(det_pts)}")
    print("真值位置(船体系):", [f"({x:.1f},{y:.1f})" for x,y in gt_boat_frame])

    for dx, dy in det_pts:
        best_d = min(math.hypot(dx-gx,dy-gy) for gx,gy in gt_boat_frame)
        status = "匹配" if best_d < MATCH_TOL else "误检"
        print(f"  候选({dx:.2f},{dy:.2f}) 最近真值距离={best_d:.2f}m [{status}]")
    printed += 1
