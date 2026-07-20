#!/usr/bin/env python3
import json
import math
import bisect

LOG_PATH = '/home/lyf040817/usv_ws/boat_log.jsonl'

gt_records = []
odom_records = []
det_records = []

with open(LOG_PATH) as f:
    for line in f:
        rec = json.loads(line)
        if rec['type'] == 'gt':
            gt_records.append(rec)
        elif rec['type'] == 'odom':
            odom_records.append(rec)
        elif rec['type'] == 'det':
            det_records.append(rec)

if gt_records and odom_records:
    _offset = gt_records[0]['stamp'] - odom_records[0]['stamp']
    for r in gt_records:
        r['stamp'] -= _offset
    print(f"[时间基准校正] gt时间戳偏移量: {_offset:.4f}s")

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

sample_det = det_records[10]
t = sample_det['stamp']
odom, odom_gap = find_nearest(odom_stamps, odom_records, t)
gt, gt_gap = find_nearest(gt_stamps, gt_records, t)

print(f"\n=== 采样点分析 (t={t:.3f}s) ===")
print(f"odom_gap: {odom_gap:.3f}s, gt_gap: {gt_gap:.3f}s")
print(f"\nodom (船在世界坐标系中的位姿):")
print(f"  x={odom['x']:.2f}, y={odom['y']:.2f}")
print(f"  qx={odom['qx']:.6e}, qy={odom['qy']:.6e}, qz={odom['qz']:.6e}, qw={odom['qw']:.6e}")
yaw = quat_to_yaw(odom['qx'], odom['qy'], odom['qz'], odom['qw'])
print(f"  yaw={yaw:.4f} rad ({yaw*180/math.pi:.1f} deg)")

print(f"\ngt (世界坐标系下的船只位置):")
for i, p in enumerate(gt['poses'][:5]):
    rx, ry = world_to_boat(p['x'], p['y'], odom['x'], odom['y'], yaw)
    print(f"  [{i}] world=({p['x']:.2f}, {p['y']:.2f}) -> boat=({rx:.2f}, {ry:.2f})")

print(f"\ndet (船体系下的检测位置):")
for i, p in enumerate(sample_det['poses'][:5]):
    print(f"  [{i}] boat=({p['x']:.2f}, {p['y']:.2f})")

print(f"\n=== 距离矩阵 (前5个gt vs 前5个det) ===")
gt_boat_frame = [world_to_boat(p['x'], p['y'], odom['x'], odom['y'], yaw) for p in gt['poses'][:5]]
det_pts = [(p['x'], p['y']) for p in sample_det['poses'][:5]]
for i, (gx, gy) in enumerate(gt_boat_frame):
    row = []
    for j, (dx, dy) in enumerate(det_pts):
        d = math.hypot(dx-gx, dy-gy)
        row.append(f"{d:.1f}")
    print(f"  gt[{i}]({gx:.1f},{gy:.1f}): {' '.join(row)}")

print(f"\n=== 所有gt转换到船体系后的统计 ===")
all_gt_rx = []
all_gt_ry = []
for g in gt_boat_frame:
    all_gt_rx.append(g[0])
    all_gt_ry.append(g[1])
print(f"  rx: min={min(all_gt_rx):.2f}, max={max(all_gt_rx):.2f}, mean={sum(all_gt_rx)/len(all_gt_rx):.2f}")
print(f"  ry: min={min(all_gt_ry):.2f}, max={max(all_gt_ry):.2f}, mean={sum(all_gt_ry)/len(all_gt_ry):.2f}")

print(f"\n=== 所有det的统计 ===")
all_det_x = [p['x'] for d in det_records[:10] for p in d['poses']]
all_det_y = [p['y'] for d in det_records[:10] for p in d['poses']]
print(f"  x: min={min(all_det_x):.2f}, max={max(all_det_x):.2f}, mean={sum(all_det_x)/len(all_det_x):.2f}")
print(f"  y: min={min(all_det_y):.2f}, max={max(all_det_y):.2f}, mean={sum(all_det_y)/len(all_det_y):.2f}")
