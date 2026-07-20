#!/usr/bin/env python3
"""
五分钟转换链路验证：挑一个gt有船、det也有候选的时刻，把完整转换链路打出来。
1. gt世界系坐标 (gx_w, gy_w)
2. 该时刻odom的 (x, y, yaw)，四元数原始值
3. world_to_boat转换后的gt船体系坐标
4. 同时刻所有det候选的船体系坐标
5. 两者的最小距离
"""
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

print(f"总记录数: gt={len(gt_records)} odom={len(odom_records)} det={len(det_records)}")

# 时间基准校正
if gt_records and odom_records:
    _offset = gt_records[0]['stamp'] - odom_records[0]['stamp']
    for r in gt_records:
        r['stamp'] -= _offset
    print(f"[时间基准校正] 偏移量: {_offset:.6f}s")

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

def quat_to_yaw(qx, qy, qz, qw):
    return math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))

def world_to_boat(wx, wy, bx, by, yaw):
    dx, dy = wx - bx, wy - by
    rx = dx*math.cos(-yaw) - dy*math.sin(-yaw)
    ry = dx*math.sin(-yaw) + dy*math.cos(-yaw)
    return rx, ry

def quat_norm(qx, qy, qz, qw):
    return math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)

# 找一个det有候选、同时刻gt也有船的时刻
print("\n=== 找一个det有候选、gt也有船的时刻 ===")
candidate_moments = []
for det in det_records:
    if not det['poses']:
        continue
    t = det['stamp']
    gt, gt_gap = find_nearest(gt_stamps, gt_records, t)
    odom, odom_gap = find_nearest(odom_stamps, odom_records, t)
    if gt is None or odom is None:
        continue
    if gt_gap > 0.5 or odom_gap > 0.5:
        continue
    if not gt['poses']:
        continue
    candidate_moments.append((det, gt, odom, t))

print(f"找到 {len(candidate_moments)} 个候选时刻")

# 挑前3个详细打印
for idx, (det, gt, odom, t) in enumerate(candidate_moments[:3]):
    print(f"\n{'='*70}")
    print(f"时刻 #{idx+1}  t={t:.3f}s")
    print(f"  det.stamp={det['stamp']:.3f}  gt.stamp={gt['stamp']:.3f}  odom.stamp={odom['stamp']:.3f}")
    print(f"  时间gap: det-gt={abs(det['stamp']-gt['stamp']):.3f}s  det-odom={abs(det['stamp']-odom['stamp']):.3f}s")

    # 第1步：gt世界系坐标
    print(f"\n--- [第1步] gt世界系坐标 ---")
    for i, p in enumerate(gt['poses']):
        print(f"  船{i+1}: gx_w={p['x']:.3f}  gy_w={p['y']:.3f}")

    # 第2步：odom位姿
    print(f"\n--- [第2步] odom位姿 ---")
    print(f"  本船位置: x={odom['x']:.3f}  y={odom['y']:.3f}  z={odom['z']:.3f}")
    print(f"  四元数原始值: qx={odom['qx']:.6f}  qy={odom['qy']:.6f}  qz={odom['qz']:.6f}  qw={odom['qw']:.6f}")
    qn = quat_norm(odom['qx'], odom['qy'], odom['qz'], odom['qw'])
    print(f"  四元数模长: {qn:.6f}  {'✓单位四元数' if abs(qn-1.0)<0.001 else '✗非单位四元数!'}")
    yaw = quat_to_yaw(odom['qx'], odom['qy'], odom['qz'], odom['qw'])
    yaw_deg = math.degrees(yaw)
    print(f"  yaw(弧度)={yaw:.6f}  yaw(度)={yaw_deg:.3f}")

    # 第3步：world_to_boat转换后的gt船体系坐标
    print(f"\n--- [第3步] world_to_boat转换后gt船体系坐标 ---")
    gt_boat = []
    for i, p in enumerate(gt['poses']):
        rx, ry = world_to_boat(p['x'], p['y'], odom['x'], odom['y'], yaw)
        gt_boat.append((rx, ry))
        dist = math.hypot(rx, ry)
        print(f"  船{i+1}: 船体系 rx={rx:.3f}  ry={ry:.3f}  距离本船={dist:.3f}m")

    # 第4步：det候选船体系坐标
    print(f"\n--- [第4步] det候选船体系坐标 ---")
    for i, p in enumerate(det['poses']):
        print(f"  检测{i+1}: dx={p['x']:.3f}  dy={p['y']:.3f}  距离本船={math.hypot(p['x'],p['y']):.3f}m")

    # 第5步：最小距离
    print(f"\n--- [第5步] det与gt_boat最小距离 ---")
    for j, p in enumerate(det['poses']):
        best_d = 1e9
        best_i = -1
        for i, (gx, gy) in enumerate(gt_boat):
            d = math.hypot(p['x']-gx, p['y']-gy)
            if d < best_d:
                best_d = d
                best_i = i
        print(f"  det{j+1}({p['x']:.2f},{p['y']:.2f}) <-> gt船{best_i+1}({gt_boat[best_i][0]:.2f},{gt_boat[best_i][1]:.2f}) 距离={best_d:.3f}m  {'✓命中' if best_d<2.0 else '✗超容差'}")

    # 额外检查：gt到本船的世界系距离
    print(f"\n--- [额外] gt船只到本船的世界系距离 ---")
    for i, p in enumerate(gt['poses']):
        wdist = math.hypot(p['x']-odom['x'], p['y']-odom['y'])
        print(f"  船{i+1}: 世界系距离={wdist:.3f}m  (对应船体系距离={math.hypot(*gt_boat[i]):.3f}m)")

print(f"\n{'='*70}")
print("验证完成")
