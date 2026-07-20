#!/usr/bin/env python3
"""
扩展验证：det检测到的障碍物位置上到底是什么？
对比gt/gt_pillar/gt_buoy/gt_block，看det到底检测到了什么
"""
import json
import math
import bisect

LOG_PATH = '/home/lyf040817/usv_ws/boat_log.jsonl'

records_by_type = {'gt': [], 'gt_pillar': [], 'gt_buoy': [], 'gt_block': [], 'odom': [], 'det': []}

with open(LOG_PATH) as f:
    for line in f:
        rec = json.loads(line)
        if rec['type'] in records_by_type:
            records_by_type[rec['type']].append(rec)

# 时间基准校正（基于gt和odom）
gt_records = records_by_type['gt']
odom_records = records_by_type['odom']
det_records = records_by_type['det']

if gt_records and odom_records:
    _offset = gt_records[0]['stamp'] - odom_records[0]['stamp']
    for t in ['gt', 'gt_pillar', 'gt_buoy', 'gt_block']:
        for r in records_by_type[t]:
            r['stamp'] -= _offset
    print(f"[时间基准校正] 偏移量: {_offset:.6f}s")

for t in records_by_type:
    records_by_type[t].sort(key=lambda r: r['stamp'])

gt_stamps = [r['stamp'] for r in records_by_type['gt']]
odom_stamps = [r['stamp'] for r in records_by_type['odom']]

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

def boat_to_world(rx, ry, bx, by, yaw):
    """把船体系坐标反算回世界系"""
    wx = bx + rx*math.cos(yaw) - ry*sin(yaw)
    wy = by + rx*sin(yaw) + ry*cos(yaw)
    return wx, wy

# 用sin/cos的别名
sin = math.sin
cos = math.cos

# 找几个有det候选的时刻，检查det位置对应的世界系坐标上有什么
print("\n=== 检查det检测位置对应的世界系坐标上有什么障碍物 ===")
candidate_count = 0
for det in det_records:
    if not det['poses']:
        continue
    t = det['stamp']
    odom, odom_gap = find_nearest(odom_stamps, odom_records, t)
    if odom is None or odom_gap > 0.5:
        continue

    yaw = quat_to_yaw(odom['qx'], odom['qy'], odom['qz'], odom['qw'])
    bx, by = odom['x'], odom['y']

    # 对每个det候选，反算其世界系位置，并查找附近所有类型的gt
    for j, p in enumerate(det['poses']):
        rx, ry = p['x'], p['y']
        wx = bx + rx*cos(yaw) - ry*sin(yaw)
        wy = by + rx*sin(yaw) + ry*cos(yaw)
        det_world_dist = math.hypot(rx, ry)

        # 检查各类gt在det世界系位置附近的最近障碍物
        best_per_type = {}
        for gt_type in ['gt', 'gt_pillar', 'gt_buoy', 'gt_block']:
            gt_list, gap = find_nearest([r['stamp'] for r in records_by_type[gt_type]], records_by_type[gt_type], t)
            if gt_list is None or gap > 0.5:
                continue
            best_d = 1e9
            best_pose = None
            for gp in gt_list['poses']:
                d = math.hypot(gp['x']-wx, gp['y']-wy)
                if d < best_d:
                    best_d = d
                    best_pose = gp
            if best_pose is not None:
                best_per_type[gt_type] = (best_d, best_pose)

        if candidate_count < 5:  # 只详细打印前5个
            print(f"\n--- det时刻 t={t:.3f}s  det候选 #{j+1} ---")
            print(f"  det船体系: rx={rx:.3f}  ry={ry:.3f}  距本船={det_world_dist:.3f}m")
            print(f"  det世界系: wx={wx:.3f}  wy={wy:.3f}")
            print(f"  本船位姿: x={bx:.3f}  y={by:.3f}  yaw={math.degrees(yaw):.3f}°")
            print(f"  各类gt最近障碍物距det世界系位置:")
            for gt_type, (d, p) in sorted(best_per_type.items(), key=lambda x: x[1][0]):
                marker = '✓可能就是这个' if d < 2.0 else ''
                print(f"    {gt_type:12s}: 距离={d:.3f}m  位置=({p['x']:.3f}, {p['y']:.3f}) {marker}")
            candidate_count += 1

# 统计：所有det候选中，最近的gt类型分布
print("\n\n=== 统计：det候选最近的gt类型分布 ===")
type_counter = {'gt': 0, 'gt_pillar': 0, 'gt_buoy': 0, 'gt_block': 0, 'none': 0}
total_det_candidates = 0
nearest_dists = []

for det in det_records:
    if not det['poses']:
        continue
    t = det['stamp']
    odom, odom_gap = find_nearest(odom_stamps, odom_records, t)
    if odom is None or odom_gap > 0.5:
        continue
    yaw = quat_to_yaw(odom['qx'], odom['qy'], odom['qz'], odom['qw'])
    bx, by = odom['x'], odom['y']

    for p in det['poses']:
        total_det_candidates += 1
        rx, ry = p['x'], p['y']
        wx = bx + rx*cos(yaw) - ry*sin(yaw)
        wy = by + rx*sin(yaw) + ry*cos(yaw)

        # 找所有类型gt中最近的
        best_type = 'none'
        best_d = 1e9
        for gt_type in ['gt', 'gt_pillar', 'gt_buoy', 'gt_block']:
            gt_list, gap = find_nearest([r['stamp'] for r in records_by_type[gt_type]], records_by_type[gt_type], t)
            if gt_list is None or gap > 0.5:
                continue
            for gp in gt_list['poses']:
                d = math.hypot(gp['x']-wx, gp['y']-wy)
                if d < best_d:
                    best_d = d
                    best_type = gt_type
        type_counter[best_type] += 1
        nearest_dists.append((best_type, best_d))

print(f"det候选总数: {total_det_candidates}")
for t, c in sorted(type_counter.items(), key=lambda x: -x[1]):
    pct = c/total_det_candidates*100 if total_det_candidates > 0 else 0
    print(f"  {t:12s}: {c:4d} ({pct:.1f}%)")

# det到各类gt的距离统计
print("\n=== det候选到最近gt的距离分布 ===")
for gt_type in ['gt', 'gt_pillar', 'gt_buoy', 'gt_block']:
    ds = [d for t, d in nearest_dists if t == gt_type]
    if ds:
        print(f"  {gt_type:12s}: 数量={len(ds):4d}  距离均值={sum(ds)/len(ds):.2f}m  最小={min(ds):.2f}m  最大={max(ds):.2f}m")
