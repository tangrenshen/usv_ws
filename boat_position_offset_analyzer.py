#!/usr/bin/env python3
"""
分析boat检测候选和最近真值船的距离分布——排查系统性位置偏移
核心方法：
1. 对所有boat检测候选，找它最近的真值船
2. 统计距离分布——如果峰值不在0而在2-4m，说明有系统性位置偏移
3. 同时分析近距和远距的分布差异
"""
import json
import math
import bisect

BOAT_LOG_PATH = '/home/lyf040817/usv_ws/boat_log.jsonl'
MATCH_TOL = 2.0
MAX_TIME_GAP = 0.5

gt_records = []
odom_records = []
det_records = []

with open(BOAT_LOG_PATH) as f:
    for line in f:
        rec = json.loads(line)
        if rec['type'] == 'gt':
            gt_records.append(rec)
        elif rec['type'] == 'odom':
            odom_records.append(rec)
        elif rec['type'] == 'det':
            det_records.append(rec)

print(f"boat_log: gt={len(gt_records)} odom={len(odom_records)} det={len(det_records)}")

if gt_records and odom_records:
    _offset = gt_records[0]['stamp'] - odom_records[0]['stamp']
    for r in gt_records:
        r['stamp'] -= _offset
    print(f"[时间基准校正] gt时间戳偏移量估计: {_offset:.4f}s")

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

def world_to_boat(wx, wy, bx, by, yaw):
    dx, dy = wx - bx, wy - by
    rx = dx*math.cos(-yaw) - dy*math.sin(-yaw)
    ry = dx*math.sin(-yaw) + dy*math.cos(-yaw)
    return rx, ry

def quat_to_yaw(qx, qy, qz, qw):
    return math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))

print(f"\n=== 任务3: boat候选↔最近真值距离分布分析 ===")

all_distances = []
near_distances = []
far_distances = []
fp_distances = []
hit_distances = []

for det in det_records:
    t = det['stamp']
    odom, odom_gap = find_nearest(odom_stamps, odom_records, t)
    gt, gt_gap = find_nearest(gt_stamps, gt_records, t)
    
    if odom is None or gt is None:
        continue
    if odom_gap > MAX_TIME_GAP or gt_gap > MAX_TIME_GAP:
        continue
    
    yaw = quat_to_yaw(odom['qx'], odom['qy'], odom['qz'], odom['qw'])
    gt_boat_frame = [world_to_boat(p['x'], p['y'], odom['x'], odom['y'], yaw) for p in gt['poses']]
    det_pts = [(p['x'], p['y']) for p in det['poses']]
    
    matched_gt = set()
    for dx, dy in det_pts:
        best_d = 1e9
        best_i = -1
        for i, (gx, gy) in enumerate(gt_boat_frame):
            d = math.hypot(dx-gx, dy-gy)
            if d < best_d:
                best_d = d
                best_i = i
        
        all_distances.append(best_d)
        
        det_dist = math.hypot(dx, dy)
        if det_dist < 20:
            near_distances.append(best_d)
        else:
            far_distances.append(best_d)
        
        if best_d < MATCH_TOL:
            hit_distances.append(best_d)
            matched_gt.add(best_i)
        else:
            fp_distances.append(best_d)

print(f"\n所有boat候选的距离分布:")
print(f"  总候选数: {len(all_distances)}")
print(f"  平均距离: {sum(all_distances)/len(all_distances):.2f}m")
print(f"  最小距离: {min(all_distances):.2f}m")
print(f"  最大距离: {max(all_distances):.2f}m")

dist_bins = [(0, 1), (1, 2), (2, 3), (3, 4), (4, 5), (5, 6), (6, 10), (10, 20), (20, 50), (50, 100)]
for name, distances in [('全部', all_distances), ('近距(0-20m)', near_distances), ('远距(20m+)', far_distances), ('命中', hit_distances), ('误检', fp_distances)]:
    if not distances:
        continue
    
    dist_counts = {rng: 0 for rng in dist_bins}
    for d in distances:
        for rng in dist_bins:
            if rng[0] <= d < rng[1]:
                dist_counts[rng] += 1
                break
    
    print(f"\n{name} ({len(distances)}个):")
    for rng in dist_bins:
        if dist_counts[rng] == 0:
            continue
        pct = dist_counts[rng] / len(distances) * 100
        print(f"  [{rng[0]:>3}, {rng[1]:>3}m): {dist_counts[rng]} ({pct:.1f}%)")
    
    print(f"  平均距离: {sum(distances)/len(distances):.2f}m")
    
    peak_bin = max(dist_bins, key=lambda r: dist_counts[r])
    print(f"  峰值区间: [{peak_bin[0]}, {peak_bin[1]}m)")

print(f"\n=== 系统性位置偏移诊断 ===")
if all_distances:
    avg_dist = sum(all_distances) / len(all_distances)
    print(f"\n1. 所有boat候选的平均距离: {avg_dist:.2f}m")
    print(f"   {'正常：峰值集中在0-1m附近' if avg_dist < 2 else '异常：平均距离较大，可能存在系统性偏移'}")
    
    hit_ratio = len(hit_distances) / len(all_distances) * 100 if all_distances else 0
    print(f"\n2. 命中比例(MATCH_TOL={MATCH_TOL}m): {hit_ratio:.1f}%")
    
    near_hit_ratio = len([d for d in near_distances if d < MATCH_TOL]) / len(near_distances) * 100 if near_distances else 0
    far_hit_ratio = len([d for d in far_distances if d < MATCH_TOL]) / len(far_distances) * 100 if far_distances else 0
    print(f"   近距命中比例: {near_hit_ratio:.1f}%")
    print(f"   远距命中比例: {far_hit_ratio:.1f}%")
    
    fp_in_near = len([d for d in fp_distances if d < 20])
    fp_in_far = len(fp_distances) - fp_in_near
    print(f"\n3. 误检分布:")
    print(f"   近距误检: {fp_in_near} ({fp_in_near/len(fp_distances)*100:.1f}%)")
    print(f"   远距误检: {fp_in_far} ({fp_in_far/len(fp_distances)*100:.1f}%)")
    
    print(f"\n4. 距离偏移假设检验:")
    if avg_dist > 2 and hit_ratio < 20:
        print(f"   ✓ 存在系统性位置偏移的迹象：平均距离>{MATCH_TOL}m且命中比例低")
        print(f"   ✓ 建议排查：标定残差、时间同步误差、点云融合变换")
    else:
        print(f"   ✓ 位置偏移不明显，问题可能在聚类或分类")

print(f"\n=== 与上一轮数据对照 ===")
print(f"上一轮第50帧: 8艘20-40m真值船，最近簇距离0-4.88m（平均~2.5m）")
print(f"本轮搜索半径5m: 30.7%漏检船附近有簇，10m内68.5%")
print(f"本轮搜索半径2m: 3.8%漏检船附近有簇")
print(f"\n结论：'96.2%无簇'是搜索半径2m太小导致的，实际10m内68.5%漏检船附近有簇")
print(f"但最近簇距离分布峰值在10-20m，暗示可能存在位置偏移")