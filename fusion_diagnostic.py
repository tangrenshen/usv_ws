#!/usr/bin/env python3
"""
融合精度评估诊断脚本
==================
针对 fusion_precision_eval.py 结果的三项验证:
1. 0-20m 原始计数（真实检测率 vs 平均检测率）
2. 0-20m 立柱数量和每立柱样本分布
3. 匹配簇的label分布（检测错配饱和问题）
"""

import json
import math
import re
import bisect
from collections import defaultdict

BOAT_LOG_PATH = '/home/lyf040817/usv_ws/boat_log.jsonl'
CLUSTER_LOG_PATH = '/home/lyf040817/usv_ws/perception_log.log'
PRIMARY_MATCH_RADIUS = 2.0
DISTANCE_BINS = [(0, 20, '0-20m'), (20, 40, '20-40m'), (40, float('inf'), '40m+')]
MAX_GT_DIST = 80.0

print("=" * 80)
print("加载数据...")
print("=" * 80)

gt_ship_records = []
odom_records = []
gt_pillar_records = []
gt_buoy_records = []
gt_block_records = []

with open(BOAT_LOG_PATH) as f:
    for line in f:
        rec = json.loads(line)
        t = rec['type']
        if t == 'gt':
            gt_ship_records.append(rec)
        elif t == 'odom':
            odom_records.append(rec)
        elif t == 'gt_pillar':
            gt_pillar_records.append(rec)
        elif t == 'gt_buoy':
            gt_buoy_records.append(rec)
        elif t == 'gt_block':
            gt_block_records.append(rec)

print(f"  odom: {len(odom_records)} 条")
print(f"  gt_pillar: {len(gt_pillar_records)} 条")
print(f"  gt_buoy: {len(gt_buoy_records)} 条")
print(f"  gt_block: {len(gt_block_records)} 条")

# 取pillar位置
gt_pillar_positions = []
for rec in gt_pillar_records:
    if rec['poses']:
        gt_pillar_positions = [(p['x'], p['y'], p['z']) for p in rec['poses']]
        break
print(f"  pillar位置数: {len(gt_pillar_positions)}")

# 时间对齐
_epoch_offset = gt_ship_records[0]['stamp'] - odom_records[0]['stamp']
for rec in gt_pillar_records:
    rec['stamp'] -= _epoch_offset
for rec in gt_buoy_records:
    rec['stamp'] -= _epoch_offset
for rec in gt_block_records:
    rec['stamp'] -= _epoch_offset

odom_records.sort(key=lambda r: r['stamp'])
odom_stamps = [r['stamp'] for r in odom_records]

def interpolate_odom(t):
    if not odom_records:
        return None
    if t <= odom_stamps[0]:
        return odom_records[0]
    if t >= odom_stamps[-1]:
        return odom_records[-1]
    idx = bisect.bisect_left(odom_stamps, t)
    r0, r1 = odom_records[idx-1], odom_records[idx]
    t0, t1 = r0['stamp'], r1['stamp']
    if abs(t1 - t0) < 1e-9:
        return r0
    alpha = (t - t0) / (t1 - t0)
    def lerp(a, b):
        return a + (b - a) * alpha
    return {
        'x': lerp(r0['x'], r1['x']), 'y': lerp(r0['y'], r1['y']), 'z': lerp(r0['z'], r1['z']),
        'qx': lerp(r0['qx'], r1['qx']), 'qy': lerp(r0['qy'], r1['qy']),
        'qz': lerp(r0['qz'], r1['qz']), 'qw': lerp(r0['qw'], r1['qw']),
    }

def quat_to_yaw(qx, qy, qz, qw):
    return math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))

def world_to_boat(wx, wy, wz, bx, by, bz, yaw):
    dx, dy = wx - bx, wy - by
    rx = dx*math.cos(-yaw) - dy*math.sin(-yaw)
    ry = dx*math.sin(-yaw) + dy*math.cos(-yaw)
    return rx, ry, wz - bz

# 解析cluster日志
NUM = r'[-+]?[\d.]+(?:[eE][-+]?\d+)?'
pending_pattern = re.compile(
    r'\[cluster_diagnostic\] t=(' + NUM + r') center=\((' + NUM + r'),(' + NUM + r'),(' + NUM + r')\)'
    r' size=\((' + NUM + r'),(' + NUM + r'),(' + NUM + r')\)'
    r' fp_max=(' + NUM + r') fp_min=(' + NUM + r') square=(' + NUM + r') pts=(\d+) -> classification pending'
)
assigned_full_pattern = re.compile(
    r'\[cluster_diagnostic\] t=(' + NUM + r') center=\((' + NUM + r'),(' + NUM + r'),(' + NUM + r')\)'
    r' fp_max=(' + NUM + r') fp_min=(' + NUM + r') dz=(' + NUM + r') square=(' + NUM + r') pts=(\d+) -> assigned=(\w+)'
)
assigned_label_pattern = re.compile(r'assigned=(\w+)')

all_clusters = []
with open(CLUSTER_LOG_PATH, encoding='utf-8', errors='replace') as f:
    current_pending = None
    for line in f:
        line = line.strip()
        pending_match = pending_pattern.match(line)
        if pending_match:
            if current_pending:
                all_clusters.append(current_pending)
            try:
                t = float(pending_match.group(1))
                current_pending = {
                    't': t, 'x': float(pending_match.group(2)), 'y': float(pending_match.group(3)),
                    'z': float(pending_match.group(4)), 'label': None,
                }
            except (ValueError, IndexError):
                current_pending = None
            continue
        assigned_full_match = assigned_full_pattern.match(line)
        if assigned_full_match:
            try:
                t = float(assigned_full_match.group(1))
                cluster = {
                    't': t, 'x': float(assigned_full_match.group(2)), 'y': float(assigned_full_match.group(3)),
                    'z': float(assigned_full_match.group(4)), 'label': assigned_full_match.group(10),
                }
                all_clusters.append(cluster)
            except (ValueError, IndexError):
                pass
            current_pending = None
            continue
        assigned_label_match = assigned_label_pattern.search(line)
        if assigned_label_match and current_pending:
            current_pending['label'] = assigned_label_match.group(1)
            all_clusters.append(current_pending)
            current_pending = None
if current_pending:
    all_clusters.append(current_pending)

# 时间校正
_cluster_sample = all_clusters[len(all_clusters)//2]['t']
_odom_mid = odom_stamps[len(odom_stamps)//2]
if abs(_cluster_sample - _odom_mid) > 1000:
    for c in all_clusters:
        c['t'] -= _epoch_offset

# 过滤
_expected_lo = odom_stamps[0] - 200
_expected_hi = odom_stamps[-1] + 200
_filtered_clusters = [c for c in all_clusters if _expected_lo <= c['t'] <= _expected_hi]

clusters_by_time = defaultdict(list)
for c in _filtered_clusters:
    clusters_by_time[c['t']].append(c)
cluster_timestamps = sorted(clusters_by_time.keys())

print(f"  有效簇数: {len(_filtered_clusters)}")
print(f"  有簇的帧数: {len(cluster_timestamps)}")
print(f"  帧时间范围: [{cluster_timestamps[0]:.3f}, {cluster_timestamps[-1]:.3f}]s")

# 构建GT静态物体
all_gt_static = []
for i, pos in enumerate(gt_pillar_positions):
    all_gt_static.append({'id': i, 'type': 'pillar', 'world_pos': pos})

print(f"  合计GT静态物体(pillar): {len(all_gt_static)}")

# ============================================================
# 诊断1: 原始计数和真实检测率
# ============================================================
print("\n" + "=" * 80)
print("【诊断1】原始计数和真实检测率")
print("=" * 80)

for radius in [1.0, 2.0]:
    print(f"\n--- 匹配半径 = {radius}m ---")
    
    gt_total_counts = defaultdict(lambda: defaultdict(int))
    gt_detection_counts = defaultdict(lambda: defaultdict(int))
    match_labels = defaultdict(lambda: defaultdict(lambda: defaultdict(int)))  # bin -> gt_id -> label -> count
    match_errors = defaultdict(list)  # bin -> list of error
    per_gt_frame_counts = defaultdict(lambda: defaultdict(int))  # gt_id -> bin -> frame_count
    per_gt_match_counts = defaultdict(lambda: defaultdict(int))  # gt_id -> bin -> match_count
    
    for t in cluster_timestamps:
        frame_clusters = clusters_by_time[t]
        odom = interpolate_odom(t)
        if odom is None:
            continue
        bx, by, bz = odom['x'], odom['y'], odom['z']
        yaw = quat_to_yaw(odom['qx'], odom['qy'], odom['qz'], odom['qw'])
        
        for gt_obj in all_gt_static:
            gx, gy, gz = gt_obj['world_pos']
            rx, ry, rz = world_to_boat(gx, gy, gz, bx, by, bz, yaw)
            dist_from_boat = math.hypot(rx, ry)
            
            if dist_from_boat > MAX_GT_DIST:
                continue
            
            if dist_from_boat < 20:
                bin_key = '0-20m'
            elif dist_from_boat < 40:
                bin_key = '20-40m'
            else:
                bin_key = '40m+'
            
            gt_total_counts[bin_key][gt_obj['id']] += 1
            per_gt_frame_counts[gt_obj['id']][bin_key] += 1
            
            # 找最近簇
            best_dist = float('inf')
            best_cluster = None
            for cluster in frame_clusters:
                dist = math.hypot(cluster['x'] - rx, cluster['y'] - ry)
                if dist < best_dist:
                    best_dist = dist
                    best_cluster = cluster
            
            if best_cluster is None or best_dist > radius:
                continue
            
            gt_detection_counts[bin_key][gt_obj['id']] += 1
            per_gt_match_counts[gt_obj['id']][bin_key] += 1
            match_labels[bin_key][gt_obj['id']][best_cluster.get('label', 'unknown')] += 1
            match_errors[bin_key].append(best_dist)
    
    print(f"\n  {'距离段':<10} {'GT物体数':<10} {'总机会数':<10} {'总匹配数':<10} {'真实检测率':<12} {'平均检测率':<12}")
    print(f"  {'-'*64}")
    
    for min_d, max_d, name in DISTANCE_BINS:
        total_gt_objects = len(gt_total_counts.get(name, {}))
        total_opportunities = sum(gt_total_counts.get(name, {}).values())
        total_matches = sum(gt_detection_counts.get(name, {}).values())
        real_det_rate = total_matches / max(total_opportunities, 1)
        
        # 平均检测率（每个GT物体的检测率平均）
        det_rates = []
        for gid, frames in gt_total_counts.get(name, {}).items():
            matched = gt_detection_counts.get(name, {}).get(gid, 0)
            det_rates.append(matched / max(frames, 1))
        avg_det_rate = sum(det_rates) / max(len(det_rates), 1)
        
        print(f"  {name:<10} {total_gt_objects:<10} {total_opportunities:<10} {total_matches:<10} {real_det_rate:<12.1%} {avg_det_rate:<12.1%}")
    
    print(f"\n  注意: '真实检测率' = 总匹配数/总机会数, '平均检测率' = 每物体检测率的平均")
    print(f"        如果两者差异巨大, 说明少数物体贡献了大部分匹配")
    
    # ============================================================
    # 诊断2: 每GT物体的样本分布（特别是0-20m）
    # ============================================================
    
    print(f"\n  --- 每GT物体样本明细 ({name}段) ---")
    for min_d, max_d, name in DISTANCE_BINS:
        if name != '0-20m':
            continue
        print(f"\n  [{name}] 每GT物体的帧数和匹配数:")
        print(f"  {'GT_ID':<8} {'帧数':<8} {'匹配数':<8} {'检测率':<10} {'说明'}")
        print(f"  {'-'*56}")
        
        for gid in sorted(per_gt_frame_counts.keys(), key=lambda x: x):
            frames = per_gt_frame_counts[gid].get(name, 0)
            matches = per_gt_match_counts[gid].get(name, 0)
            if frames > 0:
                det_rate = matches / frames
                if frames < 10:
                    note = "!! 帧数极少"
                elif det_rate >= 0.95:
                    note = "✅ 几乎全匹配"
                elif det_rate < 0.3:
                    note = "⚠️ 大部分未匹配"
                else:
                    note = ""
                print(f"  {gid:<8} {frames:<8} {matches:<8} {det_rate:<10.1%} {note}")
    
    # ============================================================
    # 诊断3: 匹配簇的label分布
    # ============================================================
    
    print(f"\n  --- 匹配簇的label分布 ---")
    print(f"  {'距离段':<10} {'总匹配':<8} {'pillar':<10} {'buoy':<10} {'block':<10} {'boat':<10} {'discarded':<12} {'其他'}")
    print(f"  {'-'*80}")
    
    for min_d, max_d, name in DISTANCE_BINS:
        label_totals = defaultdict(int)
        total = 0
        for gid, label_dict in match_labels.get(name, {}).items():
            for lbl, cnt in label_dict.items():
                label_totals[lbl] += cnt
                total += cnt
        
        pillar_cnt = label_totals.get('pillar', 0)
        buoy_cnt = label_totals.get('buoy', 0)
        block_cnt = label_totals.get('block', 0)
        boat_cnt = label_totals.get('boat', 0)
        discarded_cnt = label_totals.get('discarded_shape_implausible', 0) + label_totals.get('discarded_low_confidence', 0)
        other_cnt = total - pillar_cnt - buoy_cnt - block_cnt - boat_cnt - discarded_cnt
        
        # 其他包括什么
        other_labels = {k: v for k, v in label_totals.items() 
                       if k not in ('pillar', 'buoy', 'block', 'boat', 
                                   'discarded_shape_implausible', 'discarded_low_confidence', 'boat_fallback', 'buoy_fallback')}
        other_detail = ', '.join(f'{k}={v}' for k, v in sorted(other_labels.items()))
        
        print(f"  {name:<10} {total:<8} {pillar_cnt:<10} {buoy_cnt:<10} {block_cnt:<10} {boat_cnt:<10} {discarded_cnt:<12} {other_cnt}")
        if other_detail:
            print(f"    其他明细: {other_detail}")
    
    print(f"\n  注意: 如果非pillar label的匹配占比很高, 说明存在错配饱和问题!")
    print(f"        GT是pillar, 但匹配到的簇不是pillar → 测的不是融合精度")

# ============================================================
# 额外诊断: 只匹配pillar-label簇的"纯匹配"结果
# ============================================================
print("\n" + "=" * 80)
print("【额外诊断】限制只匹配 pillar-label 簇（排除错配）")
print("=" * 80)

radius = 2.0
strict_total = 0
strict_match = 0
strict_errors = defaultdict(list)
strict_gt_total = defaultdict(lambda: defaultdict(int))
strict_gt_match = defaultdict(lambda: defaultdict(int))

for t in cluster_timestamps:
    frame_clusters = clusters_by_time[t]
    # 只保留pillar-label的簇
    pillar_clusters = [c for c in frame_clusters if c.get('label') == 'pillar']
    
    if not pillar_clusters:
        continue
    
    odom = interpolate_odom(t)
    if odom is None:
        continue
    bx, by, bz = odom['x'], odom['y'], odom['z']
    yaw = quat_to_yaw(odom['qx'], odom['qy'], odom['qz'], odom['qw'])
    
    for gt_obj in all_gt_static:
        gx, gy, gz = gt_obj['world_pos']
        rx, ry, rz = world_to_boat(gx, gy, gz, bx, by, bz, yaw)
        dist_from_boat = math.hypot(rx, ry)
        
        if dist_from_boat > MAX_GT_DIST:
            continue
        
        if dist_from_boat < 20:
            bin_key = '0-20m'
        elif dist_from_boat < 40:
            bin_key = '20-40m'
        else:
            bin_key = '40m+'
        
        strict_gt_total[bin_key][gt_obj['id']] += 1
        
        # 只在pillar簇中找最近邻
        best_dist = float('inf')
        for cluster in pillar_clusters:
            dist = math.hypot(cluster['x'] - rx, cluster['y'] - ry)
            if dist < best_dist:
                best_dist = dist
        
        if best_dist <= radius:
            strict_errors[bin_key].append(best_dist)
            strict_gt_match[bin_key][gt_obj['id']] += 1

print(f"\n  --- 只匹配pillar-label簇（radius={radius}m） ---")
print(f"  {'距离段':<10} {'GT数':<8} {'总机会':<10} {'总匹配':<10} {'真实检测率':<12} {'误差中位数':<12} {'误差均值':<12}")
print(f"  {'-'*78}")

for min_d, max_d, name in DISTANCE_BINS:
    total_obj = len(strict_gt_total.get(name, {}))
    total_opp = sum(strict_gt_total.get(name, {}).values())
    total_match = sum(strict_gt_match.get(name, {}).values())
    det_rate = total_match / max(total_opp, 1)
    
    errs = strict_errors.get(name, [])
    if errs:
        errs.sort()
        n = len(errs)
        median = errs[n // 2]
        mean = sum(errs) / n
        print(f"  {name:<10} {total_obj:<8} {total_opp:<10} {total_match:<10} {det_rate:<12.1%} {median:<12.3f} {mean:<12.3f}")
    else:
        print(f"  {name:<10} {total_obj:<8} {total_opp:<10} {total_match:<10} {det_rate:<12.1%} {'N/A':<12} {'N/A':<12}")

print(f"\n  对比: 原始方法(不限制label) vs 纯pillar匹配")
print(f"  如果两者差异巨大, 说明原始结果被错配饱和污染了!")

# ============================================================
# 最终汇总
# ============================================================
print("\n" + "=" * 80)
print("【诊断结论】")
print("=" * 80)
print(f"""
  请检查上述数据:
  
  1. 真实检测率 vs 平均检测率是否一致?
     - 如果差异大 → 少数物体贡献假象
  
  2. 0-20m 有几个GT物体? 每个贡献多少帧?
     - 如果GT物体数很少 → 样本代表性差
  
  3. 匹配簇的label分布:
     - 如果非pillar匹配很多 → 错配饱和
  
  4. 对比"只匹配pillar"结果和原始结果:
     - 如果误差差异大 → 原始结果不可信
""")