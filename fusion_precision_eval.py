#!/usr/bin/env python3
"""
融合精度评估脚本（修正版 v3）
==========================

评估两个替代指标，衡量融合点云的精度：

指标B（重点）: 融合点云 vs GT位置误差
  - 逐帧匹配 + 纯位置匹配（不看label）
  - 分距离段统计误差分布与检测率

指标A（辅助）: 检测位置的时间稳定性
  - 同一GT物体在多帧中被检测位置的散布
  - 不是跨雷达自洽性（cluster_diagnostic日志无雷达来源信息）

判读陷阱（务必阅读）：
  1. 匹配半径截断：报告的误差中位数永远<半径，且有偏
     → 判读时必须同时看【误差中位数】+【检测率】
     → 检测率低时误差小是假象（只匹配上了恰好准的那些）
  2. 距离段偏差：20m以外的检测率低是预期的（点云稀疏→聚不出簇）
     → 0-20m的数字最有意义
  3. spread偏差：只有匹配成功的帧才进入统计（误差<半径）
     → spread被系统性低估，仅做交叉诊断用
  4. GT位置假设：假设静态物体位置全程不变（本脚本会自动验证）
  5. 多档半径：样本数变化比误差变化更有信息量

v3 修正记录（2026-07-27）：
  - 新增GT位置稳定性自检（自动验证pillar/block是否真的没动）
  - 检测率放到输出最前面，与误差同等醒目
  - 新增"有效样本评估"：检测率低时自动标注结果仅供参考
  - 完善文档注释中的判读陷阱说明
"""

import json
import math
import re
import bisect
import sys
from collections import defaultdict

# ============================================================
# 配置
# ============================================================
BOAT_LOG_PATH = '/home/lyf040817/usv_ws/boat_log.jsonl'
CLUSTER_LOG_PATH = '/home/lyf040817/usv_ws/perception_log.log'

MATCH_RADII = [1.0, 2.0, 5.0]
PRIMARY_MATCH_RADIUS = 2.0

DISTANCE_BINS = [
    (0, 20, '0-20m'),
    (20, 40, '20-40m'),
    (40, float('inf'), '40m+'),
]

MAX_GT_DIST = 80.0
DETECTION_RATE_THRESHOLD = 0.3

# ============================================================
# 1. 加载数据
# ============================================================
print("=" * 80)
print("加载数据...")
print("=" * 80)

gt_ship_records = []
odom_records = []
det_records = []
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
        elif t == 'det':
            det_records.append(rec)
        elif t == 'gt_pillar':
            gt_pillar_records.append(rec)
        elif t == 'gt_buoy':
            gt_buoy_records.append(rec)
        elif t == 'gt_block':
            gt_block_records.append(rec)

print(f"  gt(船): {len(gt_ship_records)} 条")
print(f"  odom: {len(odom_records)} 条")
print(f"  det: {len(det_records)} 条")
print(f"  gt_pillar: {len(gt_pillar_records)} 条")
print(f"  gt_buoy: {len(gt_buoy_records)} 条")
print(f"  gt_block: {len(gt_block_records)} 条")

# ============================================================
# 2. GT位置稳定性自检
# ============================================================
print("\n" + "=" * 80)
print("GT位置稳定性自检（验证静态物体假设）...")
print("=" * 80)

def check_position_stability(records, label):
    first_poses = None
    last_poses = None
    first_stamp = 0
    last_stamp = 0

    for rec in records:
        if rec['poses']:
            first_poses = [(p['x'], p['y'], p['z']) for p in rec['poses']]
            first_stamp = rec['stamp']
            break

    for rec in reversed(records):
        if rec['poses']:
            last_poses = [(p['x'], p['y'], p['z']) for p in rec['poses']]
            last_stamp = rec['stamp']
            break

    if first_poses is None or last_poses is None:
        print(f"  {label}: 无有效poses数据")
        return None

    if len(first_poses) != len(last_poses):
        print(f"  {label}: WARNING 首尾帧数不一致 ({len(first_poses)} vs {len(last_poses)})")
        return None

    max_xy_shift = 0.0
    max_z_shift = 0.0
    for i, (fp, lp) in enumerate(zip(first_poses, last_poses)):
        xy_shift = math.hypot(fp[0] - lp[0], fp[1] - lp[1])
        z_shift = abs(fp[2] - lp[2])
        max_xy_shift = max(max_xy_shift, xy_shift)
        max_z_shift = max(max_z_shift, z_shift)

    time_span = last_stamp - first_stamp
    is_stable = max_xy_shift < 0.1

    if is_stable:
        print(f"  {label}: OK 位置稳定 (首末帧XY最大偏移={max_xy_shift:.3f}m, Z最大偏移={max_z_shift:.3f}m, 时间跨度={time_span:.1f}s)")
    else:
        print(f"  {label}: WARNING 位置可能变化! (首末帧XY最大偏移={max_xy_shift:.3f}m, Z最大偏移={max_z_shift:.3f}m)")
        print(f"    首帧时间={first_stamp:.3f}s, 末帧时间={last_stamp:.3f}s, 时间跨度={time_span:.1f}s")
        print(f"    WARNING 误差评估可能虚高，建议排除此类型或逐帧取真值")

    return is_stable

pillar_stable = check_position_stability(gt_pillar_records, "pillar")
buoy_stable = check_position_stability(gt_buoy_records, "buoy")
block_stable = check_position_stability(gt_block_records, "block")

# ============================================================
# 3. 时间基准对齐
# ============================================================
_epoch_offset = 0.0
if gt_ship_records and odom_records:
    _epoch_offset = gt_ship_records[0]['stamp'] - odom_records[0]['stamp']
    print(f"\n  epoch offset (gt[0] - odom[0]): {_epoch_offset:.6f}s")

def _align_to_odom(records, label):
    if not records or not odom_records:
        return
    sample = records[len(records) // 2]['stamp']
    odom_mid = odom_records[len(odom_records) // 2]['stamp']
    if abs(sample - odom_mid) > 1000:
        for r in records:
            r['stamp'] -= _epoch_offset
        print(f"  [时间基准校正] {label}: 已统一到odom基准")

_align_to_odom(gt_ship_records, "gt(船)")
_align_to_odom(gt_pillar_records, "gt_pillar")
_align_to_odom(gt_buoy_records, "gt_buoy")
_align_to_odom(gt_block_records, "gt_block")
_align_to_odom(det_records, "det")

gt_ship_records.sort(key=lambda r: r['stamp'])
odom_records.sort(key=lambda r: r['stamp'])
gt_pillar_records.sort(key=lambda r: r['stamp'])
gt_buoy_records.sort(key=lambda r: r['stamp'])
gt_block_records.sort(key=lambda r: r['stamp'])

odom_stamps = [r['stamp'] for r in odom_records]

# ============================================================
# 4. 工具函数
# ============================================================
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
        'stamp': t,
        'x': lerp(r0['x'], r1['x']),
        'y': lerp(r0['y'], r1['y']),
        'z': lerp(r0['z'], r1['z']),
        'qx': lerp(r0['qx'], r1['qx']),
        'qy': lerp(r0['qy'], r1['qy']),
        'qz': lerp(r0['qz'], r1['qz']),
        'qw': lerp(r0['qw'], r1['qw']),
    }

def quat_to_yaw(qx, qy, qz, qw):
    return math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))

def world_to_boat(wx, wy, wz, bx, by, bz, yaw):
    dx, dy = wx - bx, wy - by
    rx = dx*math.cos(-yaw) - dy*math.sin(-yaw)
    ry = dx*math.sin(-yaw) + dy*math.cos(-yaw)
    return rx, ry, wz - bz

def boat_to_world(bx, by, bz, wx_boat, wy_boat, wz_boat, yaw):
    rx = wx_boat*math.cos(yaw) - wy_boat*math.sin(yaw)
    ry = wx_boat*math.sin(yaw) + wy_boat*math.cos(yaw)
    return rx + bx, ry + by, wz_boat + bz

# ============================================================
# 5. 构建GT静态物体位置表
# ============================================================
print("\n" + "=" * 80)
print("构建GT静态物体位置表...")
print("=" * 80)

gt_types_to_use = ['pillar', 'buoy', 'block']
if pillar_stable == False:
    print("  WARNING 排除 pillar（位置不稳定）")
    gt_types_to_use = [t for t in gt_types_to_use if t != 'pillar']
if buoy_stable == False:
    print("  WARNING 排除 buoy（位置不稳定）")
    gt_types_to_use = [t for t in gt_types_to_use if t != 'buoy']
if block_stable == False:
    print("  WARNING 排除 block（位置不稳定）")
    gt_types_to_use = [t for t in gt_types_to_use if t != 'block']

gt_pillar_positions = []
gt_buoy_positions = []
gt_block_positions = []

if 'pillar' in gt_types_to_use:
    for rec in gt_pillar_records:
        if rec['poses']:
            gt_pillar_positions = [(p['x'], p['y'], p['z']) for p in rec['poses']]
            break
    print(f"  pillar位置数: {len(gt_pillar_positions)}")

if 'buoy' in gt_types_to_use:
    for rec in gt_buoy_records:
        if rec['poses']:
            gt_buoy_positions = [(p['x'], p['y'], p['z']) for p in rec['poses']]
            break
    print(f"  buoy位置数:   {len(gt_buoy_positions)}")

if 'block' in gt_types_to_use:
    for rec in gt_block_records:
        if rec['poses']:
            gt_block_positions = [(p['x'], p['y'], p['z']) for p in rec['poses']]
            break
    print(f"  block位置数:  {len(gt_block_positions)}")

all_gt_static = []
gt_id_counter = 0
for pos in gt_pillar_positions:
    all_gt_static.append({'id': gt_id_counter, 'type': 'pillar', 'world_pos': pos})
    gt_id_counter += 1
for pos in gt_buoy_positions:
    all_gt_static.append({'id': gt_id_counter, 'type': 'buoy', 'world_pos': pos})
    gt_id_counter += 1
for pos in gt_block_positions:
    all_gt_static.append({'id': gt_id_counter, 'type': 'block', 'world_pos': pos})
    gt_id_counter += 1

print(f"  合计GT静态物体: {len(all_gt_static)}")

# ============================================================
# 6. 解析cluster_diagnostic日志
# ============================================================
print("\n" + "=" * 80)
print("解析cluster_diagnostic日志...")
print("=" * 80)

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
                    't': t,
                    'x': float(pending_match.group(2)),
                    'y': float(pending_match.group(3)),
                    'z': float(pending_match.group(4)),
                    'dx': float(pending_match.group(5)),
                    'dy': float(pending_match.group(6)),
                    'dz': float(pending_match.group(7)),
                    'fp_max': float(pending_match.group(8)),
                    'fp_min': float(pending_match.group(9)),
                    'square': float(pending_match.group(10)),
                    'pts': int(pending_match.group(11)),
                    'label': None,
                }
            except (ValueError, IndexError):
                current_pending = None
            continue

        assigned_full_match = assigned_full_pattern.match(line)
        if assigned_full_match:
            try:
                t = float(assigned_full_match.group(1))
                cluster = {
                    't': t,
                    'x': float(assigned_full_match.group(2)),
                    'y': float(assigned_full_match.group(3)),
                    'z': float(assigned_full_match.group(4)),
                    'dx': None,
                    'dy': None,
                    'dz': float(assigned_full_match.group(7)),
                    'fp_max': float(assigned_full_match.group(5)),
                    'fp_min': float(assigned_full_match.group(6)),
                    'square': float(assigned_full_match.group(8)),
                    'pts': int(assigned_full_match.group(9)),
                    'label': assigned_full_match.group(10),
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

print(f"  解析到的候选簇总数: {len(all_clusters)}")

label_counts = defaultdict(int)
for c in all_clusters:
    label = c.get('label', 'unknown')
    label_counts[label] += 1
for lbl, cnt in sorted(label_counts.items(), key=lambda x: -x[1]):
    print(f"    {lbl}: {cnt}")

# ============================================================
# 7. cluster时间基准校正与离群值过滤
# ============================================================
if all_clusters:
    _cluster_sample = all_clusters[len(all_clusters)//2]['t']
    _odom_mid = odom_stamps[len(odom_stamps)//2] if odom_stamps else 0.0
    _cluster_needs_shift = abs(_cluster_sample - _odom_mid) > 1000

    if _cluster_needs_shift:
        for c in all_clusters:
            c['t'] -= _epoch_offset
        print(f"\n  [时间基准校正] cluster时间戳已偏移到odom基准")

    _expected_lo = (odom_stamps[0] - 200) if odom_stamps else 0
    _expected_hi = (odom_stamps[-1] + 200) if odom_stamps else float('inf')

    _filtered_clusters = []
    _dropped = 0
    for c in all_clusters:
        if not (_expected_lo <= c['t'] <= _expected_hi):
            _dropped += 1
            continue
        _filtered_clusters.append(c)

    if _dropped:
        print(f"  [数据质量] 丢弃 {_dropped} 条离群值")

    print(f"  有效簇数: {len(_filtered_clusters)}")
else:
    _filtered_clusters = []
    print("\n  警告: 没有有效的cluster数据")

# ============================================================
# 8. 构建逐帧数据结构
# ============================================================
print("\n" + "=" * 80)
print("构建逐帧数据结构...")
print("=" * 80)

clusters_by_time = defaultdict(list)
for c in _filtered_clusters:
    clusters_by_time[c['t']].append(c)

cluster_timestamps = sorted(clusters_by_time.keys())
print(f"  有簇的帧数: {len(cluster_timestamps)}")
if cluster_timestamps:
    print(f"  帧时间范围: [{cluster_timestamps[0]:.3f}, {cluster_timestamps[-1]:.3f}]s")
    print(f"  每帧平均簇数: {len(_filtered_clusters) / max(len(cluster_timestamps), 1):.1f}")

# ============================================================
# 9. 指标B: 逐帧匹配（多档半径）
# ============================================================
print("\n" + "=" * 80)
print("=== 指标B: 融合点云 vs GT位置误差（逐帧匹配） ===")
print("=" * 80)
print(f"""
  方法：逐帧匹配 + 纯位置匹配（不依赖分类label）
  匹配半径：{PRIMARY_MATCH_RADIUS}m（同时测试 {MATCH_RADII} 三档）
  判读注意：
    - 误差中位数永远<匹配半径，且有偏（大误差被截断）
    - 必须同时看【检测率】才能判断真实精度
    - 0-20m 档最可信（远距离检测率低是预期的）
""")

results_by_radius = {}

for radius in MATCH_RADII:
    errors_by_bin = defaultdict(list)
    gt_detection_counts = defaultdict(lambda: defaultdict(int))
    gt_total_counts = defaultdict(lambda: defaultdict(int))
    gt_observations = defaultdict(list)

    for t in cluster_timestamps:
        frame_clusters = clusters_by_time[t]
        if not frame_clusters:
            continue

        odom = interpolate_odom(t)
        if odom is None:
            continue

        bx, by, bz = odom['x'], odom['y'], odom['z']
        yaw = quat_to_yaw(odom['qx'], odom['qy'], odom['qz'], odom['qw'])

        gt_in_boat = []
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

            gt_in_boat.append({
                'gt_id': gt_obj['id'],
                'gt_type': gt_obj['type'],
                'boat_pos': (rx, ry, rz),
                'dist_from_boat': dist_from_boat,
                'bin': bin_key,
            })
            gt_total_counts[bin_key][gt_obj['id']] += 1

        for gt_item in gt_in_boat:
            best_dist = float('inf')
            best_cluster = None

            for cluster in frame_clusters:
                dist = math.hypot(cluster['x'] - gt_item['boat_pos'][0],
                                  cluster['y'] - gt_item['boat_pos'][1])
                if dist < best_dist:
                    best_dist = dist
                    best_cluster = cluster

            if best_cluster is None or best_dist > radius:
                continue

            error_2d = best_dist

            errors_by_bin[gt_item['bin']].append({
                'error': error_2d,
                'gt_dist_from_boat': gt_item['dist_from_boat'],
                'gt_type': gt_item['gt_type'],
                'gt_id': gt_item['gt_id'],
                'cluster_pts': best_cluster.get('pts', 0),
            })

            gt_detection_counts[gt_item['bin']][gt_item['gt_id']] += 1

            wx, wy, _ = boat_to_world(bx, by, bz,
                                      best_cluster['x'], best_cluster['y'],
                                      best_cluster.get('z', 0), yaw)
            gt_observations[gt_item['gt_id']].append({
                'cluster_world_pos': (wx, wy),
                'error': error_2d,
                'timestamp': t,
                'gt_type': gt_item['gt_type'],
                'gt_dist_from_boat': gt_item['dist_from_boat'],
                'bin': gt_item['bin'],
            })

    results_by_radius[radius] = {
        'errors_by_bin': errors_by_bin,
        'gt_detection_counts': gt_detection_counts,
        'gt_total_counts': gt_total_counts,
        'gt_observations': gt_observations,
    }

# ============================================================
# 9.1 主用半径详细输出
# ============================================================
primary_result = results_by_radius[PRIMARY_MATCH_RADIUS]
errors_by_bin = primary_result['errors_by_bin']
gt_observations = primary_result['gt_observations']
gt_detection_counts = primary_result['gt_detection_counts']
gt_total_counts = primary_result['gt_total_counts']

print(f"\n{'='*70}")
print(f"[匹配半径: {PRIMARY_MATCH_RADIUS}m - 主用结果]")
print(f"{'='*70}")

total_matches = sum(len(v) for v in errors_by_bin.values())
total_gt_objects = len(all_gt_static)
print(f"  总匹配数: {total_matches}")
print(f"  GT静态物体总数: {total_gt_objects}")

for min_d, max_d, name in DISTANCE_BINS:
    data = errors_by_bin.get(name, [])
    if not data:
        print(f"\n  --- {name}: 无匹配样本")
        print(f"      (该距离段可能无GT物体或检测率为0)")
        continue

    errors = [d['error'] for d in data]
    errors.sort()
    n = len(errors)
    median = errors[n // 2]
    mean = sum(errors) / n
    rms = math.sqrt(sum(e**2 for e in errors) / n)
    p90 = errors[min(int(n * 0.9), n - 1)]
    max_err = max(errors)

    total_possible_detections = sum(gt_total_counts.get(name, {}).values())
    total_actual_detections = sum(gt_detection_counts.get(name, {}).values())
    detection_rate = total_actual_detections / max(total_possible_detections, 1)

    n_gt_objects = len(gt_total_counts.get(name, {}))
    n_gt_objects_detected = len([
        gid for gid in gt_total_counts.get(name, {})
        if gt_detection_counts.get(name, {}).get(gid, 0) > 0
    ])
    object_coverage = n_gt_objects_detected / max(n_gt_objects, 1)

    type_stats = defaultdict(list)
    for d in data:
        type_stats[d['gt_type']].append(d['error'])

    reliable = detection_rate >= DETECTION_RATE_THRESHOLD

    print(f"\n  --- {name}")
    print(f"  |")
    print(f"  |  [检测率] {detection_rate:.1%} (物体覆盖率: {object_coverage:.1%}, {n_gt_objects_detected}/{n_gt_objects})")
    if not reliable:
        print(f"  |  !!! 检测率低 (<{DETECTION_RATE_THRESHOLD:.0%}), 以下误差仅供参考!")
        print(f"  |      大部分目标未被匹配到, 统计有偏。")
    else:
        print(f"  |  OK 检测率达标, 误差统计具有参考价值")
    print(f"  |")
    print(f"  |  [位置误差] (n={n}个匹配):")
    print(f"  |    中位数: {median:.3f}m")
    print(f"  |    均值:   {mean:.3f}m")
    print(f"  |    RMS:    {rms:.3f}m")
    print(f"  |    P90:    {p90:.3f}m")
    print(f"  |    最大:   {max_err:.3f}m")

    for gt_type in ['pillar', 'buoy', 'block']:
        errs = type_stats.get(gt_type, [])
        if errs:
            errs.sort()
            n_type = len(errs)
            med_type = errs[n_type // 2]
            print(f"  |    - {gt_type}: n={n_type}, 中位数={med_type:.3f}m")
    print(f"  |")

# ============================================================
# 9.2 多档半径对比
# ============================================================
print(f"\n{'='*70}")
print(f"[多档匹配半径对比 - 判读用]")
print(f"{'='*70}")
print(f"""
  解读指南：
    - 样本数 (n) 的变化 比 误差变化 更有信息量
    - 如果5m半径的n >> 2m的n，说明有大量目标误差在2-5m之间
    - 如果误差中位数随半径变化不大，说明结果稳定
""")

header = f"  {'半径':<8}"
for min_d, max_d, name in DISTANCE_BINS:
    header += f" | {'误差中位数':<12} {'样本数':<8} {'检测率':<8}"
print(header)
print("  " + "-" * (len(header) - 2))

for radius in MATCH_RADII:
    r = results_by_radius[radius]
    row = [f"r={radius}m"]
    for min_d, max_d, name in DISTANCE_BINS:
        data = r['errors_by_bin'].get(name, [])
        if data:
            errs = [d['error'] for d in data]
            errs.sort()
            n = len(errs)
            median = errs[n // 2]
            total_possible = sum(r['gt_total_counts'].get(name, {}).values())
            total_actual = sum(r['gt_detection_counts'].get(name, {}).values())
            det_rate = total_actual / max(total_possible, 1)
            row.append(f"{median:.3f}m")
            row.append(f"n={n}")
            row.append(f"{det_rate:.1%}")
        else:
            row.append("N/A")
            row.append("N/A")
            row.append("N/A")
    print(f"  {row[0]:<6} | {row[1]:<10} {row[2]:<8} {row[3]:<8} | {row[4]:<10} {row[5]:<8} {row[6]:<8} | {row[7]:<10} {row[8]:<8} {row[9]:<8}")

# ============================================================
# 10. 指标A: 检测位置的时间稳定性
# ============================================================
print(f"\n{'='*70}")
print("=== 指标A: 检测位置的时间稳定性 ===")
print(f"{'='*70}")
print(f"""
  定性说明：
  本指标度量的是同一GT物体在多帧中被检测出的位置抖动（spread）。
  它反映的是【时间稳定性】——受点云噪声、聚类抖动、odom插值误差影响。
  它【不是】跨雷达自洽性——cluster_diagnostic日志不记录各路雷达来源。

  已知偏差：
  spread只统计匹配成功的帧（误差<{PRIMARY_MATCH_RADIUS}m），
  所有大误差观测被排除，因此spread被系统性低估。
  仅用于交叉诊断（"稳定但有偏" vs "抖动"），不读绝对值。
""")

gt_spreads = []
for gt_id, observations in gt_observations.items():
    if len(observations) < 2:
        continue

    world_positions = [obs['cluster_world_pos'] for obs in observations]
    mean_wx = sum(p[0] for p in world_positions) / len(world_positions)
    mean_wy = sum(p[1] for p in world_positions) / len(world_positions)

    spread = math.sqrt(
        sum((p[0] - mean_wx)**2 + (p[1] - mean_wy)**2
            for p in world_positions) / len(world_positions)
    )

    avg_dist = sum(obs['gt_dist_from_boat'] for obs in observations) / len(observations)
    if avg_dist < 20:
        bin_key = '0-20m'
    elif avg_dist < 40:
        bin_key = '20-40m'
    else:
        bin_key = '40m+'

    gt_spreads.append({
        'gt_id': gt_id,
        'count': len(observations),
        'spread': spread,
        'avg_dist_from_boat': avg_dist,
        'bin': bin_key,
        'gt_types': list(set(obs['gt_type'] for obs in observations)),
        'errors': [obs['error'] for obs in observations],
    })

if gt_spreads:
    print(f"\n  有多次观测的GT物体数: {len(gt_spreads)}")

    all_spreads = [s['spread'] for s in gt_spreads]
    all_spreads.sort()
    n_total = len(all_spreads)

    print(f"\n  整体时间稳定性（spread统计）:")
    print(f"    中位数spread: {all_spreads[n_total // 2]:.3f}m")
    print(f"    均值spread:   {sum(all_spreads) / n_total:.3f}m")
    print(f"    RMS spread:    {math.sqrt(sum(s**2 for s in all_spreads) / n_total):.3f}m")
    print(f"    P90 spread:    {all_spreads[min(int(n_total * 0.9), n_total - 1)]:.3f}m")
    print(f"    最大spread:    {max(all_spreads):.3f}m")

    print(f"\n  按距离段统计:")
    print(f"  {'-'*60}")

    for min_d, max_d, name in DISTANCE_BINS:
        bin_spreads = [s['spread'] for s in gt_spreads if s['bin'] == name]
        if not bin_spreads:
            print(f"\n    {name}: 无样本")
            continue

        bin_spreads.sort()
        n_bin = len(bin_spreads)
        median = bin_spreads[n_bin // 2]
        mean = sum(bin_spreads) / n_bin
        rms = math.sqrt(sum(s**2 for s in bin_spreads) / n_bin)
        p90 = bin_spreads[min(int(n_bin * 0.9), n_bin - 1)]

        print(f"\n    {name} (共{n_bin}个GT物体):")
        print(f"      中位数spread: {median:.3f}m")
        print(f"      均值spread:   {mean:.3f}m")
        print(f"      RMS spread:    {rms:.3f}m")
        print(f"      P90 spread:    {p90:.3f}m")

    print(f"\n  诊断：spread vs 误差（检测系统性偏差）")
    print(f"  {'-'*60}")
    print(f"  以下列出'稳定性好但有系统偏差'的个体:")
    print(f"  (spread<0.1m 但 avg_error>0.3m -> 稳定地偏着)")

    has_systematic_bias = False
    for min_d, max_d, name in DISTANCE_BINS:
        bin_data = [s for s in gt_spreads if s['bin'] == name]
        for s in bin_data:
            avg_error = sum(s['errors']) / len(s['errors']) if s['errors'] else 0
            if s['spread'] < 0.1 and avg_error > 0.3:
                has_systematic_bias = True
                types_str = '/'.join(s['gt_types'])
                print(f"    {name} GT#{s['gt_id']} ({types_str}): spread={s['spread']:.3f}m, avg_error={avg_error:.3f}m")

    if not has_systematic_bias:
        print(f"    无（所有GT物体要么误差小，要么同时抖动大）")

    print(f"\n  按GT类型细分:")
    print(f"  {'-'*60}")

    for gt_type in ['pillar', 'buoy', 'block']:
        type_spreads = [s['spread'] for s in gt_spreads if gt_type in s['gt_types']]
        if not type_spreads:
            print(f"\n    {gt_type}: 无样本")
            continue
        type_spreads.sort()
        n_type = len(type_spreads)
        median = type_spreads[n_type // 2]
        mean = sum(type_spreads) / n_type
        print(f"\n    {gt_type}: n={n_type}, 中位数spread={median:.3f}m, 均值spread={mean:.3f}m")
else:
    print("\n  没有足够的多次观测数据来计算时间稳定性")

# ============================================================
# 11. 汇总输出
# ============================================================
print(f"\n{'='*70}")
print("=== 汇总表（匹配半径 = 2.0m） ===")
print(f"{'='*70}")

b_cells = {}
for min_d, max_d, name in DISTANCE_BINS:
    data = errors_by_bin.get(name, [])
    det_counts = gt_detection_counts.get(name, {})
    total_counts = gt_total_counts.get(name, {})
    total_possible = sum(total_counts.values())
    total_actual = sum(det_counts.values())
    det_rate = total_actual / max(total_possible, 1)

    if data:
        errs = [d['error'] for d in data]
        errs.sort()
        n = len(errs)
        median = errs[n // 2]
        b_cells[name] = f"{median:.3f}m (n={n}, 检测率{det_rate:.1%})"
    else:
        b_cells[name] = f"N/A (检测率{det_rate:.1%})"

a_cells = {}
for min_d, max_d, name in DISTANCE_BINS:
    bin_spreads = [s['spread'] for s in gt_spreads if s['bin'] == name]
    if bin_spreads:
        bin_spreads.sort()
        n = len(bin_spreads)
        median = bin_spreads[n // 2]
        a_cells[name] = f"{median:.3f}m (n={n})"
    else:
        a_cells[name] = "N/A"

print(f"""
  +---------------------------------------------------------------------------------------------------+
  | 指标                    | 0-20m            | 20-40m           | 40m+             |
  +---------------------------------------------------------------------------------------------------+
  | 指标B: 融合vsGT误差    | {b_cells.get('0-20m', 'N/A'):<17} | {b_cells.get('20-40m', 'N/A'):<17} | {b_cells.get('40m+', 'N/A'):<17} |
  |  (逐帧匹配+纯位置)      |                  |                  |                  |
  +---------------------------------------------------------------------------------------------------+
  | 指标A: 时间稳定性spread | {a_cells.get('0-20m', 'N/A'):<17} | {a_cells.get('20-40m', 'N/A'):<17} | {a_cells.get('40m+', 'N/A'):<17} |
  |  (非跨雷达自洽性)        |                  |                  |                  |
  +---------------------------------------------------------------------------------------------------+
""")

# ============================================================
# 12. 交叉诊断
# ============================================================
print("=== 交叉诊断 ===")
print(f"{'='*70}")

for min_d, max_d, name in DISTANCE_BINS:
    a_med = None
    b_med = None
    det_rate = None

    if gt_spreads:
        bin_spreads = [s['spread'] for s in gt_spreads if s['bin'] == name]
        if bin_spreads:
            bin_spreads.sort()
            a_med = bin_spreads[len(bin_spreads) // 2]

    data = errors_by_bin.get(name, [])
    if data:
        errs = [d['error'] for d in data]
        errs.sort()
        b_med = errs[len(errs) // 2]

    total_possible = sum(gt_total_counts.get(name, {}).values())
    total_actual = sum(gt_detection_counts.get(name, {}).values())
    det_rate = total_actual / max(total_possible, 1)

    if a_med is not None and b_med is not None:
        print(f"\n  {name}:")
        print(f"    检测率={det_rate:.1%}, 稳定性spread={a_med:.3f}m, 位置误差={b_med:.3f}m")

        if det_rate < DETECTION_RATE_THRESHOLD:
            print(f"    !!! 检测率过低，误差数字参考价值有限")
            continue

        if a_med < 0.1 and b_med > 0.3:
            diag = "WARNING 自洽性好但绝对精度差 -> 可能存在整体系统偏差（外参偏移）"
        elif a_med > 0.3 and b_med > 0.3:
            diag = "WARNING 自洽性差且绝对精度差 -> 噪声大或外参标定严重不准"
        elif a_med < 0.1 and b_med < 0.3:
            diag = "OK 自洽性好且绝对精度好 -> 融合精度可信"
        else:
            diag = "INFO 中间状态，需进一步分析"
        print(f"    诊断: {diag}")

# ============================================================
# 13. 最终说明
# ============================================================
print(f"\n{'='*70}")
print("=== 指标性质与边界说明 ===")
print(f"{'='*70}")

print(f"""
  重要提示（务必阅读）：

  1. 指标B是核心
     - 逐帧匹配 + 纯位置匹配（不依赖分类label）
     - 度量融合点云的绝对位置精度
     - 匹配半径{PRIMARY_MATCH_RADIUS}m（同时输出1m/5m作参考）

  2. 判读时必须同时看【检测率】
     - 检测率高(>30%) + 误差小 -> 融合精度真的好
     - 检测率低(<30%) + 误差小 -> 假象（大部分目标没匹配上）
     - 远距离检测率低是预期的（点云稀疏->聚不出簇），0-20m最可信

  3. 指标A（时间稳定性）辅助诊断
     - 不是跨雷达自洽性（日志无雷达来源信息）
     - 用于区分"系统偏差"vs"随机噪声"

  4. 这是替代度量，不是最终评分
     - 不是"融合点云 vs 精准点云"
     - 真正的评分需要精准点云真值（比测现场）

  5. 匹配半径说明
     - {PRIMARY_MATCH_RADIUS}m与检测评估口径一致
     - 1m更严格，5m更宽松
     - 样本数变化比误差变化更有信息量
""")

print("评估完成。")