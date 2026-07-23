#!/usr/bin/env python3
"""
代价评估脚本：评估将boat_fp_max_sane从2.0放宽到2.5的真实收益与代价。

核心逻辑：
1. 解析perception_log.log中的cluster_diagnostic，提取所有候选簇
2. 解析boat_log.jsonl中的GT真值
3. 筛选符合放宽条件的候选簇（已排除被block截胡部分）
4. 计算真实命中率和噪声率，给出真假比
"""
import json
import math
import re
import bisect

BOAT_LOG_PATH = '/home/lyf040817/usv_ws/boat_log.jsonl'
CLUSTER_LOG_PATH = '/home/lyf040817/usv_ws/perception_log.log'
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

if gt_records and odom_records:
    _offset = gt_records[0]['stamp'] - odom_records[0]['stamp']
    for r in gt_records:
        r['stamp'] -= _offset

gt_records.sort(key=lambda r: r['stamp'])
odom_records.sort(key=lambda r: r['stamp'])
gt_stamps = [r['stamp'] for r in gt_records]
odom_stamps = [o['stamp'] for o in odom_records]

def interpolate_gt(t):
    idx = bisect.bisect_left(gt_stamps, t)
    if idx == 0:
        if t <= gt_stamps[0] + MAX_TIME_GAP:
            return gt_records[0]
        return None
    if idx >= len(gt_records):
        if t >= gt_stamps[-1] - MAX_TIME_GAP:
            return gt_records[-1]
        return None
    g0, g1 = gt_records[idx-1], gt_records[idx]
    t0, t1 = g0['stamp'], g1['stamp']
    if t < t0 - MAX_TIME_GAP or t > t1 + MAX_TIME_GAP:
        return None
    poses0, poses1 = g0['poses'], g1['poses']
    if len(poses0) != len(poses1):
        if abs(t - t0) < abs(t - t1):
            return g0
        else:
            return g1
    alpha = (t - t0) / (t1 - t0)
    poses = []
    for p0, p1 in zip(poses0, poses1):
        poses.append({'x': p0['x'] + (p1['x'] - p0['x']) * alpha, 'y': p0['y'] + (p1['y'] - p0['y']) * alpha})
    return {'stamp': t, 'poses': poses}

def interpolate_odom(t):
    idx = bisect.bisect_left(odom_stamps, t)
    if idx == 0:
        return odom_records[0] if odom_records else None
    if idx >= len(odom_records):
        return odom_records[-1] if odom_records else None
    r0, r1 = odom_records[idx-1], odom_records[idx]
    t0, t1 = r0['stamp'], r1['stamp']
    if abs(t1 - t0) < 1e-9:
        return r0
    alpha = (t - t0) / (t1 - t0)
    return {'x': r0['x'] + (r1['x'] - r0['x']) * alpha, 'y': r0['y'] + (r1['y'] - r0['y']) * alpha, 'qx': r0['qx'], 'qy': r0['qy'], 'qz': r0['qz'], 'qw': r0['qw']}

def quat_to_yaw(qx, qy, qz, qw):
    return math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))

def world_to_boat(wx, wy, bx, by, yaw):
    dx, dy = wx - bx, wy - by
    rx = dx*math.cos(-yaw) - dy*math.sin(-yaw)
    ry = dx*math.sin(-yaw) + dy*math.cos(-yaw)
    return rx, ry

NUM = r'[-+]?[\d.]+(?:[eE][-+]?\d+)?'
pending_pattern = re.compile(
    r'\[cluster_diagnostic\] t=(' + NUM + r') center=\((' + NUM + r'),(' + NUM + r'),(' + NUM + r')\)'
    r' size=\((' + NUM + r'),(' + NUM + r'),(' + NUM + r')\)'
    r' fp_max=(' + NUM + r') fp_min=(' + NUM + r') square=(' + NUM + r') pts=(\d+) -> classification pending'
)
assigned_pattern = re.compile(
    r'\[cluster_diagnostic\] assigned=(\w+)'
)

all_clusters = []

with open(CLUSTER_LOG_PATH, encoding='utf-8', errors='replace') as f:
    current_cluster = None
    for line in f:
        line = line.strip()
        pending_match = pending_pattern.match(line)
        if pending_match:
            if current_cluster:
                all_clusters.append(current_cluster)
            try:
                t = float(pending_match.group(1))
                current_cluster = {
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
                    'label': None
                }
            except (ValueError, IndexError):
                current_cluster = None
                continue
        else:
            assigned_match = assigned_pattern.match(line)
            if assigned_match and current_cluster:
                current_cluster['label'] = assigned_match.group(1)
                all_clusters.append(current_cluster)
                current_cluster = None

if current_cluster:
    all_clusters.append(current_cluster)

print(f"[解析统计] 共解析到 {len(all_clusters)} 个候选簇")

# cluster_diagnostic的center x,y本身已经是船体系(base_link)坐标——ObstacleDetector.hpp
# 明确文档"输入：船体坐标系(base_link)下的融合点云"，detect()的形参也直接叫cloud_boat，
# 发布话题frame_id="wamv/base_link"。不需要也不能再对它做一次world_to_boat变换，那样等于
# 把一个已经是船体系的小量级坐标，当成世界坐标去减odom的大量级世界坐标，凭空制造出几百米
# 的偏移假象（用odom≈(-702,347)实测复现过：45m的真实船体系距离，套一次多余变换后变成763m）。
# 只有gt（世界坐标）需要world_to_boat变换到船体系，cluster本身不需要。

# t=的时钟域和det一样，取决于main.cpp具体版本，不能硬编码减一个固定offset——
# 用中位数样本判断是否需要平移，和boat_evaluator.py的_align_to_odom保持同一套逻辑。
_cluster_sample = all_clusters[len(all_clusters)//2]['t'] if all_clusters else 0.0
_odom_mid = odom_stamps[len(odom_stamps)//2]
_needs_shift = abs(_cluster_sample - _odom_mid) > 1000
if _needs_shift:
    print(f"[时间基准校正] cluster_diagnostic时间戳偏移量估计: {_offset:.4f}s，已统一到odom基准")

_expected_lo, _expected_hi = odom_stamps[0] - 200, odom_stamps[-1] + 200
_dropped_outliers = 0

dynamic_threshold_candidates = []

for c in all_clusters:
    t_relative = (c['t'] - _offset) if _needs_shift else c['t']
    if not (_expected_lo <= t_relative <= _expected_hi):
        # 明显损坏的离群值（多线程并发写std::cout导致的字符级数据损坏）
        _dropped_outliers += 1
        continue

    odom = interpolate_odom(t_relative)
    gt = interpolate_gt(t_relative)

    if odom is None or gt is None or len(gt['poses']) == 0:
        continue

    dist = math.hypot(c['x'], c['y'])

    if 2.0 < c['fp_max'] <= 2.5 and 0.7 < c['dz'] <= 1.8 and c['pts'] >= 40 and dist <= 20.0:
        dynamic_threshold_candidates.append({
            't': t_relative,
            'x_ship': c['x'],
            'y_ship': c['y'],
            'fp_max': c['fp_max'],
            'dz': c['dz'],
            'pts': c['pts'],
            'dist': dist,
            'label': c['label']
        })

if _dropped_outliers:
    print(f"[数据质量] 丢弃{_dropped_outliers}条明显损坏的cluster_diagnostic离群值")

print(f"\n[筛选统计] 符合放宽条件的总簇数 (已排除被block截胡部分): {len(dynamic_threshold_candidates)}")

matched_count = 0
noise_count = 0
matched_clusters = []
noise_clusters = []

for c in dynamic_threshold_candidates:
    t = c['t']
    gt = interpolate_gt(t)
    
    if gt is None or len(gt['poses']) == 0:
        noise_count += 1
        noise_clusters.append(c)
        continue
    
    odom = interpolate_odom(t)
    yaw = quat_to_yaw(odom['qx'], odom['qy'], odom['qz'], odom['qw'])
    gt_ship = [world_to_boat(p['x'], p['y'], odom['x'], odom['y'], yaw) for p in gt['poses']]
    
    is_matched = False
    for gx, gy in gt_ship:
        if math.hypot(c['x_ship'] - gx, c['y_ship'] - gy) < MATCH_TOL:
            is_matched = True
            break
    
    if is_matched:
        matched_count += 1
        matched_clusters.append(c)
    else:
        noise_count += 1
        noise_clusters.append(c)

print(f"\n=== [代价评估] 0-20m 动态阈值放宽至 2.5 ===")
print(f"符合放宽条件的总簇数 (已排除被block截胡部分): {len(dynamic_threshold_candidates)}")
print(f"✅ 真实命中 (能救回的真船): {matched_count}")
print(f"❌ 放进的噪声 (新增的误检): {noise_count}")

if len(dynamic_threshold_candidates) > 0:
    ratio = noise_count / max(matched_count, 1)
    print(f"📊 预期真假比: 1 : {ratio:.2f}")
    if ratio > 2.0:
        print("⚠️ 警告：代价过大，这笔买卖极度不划算！")
    else:
        print("💡 提示：真假比在可接受范围内，建议修改C++代码并进行全量回测。")
else:
    print("🤷‍♂️ 提示：在这个苛刻条件下，没有找到任何候选簇。这刀改了等于没改。")

if matched_clusters:
    print("\n=== [命中簇明细] ===")
    for c in matched_clusters[:10]:
        print(f"  t={c['t']:.3f}s | dist={c['dist']:.1f}m | fp_max={c['fp_max']:.2f} | dz={c['dz']:.2f} | pts={c['pts']} | label={c['label']}")
    if len(matched_clusters) > 10:
        print(f"  ... 还有 {len(matched_clusters) - 10} 个")

if noise_clusters:
    print("\n=== [噪声簇明细] ===")
    for c in noise_clusters[:10]:
        print(f"  t={c['t']:.3f}s | dist={c['dist']:.1f}m | fp_max={c['fp_max']:.2f} | dz={c['dz']:.2f} | pts={c['pts']} | label={c['label']}")
    if len(noise_clusters) > 10:
        print(f"  ... 还有 {len(noise_clusters) - 10} 个")

print("\n=== [特征分布] ===")
if dynamic_threshold_candidates:
    fp_max_vals = [c['fp_max'] for c in dynamic_threshold_candidates]
    dz_vals = [c['dz'] for c in dynamic_threshold_candidates]
    pts_vals = [c['pts'] for c in dynamic_threshold_candidates]
    dist_vals = [c['dist'] for c in dynamic_threshold_candidates]
    
    print(f"fp_max: 最小={min(fp_max_vals):.2f}, 最大={max(fp_max_vals):.2f}, 平均={sum(fp_max_vals)/len(fp_max_vals):.2f}")
    print(f"dz: 最小={min(dz_vals):.2f}, 最大={max(dz_vals):.2f}, 平均={sum(dz_vals)/len(dz_vals):.2f}")
    print(f"pts: 最小={min(pts_vals)}, 最大={max(pts_vals)}, 平均={sum(pts_vals)/len(pts_vals):.1f}")
    print(f"dist: 最小={min(dist_vals):.1f}m, 最大={max(dist_vals):.1f}m, 平均={sum(dist_vals)/len(dist_vals):.1f}m")