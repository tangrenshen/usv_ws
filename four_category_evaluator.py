#!/usr/bin/env python3
"""
四类目标(boats/buoys/pillars/blocks)统一评估器，逻辑和boat_evaluator.py完全一致
(interpolate_odom/interpolate_gt线性插值 + _align_to_odom自适应时钟对齐)，只是把
gt/det的类型参数化，四类各自独立跑一遍，另外给一个"总召回率"(四类真值和命中数
直接相加)方便和任务书的整体评分对齐。
"""
import json
import math
import bisect

MATCH_TOL = 2.0
MAX_TIME_GAP = 0.5
LOG_PATH = '/home/lyf040817/usv_ws/boat_log.jsonl'

CATEGORIES = [
    ('boat',   'gt',        'det'),
    ('buoy',   'gt_buoy',   'det_buoy'),
    ('pillar', 'gt_pillar', 'det_pillar'),
    ('block',  'gt_block',  'det_block'),
]

records_by_type = {}
odom_records = []
with open(LOG_PATH) as f:
    for line in f:
        rec = json.loads(line)
        t = rec['type']
        if t == 'odom':
            odom_records.append(rec)
        else:
            records_by_type.setdefault(t, []).append(rec)

odom_records.sort(key=lambda r: r['stamp'])
odom_stamps = [r['stamp'] for r in odom_records]

def interpolate_odom(t):
    # 越界钳制到首/末已知位姿是有害的静默行为：2026-07-24发现odom比gt早停约9秒，
    # 钳制会让这9秒里的每一帧都用同一个冻结位姿去做world_to_boat变换，产生系统性
    # 错误坐标，污染四个类别的评估——而且不报错、不跳过，混在总账里很难发现。
    # 改成和interpolate_gt一致的边界处理：超出MAX_TIME_GAP容差直接返回None，
    # 让调用方按"真值不覆盖"处理并从统计里排除，而不是拿一个虚构的位姿继续算。
    idx = bisect.bisect_left(odom_stamps, t)
    if idx == 0:
        return odom_records[0] if (odom_records and t <= odom_stamps[0] + MAX_TIME_GAP) else None
    if idx >= len(odom_records):
        return odom_records[-1] if (odom_records and t >= odom_stamps[-1] - MAX_TIME_GAP) else None
    r0, r1 = odom_records[idx-1], odom_records[idx]
    t0, t1 = r0['stamp'], r1['stamp']
    if abs(t1 - t0) < 1e-9:
        return r0
    alpha = (t - t0) / (t1 - t0)
    def lerp(a, b): return a + (b - a) * alpha
    return {
        'x': lerp(r0['x'], r1['x']), 'y': lerp(r0['y'], r1['y']),
        'qx': lerp(r0['qx'], r1['qx']), 'qy': lerp(r0['qy'], r1['qy']),
        'qz': lerp(r0['qz'], r1['qz']), 'qw': lerp(r0['qw'], r1['qw']),
    }

# offset只能用boat的gt(记录['gt'])与odom的[0]差值估计——gt是仿真器直接广播、
# 起始延迟可忽略，是唯一可靠的offset来源。det/det_buoy/det_pillar/det_block都来自
# 同一个perception_node，且该节点有真实启动延迟(~8秒)，绝不能用它们自己的[0]算offset，
# 那样会把启动延迟当成时钟域偏移量，产生系统性错位(这个bug第一版就踩过，det[0]-odom[0]
# 和真正的epoch_offset差了~8.3秒，导致boat召回率算出3.4%而不是正确的20%)。
_canonical_epoch_offset = 0.0
if records_by_type.get('gt') and odom_records:
    _canonical_epoch_offset = records_by_type['gt'][0]['stamp'] - odom_records[0]['stamp']

def _align_to_odom(records, label):
    """每个序列(gt/gt_buoy/gt_pillar/gt_block/det/det_buoy/det_pillar/det_block)各自
    独立判断是否处于"绝对纪元"域(用中位数样本 vs odom中位数样本)，需要的话套用
    上面算出的唯一正确offset，而不是自己重新算一个。"""
    if not records or not odom_records:
        return
    sample = records[len(records) // 2]['stamp']
    odom_mid = odom_records[len(odom_records) // 2]['stamp']
    if abs(sample - odom_mid) > 1000:
        for r in records:
            r['stamp'] -= _canonical_epoch_offset
        print(f"[时间基准校正] {label}时间戳偏移量估计: {_canonical_epoch_offset:.4f}s，已统一到odom基准")

def world_to_boat(wx, wy, bx, by, yaw):
    dx, dy = wx - bx, wy - by
    rx = dx*math.cos(-yaw) - dy*math.sin(-yaw)
    ry = dx*math.sin(-yaw) + dy*math.cos(-yaw)
    return rx, ry

def quat_to_yaw(qx, qy, qz, qw):
    return math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))

def make_interpolate_gt(gt_records, gt_stamps):
    def interpolate_gt(t):
        idx = bisect.bisect_left(gt_stamps, t)
        if idx == 0:
            return gt_records[0] if t <= gt_stamps[0] + MAX_TIME_GAP else None
        if idx >= len(gt_records):
            return gt_records[-1] if t >= gt_stamps[-1] - MAX_TIME_GAP else None
        r0, r1 = gt_records[idx-1], gt_records[idx]
        t0, t1 = r0['stamp'], r1['stamp']
        if t < t0 - MAX_TIME_GAP or t > t1 + MAX_TIME_GAP:
            return None
        poses0, poses1 = r0['poses'], r1['poses']
        if len(poses0) != len(poses1):
            return r0 if abs(t - t0) < abs(t - t1) else r1
        alpha = (t - t0) / (t1 - t0)
        def lerp(a, b): return a + (b - a) * alpha
        return {'poses': [{'x': lerp(p0['x'], p1['x']), 'y': lerp(p0['y'], p1['y'])}
                          for p0, p1 in zip(poses0, poses1)]}
    return interpolate_gt

def evaluate(name, gt_key, det_key):
    gt_records = records_by_type.get(gt_key, [])
    det_records = records_by_type.get(det_key, [])

    if not gt_records or not det_records:
        print(f"\n=== [{name}] === 无数据 (gt={len(gt_records)} det={len(det_records)})")
        return None

    _align_to_odom(gt_records, gt_key)
    _align_to_odom(det_records, det_key)

    gt_records = sorted(gt_records, key=lambda r: r['stamp'])
    gt_stamps = [r['stamp'] for r in gt_records]
    interpolate_gt = make_interpolate_gt(gt_records, gt_stamps)

    hit = miss = fp = 0
    errors = []
    skipped = 0
    gt_not_covered = 0

    for det in det_records:
        t = det['stamp']
        odom = interpolate_odom(t)
        gt = interpolate_gt(t)

        if odom is None:
            skipped += 1
            continue
        if gt is None or len(gt['poses']) == 0:
            gt_not_covered += 1
            fp += len(det['poses'])
            continue

        yaw = quat_to_yaw(odom['qx'], odom['qy'], odom['qz'], odom['qw'])
        gt_frame = [world_to_boat(p['x'], p['y'], odom['x'], odom['y'], yaw) for p in gt['poses']]
        det_pts = [(p['x'], p['y']) for p in det['poses']]

        matched_gt = set()
        for dx, dy in det_pts:
            best_d, best_i = 1e9, -1
            for i, (gx, gy) in enumerate(gt_frame):
                d = math.hypot(dx-gx, dy-gy)
                if d < best_d:
                    best_d, best_i = d, i
            if best_d < MATCH_TOL:
                matched_gt.add(best_i)
                errors.append(best_d)
            else:
                fp += 1

        hit += len(matched_gt)
        miss += (len(gt_frame) - len(matched_gt))

    total = hit + miss
    det_total = hit + fp
    recall = hit/total*100 if total > 0 else 0
    fpr = fp/det_total*100 if det_total > 0 else 0
    err_mean = sum(errors)/len(errors) if errors else 0

    covered_frames = len(det_records) - skipped
    print(f"\n=== [{name}] ===")
    print(f"检测消息总数={len(det_records)} 跳过(odom不可用)={skipped} 跳过(真值不覆盖)={gt_not_covered}")
    print(f"[口径] 有效帧数(odom覆盖)={covered_frames}  真值总数(命中+遗漏)={total}")
    print(f"真值命中={hit} 真值遗漏={miss} 误检={fp}")
    print(f"召回率={recall:.1f}% 误检率={fpr:.1f}%" + (f" 位置误差均值={err_mean:.2f}m" if errors else ""))

    return {'name': name, 'hit': hit, 'miss': miss, 'fp': fp, 'recall': recall, 'fpr': fpr,
            'det_count': len(det_records), 'covered_frames': covered_frames, 'gt_total': total}

results = []
for name, gt_key, det_key in CATEGORIES:
    r = evaluate(name, gt_key, det_key)
    if r:
        results.append(r)

print("\n" + "="*60)
print("=== 四类汇总 ===")
print("="*60)
print(f"{'类别':<8} {'真值命中':>8} {'真值遗漏':>8} {'误检':>8} {'召回率':>8} {'误检率':>8}")
for r in results:
    print(f"{r['name']:<8} {r['hit']:>8} {r['miss']:>8} {r['fp']:>8} {r['recall']:>7.1f}% {r['fpr']:>7.1f}%")

total_hit = sum(r['hit'] for r in results)
total_miss = sum(r['miss'] for r in results)
total_fp = sum(r['fp'] for r in results)
overall_recall = total_hit/(total_hit+total_miss)*100 if (total_hit+total_miss) > 0 else 0
overall_fpr = total_fp/(total_hit+total_fp)*100 if (total_hit+total_fp) > 0 else 0
print(f"\n合计: 命中={total_hit} 遗漏={total_miss} 误检={total_fp} 总召回率={overall_recall:.1f}% 总误检率={overall_fpr:.1f}%")

# 口径警告：2026-07-24发现det帧数会因为主循环行为变化（比如odom截止时间、
# 新增的提前return路径）而变化，而真值总数是按det帧数统计出来的——两次跑测
# 如果det覆盖的帧数差异很大，召回率就不是同一个口径，不能直接比较涨跌。
det_count = results[0]['det_count'] if results else 0
covered = results[0]['covered_frames'] if results else 0
gt_total = sum(r['gt_total'] for r in results)
print(f"\n[口径基准] det消息总数={det_count} 有效帧数(odom覆盖)={covered} 四类真值总数合计={gt_total}")
print("[口径警告] 和其他跑测对比总召回率/总误检率之前，先比较这三个数字——")
print("           如果det消息总数或有效帧数差异超过5%，说明两次统计的是不同的帧集合，")
print("           总账不能直接比较涨跌，需要先统一时间窗口或帧集合再对比。")
