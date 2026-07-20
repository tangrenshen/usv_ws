#!/usr/bin/env python3
"""
离线裁判：读取boat_logger.py记录的时间序列，对每条检测(det)按时间戳找最近的
odom和gt记录做配对，转换到船体系后做位置匹配，输出真正的召回率/误检率/位置误差。
"""
import json
import math
import bisect
import sys

MATCH_TOL = 2.0
MAX_TIME_GAP = 0.5

LOG_PATH = '/home/lyf040817/usv_ws/boat_log.jsonl'
calib_time = None

if len(sys.argv) > 1:
    if sys.argv[1].endswith('.jsonl'):
        LOG_PATH = sys.argv[1]
        if len(sys.argv) > 2:
            calib_time = float(sys.argv[2])
    else:
        calib_time = float(sys.argv[1])

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
    print(f"[时间基准校正] gt时间戳偏移量估计: {_offset:.4f}s，已统一到odom/det基准")

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

def evaluate_segment(name, dets, offset=0.0):
    hit = 0
    miss = 0
    fp = 0
    errors = []
    skipped = 0

    for det in dets:
        t = det['stamp']
        odom, odom_gap = find_nearest(odom_stamps, odom_records, t)
        gt, gt_gap = find_nearest(gt_stamps, gt_records, t)
        if odom is None or gt is None:
            continue
        if odom_gap > MAX_TIME_GAP or gt_gap > MAX_TIME_GAP:
            skipped += 1
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
            if best_d < MATCH_TOL:
                matched_gt.add(best_i)
                errors.append(best_d)
            else:
                fp += 1

        hit += len(matched_gt)
        miss += (len(gt_boat_frame) - len(matched_gt))

    total = hit + miss
    det_total = hit + fp
    recall = hit/total*100 if total > 0 else 0
    fpr = fp/det_total*100 if det_total > 0 else 0
    err_mean = sum(errors)/len(errors) if errors else 0
    err_max = max(errors) if errors else 0
    err_min = min(errors) if errors else 0

    print(f"[{name}] 检测数={len(dets)} 跳过={skipped} 命中={hit} 遗漏={miss} 误检={fp}")
    print(f"[{name}] 召回率={recall:.1f}% 误检率={fpr:.1f}%")
    if errors:
        print(f"[{name}] 位置误差: 均值={err_mean:.2f}m 最大={err_max:.2f}m 最小={err_min:.2f}m")

    return recall, fpr, err_mean

if calib_time is not None:
    det_before = [d for d in det_records if d['stamp'] < calib_time]
    det_after = [d for d in det_records if d['stamp'] >= calib_time]

    print(f"=== 标定完成时间: {calib_time:.2f}s ===")
    print(f"标定前检测数: {len(det_before)}  标定后检测数: {len(det_after)}")
    print()

    rec_before, fpr_before, err_before = evaluate_segment("标定前", det_before)
    print()
    rec_after, fpr_after, err_after = evaluate_segment("标定后", det_after)
    print()

    print("=== 对比 ===")
    print(f"召回率差异: 标定前{rec_before:.1f}% -> 标定后{rec_after:.1f}% ({'+' if rec_after > rec_before else ''}{rec_after-rec_before:.1f}%)")
    print(f"误检率差异: 标定前{fpr_before:.1f}% -> 标定后{fpr_after:.1f}% ({'+' if fpr_after > fpr_before else ''}{fpr_after-fpr_before:.1f}%)")
    print(f"位置误差差异: 标定前{err_before:.2f}m -> 标定后{err_after:.2f}m ({'+' if err_after > err_before else ''}{err_after-err_before:.2f}m)")
else:
    total_hit = 0
    total_miss = 0
    total_fp = 0
    all_errors = []
    skipped_stale = 0

    for det in det_records:
        t = det['stamp']
        odom, odom_gap = find_nearest(odom_stamps, odom_records, t)
        gt, gt_gap = find_nearest(gt_stamps, gt_records, t)
        if odom is None or gt is None:
            continue
        if odom_gap > MAX_TIME_GAP or gt_gap > MAX_TIME_GAP:
            skipped_stale += 1
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
            if best_d < MATCH_TOL:
                matched_gt.add(best_i)
                all_errors.append(best_d)
            else:
                total_fp += 1

        total_hit += len(matched_gt)
        total_miss += (len(gt_boat_frame) - len(matched_gt))

    print(f"检测消息总数: {len(det_records)}  跳过(时间戳间隔过大): {skipped_stale}")
    print(f"真值命中(去重前累加,按帧统计): {total_hit}")
    print(f"真值遗漏(按帧统计): {total_miss}")
    print(f"误检(候选未匹配到任何真值): {total_fp}")
    if total_hit + total_miss > 0:
        print(f"平均召回率: {total_hit/(total_hit+total_miss)*100:.1f}%")
    total_det = total_hit + total_fp
    if total_det > 0:
        print(f"平均误检率: {total_fp/total_det*100:.1f}%")
    if all_errors:
        print(f"位置误差: 均值={sum(all_errors)/len(all_errors):.2f}m 最大={max(all_errors):.2f}m 最小={min(all_errors):.2f}m")
