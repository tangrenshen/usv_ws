#!/usr/bin/env python3
"""Random-baseline test for buoy recall inflation: replace each real
det_buoy candidate's (x,y) with a uniform-random point in the same-frame
GT extent (preserving per-frame candidate COUNT and rough range, but
destroying any real spatial correspondence), then run the exact same
matching logic (2m tolerance, per-frame nearest-GT, matched_gt set) used
by four_category_evaluator.py. If the random baseline recall is a large
fraction of the real 64.5%, that's the base-rate "any point in this dense
field tends to land near some buoy" effect, not real detection capability.
"""
import json
import math
import random
import sys

MATCH_TOL = 2.0
MAX_TIME_GAP = 0.5
LOG_PATH = sys.argv[1] if len(sys.argv) > 1 else "/tmp/eval_test_boat_log.jsonl"
N_TRIALS = 5

random.seed(1234)

records_by_type = {}
odom_records = []
with open(LOG_PATH) as f:
    for line in f:
        rec = json.loads(line)
        t = rec["type"]
        if t == "odom":
            odom_records.append(rec)
        else:
            records_by_type.setdefault(t, []).append(rec)

odom_records.sort(key=lambda r: r["stamp"])
odom_stamps = [r["stamp"] for r in odom_records]


def bisect_left(a, x):
    lo, hi = 0, len(a)
    while lo < hi:
        mid = (lo + hi) // 2
        if a[mid] < x:
            lo = mid + 1
        else:
            hi = mid
    return lo


def interpolate_odom(t):
    idx = bisect_left(odom_stamps, t)
    if idx == 0:
        return odom_records[0] if (odom_records and t <= odom_stamps[0] + MAX_TIME_GAP) else None
    if idx >= len(odom_records):
        return odom_records[-1] if (odom_records and t >= odom_stamps[-1] - MAX_TIME_GAP) else None
    r0, r1 = odom_records[idx - 1], odom_records[idx]
    t0, t1 = r0["stamp"], r1["stamp"]
    if abs(t1 - t0) < 1e-9:
        return r0
    alpha = (t - t0) / (t1 - t0)

    def lerp(a, b):
        return a + (b - a) * alpha

    return {
        "x": lerp(r0["x"], r1["x"]), "y": lerp(r0["y"], r1["y"]),
        "qx": lerp(r0["qx"], r1["qx"]), "qy": lerp(r0["qy"], r1["qy"]),
        "qz": lerp(r0["qz"], r1["qz"]), "qw": lerp(r0["qw"], r1["qw"]),
    }


def quat_to_yaw(qx, qy, qz, qw):
    return math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


def world_to_boat(wx, wy, bx, by, yaw):
    dx, dy = wx - bx, wy - by
    return (dx * math.cos(-yaw) - dy * math.sin(-yaw), dx * math.sin(-yaw) + dy * math.cos(-yaw))


gt_boat_for_offset = records_by_type.get("gt", [])
_canonical_epoch_offset = 0.0
if gt_boat_for_offset and odom_records:
    _canonical_epoch_offset = gt_boat_for_offset[0]["stamp"] - odom_records[0]["stamp"]


def _align_to_odom(records, label):
    # 单一canonical offset（从boat gt[0]-odom[0]算出），套用到所有需要对齐的序列，
    # 不能每个序列自己重算一个——这是这条线上反复强调过的口径纪律。
    if not records or not odom_records:
        return
    mid_r = records[len(records) // 2]["stamp"]
    mid_o = odom_records[len(odom_records) // 2]["stamp"]
    if abs(mid_r - mid_o) > 1000:
        for r in records:
            r["stamp"] -= _canonical_epoch_offset
        print(f"[epoch] applied canonical offset {_canonical_epoch_offset:.4f}s to {label}")


gt_records = records_by_type.get("gt_buoy", [])
det_records = records_by_type.get("det_buoy", [])
_align_to_odom(gt_records, "gt_buoy")
_align_to_odom(det_records, "det_buoy")
gt_records = sorted(gt_records, key=lambda r: r["stamp"])
gt_stamps = [r["stamp"] for r in gt_records]


def interpolate_gt(t):
    idx = bisect_left(gt_stamps, t)
    if idx == 0:
        return gt_records[0] if (gt_records and t <= gt_stamps[0] + MAX_TIME_GAP) else None
    if idx >= len(gt_records):
        return gt_records[-1] if (gt_records and t >= gt_stamps[-1] - MAX_TIME_GAP) else None
    r0, r1 = gt_records[idx - 1], gt_records[idx]
    t0, t1 = r0["stamp"], r1["stamp"]
    if t < t0 - MAX_TIME_GAP or t > t1 + MAX_TIME_GAP:
        return None
    poses0, poses1 = r0["poses"], r1["poses"]
    if len(poses0) != len(poses1):
        return r0 if abs(t - t0) < abs(t - t1) else r1
    alpha = (t - t0) / (t1 - t0)

    def lerp(a, b):
        return a + (b - a) * alpha

    return {"poses": [{"x": lerp(p0["x"], p1["x"]), "y": lerp(p0["y"], p1["y"])}
                       for p0, p1 in zip(poses0, poses1)]}


def real_recall():
    hit = miss = 0
    for det in det_records:
        t = det["stamp"]
        odom = interpolate_odom(t)
        gt = interpolate_gt(t)
        if odom is None or gt is None or not gt["poses"]:
            continue
        yaw = quat_to_yaw(odom["qx"], odom["qy"], odom["qz"], odom["qw"])
        gt_frame = [world_to_boat(p["x"], p["y"], odom["x"], odom["y"], yaw) for p in gt["poses"]]
        det_pts = [(p["x"], p["y"]) for p in det["poses"]]
        matched = set()
        for dx, dy in det_pts:
            best_d, best_i = 1e9, -1
            for i, (gx, gy) in enumerate(gt_frame):
                d = math.hypot(dx - gx, dy - gy)
                if d < best_d:
                    best_d, best_i = d, i
            if best_d < MATCH_TOL:
                matched.add(best_i)
        hit += len(matched)
        miss += len(gt_frame) - len(matched)
    return hit, miss


def random_recall(range_mode):
    hit = miss = 0
    for det in det_records:
        t = det["stamp"]
        odom = interpolate_odom(t)
        gt = interpolate_gt(t)
        if odom is None or gt is None or not gt["poses"]:
            continue
        yaw = quat_to_yaw(odom["qx"], odom["qy"], odom["qz"], odom["qw"])
        gt_frame = [world_to_boat(p["x"], p["y"], odom["x"], odom["y"], yaw) for p in gt["poses"]]
        n = len(det["poses"])
        if range_mode == "gt_extent":
            ranges = [math.hypot(gx, gy) for gx, gy in gt_frame]
            rmax = max(ranges) if ranges else 60.0
        else:
            rmax = 60.0
        rand_pts = []
        for _ in range(n):
            r = rmax * math.sqrt(random.random())
            theta = random.uniform(-math.pi, math.pi)
            rand_pts.append((r * math.cos(theta), r * math.sin(theta)))
        matched = set()
        for dx, dy in rand_pts:
            best_d, best_i = 1e9, -1
            for i, (gx, gy) in enumerate(gt_frame):
                d = math.hypot(dx - gx, dy - gy)
                if d < best_d:
                    best_d, best_i = d, i
            if best_d < MATCH_TOL:
                matched.add(best_i)
        hit += len(matched)
        miss += len(gt_frame) - len(matched)
    return hit, miss


rh, rm = real_recall()
print(f"real buoy recall: hit={rh} miss={rm} recall={100.0*rh/(rh+rm):.1f}%")

for mode in ("gt_extent", "fixed60"):
    trial_recalls = []
    for trial in range(N_TRIALS):
        h, m = random_recall(mode)
        trial_recalls.append(100.0 * h / (h + m) if (h + m) else 0.0)
    avg = sum(trial_recalls) / len(trial_recalls)
    print(f"random baseline ({mode}, {N_TRIALS} trials): "
          f"{['%.1f%%'%x for x in trial_recalls]} avg={avg:.1f}%")
