#!/usr/bin/env python3
"""Cross-reference BoatTracker's per-output diagnostic (x,y,misses) against
GT boat positions, bucketed by misses (0 = real match this frame, >0 = how
many frames of constant-velocity backfill). Answers: how much of the +2.5pp
recall gain comes from backfill vs how much of the +4.0pp FPR increase does.
"""
import bisect
import math
import re
import sys

MATCH_TOL = 2.0
MAX_TIME_GAP = 0.5

diag_path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/tracker_diag_run.log"
boat_log_path = sys.argv[2] if len(sys.argv) > 2 else "/tmp/tracker_diag_boat_log.jsonl"

import json

odom_records = []
gt_records = []
with open(boat_log_path) as f:
    for line in f:
        rec = json.loads(line)
        if rec["type"] == "odom":
            odom_records.append(rec)
        elif rec["type"] == "gt":
            gt_records.append(rec)

odom_records.sort(key=lambda r: r["stamp"])
gt_records.sort(key=lambda r: r["stamp"])
odom_stamps = [r["stamp"] for r in odom_records]
gt_stamps = [r["stamp"] for r in gt_records]

_epoch_offset = gt_records[0]["stamp"] - odom_records[0]["stamp"]
sample = gt_records[len(gt_records) // 2]["stamp"]
odom_mid = odom_records[len(odom_records) // 2]["stamp"]
if abs(sample - odom_mid) > 1000:
    for r in gt_records:
        r["stamp"] -= _epoch_offset
    gt_stamps = [r["stamp"] for r in gt_records]
    print(f"[epoch] applied offset {_epoch_offset:.4f}s to gt")


def interpolate_odom(t):
    idx = bisect.bisect_left(odom_stamps, t)
    if idx == 0:
        return odom_records[0] if (odom_records and t <= odom_stamps[0] + MAX_TIME_GAP) else None
    if idx >= len(odom_records):
        return odom_records[-1] if (odom_records and t >= odom_stamps[-1] - MAX_TIME_GAP) else None
    r0, r1 = odom_records[idx - 1], odom_records[idx]
    t0, t1 = r0["stamp"], r1["stamp"]
    if abs(t1 - t0) < 1e-9:
        return r0
    alpha = (t - t0) / (t1 - t0)
    def lerp(a, b): return a + (b - a) * alpha
    return {"x": lerp(r0["x"], r1["x"]), "y": lerp(r0["y"], r1["y"]),
            "qx": lerp(r0["qx"], r1["qx"]), "qy": lerp(r0["qy"], r1["qy"]),
            "qz": lerp(r0["qz"], r1["qz"]), "qw": lerp(r0["qw"], r1["qw"])}


def interpolate_gt(t):
    idx = bisect.bisect_left(gt_stamps, t)
    if idx == 0:
        return gt_records[0] if t <= gt_stamps[0] + MAX_TIME_GAP else None
    if idx >= len(gt_records):
        return gt_records[-1] if t >= gt_stamps[-1] - MAX_TIME_GAP else None
    r0, r1 = gt_records[idx - 1], gt_records[idx]
    t0, t1 = r0["stamp"], r1["stamp"]
    if t < t0 - MAX_TIME_GAP or t > t1 + MAX_TIME_GAP:
        return None
    poses0, poses1 = r0["poses"], r1["poses"]
    if len(poses0) != len(poses1):
        return r0 if abs(t - t0) < abs(t - t1) else r1
    alpha = (t - t0) / (t1 - t0)
    def lerp(a, b): return a + (b - a) * alpha
    return {"poses": [{"x": lerp(p0["x"], p1["x"]), "y": lerp(p0["y"], p1["y"])}
                       for p0, p1 in zip(poses0, poses1)]}


def quat_to_yaw(qx, qy, qz, qw):
    return math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


def world_to_boat(wx, wy, bx, by, yaw):
    dx, dy = wx - bx, wy - by
    rx = dx * math.cos(-yaw) - dy * math.sin(-yaw)
    ry = dx * math.sin(-yaw) + dy * math.cos(-yaw)
    return rx, ry


pattern = re.compile(r"\[TrackerOutputDiag\] t=([\d.]+) x=([-\d.]+) y=([-\d.]+) misses=(\d+)")
outputs = []
with open(diag_path) as f:
    for line in f:
        m = pattern.search(line)
        if m:
            outputs.append((float(m.group(1)), float(m.group(2)), float(m.group(3)), int(m.group(4))))
print(f"parsed {len(outputs)} TrackerOutputDiag lines")

buckets = {}  # misses -> [matched_count, total_count]
for t, x, y, misses in outputs:
    odom = interpolate_odom(t)
    gt = interpolate_gt(t)
    matched = False
    if odom is not None and gt is not None and gt["poses"]:
        yaw = quat_to_yaw(odom["qx"], odom["qy"], odom["qz"], odom["qw"])
        gt_frame = [world_to_boat(p["x"], p["y"], odom["x"], odom["y"], yaw) for p in gt["poses"]]
        best_d = min(math.hypot(x - gx, y - gy) for gx, gy in gt_frame)
        matched = best_d < MATCH_TOL
    b = buckets.setdefault(misses, [0, 0])
    b[1] += 1
    if matched:
        b[0] += 1

print(f"\n{'misses':>7} {'total':>7} {'matched':>8} {'match_rate':>11}")
for misses in sorted(buckets):
    m, t = buckets[misses]
    print(f"{misses:>7} {t:>7} {m:>8} {100.0*m/t if t else 0:>10.1f}%")

# aggregate: misses==0 (real) vs misses>0 (backfill)
real_m, real_t = 0, 0
backfill_m, backfill_t = 0, 0
for misses, (m, t) in buckets.items():
    if misses == 0:
        real_m += m; real_t += t
    else:
        backfill_m += m; backfill_t += t
print(f"\nreal (misses=0): {real_m}/{real_t} = {100.0*real_m/real_t if real_t else 0:.1f}% match rate")
print(f"backfill (misses>0): {backfill_m}/{backfill_t} = {100.0*backfill_m/backfill_t if backfill_t else 0:.1f}% match rate")
