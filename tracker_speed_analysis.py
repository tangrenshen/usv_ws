#!/usr/bin/env python3
"""Group BoatTracker's per-output diagnostic (id,x,y,misses,speed) by track id,
and split tracks into "ever matched GT" vs "never matched GT" to test the
hypothesis that persistently-tracked static clutter (low average_speed,
misses==0 every frame) is what drags down the misses==0 match rate.
"""
import bisect
import json
import math
import re
import sys

MATCH_TOL = 2.0
MAX_TIME_GAP = 0.5

diag_path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/tracker_speed_run.log"
boat_log_path = sys.argv[2] if len(sys.argv) > 2 else "/tmp/tracker_speed_boat_log.jsonl"

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


pattern = re.compile(
    r"\[TrackerOutputDiag\] t=([\d.]+) id=(-?\d+) x=([-\d.]+) y=([-\d.]+) "
    r"misses=(\d+) speed=([\d.eE+-]+)")
outputs = []
with open(diag_path) as f:
    for line in f:
        m = pattern.search(line)
        if m:
            outputs.append((
                float(m.group(1)), int(m.group(2)), float(m.group(3)),
                float(m.group(4)), int(m.group(5)), float(m.group(6))))
print(f"parsed {len(outputs)} TrackerOutputDiag lines")

# per-track aggregation
tracks = {}  # id -> {frames, matched_frames, speeds:[], misses0_frames, misses0_matched}
for t, tid, x, y, misses, speed in outputs:
    odom = interpolate_odom(t)
    gt = interpolate_gt(t)
    matched = False
    if odom is not None and gt is not None and gt["poses"]:
        yaw = quat_to_yaw(odom["qx"], odom["qy"], odom["qz"], odom["qw"])
        gt_frame = [world_to_boat(p["x"], p["y"], odom["x"], odom["y"], yaw) for p in gt["poses"]]
        best_d = min(math.hypot(x - gx, y - gy) for gx, gy in gt_frame)
        matched = best_d < MATCH_TOL
    tr = tracks.setdefault(tid, {
        "frames": 0, "matched_frames": 0, "speeds": [],
        "misses0_frames": 0, "misses0_matched": 0,
    })
    tr["frames"] += 1
    tr["speeds"].append(speed)
    if matched:
        tr["matched_frames"] += 1
    if misses == 0:
        tr["misses0_frames"] += 1
        if matched:
            tr["misses0_matched"] += 1

print(f"\n{len(tracks)} distinct track ids observed in output")

ever_matched = []
never_matched = []
for tid, tr in tracks.items():
    med_speed = sorted(tr["speeds"])[len(tr["speeds"]) // 2]
    last_speed = tr["speeds"][-1]
    entry = (tid, tr["frames"], tr["matched_frames"], med_speed, last_speed)
    if tr["matched_frames"] > 0:
        ever_matched.append(entry)
    else:
        never_matched.append(entry)


def summarize(label, entries):
    if not entries:
        print(f"{label}: (none)")
        return
    total_frames = sum(e[1] for e in entries)
    speeds = sorted(e[3] for e in entries)
    n = len(speeds)
    med = speeds[n // 2]
    p25 = speeds[int(n * 0.25)]
    p75 = speeds[int(n * 0.75)]
    print(f"{label}: {len(entries)} tracks, {total_frames} output-frames total")
    print(f"  median-speed median={med:.3f} p25={p25:.3f} p75={p75:.3f} "
          f"min={speeds[0]:.3f} max={speeds[-1]:.3f}")


print()
summarize("ever-matched-GT tracks", ever_matched)
summarize("never-matched-GT tracks", never_matched)

print("\n--- misses==0 bucket, split by track category ---")
m0_ever_total = sum(tracks[tid]["misses0_frames"] for tid, _, _, _, _ in ever_matched)
m0_ever_matched = sum(tracks[tid]["misses0_matched"] for tid, _, _, _, _ in ever_matched)
m0_never_total = sum(tracks[tid]["misses0_frames"] for tid, _, _, _, _ in never_matched)
m0_never_matched = sum(tracks[tid]["misses0_matched"] for tid, _, _, _, _ in never_matched)
print(f"misses=0 frames from ever-matched tracks:  {m0_ever_matched}/{m0_ever_total} = "
      f"{100.0*m0_ever_matched/m0_ever_total if m0_ever_total else 0:.1f}%")
print(f"misses=0 frames from never-matched tracks: {m0_never_matched}/{m0_never_total} = "
      f"{100.0*m0_never_matched/m0_never_total if m0_never_total else 0:.1f}%")
print(f"never-matched tracks contribute {m0_never_total}/{m0_ever_total+m0_never_total} = "
      f"{100.0*m0_never_total/(m0_ever_total+m0_never_total) if (m0_ever_total+m0_never_total) else 0:.1f}% "
      f"of all misses=0 output frames")

# speed threshold scan: if we set min_speed_mps = X, how many never-matched
# track-frames get excluded, and how many ever-matched track-frames get
# collaterally excluded (using each frame's own instantaneous avg_speed)?
print("\n--- backfill vs raw-detection split, by track category ---")
print("(misses==0 output == raw unmodified detector output that frame;")
print(" misses>0 output == extrapolated, would NOT exist with tracker off)")
for label, entries in [("ever-matched", ever_matched), ("never-matched", never_matched)]:
    total = sum(tracks[tid]["frames"] for tid, _, _, _, _ in entries)
    m0 = sum(tracks[tid]["misses0_frames"] for tid, _, _, _, _ in entries)
    bf = total - m0
    print(f"{label}: total={total} raw(misses=0)={m0} ({100.0*m0/total if total else 0:.1f}%) "
          f"backfill(misses>0)={bf} ({100.0*bf/total if total else 0:.1f}%)")

total_backfill = sum(tr["frames"] - tr["misses0_frames"] for tr in tracks.values())
never_matched_backfill = sum(
    tracks[tid]["frames"] - tracks[tid]["misses0_frames"] for tid, _, _, _, _ in never_matched)
print(f"\nnever-matched tracks' backfill frames: {never_matched_backfill}/{total_backfill} = "
      f"{100.0*never_matched_backfill/total_backfill if total_backfill else 0:.1f}% of ALL backfill output")
print("^ this backfill volume would NOT exist if tracker were off (no extrapolation) —")
print("  it is tracker-CREATED FP volume, layered on top of the detector's raw FPs above.")

print("\n--- backfill frames by exact misses count, split by track category ---")
print(f"{'misses':>7} {'ever_matched_frames':>20} {'never_matched_frames':>21} {'never_matched_share':>20}")
ever_ids = {tid for tid, _, _, _, _ in ever_matched}
never_ids = {tid for tid, _, _, _, _ in never_matched}
by_misses = {}
for t, tid, x, y, misses, speed in outputs:
    if misses == 0:
        continue
    cat = "ever" if tid in ever_ids else "never"
    b = by_misses.setdefault(misses, {"ever": 0, "never": 0})
    b[cat] += 1
for misses in sorted(by_misses):
    e, n = by_misses[misses]["ever"], by_misses[misses]["never"]
    tot = e + n
    print(f"{misses:>7} {e:>20} {n:>21} {100.0*n/tot if tot else 0:>19.1f}%")

print("\n--- min_speed_mps threshold scan (frame-level) ---")
print(f"{'thresh':>8} {'excl_never_matched_frames':>26} {'excl_ever_matched_frames':>26}")
for thresh in [0.0, 0.05, 0.1, 0.15, 0.2, 0.3, 0.5, 0.8, 1.0]:
    excl_never = 0
    total_never = 0
    excl_ever = 0
    total_ever = 0
    for t, tid, x, y, misses, speed in outputs:
        tr = tracks[tid]
        if tr["matched_frames"] > 0:
            total_ever += 1
            if speed < thresh:
                excl_ever += 1
        else:
            total_never += 1
            if speed < thresh:
                excl_never += 1
    print(f"{thresh:>8.2f} {excl_never:>10}/{total_never:<10} "
          f"({100.0*excl_never/total_never if total_never else 0:>5.1f}%)   "
          f"{excl_ever:>10}/{total_ever:<10} "
          f"({100.0*excl_ever/total_ever if total_ever else 0:>5.1f}%)")
