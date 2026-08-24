#!/usr/bin/env python3
"""Offline multi-frame accumulation feasibility analysis. Loads the raw
capture (clouds + odom + static GT), interpolates odom to each cloud's own
timestamp (proper time alignment, not "whatever was latest"), transforms
each frame's near-target points into a COMMON reference frame (the last
frame's base_link) using odom deltas - this is the natural "motion
compensation" baseline, since we have odom, so there's no reason to also
report the truly-naive (uncompensated) case: the target itself moves in
raw base_link between frames purely because the boat moves, which isn't
the same drift question as compensated point spread.

For a handful of known far-range (>20m) static targets, reports: per-frame
point count, cumulative point count under compensated accumulation, and
the accumulated cluster's centroid drift/spread - answering how many
frames it takes to cross min_cluster_pts=4 and each category's own min_pts
threshold.
"""

import json
import math
import statistics
import sys
from collections import defaultdict

PATH = sys.argv[1] if len(sys.argv) > 1 else "multiframe_accum.jsonl"
N_FRAMES_TO_ACCUM = int(sys.argv[2]) if len(sys.argv) > 2 else 5
NEAR_RADIUS = 3.0


def world_to_boat(px, py, ox, oy, oyaw):
    dx, dy = px - ox, py - oy
    c, s = math.cos(-oyaw), math.sin(-oyaw)
    return dx * c - dy * s, dx * s + dy * c


def boat_to_boat(bx, by, ox1, oy1, oyaw1, ox2, oy2, oyaw2):
    """Transform a point given in base_link-at-time-1 into base_link-at-time-2,
    via world frame (uses odom poses at both times)."""
    c1, s1 = math.cos(oyaw1), math.sin(oyaw1)
    wx = ox1 + bx * c1 - by * s1
    wy = oy1 + bx * s1 + by * c1
    return world_to_boat(wx, wy, ox2, oy2, oyaw2)


odoms = []
clouds = []
gt = {"pillar": [], "buoy": [], "block": []}
gt_key_map = {"gt_pillar": "pillar", "gt_buoy": "buoy", "gt_block": "block"}

print("loading...", file=sys.stderr)
with open(PATH) as f:
    for line in f:
        rec = json.loads(line)
        if rec["type"] == "odom":
            odoms.append((rec["t"], rec["x"], rec["y"], rec["yaw"]))
        elif rec["type"] == "cloud":
            clouds.append(rec)
        elif rec["type"] in gt_key_map:
            cat = gt_key_map[rec["type"]]
            if not gt[cat]:
                gt[cat] = rec["poses"]

odoms.sort(key=lambda o: o[0])
odom_ts = [o[0] for o in odoms]
print(f"loaded {len(clouds)} clouds, {len(odoms)} odom, "
      f"gt: pillar={len(gt['pillar'])} buoy={len(gt['buoy'])} block={len(gt['block'])}",
      file=sys.stderr)


def interp_odom(t):
    import bisect
    i = bisect.bisect_left(odom_ts, t)
    if i == 0:
        return odoms[0][1:]
    if i >= len(odoms):
        return odoms[-1][1:]
    t0, x0, y0, yaw0 = odoms[i - 1]
    t1, x1, y1, yaw1 = odoms[i]
    if t1 == t0:
        return x0, y0, yaw0
    r = (t - t0) / (t1 - t0)
    x = x0 + r * (x1 - x0)
    y = y0 + r * (y1 - y0)
    dyaw = math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0))
    yaw = yaw0 + r * dyaw
    return x, y, yaw


clouds.sort(key=lambda c: c["t"])

# pick a handful of far-range (>20m) static targets to analyze
targets = []
if clouds:
    ox0, oy0, oyaw0 = interp_odom(clouds[-1]["t"])
    for cat, poses in gt.items():
        for ti, (px, py) in enumerate(poses):
            bx, by = world_to_boat(px, py, ox0, oy0, oyaw0)
            d = math.hypot(bx, by)
            if 20.0 < d < 65.0:
                targets.append((cat, ti, px, py, d))
targets.sort(key=lambda t: t[4])
targets = targets[:6]
print(f"analyzing {len(targets)} far-range targets: "
      + ", ".join(f"{c}_{i}@{d:.0f}m" for c, i, _, _, d in targets), file=sys.stderr)

for cat, ti, wx, wy, approx_dist in targets:
    print(f"\n=== {cat}_{ti} (~{approx_dist:.1f}m) ===")
    per_frame_counts = []
    per_frame_centroids_ref = []  # in reference (last) frame's base_link
    ref_t = clouds[-1]["t"]
    ox_ref, oy_ref, oyaw_ref = interp_odom(ref_t)
    bx_ref, by_ref = world_to_boat(wx, wy, ox_ref, oy_ref, oyaw_ref)

    cumulative_points = []
    # consecutive frames ending at the reference frame - this is what real
    # accumulation would actually use (the last N frames), not samples
    # scattered across the whole capture window.
    sampled = clouds[-(N_FRAMES_TO_ACCUM * 2):]

    prev_near_set = None
    per_frame_dedup_counts = []
    for cloud in sampled:
        ox, oy, oyaw = interp_odom(cloud["t"])
        bx, by = world_to_boat(wx, wy, ox, oy, oyaw)
        xs, ys, zs = cloud["x"], cloud["y"], cloud["z"]
        near = []
        for x, y, z in zip(xs, ys, zs):
            if math.hypot(x - bx, y - by) <= NEAR_RADIUS:
                near.append((x, y, z))
        near_set = set(near)
        dup = len(near_set & prev_near_set) if prev_near_set is not None else 0
        per_frame_dedup_counts.append(len(near) - dup)
        prev_near_set = near_set
        per_frame_counts.append(len(near))
        if near:
            fcx = sum(p[0] for p in near) / len(near)
            fcy = sum(p[1] for p in near) / len(near)
            per_frame_centroids_ref.append((fcx - bx + bx_ref, fcy - by + by_ref))
        # transform into reference frame for accumulation
        for x, y, z in near:
            tbx, tby = boat_to_boat(x, y, ox, oy, oyaw, ox_ref, oy_ref, oyaw_ref)
            cumulative_points.append((tbx, tby, z))

    nonzero_counts = [c for c in per_frame_counts if c > 0]
    print(f"  frames sampled: {len(sampled)}, frames with any near point: {len(nonzero_counts)}")
    if nonzero_counts:
        print(f"  per-frame point count: min={min(nonzero_counts)} median={statistics.median(nonzero_counts):.0f} "
              f"max={max(nonzero_counts)}")

    if len(cumulative_points) < 2:
        print("  too few points to analyze accumulation")
        continue

    for n in (1, 2, 3, 5, 10):
        pts_n = cumulative_points[:sum(per_frame_counts[:n])] if n <= len(per_frame_counts) else cumulative_points
        # simpler: accumulate frame-by-frame in order
        acc = []
        fi = 0
        idx = 0
        for c in per_frame_counts[:n]:
            acc.extend(cumulative_points[idx:idx + c])
            idx += c
        if not acc:
            print(f"  after {n} frames: 0 points")
            continue
        cx = sum(p[0] for p in acc) / len(acc)
        cy = sum(p[1] for p in acc) / len(acc)
        spread = max(math.hypot(p[0] - cx, p[1] - cy) for p in acc)
        dedup_n = sum(per_frame_dedup_counts[:n]) if n <= len(per_frame_dedup_counts) else sum(per_frame_dedup_counts)
        print(f"  after {n} frames: {len(acc)} points accumulated (compensated), "
              f"spread(max radius from centroid)={spread:.3f}m, "
              f"dedup(exact-match to prev frame only, per-frame stamp not available here)={dedup_n}")

    if len(per_frame_centroids_ref) >= 2:
        max_drift = 0.0
        for i in range(len(per_frame_centroids_ref)):
            for j in range(i + 1, len(per_frame_centroids_ref)):
                d = math.hypot(per_frame_centroids_ref[i][0] - per_frame_centroids_ref[j][0],
                                per_frame_centroids_ref[i][1] - per_frame_centroids_ref[j][1])
                max_drift = max(max_drift, d)
        print(f"  max per-frame centroid drift (compensated, should be near 0 for a true static target): {max_drift:.3f}m")
