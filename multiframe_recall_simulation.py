#!/usr/bin/env python3
"""Unconditional recall-improvement estimate for multi-frame pillar
accumulation, addressing the anchor-selection bias in the earlier
per-target "most recent sighting" check (that answered "can accumulation
detect a pillar once it becomes visible", a conditional question - this
answers "how much does recall actually improve across the whole run",
which is what the scoring rule's frame-by-frame, no-distance-exemption
denominator actually asks for).

For a stride-sampled set of frames across the whole capture, and for every
pillar within max_range at that frame (the scoring denominator - present,
whether or not it has any returns), computes single-frame vs N-frame
(deduplicated, odom-compensated) cluster-and-classify success, and reports
the aggregate recall for each - no per-target anchor selection, no
conditioning on visibility.
"""

import json
import math
import sys
import numpy as np

PATH = sys.argv[1] if len(sys.argv) > 1 else "outputs/multiframe_accum2/multiframe_accum.jsonl"
N_ACCUM = int(sys.argv[2]) if len(sys.argv) > 2 else 10
FRAME_STRIDE = int(sys.argv[3]) if len(sys.argv) > 3 else 4
NEAR_RADIUS = 1.0
MIN_CLUSTER_PTS = 4
MAX_RANGE = 130.0


def world_to_boat_np(px, py, ox, oy, oyaw):
    dx, dy = px - ox, py - oy
    c, s = math.cos(-oyaw), math.sin(-oyaw)
    return dx * c - dy * s, dx * s + dy * c


def pillar_passes(fp_max, dz, n_pts):
    slenderness = dz / max(fp_max, 0.05)
    return fp_max <= 2.0 and dz >= 1.5 and slenderness >= 2.5 and n_pts <= 3000


odoms, clouds = [], []
gt_pillar = []

print("loading...", file=sys.stderr)
with open(PATH) as f:
    for line in f:
        rec = json.loads(line)
        if rec["type"] == "odom":
            odoms.append((rec["t"], rec["x"], rec["y"], rec["yaw"]))
        elif rec["type"] == "cloud":
            clouds.append(rec)
        elif rec["type"] == "gt_pillar" and not gt_pillar:
            gt_pillar = rec["poses"]

odoms.sort(key=lambda o: o[0])
odom_ts = np.array([o[0] for o in odoms])
odom_arr = np.array([[o[1], o[2], o[3]] for o in odoms])
clouds.sort(key=lambda c: c["t"])
print(f"loaded {len(clouds)} clouds, {len(odoms)} odom, {len(gt_pillar)} pillars", file=sys.stderr)

# preconvert clouds to numpy arrays once
cloud_np = []
for c in clouds:
    cloud_np.append({
        "t": c["t"],
        "xy": np.array(list(zip(c["x"], c["y"]))),
        "z": np.array(c["z"]),
        "xyz_set": None,  # computed lazily for dedup
    })


def interp_odom(t):
    i = np.searchsorted(odom_ts, t)
    if i <= 0:
        return odom_arr[0]
    if i >= len(odom_ts):
        return odom_arr[-1]
    t0, t1 = odom_ts[i - 1], odom_ts[i]
    if t1 == t0:
        return odom_arr[i - 1]
    r = (t - t0) / (t1 - t0)
    x0, y0, yaw0 = odom_arr[i - 1]
    x1, y1, yaw1 = odom_arr[i]
    x = x0 + r * (x1 - x0)
    y = y0 + r * (y1 - y0)
    dyaw = math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0))
    return np.array([x, y, yaw0 + r * dyaw])


gt_arr = np.array(gt_pillar)  # (n_pillar, 2) world xy

frame_indices = list(range(N_ACCUM, len(cloud_np), FRAME_STRIDE))
print(f"simulating over {len(frame_indices)} sampled frames (stride={FRAME_STRIDE}), "
      f"accumulating last {N_ACCUM} frames each", file=sys.stderr)

total_in_range = 0
total_single_hit = 0
total_accum_hit = 0

for count, fi in enumerate(frame_indices):
    ref = cloud_np[fi]
    ox, oy, oyaw = interp_odom(ref["t"])
    bx = (gt_arr[:, 0] - ox) * math.cos(-oyaw) - (gt_arr[:, 1] - oy) * math.sin(-oyaw)
    by = (gt_arr[:, 0] - ox) * math.sin(-oyaw) + (gt_arr[:, 1] - oy) * math.cos(-oyaw)
    dist = np.hypot(bx, by)
    in_range_mask = dist <= MAX_RANGE
    in_range_idx = np.where(in_range_mask)[0]
    if len(in_range_idx) == 0:
        continue
    total_in_range += len(in_range_idx)

    # single-frame: points in ref frame only
    ref_xy = ref["xy"]
    ref_z = ref["z"]
    for ti in in_range_idx:
        d = np.hypot(ref_xy[:, 0] - bx[ti], ref_xy[:, 1] - by[ti])
        near_mask = d <= NEAR_RADIUS
        n = int(near_mask.sum())
        hit = False
        if n >= MIN_CLUSTER_PTS:
            xs = ref_xy[near_mask, 0]; ys = ref_xy[near_mask, 1]; zs = ref_z[near_mask]
            fp_max = max(xs.max() - xs.min(), ys.max() - ys.min())
            dz = float(zs.max() - zs.min())
            hit = pillar_passes(fp_max, dz, n)
        if hit:
            total_single_hit += 1

    # accumulated: last N_ACCUM frames incl. ref, deduplicated vs immediately
    # preceding frame, transformed into ref frame's base_link
    window = cloud_np[fi - N_ACCUM + 1: fi + 1]
    acc_xy = {ti: [] for ti in in_range_idx}
    acc_z = {ti: [] for ti in in_range_idx}
    prev_sets = {ti: None for ti in in_range_idx}
    for c in window:
        cox, coy, coyaw = interp_odom(c["t"])
        cbx = (gt_arr[in_range_idx, 0] - cox) * math.cos(-coyaw) - (gt_arr[in_range_idx, 1] - coy) * math.sin(-coyaw)
        cby = (gt_arr[in_range_idx, 0] - cox) * math.sin(-coyaw) + (gt_arr[in_range_idx, 1] - coy) * math.cos(-coyaw)
        c1, s1 = math.cos(coyaw), math.sin(coyaw)
        for k, ti in enumerate(in_range_idx):
            d = np.hypot(c["xy"][:, 0] - cbx[k], c["xy"][:, 1] - cby[k])
            near_mask = d <= NEAR_RADIUS
            pts = list(zip(c["xy"][near_mask, 0].tolist(), c["xy"][near_mask, 1].tolist(), c["z"][near_mask].tolist()))
            pset = set(pts)
            if prev_sets[ti] is not None:
                pts = [p for p in pts if p not in prev_sets[ti]]
            prev_sets[ti] = pset
            for x, y, z in pts:
                wx = cox + x * c1 - y * s1
                wy = coy + x * s1 + y * c1
                tbx = (wx - ox) * math.cos(-oyaw) - (wy - oy) * math.sin(-oyaw)
                tby = (wx - ox) * math.sin(-oyaw) + (wy - oy) * math.cos(-oyaw)
                acc_xy[ti].append((tbx, tby))
                acc_z[ti].append(z)

    for ti in in_range_idx:
        pts_xy = acc_xy[ti]
        n = len(pts_xy)
        hit = False
        if n >= MIN_CLUSTER_PTS:
            xs = [p[0] for p in pts_xy]; ys = [p[1] for p in pts_xy]; zs = acc_z[ti]
            fp_max = max(max(xs) - min(xs), max(ys) - min(ys))
            dz = max(zs) - min(zs)
            hit = pillar_passes(fp_max, dz, n)
        if hit:
            total_accum_hit += 1

    if (count + 1) % 20 == 0:
        print(f"  progress: {count+1}/{len(frame_indices)} frames, "
              f"single_recall={100.0*total_single_hit/total_in_range:.1f}% "
              f"accum_recall={100.0*total_accum_hit/total_in_range:.1f}%", file=sys.stderr)

print(f"\n=== unconditional pillar recall, single-frame vs {N_ACCUM}-frame accumulation ===")
print(f"total (target,frame) in-range samples: {total_in_range}")
print(f"single-frame recall: {total_single_hit}/{total_in_range} = {100.0*total_single_hit/total_in_range:.2f}%")
print(f"accumulated recall:  {total_accum_hit}/{total_in_range} = {100.0*total_accum_hit/total_in_range:.2f}%")
print(f"delta: {100.0*(total_accum_hit-total_single_hit)/total_in_range:+.2f}pp")
