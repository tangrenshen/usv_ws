#!/usr/bin/env python3
"""Estimate multi-frame accumulation's benefit ceiling for pillar (and buoy/
block as secondary), reusing the existing full-bag capture from the
feasibility check. For each static target beyond a distance threshold:
1) single-frame point count distribution (0 / 1-3 / 4-10 / >10, 1m radius -
   matches the methodology behind the earlier-cited "86.6% zero points"
   pillar figure)
2) accumulated (odom-compensated, deduplicated against the immediately
   preceding frame) point count after N frames, and whether it crosses
   min_cluster_pts=4
3) for targets that do cross, the accumulated bounding box's fp_max/dz/
   slenderness against pillar's own predicates - clustering past the point
   threshold doesn't mean passing classification.
"""

import json
import math
import sys
from collections import defaultdict

PATH = sys.argv[1] if len(sys.argv) > 1 else "outputs/multiframe_accum2/multiframe_accum.jsonl"
DIST_MIN = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0
NEAR_RADIUS = 1.0
N_FRAMES = (5, 10, 15)

MIN_CLUSTER_PTS = 4


def pillar_passes(fp_max, fp_min, dz, n_pts):
    slenderness = dz / max(fp_max, 0.05)
    return fp_max <= 2.0 and dz >= 1.5 and slenderness >= 2.5 and n_pts <= 3000


def block_passes(fp_max, fp_min, dz, n_pts, center_z):
    square = fp_min / max(fp_max, 1e-3)
    return (1.65 <= fp_max <= 3.2 and dz <= 1.5 and square >= 0.55 and
            n_pts >= 10 and abs(center_z) <= 1.0)


def buoy_passes(fp_max, fp_min, dz, n_pts):
    return fp_max <= 1.0 and dz <= 1.5


PREDICATE_FN = {"pillar": pillar_passes, "buoy": buoy_passes, "block": block_passes}


def world_to_boat(px, py, ox, oy, oyaw):
    dx, dy = px - ox, py - oy
    c, s = math.cos(-oyaw), math.sin(-oyaw)
    return dx * c - dy * s, dx * s + dy * c


def boat_to_boat(bx, by, ox1, oy1, oyaw1, ox2, oy2, oyaw2):
    c1, s1 = math.cos(oyaw1), math.sin(oyaw1)
    wx = ox1 + bx * c1 - by * s1
    wy = oy1 + bx * s1 + by * c1
    return world_to_boat(wx, wy, ox2, oy2, oyaw2)


odoms, clouds = [], []
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
clouds.sort(key=lambda c: c["t"])
print(f"loaded {len(clouds)} clouds, {len(odoms)} odom, "
      f"gt: pillar={len(gt['pillar'])} buoy={len(gt['buoy'])} block={len(gt['block'])}", file=sys.stderr)


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
    return x0 + r * (x1 - x0), y0 + r * (y1 - y0), yaw0 + r * dyaw


ref_t = clouds[-1]["t"]
ox_ref, oy_ref, oyaw_ref = interp_odom(ref_t)

# sample distance/point-count over MANY frames across the whole run (not
# just the last one) - a target's single-frame point count varies a lot
# with viewing geometry, so one snapshot isn't representative.
SAMPLE_STRIDE = max(1, len(clouds) // 60)
sample_frames = clouds[::SAMPLE_STRIDE]

for cat, min_dist in (("pillar", DIST_MIN), ("buoy", 40.0), ("block", 20.0)):
    poses = gt[cat]

    print(f"\n########## {cat}: distance>= {min_dist:.0f}m, sampled across {len(sample_frames)} frames spanning the run ##########")
    if not poses:
        continue

    # for each target, collect (frame, dist, single_frame_point_count) for
    # every sampled frame where dist >= min_dist
    per_target_far_samples = defaultdict(list)
    for cloud in sample_frames:
        ox, oy, oyaw = interp_odom(cloud["t"])
        xs, ys, zs = cloud["x"], cloud["y"], cloud["z"]
        for ti, (px, py) in enumerate(poses):
            bx, by = world_to_boat(px, py, ox, oy, oyaw)
            d = math.hypot(bx, by)
            if d < min_dist:
                continue
            cnt = sum(1 for x, y in zip(xs, ys) if math.hypot(x - bx, y - by) <= NEAR_RADIUS)
            per_target_far_samples[ti].append((cloud["t"], d, cnt))

    if not per_target_far_samples:
        print("  no target ever exceeds this distance in the sampled frames")
        continue

    buckets = {"0": 0, "1-3": 0, "4-10": 0, ">10": 0}
    total_far_samples = 0
    for ti, samples in per_target_far_samples.items():
        for _, _, cnt in samples:
            total_far_samples += 1
            if cnt == 0:
                buckets["0"] += 1
            elif cnt <= 3:
                buckets["1-3"] += 1
            elif cnt <= 10:
                buckets["4-10"] += 1
            else:
                buckets[">10"] += 1

    print(f"  {len(per_target_far_samples)}/{len(poses)} distinct targets observed >= {min_dist:.0f}m at some point; "
          f"{total_far_samples} (target,frame) samples total")
    print(f"  per-(target,frame) point count distribution (radius={NEAR_RADIUS}m):")
    for k in ("0", "1-3", "4-10", ">10"):
        print(f"    {k:>5}: {buckets[k]:>5} ({100.0*buckets[k]/total_far_samples:.1f}%)")

    # pick, for each target, its most-recent far-range sample as the anchor
    # for the accumulation check (closest available proxy to "the last time
    # before it possibly moved into <min_dist or out of view")
    far_targets = []
    for ti, samples in per_target_far_samples.items():
        samples.sort(key=lambda s: s[0])
        anchor_t, anchor_d, anchor_cnt = samples[-1]
        px, py = poses[ti]
        far_targets.append((ti, px, py, anchor_d, anchor_t, anchor_cnt))

    accum_results = []
    for ti, wx, wy, dist, anchor_t, single in far_targets:
        # accumulate the N frames in `clouds` ending at/near anchor_t
        import bisect
        cloud_ts = [c["t"] for c in clouds]
        end_idx = bisect.bisect_right(cloud_ts, anchor_t)
        sampled = clouds[max(0, end_idx - max(N_FRAMES)):end_idx]
        if not sampled:
            continue
        ox_ref, oy_ref, oyaw_ref = interp_odom(sampled[-1]["t"])
        prev_set = None
        acc_pts_by_n = {n: [] for n in N_FRAMES}
        running = []
        for fi, c in enumerate(sampled):
            ox, oy, oyaw = interp_odom(c["t"])
            fbx, fby = world_to_boat(wx, wy, ox, oy, oyaw)
            near = [(x, y, z) for x, y, z in zip(c["x"], c["y"], c["z"])
                    if math.hypot(x - fbx, y - fby) <= NEAR_RADIUS]
            near_set = set(near)
            if prev_set is not None:
                near = [p for p in near if p not in prev_set]
            prev_set = near_set
            for x, y, z in near:
                tbx, tby = boat_to_boat(x, y, ox, oy, oyaw, ox_ref, oy_ref, oyaw_ref)
                running.append((tbx, tby, z))
            frames_so_far = fi + 1
            for n in N_FRAMES:
                if frames_so_far == n:
                    acc_pts_by_n[n] = list(running)

        result = {"ti": ti, "dist": dist, "single": single}
        for n in N_FRAMES:
            pts = acc_pts_by_n[n]
            result[f"n{n}_count"] = len(pts)
            if len(pts) >= MIN_CLUSTER_PTS:
                xs_ = [p[0] for p in pts]; ys_ = [p[1] for p in pts]; zs_ = [p[2] for p in pts]
                dx = max(xs_) - min(xs_); dy = max(ys_) - min(ys_); dz = max(zs_) - min(zs_)
                fp_max = max(dx, dy); fp_min = min(dx, dy)
                cz = sum(zs_) / len(zs_)
                if cat == "block":
                    passes = block_passes(fp_max, fp_min, dz, len(pts), cz)
                elif cat == "buoy":
                    passes = buoy_passes(fp_max, fp_min, dz, len(pts))
                else:
                    passes = pillar_passes(fp_max, fp_min, dz, len(pts))
                result[f"n{n}_bbox"] = (fp_max, dz, passes)
            else:
                result[f"n{n}_bbox"] = None
        accum_results.append(result)

    total = len(far_targets)
    print(f"  accumulation check anchored on each target's most recent far-range observation "
          f"({total} distinct targets):")
    for n in N_FRAMES:
        crossed = sum(1 for r in accum_results if r[f"n{n}_count"] >= MIN_CLUSTER_PTS)
        passed_pred = sum(1 for r in accum_results if r[f"n{n}_bbox"] and r[f"n{n}_bbox"][2])
        was_zero_or_sparse = sum(1 for r in accum_results if r["single"] <= 3)
        crossed_from_sparse = sum(1 for r in accum_results if r["single"] <= 3 and r[f"n{n}_count"] >= MIN_CLUSTER_PTS)
        print(f"  after {n} frames: crosses min_cluster_pts={crossed}/{total} ({100.0*crossed/total:.1f}%), "
              f"of which also passes pillar predicates={passed_pred} "
              f"(of the {was_zero_or_sparse} originally-sparse(<=3pt) targets, "
              f"{crossed_from_sparse} cross min_cluster_pts)")
