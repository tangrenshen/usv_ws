#!/usr/bin/env python3
"""Decomposes position error into radial (toward/away from boat) and
tangential components, per category and per distance bucket. Radial
component = dot(detection - target, unit_vector_from_boat_to_target):
negative means the detection sits closer to the boat than the true target
center (near-face-offset signature); positive means it overshoots away.
Also reports error magnitude vs the detection's own reported size
(fp_max = max(dx,dy) from the MarkerArray scale, now logged), to check
whether the offset scales with target size the way a near-face-offset
compensation would need."""

import argparse
import importlib.util
import math
import statistics
from collections import defaultdict


def load_evaluator(path):
    spec = importlib.util.spec_from_file_location("evaluator_v2", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("log_path")
    parser.add_argument("--evaluator", required=True)
    parser.add_argument("--epoch-offset", type=float, required=True)
    parser.add_argument("--match-tol", type=float, default=2.0)
    return parser.parse_args()


DIST_BUCKETS = [(0.0, 10.0), (10.0, 20.0), (20.0, 40.0), (40.0, 60.0), (60.0, 1e9)]


def bucket_of(d):
    for lo, hi in DIST_BUCKETS:
        if lo <= d < hi:
            return f"{lo:g}-{hi:g}" if hi < 1e8 else f"{lo:g}+"
    return "?"


OMEGA_BUCKETS_DEG = [(0.0, 0.2), (0.2, 0.4), (0.4, 0.6), (0.6, 0.8), (0.8, 1.5)]  # deg/s magnitude


def abs_bucket(omega_rad_s):
    deg = abs(math.degrees(omega_rad_s))
    for lo, hi in OMEGA_BUCKETS_DEG:
        if lo <= deg < hi:
            return f"{lo:g}-{hi:g}" if hi < 1e8 else f"{lo:g}+"
    return "?"


def main():
    args = parse_args()
    evaluator = load_evaluator(args.evaluator)
    records = evaluator.load_records(args.log_path)
    odom_mid = records["odom"][len(records["odom"]) // 2]["stamp"]
    for record_type, items in records.items():
        if record_type not in {"odom", "cluster"}:
            evaluator.align_to_odom(items, args.epoch_offset, odom_mid)

    interpolate_odom = evaluator.make_odom_interpolator(records["odom"])

    # yaw rate at time t: finite difference between the two odom samples
    # bracketing t (same records the interpolator itself uses)
    odom_list = sorted(records["odom"], key=lambda o: o["stamp"])
    odom_ts_arr = [o["stamp"] for o in odom_list]

    def yaw_rate_at(t):
        import bisect
        i = bisect.bisect_left(odom_ts_arr, t)
        if i <= 0 or i >= len(odom_list):
            return None
        o0, o1 = odom_list[i - 1], odom_list[i]
        dt = o1["stamp"] - o0["stamp"]
        if dt <= 1e-6:
            return None
        yaw0 = math.atan2(2.0 * (o0["qw"] * o0["qz"] + o0["qx"] * o0["qy"]),
                           1.0 - 2.0 * (o0["qy"]**2 + o0["qz"]**2))
        yaw1 = math.atan2(2.0 * (o1["qw"] * o1["qz"] + o1["qx"] * o1["qy"]),
                           1.0 - 2.0 * (o1["qy"]**2 + o1["qz"]**2))
        dyaw = math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0))
        return dyaw / dt

    tangential_by_omega_all = defaultdict(list)  # combined across all categories

    for name, gt_key, detection_key in evaluator.CATEGORIES:
        interpolate_gt = evaluator.make_gt_interpolator(records[gt_key])
        detection_frames, _, _ = evaluator.deduplicate_frames(records[detection_key])

        radial_by_bucket = defaultdict(list)
        tangential_by_bucket = defaultdict(list)
        radial_over_size = []
        raw_ex, raw_ey = [], []
        bearings = []
        tangential_by_bearing = defaultdict(list)  # bearing bucket -> tangential list

        for frame in detection_frames:
            odom = interpolate_odom(frame["stamp"])
            gt = interpolate_gt(frame["stamp"])
            if odom is None or gt is None:
                continue
            detections = [(pose["x"], pose["y"]) for pose in frame["poses"]]
            targets = [evaluator.world_to_boat(pose, odom) for pose in gt["poses"]]
            pairs, _ = evaluator.maximum_cardinality_matches(detections, targets, args.match_tol)
            for detection_index, target_index in pairs:
                dx, dy = detections[detection_index]
                tx, ty = targets[target_index]
                target_dist = math.hypot(tx, ty)
                if target_dist < 1e-6:
                    continue
                ux, uy = tx / target_dist, ty / target_dist
                ex, ey = dx - tx, dy - ty
                radial = ex * ux + ey * uy
                tangential = ex * (-uy) + ey * ux
                b = bucket_of(target_dist)
                radial_by_bucket[b].append(radial)
                tangential_by_bucket[b].append(tangential)
                raw_ex.append(ex)
                raw_ey.append(ey)
                bearing_deg = math.degrees(math.atan2(ty, tx))
                bearings.append(bearing_deg)
                # REP103: x-forward, y-left(port), z-up. bearing=atan2(ty,tx):
                # 0=bow, +90=port(left), -90=starboard(right), ±180=stern.
                if -45 <= bearing_deg < 45:
                    bb = "bow(±45°)"
                elif 45 <= bearing_deg < 135:
                    bb = "port(45-135°)"
                elif -135 <= bearing_deg < -45:
                    bb = "starboard(-135..-45°)"
                else:
                    bb = "stern(±180..135°)"
                tangential_by_bearing[bb].append(tangential)

                omega = yaw_rate_at(frame["stamp"])
                if omega is not None and target_dist > 1e-6:
                    # rotation-induced tangential offset predicts:
                    # tangential ~= target_dist * omega * dt_bias (constant dt_bias)
                    # so normalize by distance to get an angle-rate-independent
                    # quantity, and separately bucket by |omega|.
                    tangential_by_omega_all[abs_bucket(omega)].append((tangential, target_dist, omega))

                pose = frame["poses"][detection_index]
                if "dx" in pose and "dy" in pose:
                    size_est = max(pose["dx"], pose["dy"])
                    if size_est > 1e-3:
                        radial_over_size.append(radial / size_est)

        print(f"=== {name} ===")
        for lo, hi in DIST_BUCKETS:
            b = f"{lo:g}-{hi:g}" if hi < 1e8 else f"{lo:g}+"
            rad = radial_by_bucket[b]
            tan = tangential_by_bucket[b]
            if not rad:
                print(f"  {b:>8}: n=0")
                continue
            toward_boat_frac = 100.0 * sum(1 for r in rad if r < 0) / len(rad)
            print(f"  {b:>8}: n={len(rad):>5} "
                  f"radial(mean={statistics.mean(rad):+.3f}m median={statistics.median(rad):+.3f}m "
                  f"toward_boat%={toward_boat_frac:.1f}%) "
                  f"tangential(mean={statistics.mean(tan):+.3f}m |mean|={abs(statistics.mean(tan)):.3f}m, "
                  f"stdev={statistics.pstdev(tan):.3f}m)")
        if radial_over_size:
            print(f"  radial/size ratio: mean={statistics.mean(radial_over_size):+.3f} "
                  f"median={statistics.median(radial_over_size):+.3f} "
                  f"(negative = detection biased toward boat, magnitude relative to detected fp_max)")
        if raw_ex:
            print(f"  raw Cartesian mean error vector (boat frame): "
                  f"ex={statistics.mean(raw_ex):+.3f}m ey={statistics.mean(raw_ey):+.3f}m")
            print(f"  target bearing (deg, atan2(ty,tx), 0=bow, +90=port/left per REP103): "
                  f"min={min(bearings):.1f} max={max(bearings):.1f} mean={statistics.mean(bearings):.1f}")
        print("  tangential by target bearing sector:")
        for bb in ("bow(±45°)", "port(45-135°)", "stern(±180..135°)", "starboard(-135..-45°)"):
            vals = tangential_by_bearing[bb]
            if vals:
                print(f"    {bb:<22}: n={len(vals):>5} mean={statistics.mean(vals):+.3f}m")
            else:
                print(f"    {bb:<22}: n=0")
        print()

    print("=== tangential vs boat yaw rate (all categories combined) ===")
    print("hypothesis: if tangential offset comes from a GT/detection timestamp")
    print("misalignment while the boat is turning, tangential should grow with |omega|,")
    print("and tangential/(distance*omega) should be roughly constant (= the time offset).")
    for lo, hi in OMEGA_BUCKETS_DEG:
        b = f"{lo:g}-{hi:g}" if hi < 1e8 else f"{lo:g}+"
        entries = tangential_by_omega_all[b]
        if not entries:
            print(f"  omega {b:>8} deg/s: n=0")
            continue
        tans = [e[0] for e in entries]
        # floor |omega| at the bucket's own lower bound to avoid the near-zero
        # denominator blowup (a sample right at the bucket's edge with tiny
        # actual omega otherwise dominates the estimate with noise)
        omega_floor = math.radians(lo) if lo > 0 else math.radians(0.2)
        dt_estimates = []
        for tan, dist, omega in entries:
            eff_omega = max(abs(omega), omega_floor)
            if dist > 1.0:
                dt_estimates.append(tan / (dist * eff_omega) * (1 if omega >= 0 else -1))
        dt_str = (f"implied_dt_mean={statistics.mean(dt_estimates)*1000:+.1f}ms "
                  f"implied_dt_median={statistics.median(dt_estimates)*1000:+.1f}ms") if dt_estimates else "n/a"
        print(f"  omega {b:>8} deg/s: n={len(entries):>6} tangential_mean={statistics.mean(tans):+.3f}m "
              f"tangential_median={statistics.median(tans):+.3f}m {dt_str}")


if __name__ == "__main__":
    main()
