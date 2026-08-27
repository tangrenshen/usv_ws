#!/usr/bin/env python3
"""dt-shift scan for position error: for each candidate dt, re-sample the
odom pose at (detection_stamp + dt) instead of detection_stamp when
projecting GT into base_link, re-match, and report the mean position
error. If a single dt minimizes error consistently across all four
categories, that's a real timestamp misalignment - same method that
previously found and confirmed the ~8-9s header.stamp lag at the
recall/match level. This time restricted to turning frames (|yaw rate|
above a threshold) to concentrate the signal, since straight-line frames
show ~zero tangential offset already."""

import argparse
import importlib.util
import math
import statistics
import bisect
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
    parser.add_argument("--omega-min-deg", type=float, default=0.5,
                         help="only include frames with |yaw rate| >= this many deg/s")
    return parser.parse_args()


import os
DT_CANDIDATES = [float(x) for x in os.environ.get(
    "DT_CANDIDATES", "-3.0,-2.0,-1.0,-0.5,-0.2,-0.1,0.0,0.1,0.2,0.5,1.0,2.0,3.0").split(",")]


def main():
    args = parse_args()
    evaluator = load_evaluator(args.evaluator)
    records = evaluator.load_records(args.log_path)
    odom_mid = records["odom"][len(records["odom"]) // 2]["stamp"]
    for record_type, items in records.items():
        if record_type not in {"odom", "cluster"}:
            evaluator.align_to_odom(items, args.epoch_offset, odom_mid)

    interpolate_odom = evaluator.make_odom_interpolator(records["odom"])

    odom_list = sorted(records["odom"], key=lambda o: o["stamp"])
    odom_ts_arr = [o["stamp"] for o in odom_list]

    def yaw_of(o):
        return math.atan2(2.0 * (o["qw"] * o["qz"] + o["qx"] * o["qy"]),
                           1.0 - 2.0 * (o["qy"] ** 2 + o["qz"] ** 2))

    def yaw_rate_at(t):
        i = bisect.bisect_left(odom_ts_arr, t)
        if i <= 0 or i >= len(odom_list):
            return None
        o0, o1 = odom_list[i - 1], odom_list[i]
        dt = o1["stamp"] - o0["stamp"]
        if dt <= 1e-6:
            return None
        dyaw = math.atan2(math.sin(yaw_of(o1) - yaw_of(o0)), math.cos(yaw_of(o1) - yaw_of(o0)))
        return dyaw / dt

    omega_min = math.radians(args.omega_min_deg)

    # pre-filter: which frame stamps (by category detection_key) are "turning"
    results_by_dt = {dt: defaultdict(list) for dt in DT_CANDIDATES}
    per_category_best = {}

    for name, gt_key, detection_key in evaluator.CATEGORIES:
        interpolate_gt = evaluator.make_gt_interpolator(records[gt_key])
        detection_frames, _, _ = evaluator.deduplicate_frames(records[detection_key])

        turning_frames = []
        for frame in detection_frames:
            omega = yaw_rate_at(frame["stamp"])
            if omega is not None and abs(omega) >= omega_min:
                turning_frames.append(frame)

        print(f"{name}: {len(turning_frames)}/{len(detection_frames)} frames pass |omega|>={args.omega_min_deg}deg/s",
              flush=True)

        cat_errs_by_dt = {}
        for dt in DT_CANDIDATES:
            errs = []
            for frame in turning_frames:
                odom = interpolate_odom(frame["stamp"] + dt)
                gt = interpolate_gt(frame["stamp"])
                if odom is None or gt is None:
                    continue
                detections = [(pose["x"], pose["y"]) for pose in frame["poses"]]
                targets = [evaluator.world_to_boat(pose, odom) for pose in gt["poses"]]
                pairs, _ = evaluator.maximum_cardinality_matches(detections, targets, args.match_tol)
                for detection_index, target_index in pairs:
                    dx, dy = detections[detection_index]
                    tx, ty = targets[target_index]
                    errs.append(math.hypot(dx - tx, dy - ty))
            cat_errs_by_dt[dt] = errs
            results_by_dt[dt][name] = errs

        print(f"  {name}: mean position error by dt(s):")
        best_dt, best_mean = None, 1e9
        for dt in DT_CANDIDATES:
            errs = cat_errs_by_dt[dt]
            m = statistics.mean(errs) if errs else float("nan")
            n = len(errs)
            marker = ""
            if errs and m < best_mean:
                best_mean, best_dt = m, dt
            print(f"    dt={dt:+.1f}s: n={n:>5} mean_err={m:.3f}m")
        per_category_best[name] = (best_dt, best_mean)
        print(f"  {name}: best dt = {best_dt:+.1f}s (mean_err={best_mean:.3f}m)\n", flush=True)

    print("=== summary: best dt per category ===")
    for name, (bd, bm) in per_category_best.items():
        print(f"  {name}: best_dt={bd:+.1f}s mean_err_at_best={bm:.3f}m")

    print("\n=== combined across all categories ===")
    for dt in DT_CANDIDATES:
        all_errs = []
        for name in results_by_dt[dt]:
            all_errs.extend(results_by_dt[dt][name])
        if all_errs:
            print(f"  dt={dt:+.1f}s: n={len(all_errs):>6} mean_err={statistics.mean(all_errs):.3f}m "
                  f"median={statistics.median(all_errs):.3f}m")


if __name__ == "__main__":
    main()
