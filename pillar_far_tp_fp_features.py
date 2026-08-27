#!/usr/bin/env python3
"""Compares pillar TP vs pure-noise-FP candidates beyond 60m across several
features (pts, dz, slenderness, fp_max, forward_ratio, temporal stability)
to see if any cleanly separates real detections from noise - avoiding a
blind recall/FP tradeoff if a discriminating feature exists."""

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
    parser.add_argument("--far-min-dist", type=float, default=60.0)
    parser.add_argument("--temporal-radius", type=float, default=1.0)
    return parser.parse_args()


def main():
    args = parse_args()
    evaluator = load_evaluator(args.evaluator)
    records = evaluator.load_records(args.log_path)
    odom_mid = records["odom"][len(records["odom"]) // 2]["stamp"]
    for record_type, items in records.items():
        if record_type not in {"odom", "cluster"}:
            evaluator.align_to_odom(items, args.epoch_offset, odom_mid)

    interpolate_odom = evaluator.make_odom_interpolator(records["odom"])
    gt_interp = {
        "boat": evaluator.make_gt_interpolator(records["gt"]),
        "buoy": evaluator.make_gt_interpolator(records["gt_buoy"]),
        "pillar": evaluator.make_gt_interpolator(records["gt_pillar"]),
        "block": evaluator.make_gt_interpolator(records["gt_block"]),
    }

    clusters_by_stamp = defaultdict(list)
    for cluster in records["cluster"]:
        if cluster["label"] not in ("pillar",):
            continue
        stamp = round(cluster["t"] - args.epoch_offset, 6)
        clusters_by_stamp[stamp].append(cluster)

    detection_frames, _, _ = evaluator.deduplicate_frames(records["det_pillar"])
    detection_frames.sort(key=lambda f: f["stamp"])

    # build per-frame world-frame detection positions, for temporal stability check
    frame_world_positions = {}  # frame index -> list of (wx, wy)
    for fi, frame in enumerate(detection_frames):
        odom = interpolate_odom(frame["stamp"])
        if odom is None:
            frame_world_positions[fi] = []
            continue
        ox, oy, oyaw = odom["x"], odom["y"], evaluator.quat_to_yaw(odom)
        c, s = math.cos(oyaw), math.sin(oyaw)
        positions = []
        for pose in frame["poses"]:
            wx = ox + pose["x"] * c - pose["y"] * s
            wy = oy + pose["x"] * s + pose["y"] * c
            positions.append((wx, wy))
        frame_world_positions[fi] = positions

    tp_features = defaultdict(list)
    fp_features = defaultdict(list)
    tp_n, fp_n = 0, 0

    for fi, frame in enumerate(detection_frames):
        odom = interpolate_odom(frame["stamp"])
        gt = gt_interp["pillar"](frame["stamp"])
        if odom is None or gt is None:
            continue
        detections = [(pose["x"], pose["y"]) for pose in frame["poses"]]
        targets = [evaluator.world_to_boat(pose, odom) for pose in gt["poses"]]
        pairs, _ = evaluator.maximum_cardinality_matches(detections, targets, args.match_tol)
        matched_indices = {i for i, _ in pairs}

        available = clusters_by_stamp.get(round(frame["stamp"], 6), [])
        used = set()

        ox, oy, oyaw = odom["x"], odom["y"], evaluator.quat_to_yaw(odom)
        c, s = math.cos(oyaw), math.sin(oyaw)

        for di, det in enumerate(detections):
            dist = math.hypot(det[0], det[1])
            if dist < args.far_min_dist:
                continue
            is_tp = di in matched_indices
            if not is_tp:
                # must be pure noise (not matching any other category)
                is_pure_noise = True
                for cat, interp in gt_interp.items():
                    if cat == "pillar":
                        continue
                    other_gt = interp(frame["stamp"])
                    if other_gt is None:
                        continue
                    other_targets = [evaluator.world_to_boat(p, odom) for p in other_gt["poses"]]
                    if any(math.hypot(det[0]-tx, det[1]-ty) <= args.match_tol for tx, ty in other_targets):
                        is_pure_noise = False
                        break
                if not is_pure_noise:
                    continue

            # find matching cluster record for geometry features
            options = []
            for ci, cl in enumerate(available):
                if ci in used:
                    continue
                d = math.hypot(cl["x"] - det[0], cl["y"] - det[1])
                if d <= 0.05:
                    options.append((d, ci))
            cl = None
            if options:
                _, ci = min(options)
                used.add(ci)
                cl = available[ci]

            # temporal stability: is a detection present within temporal_radius
            # (world frame) in both the immediately preceding and following frame?
            wx = ox + det[0] * c - det[1] * s
            wy = oy + det[0] * s + det[1] * c
            prev_hit = any(math.hypot(wx-px, wy-py) <= args.temporal_radius
                            for px, py in frame_world_positions.get(fi-1, []))
            next_hit = any(math.hypot(wx-px, wy-py) <= args.temporal_radius
                            for px, py in frame_world_positions.get(fi+1, []))
            stable = prev_hit and next_hit

            bucket = tp_features if is_tp else fp_features
            if is_tp:
                tp_n += 1
            else:
                fp_n += 1
            bucket["stable"].append(1 if stable else 0)
            if cl is not None:
                bucket["pts"].append(cl["pts"])
                bucket["dz"].append(cl["dz"])
                bucket["fp_max"].append(cl["fp_max"])
                slenderness = cl["dz"] / max(cl["fp_max"], 0.05)
                bucket["slenderness"].append(slenderness)
                if cl.get("forward_ratio") is not None:
                    bucket["forward_ratio"].append(cl["forward_ratio"])

    def summarize(bucket, n, label):
        print(f"  {label} (n={n}):")
        for key in ("pts", "dz", "fp_max", "slenderness", "forward_ratio"):
            vals = bucket[key]
            if vals:
                print(f"    {key:<12}: n={len(vals):>5} mean={statistics.mean(vals):.3f} "
                      f"median={statistics.median(vals):.3f}")
        stable_frac = 100.0 * sum(bucket["stable"]) / len(bucket["stable"]) if bucket["stable"] else 0.0
        print(f"    {'temporal_stable':<12}: {stable_frac:.1f}% (present in both neighbor frames)")

    print(f"=== pillar >= {args.far_min_dist:g}m: TP vs pure-noise-FP feature comparison ===")
    summarize(tp_features, tp_n, "TP")
    summarize(fp_features, fp_n, "FP(pure noise)")


if __name__ == "__main__":
    main()
