#!/usr/bin/env python3
"""Clusters the world-frame positions of pillar's far-range (>=60m)
pure-noise false positives to check whether they concentrate at a small
number of fixed locations (suggesting a real, unlabeled static structure)
versus scattering randomly."""

import argparse
import importlib.util
import math
import json
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
    parser.add_argument("--cluster-radius", type=float, default=3.0)
    parser.add_argument("--out-json", default=None)
    return parser.parse_args()


def greedy_cluster(points, radius):
    """Simple greedy spatial clustering: assign each point to the nearest
    existing cluster centroid if within radius, else start a new cluster."""
    clusters = []  # list of [sum_x, sum_y, count]
    assignments = []
    for x, y in points:
        best = None
        best_d = radius
        for i, (cx, cy, n) in enumerate(clusters):
            d = math.hypot(x - cx / n, y - cy / n)
            if d < best_d:
                best_d = d
                best = i
        if best is None:
            clusters.append([x, y, 1])
            assignments.append(len(clusters) - 1)
        else:
            clusters[best][0] += x
            clusters[best][1] += y
            clusters[best][2] += 1
            assignments.append(best)
    return clusters, assignments


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
    all_gt_world = {
        "boat": records["gt"][0]["poses"] if records["gt"] else [],
        "buoy": records["gt_buoy"][0]["poses"] if records["gt_buoy"] else [],
        "pillar": records["gt_pillar"][0]["poses"] if records["gt_pillar"] else [],
        "block": records["gt_block"][0]["poses"] if records["gt_block"] else [],
    }
    # poses are dicts {"x":..,"y":..}; normalize to plain (x, y) tuples
    for cat in all_gt_world:
        normalized = []
        for p in all_gt_world[cat]:
            if isinstance(p, dict):
                normalized.append((p["x"], p["y"]))
            else:
                normalized.append((p[0], p[1]))
        all_gt_world[cat] = normalized

    detection_frames, _, _ = evaluator.deduplicate_frames(records["det_pillar"])

    world_points = []

    for frame in detection_frames:
        odom = interpolate_odom(frame["stamp"])
        gt = gt_interp["pillar"](frame["stamp"])
        if odom is None or gt is None:
            continue
        detections = [(pose["x"], pose["y"]) for pose in frame["poses"]]
        targets = [evaluator.world_to_boat(pose, odom) for pose in gt["poses"]]
        pairs, _ = evaluator.maximum_cardinality_matches(detections, targets, args.match_tol)
        matched_indices = {i for i, _ in pairs}

        ox, oy, oyaw = odom["x"], odom["y"], evaluator.quat_to_yaw(odom)
        c, s = math.cos(oyaw), math.sin(oyaw)

        for di, det in enumerate(detections):
            dist = math.hypot(det[0], det[1])
            if dist < args.far_min_dist or di in matched_indices:
                continue
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
            wx = ox + det[0] * c - det[1] * s
            wy = oy + det[0] * s + det[1] * c
            world_points.append((wx, wy))

    print(f"total far-range pure-noise pillar FP points: {len(world_points)}")
    clusters, _ = greedy_cluster(world_points, args.cluster_radius)
    clusters.sort(key=lambda c: -c[2])
    print(f"spatial clusters (radius={args.cluster_radius}m): {len(clusters)}")
    print(f"{'rank':>4} {'centroid(x,y)':>22} {'count':>6} {'nearest GT (any cat)':>28}")

    out = []
    for i, (sx, sy, n) in enumerate(clusters[:30]):
        cx, cy = sx / n, sy / n
        best_cat, best_dist = None, 1e9
        for cat, pts in all_gt_world.items():
            for gx, gy in pts:
                d = math.hypot(cx - gx, cy - gy)
                if d < best_dist:
                    best_dist = d
                    best_cat = cat
        print(f"{i+1:>4} ({cx:>8.2f},{cy:>8.2f}) {n:>6} {best_cat}@{best_dist:.1f}m")
        out.append({"x": cx, "y": cy, "count": n, "nearest_gt_cat": best_cat, "nearest_gt_dist": best_dist})

    top10_total = sum(c[2] for c in clusters[:10])
    print(f"\ntop 10 clusters account for {top10_total}/{len(world_points)} "
          f"({100.0*top10_total/len(world_points):.1f}%) of all far-range noise points")

    if args.out_json:
        with open(args.out_json, "w") as f:
            json.dump(out, f, indent=2)
        print(f"wrote top clusters to {args.out_json}")


if __name__ == "__main__":
    main()
