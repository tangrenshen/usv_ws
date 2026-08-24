#!/usr/bin/env python3
"""Compare independent Airy/forward obstacle clusters in their overlap band."""

import argparse
import json
import math
import statistics

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from scipy.spatial import cKDTree
from scipy.optimize import linear_sum_assignment
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


LIDARS = [
    ("front", "front_lidar", 7.0, 0.0, 2.9, 0.0, 115.0, 0.0),
    ("right", "right_lidar", -6.0, -1.626, 2.9, 0.0, 115.0, -90.0),
    ("right2", "right2_lidar", 6.0, -1.626, 2.9, 0.0, 115.0, -90.0),
    ("back", "back_lidar", -7.0, 0.0, 2.9, 0.0, 115.0, 180.0),
    ("left2", "left2_lidar", 6.0, 1.626, 2.9, 0.0, 115.0, 90.0),
    ("left", "left_lidar", -6.0, 1.626, 2.9, 0.0, 115.0, 90.0),
    ("forward", "forward_lidar", 7.0, 0.0, 2.0, 0.0, 0.0, 0.0),
]
SEED0_DELTAS = [
    (0.0928, 0.3443, 0.3579, 6.945, 2.471, -2.312),
    (-0.2025, -0.4433, -0.2273, -0.447, 6.243, -0.400),
    (-0.1072, 0.3361, -0.1626, 2.963, -2.635, 9.143),
    (-0.3596, 0.3701, -0.0264, 6.018, 0.410, 3.578),
    (0.2206, 0.0820, 0.0374, 5.172, -7.882, -0.528),
    (-0.3137, 0.2369, -0.2834, -7.296, -3.517, -7.007),
    (-0.2777, -0.1135, 0.4026, -1.001, 2.261, 8.047),
]
TOPIC_TO_INDEX = {
    f"/wamv/sensors/lidars/{full_name}_sensor/points": index
    for index, (_short, full_name, *_rest) in enumerate(LIDARS)
}


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "bag_path", nargs="?", default="/home/lyf040817/bags/bag_09_03_17"
    )
    parser.add_argument(
        "--stamps",
        type=float,
        nargs="+",
        default=tuple(float(value) for value in range(5, 75, 5)),
    )
    parser.add_argument("--range-min", type=float, default=15.0)
    parser.add_argument("--range-max", type=float, default=25.0)
    parser.add_argument("--voxel", type=float, default=0.2)
    parser.add_argument("--cluster-tol", type=float, default=0.9)
    parser.add_argument("--min-points", type=int, default=8)
    parser.add_argument("--output", required=True)
    return parser.parse_args()


def rotation_matrix(roll_deg, pitch_deg, yaw_deg):
    roll, pitch, yaw = np.deg2rad([roll_deg, pitch_deg, yaw_deg])
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def load_frames(bag_path, target_stamps):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(TOPIC_TO_INDEX)))
    selected = {
        target: {index: None for index in range(len(LIDARS))}
        for target in target_stamps
    }
    while reader.has_next():
        topic, data, _ = reader.read_next()
        msg = deserialize_message(data, PointCloud2)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        index = TOPIC_TO_INDEX[topic]
        for target in target_stamps:
            gap = abs(stamp - target)
            current = selected[target][index]
            if current is None or gap < current["gap"]:
                selected[target][index] = {"gap": gap, "data": data}
    return selected


def raw_points(item):
    msg = deserialize_message(item["data"], PointCloud2)
    points = np.asarray(
        point_cloud2.read_points_numpy(
            msg, field_names=("x", "y", "z"), skip_nans=False
        ),
        dtype=np.float64,
    ).reshape(-1, 3)
    return points[np.isfinite(points).all(axis=1)]


def extrinsics(perturbed):
    output = []
    for index, item in enumerate(LIDARS):
        values = np.array(item[2:], dtype=np.float64)
        if perturbed:
            values += np.array(SEED0_DELTAS[index])
        output.append(values)
    return output


def transform(points, values):
    x, y, z, roll, pitch, yaw = values
    return points @ rotation_matrix(roll, pitch, yaw).T + np.array([x, y, z])


def voxelize(points, voxel):
    if len(points) == 0:
        return points, np.empty(0, dtype=np.int64)
    cells = np.floor(points / voxel).astype(np.int64)
    unique, inverse = np.unique(cells, axis=0, return_inverse=True)
    sums = np.zeros((len(unique), 3), dtype=np.float64)
    counts = np.bincount(inverse)
    np.add.at(sums, inverse, points)
    return sums / counts[:, None], counts


def cluster_centers(points, weights, tolerance, min_points):
    if len(points) == 0:
        return np.empty((0, 3))
    pairs = cKDTree(points).query_pairs(tolerance, output_type="ndarray")
    parent = np.arange(len(points))

    def find(index):
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return index

    for left, right in pairs:
        root_left, root_right = find(left), find(right)
        if root_left != root_right:
            parent[root_right] = root_left

    groups = {}
    for index in range(len(points)):
        groups.setdefault(find(index), []).append(index)

    centers = []
    for indices in groups.values():
        indices = np.asarray(indices)
        total_weight = int(weights[indices].sum())
        if total_weight < min_points or total_weight > 40000:
            continue
        center = np.average(points[indices], axis=0, weights=weights[indices])
        if abs(center[2]) <= 5.0:
            centers.append(center)
    return np.asarray(centers).reshape(-1, 3)


def maximum_matches(points_a, points_b, threshold):
    if not len(points_a) or not len(points_b):
        return []
    distances = np.linalg.norm(
        points_a[:, None, :2] - points_b[None, :, :2], axis=2
    )
    costs = distances.copy()
    costs[costs > threshold] = 1e6
    rows, columns = linear_sum_assignment(costs)
    return [
        (int(row), int(column), float(distances[row, column]))
        for row, column in zip(rows, columns)
        if distances[row, column] <= threshold
    ]


def state_frame(raw_by_sensor, state_extrinsics, args):
    transformed = [
        transform(raw_by_sensor[index], state_extrinsics[index])
        for index in range(len(LIDARS))
    ]
    combined = np.concatenate(transformed)
    radii = np.linalg.norm(combined[:, :2], axis=1)
    crop_mask = (
        (radii <= 130.0)
        & ~(
            (np.abs(combined[:, 0]) < 7.5)
            & (np.abs(combined[:, 1]) < 2.75)
        )
    )
    cropped = combined[crop_mask]
    water_z = float(np.quantile(cropped[:, 2], 0.05))

    centers = []
    for group in (np.concatenate(transformed[:6]), transformed[6]):
        radii = np.linalg.norm(group[:, :2], axis=1)
        mask = (
            (radii >= args.range_min)
            & (radii < args.range_max)
            & (group[:, 2] > water_z + 0.15)
            & (np.abs(group[:, 2]) <= 8.0)
        )
        voxels, weights = voxelize(group[mask], args.voxel)
        centers.append(
            cluster_centers(
                voxels, weights, args.cluster_tol, args.min_points
            )
        )
    forward_values = state_extrinsics[6]
    forward_rotation = rotation_matrix(*forward_values[3:])
    forward_translation = forward_values[:3]
    forward_centers_sensor = (
        (centers[1] - forward_translation) @ forward_rotation
        if len(centers[1]) else np.empty((0, 3))
    )
    return centers[0], centers[1], forward_centers_sensor, water_z


def summarize_state(frames):
    summary = {
        "frames": len(frames),
        "airy_clusters": sum(item["airy_clusters"] for item in frames),
        "forward_clusters": sum(item["forward_clusters"] for item in frames),
    }
    for threshold in (1.0, 2.0, 5.0):
        key = str(threshold)
        per_frame_counts = [
            len(item["matches"][key]) for item in frames
        ]
        distances = [
            match[2]
            for item in frames
            for match in item["matches"][key]
        ]
        summary[f"matches_within_{key}m"] = len(distances)
        summary[f"matches_within_{key}m_per_frame_median"] = (
            statistics.median(per_frame_counts)
        )
        summary[f"frames_with_at_least_2_matches_within_{key}m"] = sum(
            count >= 2 for count in per_frame_counts
        )
        summary[f"frames_with_at_least_3_matches_within_{key}m"] = sum(
            count >= 3 for count in per_frame_counts
        )
    distances_5m = [
        match[2] for item in frames for match in item["matches"]["5.0"]
    ]
    summary["strict_1m_fraction_of_5m_matches"] = (
        summary["matches_within_1.0m"]
        / summary["matches_within_5.0m"]
        if summary["matches_within_5.0m"] else 0.0
    )
    summary["matched_distance_median"] = (
        statistics.median(distances_5m) if distances_5m else None
    )
    summary["matched_distance_p75"] = (
        float(np.quantile(distances_5m, 0.75)) if distances_5m else None
    )
    return summary


def map_cluster_identities(reference, candidate, tolerance):
    if not len(reference) or not len(candidate):
        return {}
    distances = np.linalg.norm(
        candidate[:, None, :] - reference[None, :, :], axis=2
    )
    costs = distances.copy()
    costs[costs > tolerance] = 1e6
    candidate_rows, reference_columns = linear_sum_assignment(costs)
    return {
        int(candidate_index): int(reference_index)
        for candidate_index, reference_index in zip(
            candidate_rows, reference_columns
        )
        if distances[candidate_index, reference_index] <= tolerance
    }


def compare_pair_content(reference_frames, candidate_frames):
    per_frame = []
    for reference, candidate in zip(reference_frames, candidate_frames):
        reference_airy = np.asarray(reference["airy_centers"])
        candidate_airy = np.asarray(candidate["airy_centers"])
        reference_forward = np.asarray(reference["forward_centers_sensor"])
        candidate_forward = np.asarray(candidate["forward_centers_sensor"])
        airy_map = map_cluster_identities(
            reference_airy, candidate_airy, tolerance=0.10
        )
        forward_map = map_cluster_identities(
            reference_forward, candidate_forward, tolerance=0.75
        )

        reference_pairs = {
            (match[0], match[1])
            for match in reference["matches"]["5.0"]
        }
        mapped_candidate_pairs = {
            (airy_map[match[0]], forward_map[match[1]])
            for match in candidate["matches"]["5.0"]
            if match[0] in airy_map and match[1] in forward_map
        }
        intersection = reference_pairs & mapped_candidate_pairs
        per_frame.append(
            {
                "stamp": reference["stamp"],
                "reference_pairs": len(reference_pairs),
                "candidate_pairs": len(candidate["matches"]["5.0"]),
                "mapped_candidate_pairs": len(mapped_candidate_pairs),
                "same_pairs": len(intersection),
                "same_fraction_of_reference": (
                    len(intersection) / len(reference_pairs)
                    if reference_pairs else 0.0
                ),
            }
        )
    reference_total = sum(item["reference_pairs"] for item in per_frame)
    candidate_total = sum(item["candidate_pairs"] for item in per_frame)
    mapped_total = sum(item["mapped_candidate_pairs"] for item in per_frame)
    same_total = sum(item["same_pairs"] for item in per_frame)
    return {
        "summary": {
            "reference_pairs": reference_total,
            "candidate_pairs": candidate_total,
            "mapped_candidate_pairs": mapped_total,
            "same_pairs": same_total,
            "same_fraction_of_reference": (
                same_total / reference_total if reference_total else 0.0
            ),
            "same_fraction_of_mapped_candidate": (
                same_total / mapped_total if mapped_total else 0.0
            ),
            "frames_with_all_reference_pairs_preserved": sum(
                item["same_pairs"] == item["reference_pairs"]
                for item in per_frame
            ),
        },
        "frames": per_frame,
    }


def main():
    args = parse_args()
    selected = load_frames(args.bag_path, args.stamps)
    clean_extrinsics = extrinsics(False)
    seed0_extrinsics = extrinsics(True)
    forward_only_extrinsics = [
        values.copy() for values in clean_extrinsics
    ]
    forward_only_extrinsics[6] = seed0_extrinsics[6].copy()
    forward_yaw_fixed_extrinsics = [
        values.copy() for values in forward_only_extrinsics
    ]
    forward_yaw_fixed_extrinsics[6][5] = clean_extrinsics[6][5]
    states = {
        "clean": clean_extrinsics,
        "forward_only_seed0": forward_only_extrinsics,
        "forward_yaw_fixed": forward_yaw_fixed_extrinsics,
        "seed0_all": seed0_extrinsics,
    }
    report = {"settings": vars(args), "states": {}}

    for state_name, state_extrinsics in states.items():
        frames = []
        for stamp in args.stamps:
            raw_by_sensor = [
                raw_points(selected[stamp][index])
                for index in range(len(LIDARS))
            ]
            airy, forward, forward_sensor, water_z = state_frame(
                raw_by_sensor, state_extrinsics, args
            )
            frames.append(
                {
                    "stamp": stamp,
                    "airy_clusters": len(airy),
                    "forward_clusters": len(forward),
                    "water_z": water_z,
                    "airy_centers": airy.tolist(),
                    "forward_centers_sensor": forward_sensor.tolist(),
                    "matches": {
                        str(threshold): maximum_matches(
                            airy, forward, threshold
                        )
                        for threshold in (1.0, 2.0, 5.0)
                    },
                }
            )
        report["states"][state_name] = {
            "summary": summarize_state(frames),
            "frames": frames,
        }

    clean_frames = report["states"]["clean"]["frames"]
    report["pair_content_comparison"] = {
        state_name: compare_pair_content(
            clean_frames, report["states"][state_name]["frames"]
        )
        for state_name in (
            "forward_only_seed0",
            "forward_yaw_fixed",
            "seed0_all",
        )
    }

    with open(args.output, "w", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2)

    for state_name, state in report["states"].items():
        summary = state["summary"]
        print(
            f"{state_name}: airy_clusters={summary['airy_clusters']} "
            f"forward_clusters={summary['forward_clusters']} "
            f"matches<=1m={summary['matches_within_1.0m']} "
            f"<=2m={summary['matches_within_2.0m']} "
            f"<=5m={summary['matches_within_5.0m']} "
            f"strict_fraction="
            f"{summary['strict_1m_fraction_of_5m_matches']:.3f} "
            f"strict_per_frame_median="
            f"{summary['matches_within_1.0m_per_frame_median']:.1f} "
            f"frames_strict>=2="
            f"{summary['frames_with_at_least_2_matches_within_1.0m']}/"
            f"{summary['frames']} "
            f"matched_distance_median={summary['matched_distance_median']} "
            f"p75={summary['matched_distance_p75']}"
        )
    print("\n5m pair-content identity relative to clean:")
    for state_name, comparison in report["pair_content_comparison"].items():
        summary = comparison["summary"]
        print(
            f"{state_name}: reference={summary['reference_pairs']} "
            f"candidate={summary['candidate_pairs']} "
            f"mapped={summary['mapped_candidate_pairs']} "
            f"same={summary['same_pairs']} "
            f"same/reference={summary['same_fraction_of_reference']:.3f} "
            f"same/mapped={summary['same_fraction_of_mapped_candidate']:.3f} "
            f"all_preserved_frames="
            f"{summary['frames_with_all_reference_pairs_preserved']}/"
            f"{len(args.stamps)}"
        )
        mismatched_frames = [
            frame for frame in comparison["frames"]
            if frame["same_pairs"] != frame["reference_pairs"]
        ]
        if mismatched_frames:
            print(
                "  mismatched_frames: "
                + ", ".join(
                    f"t={frame['stamp']:.0f}s "
                    f"same={frame['same_pairs']}/"
                    f"{frame['reference_pairs']}"
                    for frame in mismatched_frames
                )
            )


if __name__ == "__main__":
    main()
