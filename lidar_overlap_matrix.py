#!/usr/bin/env python3
import argparse
import json
import math
import statistics

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from scipy.spatial import cKDTree
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
        default=(10.0, 25.0, 40.0, 55.0, 70.0),
    )
    parser.add_argument("--voxel", type=float, default=0.1)
    parser.add_argument(
        "--thresholds", type=float, nargs="+", default=(0.3, 0.5, 0.8)
    )
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


def voxel_centroids(points, voxel):
    cells = np.floor(points / voxel).astype(np.int64)
    unique_cells, inverse = np.unique(cells, axis=0, return_inverse=True)
    sums = np.zeros((len(unique_cells), 3), dtype=np.float64)
    counts = np.bincount(inverse)
    np.add.at(sums, inverse, points)
    return sums / counts[:, None]


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
        if topic not in TOPIC_TO_INDEX:
            continue
        msg = deserialize_message(data, PointCloud2)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        index = TOPIC_TO_INDEX[topic]
        for target in target_stamps:
            gap = abs(stamp - target)
            current = selected[target][index]
            if current is None or gap < current["gap"]:
                selected[target][index] = {
                    "gap": gap,
                    "stamp": stamp,
                    "frame_id": msg.header.frame_id,
                    "data": data,
                }

    return selected


def transform_selected(selected, voxel):
    frames = {}
    metadata = {}
    for index, item in selected.items():
        if item is None:
            raise RuntimeError(f"missing lidar index {index}")
        msg = deserialize_message(item["data"], PointCloud2)
        array = point_cloud2.read_points_numpy(
            msg, field_names=("x", "y", "z"), skip_nans=False
        )
        all_points = np.asarray(array, dtype=np.float64).reshape(-1, 3)
        finite_mask = np.isfinite(all_points).all(axis=1)
        points = all_points[finite_mask]
        downsampled = voxel_centroids(points, voxel)

        _short, _full, x, y, z, roll, pitch, yaw = LIDARS[index]
        transformed = (
            downsampled @ rotation_matrix(roll, pitch, yaw).T
            + np.array([x, y, z])
        )
        frames[index] = transformed
        metadata[index] = {
            "stamp": item["stamp"],
            "target_gap_sec": item["gap"],
            "frame_id": item["frame_id"],
            "message_points": int(len(all_points)),
            "finite_points": int(len(points)),
            "voxel_points": int(len(downsampled)),
        }
    return frames, metadata


def pair_metrics(points_a, points_b, thresholds):
    distances_ab, _ = cKDTree(points_b).query(points_a, k=1, workers=-1)
    distances_ba, _ = cKDTree(points_a).query(points_b, k=1, workers=-1)
    result = {}
    for threshold in thresholds:
        count_ab = int(np.count_nonzero(distances_ab <= threshold))
        count_ba = int(np.count_nonzero(distances_ba <= threshold))
        rate_ab = count_ab / len(points_a) if len(points_a) else 0.0
        rate_ba = count_ba / len(points_b) if len(points_b) else 0.0
        result[str(threshold)] = {
            "count_ab": count_ab,
            "count_ba": count_ba,
            "rate_ab": rate_ab,
            "rate_ba": rate_ba,
            "conservative_count": min(count_ab, count_ba),
            "conservative_rate": min(rate_ab, rate_ba),
        }
    return result


def median(values):
    return float(statistics.median(values))


def main():
    args = parse_args()
    selected_by_stamp = load_frames(args.bag_path, args.stamps)
    per_stamp = {}
    samples_by_pair = {}

    for target_stamp in args.stamps:
        frames, metadata = transform_selected(
            selected_by_stamp[target_stamp], args.voxel
        )
        stamp_result = {
            "metadata": {
                LIDARS[index][0]: item for index, item in metadata.items()
            },
            "pairs": {},
        }
        for index_a in range(len(LIDARS)):
            for index_b in range(index_a + 1, len(LIDARS)):
                name_a = LIDARS[index_a][0]
                name_b = LIDARS[index_b][0]
                key = f"{name_a}<->{name_b}"
                metrics = pair_metrics(
                    frames[index_a],
                    frames[index_b],
                    args.thresholds,
                )
                stamp_result["pairs"][key] = metrics
                samples_by_pair.setdefault(key, []).append(metrics)
        per_stamp[str(target_stamp)] = stamp_result

    aggregate = {}
    for key, samples in samples_by_pair.items():
        aggregate[key] = {}
        for threshold in args.thresholds:
            threshold_key = str(threshold)
            aggregate[key][threshold_key] = {
                metric: median(
                    [sample[threshold_key][metric] for sample in samples]
                )
                for metric in (
                    "count_ab",
                    "count_ba",
                    "rate_ab",
                    "rate_ba",
                    "conservative_count",
                    "conservative_rate",
                )
            }

    report = {
        "bag_path": args.bag_path,
        "target_stamps_sec": args.stamps,
        "voxel_m": args.voxel,
        "thresholds_m": args.thresholds,
        "sensor_order": [item[0] for item in LIDARS],
        "per_stamp": per_stamp,
        "aggregate_median": aggregate,
    }
    with open(args.output, "w", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2)

    names = report["sensor_order"]
    for threshold in args.thresholds:
        threshold_key = str(threshold)
        print(f"\nthreshold={threshold:.1f}m, cell=median conservative overlap % / count")
        print("".ljust(10) + "".join(name.rjust(14) for name in names))
        for row, name_a in enumerate(names):
            cells = []
            for column, name_b in enumerate(names):
                if row == column:
                    cells.append("100.0/--")
                    continue
                first, second = sorted((row, column))
                key = f"{names[first]}<->{names[second]}"
                item = aggregate[key][threshold_key]
                cells.append(
                    f"{100.0 * item['conservative_rate']:.1f}/"
                    f"{item['conservative_count']:.0f}"
                )
            print(name_a.ljust(10) + "".join(cell.rjust(14) for cell in cells))

    ranking_threshold = str(args.thresholds[0])
    ranking = sorted(
        (
            (
                value[ranking_threshold]["conservative_rate"],
                value[ranking_threshold]["conservative_count"],
                key,
            )
            for key, value in aggregate.items()
        ),
        reverse=True,
    )
    print(f"\nPair ranking at {ranking_threshold}m:")
    for rate, count, key in ranking:
        print(f"{key:24s} {100.0 * rate:6.2f}% min_count={count:.0f}")


if __name__ == "__main__":
    main()
