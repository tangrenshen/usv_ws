#!/usr/bin/env python3
import json
import math
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from scipy.spatial import cKDTree
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


LIDARS = [
    ("front_lidar", 7.0, 0.0, 2.9, 0.0, 115.0, 0.0),
    ("right_lidar", -6.0, -1.626, 2.9, 0.0, 115.0, -90.0),
    ("right2_lidar", 6.0, -1.626, 2.9, 0.0, 115.0, -90.0),
    ("back_lidar", -7.0, 0.0, 2.9, 0.0, 115.0, 180.0),
    ("left2_lidar", 6.0, 1.626, 2.9, 0.0, 115.0, 90.0),
    ("left_lidar", -6.0, 1.626, 2.9, 0.0, 115.0, 90.0),
    ("forward_lidar", 7.0, 0.0, 2.0, 0.0, 0.0, 0.0),
]
TOPIC_TO_INDEX = {
    f"/wamv/sensors/lidars/{name}_sensor/points": index
    for index, (name, *_rest) in enumerate(LIDARS)
}


def rotation_matrix(roll_deg, pitch_deg, yaw_deg):
    roll, pitch, yaw = np.deg2rad([roll_deg, pitch_deg, yaw_deg])
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def voxel_downsample(points, voxel):
    cells = np.floor(points / voxel).astype(np.int64)
    _, unique_indices = np.unique(cells, axis=0, return_index=True)
    return points[np.sort(unique_indices)]


def main():
    bag_path = sys.argv[1] if len(sys.argv) > 1 else "/home/lyf040817/bags/bag_09_03_17"
    target_stamp = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(TOPIC_TO_INDEX)))

    selected = {}
    headers = {}
    while reader.has_next() and len(selected) < len(LIDARS):
        topic, data, _ = reader.read_next()
        if topic not in TOPIC_TO_INDEX:
            continue
        msg = deserialize_message(data, PointCloud2)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        index = TOPIC_TO_INDEX[topic]
        if stamp < target_stamp or index in selected:
            continue
        array = point_cloud2.read_points_numpy(
            msg, field_names=("x", "y", "z"), skip_nans=False
        )
        points = np.asarray(array, dtype=np.float64).reshape(-1, 3)
        total_points = len(points)
        nan_points = int(np.count_nonzero(np.isnan(points).any(axis=1)))
        inf_points = int(np.count_nonzero(np.isinf(points).any(axis=1)))
        points = points[np.isfinite(points).all(axis=1)]
        selected[index] = points
        headers[index] = {
            "stamp": stamp,
            "frame_id": msg.header.frame_id,
            "message_points": int(total_points),
            "nan_points": nan_points,
            "inf_points": inf_points,
            "raw_points": int(len(points)),
        }

    if len(selected) != len(LIDARS):
        raise RuntimeError(f"only found {len(selected)}/{len(LIDARS)} lidar frames")

    transformed = {}
    for index, (name, x, y, z, roll, pitch, yaw) in enumerate(LIDARS):
        downsampled = voxel_downsample(selected[index], 0.1)
        rotation = rotation_matrix(roll, pitch, yaw)
        transformed[index] = downsampled @ rotation.T + np.array([x, y, z])
        headers[index]["voxel_points"] = int(len(downsampled))

    thresholds = (0.3, 0.8, 1.5, 2.0, 3.0)
    per_sensor = {}
    pairwise = {}
    for index, (name, *_rest) in enumerate(LIDARS):
        source = transformed[index]
        target = np.concatenate(
            [points for other, points in transformed.items() if other != index],
            axis=0,
        )
        distances, _ = cKDTree(target).query(source, k=1, workers=-1)
        per_sensor[name] = {
            **headers[index],
            "nearest_distance_median_m": float(np.median(distances)),
            "nearest_distance_p10_m": float(np.percentile(distances, 10)),
            "nearest_distance_p90_m": float(np.percentile(distances, 90)),
            "matches": {
                str(threshold): int(np.count_nonzero(distances <= threshold))
                for threshold in thresholds
            },
        }

        for other_index in range(index + 1, len(LIDARS)):
            other_name = LIDARS[other_index][0]
            pair_distances, _ = cKDTree(transformed[other_index]).query(
                source, k=1, workers=-1
            )
            pairwise[f"{name}->{other_name}"] = {
                str(threshold): int(
                    np.count_nonzero(pair_distances <= threshold)
                )
                for threshold in thresholds
            }

    print(
        json.dumps(
            {
                "target_stamp_sec": target_stamp,
                "per_sensor_to_union": per_sensor,
                "pairwise_source_match_counts": pairwise,
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
