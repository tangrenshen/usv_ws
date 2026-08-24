#!/usr/bin/env python3
"""Multi-frame accumulation feasibility capture, take 2: just dump raw data
fast (no per-message heavy computation in the callback - that caused a
synchronization bug in the first version, where a slow callback meant
self.latest_odom was stale relative to the point cloud by the time it was
used, corrupting the near-target filter itself). Records fused-cloud xyz
(numpy-based unpack, fast) and odom, both with their own timestamps, to a
single JSONL. Offline analysis does proper timestamp interpolation."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
import numpy as np
import json
import math
import os

OUT_PATH = os.environ.get('ACCUM_LOG_PATH', '/home/lyf040817/usv_ws/multiframe_accum.jsonl')


def yaw_from_quat(qz, qw):
    return 2.0 * math.atan2(qz, qw)


class AccumCapture(Node):
    def __init__(self):
        super().__init__('multiframe_accum_capture')
        self.f = open(OUT_PATH, 'w')
        self.gt_written = False

        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Odometry, '/wamv/sensors/position/ground_truth_odometry', self.on_odom, qos)
        self.create_subscription(PoseArray, '/world/obstacles/pillars', self.on_gt_pillar, qos)
        self.create_subscription(PoseArray, '/world/obstacles/buoys', self.on_gt_buoy, qos)
        self.create_subscription(PoseArray, '/world/obstacles/blocks', self.on_gt_block, qos)
        self.create_subscription(PointCloud2, '/world/obstacles/lidar_check', self.on_cloud, qos)
        self.cloud_count = 0
        self.odom_count = 0
        self.get_logger().info(f"capturing to {OUT_PATH}")

    def stamp_sec(self, header):
        return header.stamp.sec + header.stamp.nanosec * 1e-9

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        rec = {'type': 'odom', 't': self.stamp_sec(msg.header),
               'x': p.x, 'y': p.y, 'yaw': yaw_from_quat(q.z, q.w)}
        self.f.write(json.dumps(rec) + '\n')
        self.odom_count += 1

    def on_gt_pillar(self, msg):
        self._write_gt('gt_pillar', msg)

    def on_gt_buoy(self, msg):
        self._write_gt('gt_buoy', msg)

    def on_gt_block(self, msg):
        self._write_gt('gt_block', msg)

    def _write_gt(self, type_name, msg):
        rec = {'type': type_name, 't': self.stamp_sec(msg.header),
               'poses': [[p.position.x, p.position.y] for p in msg.poses]}
        self.f.write(json.dumps(rec) + '\n')

    def on_cloud(self, msg):
        # fast numpy unpack: PointCloud2 with float32 x,y,z fields, standard layout
        dtype = np.dtype([('x', np.float32), ('y', np.float32), ('z', np.float32)])
        # point_step may include extra padding fields; use offsets from msg.fields
        offs = {f.name: f.offset for f in msg.fields}
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
        xs = raw[:, offs['x']:offs['x']+4].copy().view(np.float32).reshape(-1)
        ys = raw[:, offs['y']:offs['y']+4].copy().view(np.float32).reshape(-1)
        zs = raw[:, offs['z']:offs['z']+4].copy().view(np.float32).reshape(-1)
        valid = np.isfinite(xs) & np.isfinite(ys) & np.isfinite(zs)
        xs, ys, zs = xs[valid], ys[valid], zs[valid]
        rec = {
            'type': 'cloud', 't': self.stamp_sec(msg.header),
            'x': xs.tolist(), 'y': ys.tolist(), 'z': zs.tolist(),
        }
        self.f.write(json.dumps(rec) + '\n')
        self.cloud_count += 1
        if self.cloud_count % 20 == 0:
            self.get_logger().info(f"clouds={self.cloud_count} odom={self.odom_count}")


def main():
    rclpy.init()
    node = AccumCapture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.f.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
