#!/usr/bin/env python3
import json
import math
import sys
import time
from collections import Counter

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2


TOPICS = {
    "bev": ("/world/obstacles/bev_check", Image),
    "lidar": ("/world/obstacles/lidar_check", PointCloud2),
    "buoys": ("/world/obstacles/buoys_check", PoseArray),
    "boats": ("/world/obstacles/boats_check", PoseArray),
    "pillars": ("/world/obstacles/pillars_check", PoseArray),
    "blocks": ("/world/obstacles/blocks_check", PoseArray),
}


class InterfaceAudit(Node):
    def __init__(self, duration_sec):
        super().__init__("perception_interface_audit")
        self.duration_sec = duration_sec
        self.started = time.monotonic()
        self.records = {}
        for name, (topic, msg_type) in TOPICS.items():
            self.records[name] = {
                "topic": topic,
                "count": 0,
                "first_wall": None,
                "last_wall": None,
                "first_stamp": None,
                "last_stamp": None,
                "frame_ids": Counter(),
                "stamp_zero": 0,
                "nonmonotonic_stamp": 0,
                "last_seen_stamp": None,
                "sample": None,
            }
            self.create_subscription(
                msg_type,
                topic,
                lambda msg, key=name: self.on_message(key, msg),
                20,
            )
        self.create_timer(0.2, self.check_done)

    def on_message(self, name, msg):
        now_wall = time.monotonic()
        record = self.records[name]
        stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        record["count"] += 1
        record["first_wall"] = record["first_wall"] or now_wall
        record["last_wall"] = now_wall
        record["first_stamp"] = record["first_stamp"] if record["first_stamp"] is not None else stamp
        record["last_stamp"] = stamp
        record["frame_ids"][msg.header.frame_id] += 1
        if stamp == 0.0:
            record["stamp_zero"] += 1
        if record["last_seen_stamp"] is not None and stamp < record["last_seen_stamp"]:
            record["nonmonotonic_stamp"] += 1
        record["last_seen_stamp"] = stamp

        if record["sample"] is None:
            if isinstance(msg, Image):
                record["sample"] = {
                    "height": msg.height,
                    "width": msg.width,
                    "encoding": msg.encoding,
                    "step": msg.step,
                }
            elif isinstance(msg, PointCloud2):
                points = point_cloud2.read_points_numpy(
                    msg, field_names=("x", "y", "z"), skip_nans=False
                )
                points = np.asarray(points).reshape(-1, 3)
                record["sample"] = {
                    "height": msg.height,
                    "width": msg.width,
                    "is_dense": msg.is_dense,
                    "fields": [field.name for field in msg.fields],
                    "nonfinite_points": int(
                        np.count_nonzero(~np.isfinite(points).all(axis=1))
                    ),
                }
            else:
                record["sample"] = {
                    "pose_count_first": len(msg.poses),
                    "orientation_norm_first": (
                        math.sqrt(
                            msg.poses[0].orientation.x ** 2
                            + msg.poses[0].orientation.y ** 2
                            + msg.poses[0].orientation.z ** 2
                            + msg.poses[0].orientation.w ** 2
                        )
                        if msg.poses
                        else None
                    ),
                }

    def check_done(self):
        if time.monotonic() - self.started >= self.duration_sec:
            self.finish()

    def finish(self):
        report = {}
        for name, record in self.records.items():
            wall_span = (
                record["last_wall"] - record["first_wall"]
                if record["first_wall"] is not None
                and record["last_wall"] is not None
                and record["count"] > 1
                else 0.0
            )
            stamp_span = (
                record["last_stamp"] - record["first_stamp"]
                if record["first_stamp"] is not None
                and record["last_stamp"] is not None
                and record["count"] > 1
                else 0.0
            )
            report[name] = {
                "topic": record["topic"],
                "count": record["count"],
                "wall_rate_hz": (
                    (record["count"] - 1) / wall_span if wall_span > 0.0 else 0.0
                ),
                "stamp_rate_hz": (
                    (record["count"] - 1) / stamp_span if stamp_span > 0.0 else 0.0
                ),
                "frame_ids": dict(record["frame_ids"]),
                "stamp_zero": record["stamp_zero"],
                "nonmonotonic_stamp": record["nonmonotonic_stamp"],
                "sample": record["sample"],
            }
        print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
        rclpy.shutdown()


def main():
    duration_sec = float(sys.argv[1]) if len(sys.argv) > 1 else 90.0
    rclpy.init()
    node = InterfaceAudit(duration_sec)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            node.finish()
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
