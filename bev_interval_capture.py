#!/usr/bin/env python3
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class IntervalCapture(Node):
    def __init__(self, out_path):
        super().__init__("bev_interval_capture")
        self.out_path = out_path
        self.last_t = None
        self.intervals = []
        self.sub = self.create_subscription(Image, "/world/obstacles/bev_check", self.cb, 10)
        self.timer = self.create_timer(30.0, self.finish)

    def cb(self, msg):
        t = time.monotonic()
        if self.last_t is not None:
            self.intervals.append(t - self.last_t)
        self.last_t = t

    def finish(self):
        with open(self.out_path, "w") as f:
            for iv in self.intervals:
                f.write(f"{iv:.4f}\n")
        self.get_logger().info(f"wrote {len(self.intervals)} intervals to {self.out_path}")
        rclpy.shutdown()


def main():
    out_path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/bev_intervals.txt"
    rclpy.init()
    node = IntervalCapture(out_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
