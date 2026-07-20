#!/usr/bin/env python3
"""
同步采集节点：在同一处理周期内，把浮球+立柱+浮块真值、船体位姿、六路原始图像一起存盘。
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import json
import math

CAMERA_NAMES = ["front_camera", "right_camera", "right2_camera",
                "back_camera", "left2_camera", "left_camera"]

class SyncCapture(Node):
    def __init__(self):
        super().__init__('sync_capture')
        self.bridge = CvBridge()
        self.latest_buoys = None
        self.latest_pillars = None
        self.latest_blocks = None
        self.latest_odom = None
        self.latest_images = {}
        self.captured = False

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST)

        self.create_subscription(PoseArray, '/world/obstacles/buoys',
                                  self.on_buoys, qos)
        self.create_subscription(PoseArray, '/world/obstacles/pillars',
                                  self.on_pillars, qos)
        self.create_subscription(PoseArray, '/world/obstacles/blocks',
                                  self.on_blocks, qos)
        self.create_subscription(Odometry, '/wamv/sensors/position/ground_truth_odometry',
                                  self.on_odom, qos)
        for i, name in enumerate(CAMERA_NAMES):
            topic = f'/wamv/sensors/cameras/{name}_sensor/optical/image_raw'
            self.create_subscription(
                Image, topic,
                lambda msg, idx=i, n=name: self.on_image(msg, idx, n), qos)

        self.timer = self.create_timer(0.5, self.try_capture)
        self.get_logger().info("同步采集节点已启动，等待所有数据到齐...")

    def on_buoys(self, msg):
        self.latest_buoys = msg

    def on_pillars(self, msg):
        self.latest_pillars = msg

    def on_blocks(self, msg):
        self.latest_blocks = msg

    def on_odom(self, msg):
        self.latest_odom = msg

    def on_image(self, msg, idx, name):
        self.latest_images[idx] = (msg, name)

    def try_capture(self):
        if self.captured:
            return
        ready = (self.latest_buoys is not None and self.latest_pillars is not None
                 and self.latest_blocks is not None and self.latest_odom is not None
                 and len(self.latest_images) >= 6)
        if not ready:
            self.get_logger().info(
                f"等待中... buoys={self.latest_buoys is not None} "
                f"pillars={self.latest_pillars is not None} "
                f"blocks={self.latest_blocks is not None} "
                f"odom={self.latest_odom is not None} images={len(self.latest_images)}/6")
            return

        self.captured = True

        gt_buoys = [{"x": p.position.x, "y": p.position.y, "z": p.position.z}
                    for p in self.latest_buoys.poses]
        gt_pillars = [{"x": p.position.x, "y": p.position.y, "z": p.position.z}
                      for p in self.latest_pillars.poses]
        gt_blocks = [{"x": p.position.x, "y": p.position.y, "z": p.position.z}
                     for p in self.latest_blocks.poses]

        p = self.latest_odom.pose.pose.position
        q = self.latest_odom.pose.pose.orientation
        odom = {"x": p.x, "y": p.y, "z": p.z,
                "qx": q.x, "qy": q.y, "qz": q.z, "qw": q.w}

        with open('/tmp/sync_data.json', 'w') as f:
            json.dump({
                "buoys": gt_buoys,
                "pillars": gt_pillars,
                "blocks": gt_blocks,
                "odom": odom,
            }, f, indent=2)

        for idx in range(6):
            msg, name = self.latest_images[idx]
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imwrite(f'/tmp/sync_cam{idx}.png', cv_img)

        self.get_logger().info(
            f"真值浮球数: {len(gt_buoys)}  真值立柱数: {len(gt_pillars)}  真值浮块数: {len(gt_blocks)}")
        self.get_logger().info(f"船位姿: x={odom['x']:.2f} y={odom['y']:.2f}")
        self.get_logger().info("数据已保存: /tmp/sync_data.json, /tmp/sync_cam0~5.png")

def main():
    rclpy.init()
    node = SyncCapture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
