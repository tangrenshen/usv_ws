#!/usr/bin/env python3
"""快速检查bag的点云消息里有没有NaN/Inf点。
只读前几条消息，统计NaN比例。
"""
import sys
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import threading

class NaNChecker(Node):
    def __init__(self):
        super().__init__('nan_checker')
        self.topic = '/wamv/sensors/lidars/front_lidar_sensor/points'
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(
            PointCloud2, self.topic, self.cb, qos)
        self.msg_count = 0
        self.total_pts = 0
        self.nan_pts = 0
        self.inf_pts = 0
        self.stop_after = 5  # 只看前5帧
        self.lock = threading.Lock()

    def cb(self, msg):
        with self.lock:
            if self.msg_count >= self.stop_after:
                return
            self.msg_count += 1
            field_names = [f.name for f in msg.fields]
            nan_in_msg = 0
            inf_in_msg = 0
            total_in_msg = 0
            for pt in point_cloud2.read_points(msg, field_names=['x','y','z'], skip_nans=False):
                x, y, z = pt[0], pt[1], pt[2]
                total_in_msg += 1
                if math.isnan(x) or math.isnan(y) or math.isnan(z):
                    nan_in_msg += 1
                if math.isinf(x) or math.isinf(y) or math.isinf(z):
                    inf_in_msg += 1
            self.total_pts += total_in_msg
            self.nan_pts += nan_in_msg
            self.inf_pts += inf_in_msg
            print(f"msg#{self.msg_count}: pts={total_in_msg} nan={nan_in_msg}({100*nan_in_msg/max(total_in_msg,1):.1f}%) inf={inf_in_msg}({100*inf_in_msg/max(total_in_msg,1):.1f}%) fields={field_names}", flush=True)
            if self.msg_count >= self.stop_after:
                print(f"\n=== 汇总 ===", flush=True)
                print(f"总点数={self.total_pts} NaN={self.nan_pts}({100*self.nan_pts/max(self.total_pts,1):.1f}%) Inf={self.inf_pts}({100*self.inf_pts/max(self.total_pts,1):.1f}%)", flush=True)
                rclpy.shutdown()

def main():
    rclpy.init()
    node = NaNChecker()
    print(f"订阅 {node.topic}，等待5帧...", flush=True)
    rclpy.spin(node)

if __name__ == '__main__':
    main()
