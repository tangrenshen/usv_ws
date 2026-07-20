#!/usr/bin/env python3
"""
纯记录节点：订阅船只/立柱/浮球/浮块真值、位姿、检测结果，按各自消息到达时刻的
header.stamp原样记录到JSONL，不做任何跨话题计算——配对逻辑放到离线evaluator里做。
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
import json

OUT_PATH = '/home/lyf040817/usv_ws/boat_log.jsonl'

class BoatLogger(Node):
    def __init__(self):
        super().__init__('boat_logger')
        self.f = open(OUT_PATH, 'w')
        self.count = {'gt': 0, 'gt_pillar': 0, 'gt_buoy': 0, 'gt_block': 0, 'odom': 0, 'det': 0}

        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST)

        self.create_subscription(PoseArray, '/world/obstacles/boats', self.on_gt, qos)
        self.create_subscription(PoseArray, '/world/obstacles/pillars', self.on_gt_pillar, qos)
        self.create_subscription(PoseArray, '/world/obstacles/buoys', self.on_gt_buoy, qos)
        self.create_subscription(PoseArray, '/world/obstacles/blocks', self.on_gt_block, qos)
        self.create_subscription(Odometry, '/wamv/sensors/position/ground_truth_odometry', self.on_odom, qos)
        self.create_subscription(PoseArray, '/world/obstacles/boats_check', self.on_det, qos)

        self.get_logger().info(f"boat_logger 已启动，记录到 {OUT_PATH}")
        self.timer = self.create_timer(3.0, self.report)

    def stamp_sec(self, header):
        return header.stamp.sec + header.stamp.nanosec * 1e-9

    def _mk(self, msg):
        return {'stamp': self.stamp_sec(msg.header),
                'poses': [{'x': p.position.x, 'y': p.position.y, 'z': p.position.z} for p in msg.poses]}

    def on_gt(self, msg):
        rec = self._mk(msg); rec['type'] = 'gt'
        self.f.write(json.dumps(rec) + '\n'); self.count['gt'] += 1

    def on_gt_pillar(self, msg):
        rec = self._mk(msg); rec['type'] = 'gt_pillar'
        self.f.write(json.dumps(rec) + '\n'); self.count['gt_pillar'] += 1

    def on_gt_buoy(self, msg):
        rec = self._mk(msg); rec['type'] = 'gt_buoy'
        self.f.write(json.dumps(rec) + '\n'); self.count['gt_buoy'] += 1

    def on_gt_block(self, msg):
        rec = self._mk(msg); rec['type'] = 'gt_block'
        self.f.write(json.dumps(rec) + '\n'); self.count['gt_block'] += 1

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        rec = {'type': 'odom', 'stamp': self.stamp_sec(msg.header),
               'x': p.x, 'y': p.y, 'z': p.z, 'qx': q.x, 'qy': q.y, 'qz': q.z, 'qw': q.w}
        self.f.write(json.dumps(rec) + '\n'); self.count['odom'] += 1

    def on_det(self, msg):
        rec = self._mk(msg); rec['type'] = 'det'
        self.f.write(json.dumps(rec) + '\n'); self.count['det'] += 1

    def report(self):
        c = self.count
        self.get_logger().info(f"已记录: boat={c['gt']} pillar={c['gt_pillar']} buoy={c['gt_buoy']} block={c['gt_block']} odom={c['odom']} det={c['det']}")
        self.f.flush()

def main():
    rclpy.init()
    node = BoatLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.f.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
