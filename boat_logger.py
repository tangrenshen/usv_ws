#!/usr/bin/env python3
"""
纯记录节点：订阅船只/立柱/浮球/浮块真值、位姿、检测结果，按各自消息到达时刻的
header.stamp原样记录到JSONL，不做任何跨话题计算——配对逻辑放到离线evaluator里做。

新增：同时记录cluster_diagnostic信息，包含每个候选簇的详细特征（size/pts/label等）
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
import json
import re
import os

OUT_PATH = os.environ.get(
    'BOAT_LOG_PATH', '/home/lyf040817/usv_ws/boat_log.jsonl')
CLUSTER_LOG_PATH = os.environ.get(
    'CLUSTER_LOG_PATH', '/home/lyf040817/usv_ws/perception_log.log')

NUM = r'[-+]?[\d.]+(?:[eE][-+]?\d+)?'
pending_pattern = re.compile(
    r'\[cluster_diagnostic\] t=(' + NUM + r') center=\((' + NUM + r'),(' + NUM + r'),(' + NUM + r')\)'
    r' size=\((' + NUM + r'),(' + NUM + r'),(' + NUM + r')\)'
    r' fp_max=(' + NUM + r') fp_min=(' + NUM + r') square=(' + NUM + r') pts=(\d+)'
    r'(?: forward_pts=(\d+) forward_ratio=(' + NUM + r'))?'
    r'(?: measurement_t=(' + NUM + r'))? -> classification pending'
)
# 成功分类(block/buoy/pillar/boat/*_fallback)打印格式是"t=... -> assigned=X"，assigned=
# 不在行首；只有discarded_*两条丢弃路径是行首简短格式"[cluster_diagnostic] assigned=X"。
# 不能锚定行首(.match)，否则所有成功分类的簇都不会被写入'cluster'记录，只有被丢弃的会。
assigned_pattern = re.compile(r'assigned=(\w+)')

class BoatLogger(Node):
    def __init__(self):
        super().__init__('boat_logger')
        self.f = open(OUT_PATH, 'w')
        self.count = {'gt': 0, 'gt_pillar': 0, 'gt_buoy': 0, 'gt_block': 0, 'odom': 0,
                      'det': 0, 'det_buoy': 0, 'det_pillar': 0, 'det_block': 0, 'cluster': 0}
        self.cluster_frames = {}
        self.last_cluster_pos = 0
        self.latest_clock = None

        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST)

        self.create_subscription(PoseArray, '/world/obstacles/boats', self.on_gt, qos)
        self.create_subscription(PoseArray, '/world/obstacles/pillars', self.on_gt_pillar, qos)
        self.create_subscription(PoseArray, '/world/obstacles/buoys', self.on_gt_buoy, qos)
        self.create_subscription(PoseArray, '/world/obstacles/blocks', self.on_gt_block, qos)
        self.create_subscription(Odometry, '/wamv/sensors/position/ground_truth_odometry', self.on_odom, qos)
        # four_category_evaluator_v2.resolve_epoch_offset()的首选方法
        # (median(clock_stamp - odom_stamp))此前一直没有任何logger产出clock_stamp
        # 字段，实际上从未被真正用过——一直靠--epoch-offset显式值或标了"not robust"
        # 的legacy回退。这里补上，让该方法真正可用，避免contour诊断继续依赖
        # 未经本次重新验证的历史epoch-offset数字。
        self.create_subscription(Clock, '/clock', self.on_clock, qos)
        self.create_subscription(MarkerArray, '/world/obstacles/boats_check', self.on_det, qos)
        self.create_subscription(MarkerArray, '/world/obstacles/buoys_check', self.on_det_buoy, qos)
        self.create_subscription(MarkerArray, '/world/obstacles/pillars_check', self.on_det_pillar, qos)
        self.create_subscription(MarkerArray, '/world/obstacles/blocks_check', self.on_det_block, qos)

        self.get_logger().info(f"boat_logger 已启动，记录到 {OUT_PATH}")
        self.timer = self.create_timer(3.0, self.report)
        self.cluster_timer = self.create_timer(0.5, self.read_cluster_diagnostic)

    def stamp_sec(self, header):
        return header.stamp.sec + header.stamp.nanosec * 1e-9

    def _mk(self, msg):
        return {'stamp': self.stamp_sec(msg.header),
                'poses': [{'x': p.position.x, 'y': p.position.y, 'z': p.position.z} for p in msg.poses]}

    def _mk_oriented(self, msg):
        # 同_mk，额外带朝向四元数——仅block轮廓旋转修正需要（第五节：block是唯一
        # 非轴对称、需要修正的类别，boat同理但本轮推后，buoy/pillar在XY上旋转不变，
        # 不需要）。
        return {'stamp': self.stamp_sec(msg.header),
                'poses': [{'x': p.position.x, 'y': p.position.y, 'z': p.position.z,
                           'qx': p.orientation.x, 'qy': p.orientation.y,
                           'qz': p.orientation.z, 'qw': p.orientation.w} for p in msg.poses]}

    def _mk_markers(self, msg):
        # *_check话题2026-08-24改用MarkerArray（赛会口径变更，PoseArray->MarkerArray，
        # 携带长宽高）。每帧先发一个DELETEALL(action=3)清空上一帧的marker，只有
        # action=ADD(0)的才是真实目标，stamp取第一个真实marker的（DELETEALL用的是
        # 同一帧的stamp，没有真实marker时才退化到用DELETEALL自己的stamp）。
        adds = [m for m in msg.markers if m.action == Marker.ADD]
        # MarkerArray本身没有header，取任意一个marker的（同一帧发布的所有marker
        # stamp都相同）；理论上msg.markers不会为空，因为我们的发布端总是先塞一个
        # DELETEALL，这里仍加个防御分支避免极端情况下崩溃。
        stamp_src = (adds[0] if adds else msg.markers[0]).header if msg.markers else None
        if stamp_src is None:
            return {'stamp': self.get_clock().now().nanoseconds * 1e-9, 'poses': []}
        return {'stamp': self.stamp_sec(stamp_src),
                'poses': [{'x': m.pose.position.x, 'y': m.pose.position.y, 'z': m.pose.position.z,
                           'dx': m.scale.x, 'dy': m.scale.y, 'dz': m.scale.z} for m in adds]}

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
        rec = self._mk_oriented(msg); rec['type'] = 'gt_block'
        self.f.write(json.dumps(rec) + '\n'); self.count['gt_block'] += 1

    def on_clock(self, msg):
        self.latest_clock = msg.clock.sec + msg.clock.nanosec * 1e-9

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        rec = {'type': 'odom', 'stamp': self.stamp_sec(msg.header),
               'x': p.x, 'y': p.y, 'z': p.z, 'qx': q.x, 'qy': q.y, 'qz': q.z, 'qw': q.w}
        if self.latest_clock is not None:
            rec['clock_stamp'] = self.latest_clock
        self.f.write(json.dumps(rec) + '\n'); self.count['odom'] += 1

    def on_det(self, msg):
        rec = self._mk_markers(msg); rec['type'] = 'det'
        self.f.write(json.dumps(rec) + '\n'); self.count['det'] += 1

    def on_det_buoy(self, msg):
        rec = self._mk_markers(msg); rec['type'] = 'det_buoy'
        self.f.write(json.dumps(rec) + '\n'); self.count['det_buoy'] += 1

    def on_det_pillar(self, msg):
        rec = self._mk_markers(msg); rec['type'] = 'det_pillar'
        self.f.write(json.dumps(rec) + '\n'); self.count['det_pillar'] += 1

    def on_det_block(self, msg):
        rec = self._mk_markers(msg); rec['type'] = 'det_block'
        self.f.write(json.dumps(rec) + '\n'); self.count['det_block'] += 1

    def read_cluster_diagnostic(self):
        if not os.path.exists(CLUSTER_LOG_PATH):
            return
        
        try:
            with open(CLUSTER_LOG_PATH, 'r', encoding='utf-8', errors='replace') as f:
                f.seek(self.last_cluster_pos)
                current_cluster = None
                
                for line in f:
                    line = line.strip()
                    pending_match = pending_pattern.match(line)
                    if pending_match:
                        if current_cluster:
                            self.cluster_frames.setdefault(current_cluster['t'], []).append(current_cluster)
                        try:
                            t = float(pending_match.group(1))
                            current_cluster = {
                                't': t,
                                'x': float(pending_match.group(2)),
                                'y': float(pending_match.group(3)),
                                'z': float(pending_match.group(4)),
                                'dx': float(pending_match.group(5)),
                                'dy': float(pending_match.group(6)),
                                'dz': float(pending_match.group(7)),
                                'fp_max': float(pending_match.group(8)),
                                'fp_min': float(pending_match.group(9)),
                                'square': float(pending_match.group(10)),
                                'pts': int(pending_match.group(11)),
                                'forward_pts': (
                                    int(pending_match.group(12))
                                    if pending_match.group(12) is not None
                                    else None
                                ),
                                'forward_ratio': (
                                    float(pending_match.group(13))
                                    if pending_match.group(13) is not None
                                    else None
                                ),
                                'measurement_t': (
                                    float(pending_match.group(14))
                                    if pending_match.group(14) is not None
                                    else None
                                ),
                                'label': None
                            }
                        except (ValueError, IndexError):
                            current_cluster = None
                            continue
                    else:
                        assigned_match = assigned_pattern.search(line)
                        if assigned_match and current_cluster:
                            current_cluster['label'] = assigned_match.group(1)
                            self.cluster_frames.setdefault(current_cluster['t'], []).append(current_cluster)
                            rec = {
                                'type': 'cluster',
                                't': current_cluster['t'],
                                'x': current_cluster['x'],
                                'y': current_cluster['y'],
                                'z': current_cluster['z'],
                                'dx': current_cluster['dx'],
                                'dy': current_cluster['dy'],
                                'dz': current_cluster['dz'],
                                'fp_max': current_cluster['fp_max'],
                                'fp_min': current_cluster['fp_min'],
                                'square': current_cluster['square'],
                                'pts': current_cluster['pts'],
                                'forward_pts': current_cluster['forward_pts'],
                                'forward_ratio': current_cluster['forward_ratio'],
                                'measurement_t': current_cluster['measurement_t'],
                                'label': current_cluster['label']
                            }
                            self.f.write(json.dumps(rec) + '\n')
                            self.count['cluster'] += 1
                            current_cluster = None
                
                self.last_cluster_pos = f.tell()
        except Exception as e:
            self.get_logger().warn(f"读取cluster_diagnostic失败: {e}")

    def report(self):
        c = self.count
        self.get_logger().info(
            f"已记录: boat={c['gt']} pillar={c['gt_pillar']} buoy={c['gt_buoy']} block={c['gt_block']} "
            f"odom={c['odom']} det={c['det']} det_buoy={c['det_buoy']} det_pillar={c['det_pillar']} "
            f"det_block={c['det_block']} cluster={c['cluster']}")
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
