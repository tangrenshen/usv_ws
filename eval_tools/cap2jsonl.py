# -*- coding: utf-8 -*-
"""录制包 → boat_logger 同格式 jsonl。所有记录的 stamp 一律取录制器接收时刻（单一时钟），
供 four_category_evaluator_v4.py --common-clock 使用。
用法: python3 cap2jsonl.py <bag_dir> <out.jsonl>"""
import sys, json, math
import rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray

T = '/world/obstacles/'
GT = {T + 'boats': 'gt', T + 'buoys': 'gt_buoy', T + 'pillars': 'gt_pillar', T + 'blocks': 'gt_block'}
DET = {T + 'boats_check': 'det', T + 'buoys_check': 'det_buoy', T + 'pillars_check': 'det_pillar',
       T + 'blocks_check': 'det_block'}
ODOM = '/wamv/sensors/position/ground_truth_odometry'
rd = rosbag2_py.SequentialReader()
rd.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
rd.set_filter(rosbag2_py.StorageFilter(topics=list(GT) + list(DET) + [ODOM]))
n = {}
with open(sys.argv[2], 'w') as g:
    while rd.has_next():
        topic, data, t = rd.read_next(); t *= 1e-9
        if topic in GT:
            m = deserialize_message(data, PoseArray)
            r = {'type': GT[topic], 'stamp': t, 'poses': [{'x': p.position.x, 'y': p.position.y} for p in m.poses]}
        elif topic in DET:
            m = deserialize_message(data, MarkerArray)
            h = m.markers[0].header.stamp if m.markers else None   # 节点发布时刻(仿真时)，与 cluster_diagnostic 的 t 相同
            r = {'type': DET[topic], 'stamp': t, 'hstamp': (h.sec + h.nanosec * 1e-9) if h else None,
                 'poses': [{'x': k.pose.position.x, 'y': k.pose.position.y, 'z': k.pose.position.z}
                           for k in m.markers if k.action == 0]}
        else:
            m = deserialize_message(data, Odometry); p, q = m.pose.pose.position, m.pose.pose.orientation
            r = {'type': 'odom', 'stamp': t, 'x': p.x, 'y': p.y, 'z': p.z, 'qx': q.x, 'qy': q.y, 'qz': q.z, 'qw': q.w}
        n[r['type']] = n.get(r['type'], 0) + 1
        g.write(json.dumps(r) + '\n')
print(sys.argv[2], n)
