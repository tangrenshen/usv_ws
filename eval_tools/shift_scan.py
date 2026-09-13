# -*- coding: utf-8 -*-
"""对齐可证伪检验：给真值时间加偏移 Δ，看一对一匹配命中率是否在 Δ≈0 取峰值。
同时对四类里的船只算出与 v3 同口径的遗漏率（最大基数一对一匹配，容差2m）。"""
import sys, math, bisect
sys.path.insert(0, '/home/lyf040817/usv_ws')
import four_category_evaluator_v2 as ev
import rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray

BAG = sys.argv[1]
rd = rosbag2_py.SequentialReader()
rd.open(rosbag2_py.StorageOptions(uri=BAG, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
rd.set_filter(rosbag2_py.StorageFilter(topics=['/world/obstacles/boats', '/wamv/sensors/position/ground_truth_odometry',
                                                '/world/obstacles/boats_check']))
gts, odoms, dets = [], [], []
while rd.has_next():
    topic, data, t = rd.read_next(); t *= 1e-9
    if topic.endswith('boats'):
        m = deserialize_message(data, PoseArray)
        gts.append({'stamp': t, 'poses': [{'x': p.position.x, 'y': p.position.y} for p in m.poses]})
    elif topic.endswith('odometry'):
        m = deserialize_message(data, Odometry); p, q = m.pose.pose.position, m.pose.pose.orientation
        odoms.append({'stamp': t, 'x': p.x, 'y': p.y, 'qx': q.x, 'qy': q.y, 'qz': q.z, 'qw': q.w})
    else:
        m = deserialize_message(data, MarkerArray)
        dets.append({'stamp': t, 'poses': [(mk.pose.position.x, mk.pose.position.y) for mk in m.markers if mk.action == 0]})
io = ev.make_odom_interpolator(odoms)
for shift in [-4, -2, -1, -0.5, -0.25, 0, 0.25, 0.5, 1, 2, 4]:
    ig = ev.make_gt_interpolator([{'stamp': g['stamp'] + shift, 'poses': g['poses']} for g in gts])
    hit = tot = nd = 0
    for d in dets:
        od = io(d['stamp']); g = ig(d['stamp'])
        if od is None or g is None: continue
        tg = [ev.world_to_boat(p, od) for p in g['poses']]
        pairs, _ = ev.maximum_cardinality_matches(d['poses'], tg, 2.0)
        hit += len(pairs); tot += len(tg); nd += len(d['poses'])
    print(f'Δ={shift:+5.2f}s  真值 {tot}  检测 {nd}  命中 {hit}  遗漏率 {100*(1-hit/tot):5.1f}%  误检(未匹配船) {100*(1-hit/max(nd,1)):5.1f}%')
