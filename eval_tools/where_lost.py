# -*- coding: utf-8 -*-
"""逐帧逐条真值船：统计节点实际融合点云中落在船身附近的点，定位在哪一步丢失。
用录制接收时刻统一对齐（点云/真值/odom 都在本次回放中被同一个 recorder 接收）。
用法: python3 where_lost.py <capture_bag_dir> [半径m]
"""
import sys, math, bisect, collections
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray

BAG = sys.argv[1]
R = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
MAXR, SXH, SYH, WP, MINH, VOX, TOL = 130.0, 7.5, 2.75, 0.05, 0.15, 0.20, 0.90

rd = rosbag2_py.SequentialReader()
rd.open(rosbag2_py.StorageOptions(uri=BAG, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
TY = {'/world/obstacles/lidar_check': PointCloud2, '/world/obstacles/boats': PoseArray,
      '/wamv/sensors/position/ground_truth_odometry': Odometry, '/world/obstacles/boats_check': MarkerArray}
clouds, gts, odoms, dets = [], [], [], []
while rd.has_next():
    topic, data, t = rd.read_next()
    t *= 1e-9
    m = deserialize_message(data, TY[topic])
    if topic.endswith('lidar_check'):
        a = np.frombuffer(bytes(m.data), dtype=np.uint8).reshape(-1, m.point_step)
        off = {f.name: f.offset for f in m.fields}
        xyz = np.stack([a[:, off[k]:off[k] + 4].copy().view(np.float32)[:, 0] for k in 'xyz'], 1)
        clouds.append((t, xyz))
    elif topic.endswith('boats'):
        gts.append((t, [(p.position.x, p.position.y) for p in m.poses]))
    elif topic.endswith('odometry'):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
        odoms.append((t, p.x, p.y, yaw))
    else:
        dets.append((t, [(mk.pose.position.x, mk.pose.position.y) for mk in m.markers if mk.action == 0]))
print(f'点云帧 {len(clouds)}  真值 {len(gts)}  odom {len(odoms)}  检测 {len(dets)}  每帧点数中位 {int(np.median([len(c[1]) for c in clouds]))}')

ot = [o[0] for o in odoms]; gt_t = [g[0] for g in gts]; dt_ = [d[0] for d in dets]
def odom_at(t):
    i = bisect.bisect_left(ot, t)
    if i == 0 or i >= len(ot): return None
    a, b = odoms[i - 1], odoms[i]; s = (t - a[0]) / (b[0] - a[0])
    dyaw = math.atan2(math.sin(b[3] - a[3]), math.cos(b[3] - a[3]))
    return (a[1] + s * (b[1] - a[1]), a[2] + s * (b[2] - a[2]), a[3] + s * dyaw)
def gt_at(t):
    i = bisect.bisect_left(gt_t, t)
    if i == 0 or i >= len(gts): return None
    a, b = gts[i - 1], gts[i]
    if len(a[1]) != len(b[1]): return a[1]
    s = (t - a[0]) / (b[0] - a[0])
    return [(p[0] + s * (q[0] - p[0]), p[1] + s * (q[1] - p[1])) for p, q in zip(a[1], b[1])]
def det_at(t):
    i = bisect.bisect_right(dt_, t + 0.15)   # 该帧点云之后最近发布的检测
    return dets[i - 1][1] if i > 0 else []

def components(P):
    """体素降采样 + 0.9m 连通分量，返回 (代表点, 权重, 标签)"""
    k = np.floor(P / VOX).astype(np.int64)
    uk, inv, cnt = np.unique(k, axis=0, return_inverse=True, return_counts=True)
    inv = inv.ravel()
    rep = np.zeros((len(uk), 3)); np.add.at(rep, inv, P); rep /= cnt[:, None]
    par = list(range(len(rep)))
    def f(x):
        while par[x] != x: par[x] = par[par[x]]; x = par[x]
        return x
    g = collections.defaultdict(list)
    cell = np.floor(rep / TOL).astype(np.int64)
    for i, c in enumerate(map(tuple, cell)): g[c].append(i)
    for i, c in enumerate(map(tuple, cell)):
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    for j in g.get((c[0] + dx, c[1] + dy, c[2] + dz), ()):
                        if j > i and np.sum((rep[i] - rep[j]) ** 2) <= TOL * TOL:
                            a, b = f(i), f(j)
                            if a != b: par[a] = b
    lab = np.array([f(i) for i in range(len(rep))])
    return rep, cnt, lab

stage = collections.Counter(); rows = []
for t, xyz in clouds:
    od = odom_at(t); g = gt_at(t)
    if od is None or g is None: continue
    ok = np.isfinite(xyz).all(1)
    P = xyz[ok]
    r2 = P[:, 0] ** 2 + P[:, 1] ** 2
    keep = (r2 <= MAXR ** 2) & ~((np.abs(P[:, 0]) < SXH) & (np.abs(P[:, 1]) < SYH))
    C = P[keep]
    if len(C) == 0: continue
    wz = np.partition(C[:, 2], int(WP * len(C)))[int(WP * len(C))]
    zmin = wz + MINH
    det = det_at(t)
    c, s = math.cos(-od[2]), math.sin(-od[2])
    for (wx, wy) in g:
        dx, dy = wx - od[0], wy - od[1]
        bx, by = dx * c - dy * s, dx * s + dy * c
        dist = math.hypot(bx, by)
        hit = any(math.hypot(d[0] - bx, d[1] - by) < 2.0 for d in det)
        near = (C[:, 0] - bx) ** 2 + (C[:, 1] - by) ** 2 <= R * R
        n_raw = int(near.sum()); n_up = int((near & (C[:, 2] > zmin)).sum())
        info = dict(dist=dist, n_raw=n_raw, n_up=n_up, hit=hit, zmin=zmin,
                    zmax_near=float(C[near, 2].max()) if n_raw else None,
                    bearing=math.degrees(math.atan2(by, bx)))
        if n_raw == 0: st = '0 船身1m内无任何点(未扫到)'
        elif n_up == 0: st = '1 有点但全被水面滤除'
        else:
            loc = (C[:, 2] > zmin) & ((C[:, 0] - bx) ** 2 + (C[:, 1] - by) ** 2 <= 64)
            rep, cnt, lab = components(C[loc].astype(np.float64))
            m = (rep[:, 0] - bx) ** 2 + (rep[:, 1] - by) ** 2 <= R * R
            best = max(set(lab[m]), key=lambda L: cnt[(lab == L) & m].sum())
            sel = lab == best
            tot = int(cnt[sel].sum()); cc = (rep[sel] * cnt[sel, None]).sum(0) / tot
            ext = rep[sel].max(0) - rep[sel].min(0) + VOX
            info.update(cl_pts=tot, cl_fp=float(max(ext[0], ext[1])), cl_dz=float(ext[2]),
                        cl_off=float(math.hypot(cc[0] - bx, cc[1] - by)))
            if tot < 4: st = '2 成簇点数<4(min_cluster_pts)'
            elif info['cl_off'] >= 2.0: st = '3 簇心被拉偏>=2m(粘连)'
            else: st = '4 形成簇且簇心在2m内'
        info['st'] = st; rows.append(info); stage[st] += 1

N = len(rows)
print(f'\n真值船实例 N={N}  (半径 {R} m)   其中节点实际命中 {sum(r["hit"] for r in rows)} ({sum(r["hit"] for r in rows)/N*100:.1f}%)')
for k in sorted(stage): print(f'  {k:28s} {stage[k]:6d}  {stage[k]/N*100:5.1f}%   其中命中 {sum(r["hit"] for r in rows if r["st"]==k)}')
print('\n按距离分段 (各阶段占比 %):')
bins = [0, 10, 15, 20, 25, 30, 40, 60, 200]
for lo, hi in zip(bins, bins[1:]):
    sub = [r for r in rows if lo <= r['dist'] < hi]
    if not sub: continue
    cc = collections.Counter(r['st'][0] for r in sub)
    print(f'  {lo:3d}-{hi:<3d}m n={len(sub):5d}  ' + '  '.join(f'{k}:{cc[k]/len(sub)*100:5.1f}' for k in '01234') +
          f'   命中 {sum(r["hit"] for r in sub)/len(sub)*100:5.1f}%   原始点中位 {int(np.median([r["n_raw"] for r in sub]))}')
import pickle; pickle.dump(rows, open(BAG.rstrip('/') + '_rows.pkl', 'wb'))
