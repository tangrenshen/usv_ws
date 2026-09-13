# -*- coding: utf-8 -*-
"""鸟瞰图 A/B：与俯视相机的逐像素误差（赛会口径）。
关键点：
 - 俯视图按实测尺度 31.40 px/m 裁 ±10 m（628 px）再缩放到鸟瞰画布 1000 px；
 - 朝向变换（8 种二面体）只在基线上定一次，两个模式共用，保证对照公平；
 - 逐帧先做偏置对齐（扣除均值差）再算 MAE/RMSE，与报告口径一致；
 - 有效区 = 鸟瞰图非黑像素（排除本船遮挡矩形与未覆盖角区）；
 - 另按真值目标邻域（半径 1.2 m 圆盘）拆分"目标邻域 / 纯水面"。
用法: python3 cmp_bev.py <mode0包> <mode1包>
"""
import sys, math, bisect
import numpy as np, cv2
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray

S_PX_PER_M, RANGE_M, BEV_PX = 31.40, 10.0, 1000
GT = ['/world/obstacles/buoys', '/world/obstacles/blocks', '/world/obstacles/boats', '/world/obstacles/pillars']
def load(bag):
    rd = rosbag2_py.SequentialReader()
    rd.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    bev, oh, od, gt = [], [], [], []
    while rd.has_next():
        t, data, ts = rd.read_next(); ts *= 1e-9
        if t.endswith('bev_check') or 'overhead' in t:
            m = deserialize_message(data, Image)
            a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
            if m.encoding == 'rgb8': a = cv2.cvtColor(a, cv2.COLOR_RGB2BGR)
            (bev if t.endswith('bev_check') else oh).append((ts, a))
        elif t.endswith('odometry'):
            m = deserialize_message(data, Odometry); p, q = m.pose.pose.position, m.pose.pose.orientation
            od.append((ts, p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y**2+q.z**2))))
        elif t in GT:
            m = deserialize_message(data, PoseArray)
            gt.append((ts, [(p.position.x, p.position.y) for p in m.poses]))
    return bev, oh, od, gt

def crop_overhead(img):
    h = int(round(RANGE_M * S_PX_PER_M))          # 314 px = 10 m
    c = img.shape[0] // 2
    return cv2.resize(img[c-h:c+h, c-h:c+h], (BEV_PX, BEV_PX), interpolation=cv2.INTER_AREA)

DIHEDRAL = [('原样', lambda a: a), ('旋90', lambda a: cv2.rotate(a, cv2.ROTATE_90_CLOCKWISE)),
            ('旋180', lambda a: cv2.rotate(a, cv2.ROTATE_180)), ('旋270', lambda a: cv2.rotate(a, cv2.ROTATE_90_COUNTERCLOCKWISE)),
            ('镜像', lambda a: cv2.flip(a, 1)), ('镜像+90', lambda a: cv2.rotate(cv2.flip(a, 1), cv2.ROTATE_90_CLOCKWISE)),
            ('镜像+180', lambda a: cv2.rotate(cv2.flip(a, 1), cv2.ROTATE_180)), ('镜像+270', lambda a: cv2.rotate(cv2.flip(a, 1), cv2.ROTATE_90_COUNTERCLOCKWISE))]

def pair(bev, oh):
    ot = [x[0] for x in oh]; out = []
    for t, b in bev:
        i = bisect.bisect_left(ot, t)
        cand = [j for j in (i-1, i) if 0 <= j < len(oh)]
        if not cand: continue
        j = min(cand, key=lambda j: abs(ot[j]-t))
        if abs(ot[j]-t) <= 0.20: out.append((t, b, oh[j][1]))
    return out

def metrics(bg, og, mask):
    d = bg.astype(np.float32) - og.astype(np.float32)
    d -= d[mask].mean()                            # 仅偏置对齐
    return float(np.abs(d[mask]).mean()), float(np.sqrt((d[mask]**2).mean())), d

def target_mask(t, od, gt):
    m = np.zeros((BEV_PX, BEV_PX), np.uint8)
    if not od: return m
    o = min(od, key=lambda x: abs(x[0]-t))
    c, s = math.cos(-o[3]), math.sin(-o[3])
    for gts, poses in [(g[0], g[1]) for g in gt if abs(g[0]-t) < 0.5]:
        for wx, wy in poses:
            dx, dy = wx-o[1], wy-o[2]
            bx, by = dx*c-dy*s, dx*s+dy*c
            if abs(bx) > RANGE_M or abs(by) > RANGE_M: continue
            u = int((RANGE_M-by)/(2*RANGE_M)*BEV_PX); v = int((RANGE_M-bx)/(2*RANGE_M)*BEV_PX)
            cv2.circle(m, (u, v), int(1.2*BEV_PX/(2*RANGE_M)), 255, -1)
    return m > 0

res = {}
tf = None
for tag, bag in (('基线 mode0', sys.argv[1]), ('A 保留属主臂 mode1', sys.argv[2])):
    bev, oh, od, gt = load(bag)
    pairs = pair(bev, oh)
    print(f'{tag}: 鸟瞰 {len(bev)} 帧, 俯视 {len(oh)} 帧, 配对 {len(pairs)} 对')
    if tf is None:      # 朝向只在基线上定一次
        t, b, o = pairs[len(pairs)//2]
        bg = cv2.cvtColor(b, cv2.COLOR_BGR2GRAY); oc = crop_overhead(o)
        best = None
        for name, fn in DIHEDRAL:
            og = cv2.cvtColor(fn(oc), cv2.COLOR_BGR2GRAY)
            msk = bg > 0
            mae, _, _ = metrics(bg, og, msk)
            if best is None or mae < best[1]: best = (name, mae, fn)
        tf = best; print(f'  朝向变换选定：{best[0]}（MAE {best[1]:.2f}，两模式共用）')
    rows = []
    for t, b, o in pairs:
        bg = cv2.cvtColor(b, cv2.COLOR_BGR2GRAY)
        og = cv2.cvtColor(tf[2](crop_overhead(o)), cv2.COLOR_BGR2GRAY)
        valid = bg > 0
        if valid.sum() < 1000: continue
        mae, rmse, d = metrics(bg, og, valid)
        tm = target_mask(t, od, gt) & valid
        wm = valid & ~tm
        rows.append((t, mae, rmse,
                     float(np.abs(d[tm]).mean()) if tm.sum() > 50 else np.nan,
                     float(np.abs(d[wm]).mean()) if wm.sum() > 50 else np.nan,
                     100*valid.mean(), 100*tm.mean()))
    a = np.array([r[1:] for r in rows], float)
    res[tag] = (rows, a)
    print(f'  n={len(rows)}  MAE 中位 {np.median(a[:,0]):.2f}  RMSE 中位 {np.median(a[:,1]):.2f}'
          f'  目标邻域 MAE {np.nanmedian(a[:,2]):.2f}  纯水面 MAE {np.nanmedian(a[:,3]):.2f}'
          f'  有效区 {np.median(a[:,4]):.1f}%  目标邻域占 {np.median(a[:,5]):.1f}%')
k = list(res)
r0, r1 = res[k[0]][1], res[k[1]][1]
n = min(len(r0), len(r1))
print(f'\n配对差（A − 基线，逐帧按序配对 n={n}，负值=A更好）:')
for j, nm in ((0, '整图 MAE'), (1, 'RMSE'), (2, '目标邻域 MAE'), (3, '纯水面 MAE')):
    d = r1[:n, j] - r0[:n, j]
    print(f'  {nm:12s} 中位 {np.nanmedian(d):+.3f}   A更好的帧占 {100*np.nanmean(d<0):.0f}%')
