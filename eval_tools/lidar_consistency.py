#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""融合精度：跨雷达一致性。

口径说明（与赛会答复对齐）：赛会的"精准点云"未提供，无法直接比对。
但融合环节本身不做取舍——`fuseLidarClouds` 只做"按外参变换 + 跳过无效坐标 +
追加"，无体素下采样、无距离裁剪、无本船点过滤。因此"朴素合并"就是输出本身，
拿它当基准是自己比自己，测不出任何东西。

有意义的量是**跨雷达一致性**：同一物理表面被多路扫到时，若外参准确，各路的
点应当重合；外参有误，同一表面会分裂成层。度量方式是逐点最近邻距离
（Chamfer 的单向分量），不需要外部基准，也不循环论证。

分组报告：六路 Airy 之间（360°、重叠大）与 forward EMX 对 Airy（重叠仅约3.5%、
样本少）性质不同，合并会掩盖后者的样本不足。
"""
import numpy as np, math, sys, glob, os, json
from scipy.spatial import cKDTree

# 生效的名义外参（来自 main.cpp nominal_lidars_；z=1.85 为 TF 实测值，
# 任务书标称 2.90，差 1.05m，该差异单独说明，不作为基准混入）
EXT = {
 "front_lidar":  ( 7.0,  0.0,   1.85, 0.0, 115.0,   0.0),
 "right_lidar":  (-6.0, -1.626, 1.85, 0.0, 115.0, -90.0),
 "right2_lidar": ( 6.0, -1.626, 1.85, 0.0, 115.0, -90.0),
 "back_lidar":   (-7.0,  0.0,   1.85, 0.0, 115.0, 180.0),
 "left2_lidar":  ( 6.0,  1.626, 1.85, 0.0, 115.0,  90.0),
 "left_lidar":   (-6.0,  1.626, 1.85, 0.0, 115.0,  90.0),
 "forward_lidar":( 7.0,  0.0,   1.58, 0.0,   0.0,   0.0),
}
AIRY=[k for k in EXT if k!="forward_lidar"]

def R_of(roll,pitch,yaw):
    """照抄 ImageStitcher::buildSensorToBoatRotation（ZYX欧拉角），不另行推导。"""
    D=math.pi/180.0
    cy,sy=math.cos(yaw*D),math.sin(yaw*D)
    cp,sp=math.cos(pitch*D),math.sin(pitch*D)
    cr,sr=math.cos(roll*D),math.sin(roll*D)
    return np.array([
        [ cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
        [ sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
        [-sp,    cp*sr,          cp*cr        ]])

def to_boat(pts,name):
    x,y,z,r,p,yw=EXT[name]
    return pts @ R_of(r,p,yw).T + np.array([x,y,z])

def analyse(clouds, eps_list, rmax=60.0):
    """clouds: {name: Nx3 船体系}。返回逐路到'其余各路合并'的最近邻距离统计。"""
    out={}
    for grp_name, members in (("Airy六路", AIRY), ("EMX对Airy", ["forward_lidar"])):
        ds=[]
        for name in members:
            if name not in clouds: continue
            others=[c for k,c in clouds.items() if k!=name and (grp_name!="EMX对Airy" or k in AIRY)]
            if not others: continue
            ref=np.vstack(others)
            src=clouds[name]
            m=np.linalg.norm(src[:,:2],axis=1)<=rmax
            src=src[m]
            mr=np.linalg.norm(ref[:,:2],axis=1)<=rmax
            ref=ref[mr]
            if len(src)<100 or len(ref)<100: continue
            if len(src)>60000:
                src=src[np.random.default_rng(0).choice(len(src),60000,replace=False)]
            d,_=cKDTree(ref).query(src,k=1)
            ds.append(d)
        if ds:
            d=np.concatenate(ds)
            out[grp_name]=dict(n=len(d), med=float(np.median(d)),
                               p90=float(np.percentile(d,90)),
                               p99=float(np.percentile(d,99)),
                               frac={f"{e}": float((d<e).mean()) for e in eps_list})
    return out

# ── 从 bag 直接读逐路点云（不经节点，避免把评估依赖在被评估对象上）──
def read_bag(bag, max_groups=8):
    import rclpy
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2 as pc2
    rd=SequentialReader()
    rd.open(StorageOptions(uri=bag), ConverterOptions("",""))
    want={f"/wamv/sensors/lidars/{k.replace('_lidar','')}_lidar_sensor/points":k for k in EXT}
    groups=[]; cur={}
    while rd.has_next():
        topic,data,_=rd.read_next()
        if topic not in want: continue
        name=want[topic]
        m=deserialize_message(data,PointCloud2)
        a=pc2.read_points_numpy(m,field_names=("x","y","z"),skip_nans=True)
        a=a[np.isfinite(a).all(axis=1)]
        if len(a)<50: continue
        cur[name]=to_boat(a.astype(float),name)
        if len(cur)==len(EXT):
            groups.append(cur); cur={}
            if len(groups)>=max_groups: break
    return groups

if __name__=="__main__":
    EPS=[0.05,0.10,0.15,0.20,0.30]
    bags=sorted(glob.glob("/home/lyf040817/bags_5run/run*"))
    allres={}
    for bag in bags:
        try: groups=read_bag(bag, max_groups=int(sys.argv[1]) if len(sys.argv)>1 else 6)
        except Exception as e: print(f"{os.path.basename(bag)}: 读取失败 {e}"); continue
        if not groups: print(f"{os.path.basename(bag)}: 无完整组"); continue
        per=[analyse(g,EPS) for g in groups]
        for grp in ("Airy六路","EMX对Airy"):
            v=[p[grp] for p in per if grp in p]
            if not v: continue
            allres.setdefault(grp,[]).append(dict(
                bag=os.path.basename(bag),
                med=float(np.median([x["med"] for x in v])),
                p90=float(np.median([x["p90"] for x in v])),
                p99=float(np.median([x["p99"] for x in v])),
                frac={e: float(np.median([x["frac"][e] for x in v])) for e in map(str,EPS)},
                n=int(np.median([x["n"] for x in v])), groups=len(v)))
    print("融合精度 · 跨雷达一致性（逐点到其余各路合并点云的最近邻距离）")
    print("外参：运行时生效的名义外参（z=1.85m 为TF实测；任务书标称2.90m，差1.05m）\n")
    for grp,rows in allres.items():
        print(f"【{grp}】")
        print(f"  {'数据':<8}{'组数':>5}{'样本':>9}{'中位(m)':>10}{'p90':>8}{'p99':>8}"
              + "".join(f"{'<'+e+'m':>9}" for e in map(str,EPS)))
        for r in rows:
            print(f"  {r['bag']:<8}{r['groups']:>5}{r['n']:>9}{r['med']:>10.3f}"
                  f"{r['p90']:>8.3f}{r['p99']:>8.3f}"
                  + "".join(f"{100*r['frac'][e]:>8.1f}%" for e in map(str,EPS)))
        med=[r["med"] for r in rows]; p90=[r["p90"] for r in rows]
        print(f"  → 跨{len(rows)}份: 中位 {min(med):.3f}~{max(med):.3f}m,"
              f" p90 {min(p90):.3f}~{max(p90):.3f}m\n")
