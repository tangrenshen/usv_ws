#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""融合精度 · 跨雷达一致性（面到面口径）。

为什么不用点到点：点到点最近邻距离对**采样密度**极其敏感，对**配准精度**反而
不敏感。实测反证——EMX(水平雷达、重叠仅3.5%)本应更差，点到点却比Airy六路
好三倍(0.12~0.23m vs 0.63~0.73m)，因为它打在近处高密度结构上。那个口径
算得出数字，但测的不是我们声称要测的量。

面到面：各路分别做局部平面拟合，再比较同一区域两路拟合平面之间的距离。
平面参数由邻域整体决定，不依赖某一路是否恰好有光束落在对方的采样点上。

三道过滤，缺一不可：
  平面性  —— 特征值比不满足则该邻域不是平面（水面波纹、稀疏区、边缘）
  法向一致 —— 两片法向夹角>30°说明是两个不同表面碰巧靠近，不配对
  排除水面 —— 水面在波动，各路在不同时刻打到的形状本就不同，
              这部分差异不属于配准误差
"""
import numpy as np, math, sys, glob, os, zlib
from scipy.spatial import cKDTree
sys.path.insert(0,"/home/lyf040817")
from lidar_consistency import EXT, AIRY, R_of

# ── 竞赛级外参扰动 ────────────────────────────────────────────────
# 评分细则规定比测时施加 ±0.5m / ±10° 的随机扰动。无扰动条件下外参直接取自
# TF真值，跨雷达一致性当然接近完美——8~12mm 是方法噪声底而非系统性能。
# 只有在扰动条件下，这项指标才真正反映"外参误差对融合质量的影响"。
# 逐雷达在 x/y/z 与 roll/pitch/yaw 上各加一个 ±max 均匀分布量，
# 与 main.cpp 的 test_extrinsic_perturb_* 同分布；用多个seed取中位，
# 以代表"随机扰动"整体而非某一次特定抽样。
PERTURB_T=0.0; PERTURB_R=0.0; PERTURB_SEED=0
def _ext_perturbed(name):
    x,y,z,r,p,yw=EXT[name]
    if PERTURB_T<=0 and PERTURB_R<=0: return x,y,z,r,p,yw
    # 2026-09-12：原用 abs(hash(name))%997，而 Python 字符串 hash 每进程带随机盐，
    # 同一 seed 每次运行的扰动量都不同——结果不可复现。改用 crc32，确定且跨进程一致。
    g=np.random.default_rng(PERTURB_SEED*1000+zlib.crc32(name.encode())%997)
    dt=g.uniform(-PERTURB_T,PERTURB_T,3); dr=g.uniform(-PERTURB_R,PERTURB_R,3)
    return x+dt[0],y+dt[1],z+dt[2],r+dr[0],p+dr[1],yw+dr[2]

def to_boat(pts,name):
    x,y,z,r,p,yw=_ext_perturbed(name)
    return pts @ R_of(r,p,yw).T + np.array([x,y,z])

VOX=0.60          # 局部邻域体素边长(m)
MINPTS=6          # 邻域最少点数
PLANARITY=0.22    # λ0/(λ0+λ1+λ2) 上限：越小越像平面
PAIR_R=0.60       # 两片中心配对的最大距离(m)
NORMAL_DEG=35.0   # 法向夹角上限
WATER_Z=0.35      # |z|<该值且法向近垂直 → 判为水面，排除
RMAX=60.0

def patches(pts):
    """体素内PCA局部平面拟合。返回 (中心Nx3, 法向Nx3)。"""
    m=np.linalg.norm(pts[:,:2],axis=1)<=RMAX
    p=pts[m]
    if len(p)<MINPTS: return np.empty((0,3)),np.empty((0,3))
    key=np.floor(p/VOX).astype(np.int64)
    _,inv,cnt=np.unique(key,axis=0,return_inverse=True,return_counts=True)
    C=[];Nrm=[]
    order=np.argsort(inv); inv_s=inv[order]; p_s=p[order]
    bounds=np.searchsorted(inv_s,np.arange(len(cnt)))
    bounds=np.append(bounds,len(inv_s))
    for i in range(len(cnt)):
        if cnt[i]<MINPTS: continue
        q=p_s[bounds[i]:bounds[i+1]]
        c=q.mean(axis=0)
        w,v=np.linalg.eigh(np.cov((q-c).T))
        if w.sum()<=0: continue
        if w[0]/w.sum()>PLANARITY: continue          # 不够平
        n=v[:,0]/ (np.linalg.norm(v[:,0])+1e-12)
        if abs(c[2])<WATER_Z and abs(n[2])>0.85: continue   # 水面，排除
        C.append(c); Nrm.append(n)
    return (np.array(C) if C else np.empty((0,3)),
            np.array(Nrm) if Nrm else np.empty((0,3)))

def plane_dists(cA,nA,cB,nB):
    if len(cA)==0 or len(cB)==0: return np.array([])
    tree=cKDTree(cB)
    idx=tree.query_ball_point(cA,PAIR_R)
    out=[]
    cosmin=math.cos(math.radians(NORMAL_DEG))
    for i,js in enumerate(idx):
        if not js: continue
        best=None
        for j in js:
            c=abs(float(np.dot(nA[i],nB[j])))
            if c<cosmin: continue
            d=abs(float(np.dot(nB[j], cA[i]-cB[j])))   # A中心到B平面的垂距
            if best is None or d<best: best=d
        if best is not None: out.append(best)
    return np.array(out)

def analyse_group(pat, members, pool):
    ds=[]
    for a in members:
        if a not in pat: continue
        cA,nA=pat[a]
        for b in pool:
            if b==a or b not in pat: continue
            cB,nB=pat[b]
            d=plane_dists(cA,nA,cB,nB)
            if len(d): ds.append(d)
    if not ds: return None
    d=np.concatenate(ds)
    return dict(n=len(d), med=float(np.median(d)), p90=float(np.percentile(d,90)),
                frac10=float((d<0.10).mean()), frac20=float((d<0.20).mean()))

def read_bag_raw(bag, max_groups=3):
    """返回逐组 {name: Nx3 传感器系原始点}。变换放在调用侧，
    否则扰动参数不会作用到坐标变换上。"""
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2 as pc2
    rd=SequentialReader(); rd.open(StorageOptions(uri=bag), ConverterOptions("",""))
    want={f"/wamv/sensors/lidars/{k.replace('_lidar','')}_lidar_sensor/points":k for k in EXT}
    groups=[]; cur={}
    while rd.has_next():
        topic,data,_=rd.read_next()
        if topic not in want: continue
        m=deserialize_message(data,PointCloud2)
        a=pc2.read_points_numpy(m,field_names=("x","y","z"),skip_nans=True)
        a=a[np.isfinite(a).all(axis=1)]
        if len(a)<50: continue
        cur[want[topic]]=a.astype(float)
        if len(cur)==len(EXT):
            groups.append(cur); cur={}
            if len(groups)>=max_groups: break
    return groups

if __name__=="__main__":
    NG=int(sys.argv[1]) if len(sys.argv)>1 else 3
    if len(sys.argv)>3:
        PERTURB_T=float(sys.argv[2]); PERTURB_R=float(sys.argv[3])
        PERTURB_SEED=int(sys.argv[4]) if len(sys.argv)>4 else 0
        globals()["PERTURB_T"]=PERTURB_T; globals()["PERTURB_R"]=PERTURB_R
        globals()["PERTURB_SEED"]=PERTURB_SEED
        print(f"[扰动] ±{PERTURB_T}m / ±{PERTURB_R}° seed={PERTURB_SEED}")
    res={}
    for bag in sorted(glob.glob("/home/lyf040817/bags_5run/run*")):
        raw=read_bag_raw(bag,max_groups=NG)
        groups=[{k:to_boat(v,k) for k,v in g.items()} for g in raw]
        if not groups: continue
        acc={}
        for g in groups:
            pat={k:patches(v) for k,v in g.items()}
            for gn,(mem,pool) in (("Airy六路",(AIRY,AIRY)),
                                  ("EMX对Airy",(["forward_lidar"],AIRY))):
                r=analyse_group(pat,mem,pool)
                if r: acc.setdefault(gn,[]).append(r)
        for gn,v in acc.items():
            res.setdefault(gn,[]).append(dict(
                bag=os.path.basename(bag), groups=len(v),
                n=int(np.median([x["n"] for x in v])),
                med=float(np.median([x["med"] for x in v])),
                p90=float(np.median([x["p90"] for x in v])),
                f10=float(np.median([x["frac10"] for x in v])),
                f20=float(np.median([x["frac20"] for x in v]))))
    print("融合精度 · 跨雷达一致性（面到面口径）")
    print(f"体素{VOX}m 最少{MINPTS}点 平面性<{PLANARITY} 配对半径{PAIR_R}m "
          f"法向夹角<{NORMAL_DEG}° 已排除水面\n")
    for gn,rows in res.items():
        print(f"【{gn}】")
        print(f"  {'数据':<8}{'组数':>5}{'配对数':>8}{'中位(m)':>10}{'p90':>9}{'<0.1m':>9}{'<0.2m':>9}")
        for r in rows:
            print(f"  {r['bag']:<8}{r['groups']:>5}{r['n']:>8}{r['med']:>10.3f}"
                  f"{r['p90']:>9.3f}{100*r['f10']:>8.1f}%{100*r['f20']:>8.1f}%")
        med=[r["med"] for r in rows]
        print(f"  → 跨{len(rows)}份: 中位 {min(med):.3f}~{max(med):.3f}m\n")
    # 效度自检
    if "Airy六路" in res and "EMX对Airy" in res:
        a=np.median([r["med"] for r in res["Airy六路"]])
        e=np.median([r["med"] for r in res["EMX对Airy"]])
        print(f"【效度自检】点到点口径下 EMX/Airy = 0.175/0.667 = 0.26x（EMX反常地好4倍，"
              f"说明该口径主要反映点密度）")
        print(f"           面到面口径下 EMX/Airy = {e:.3f}/{a:.3f} = {e/a:.2f}x")
        print("           判据：比值接近1 → 密度影响已消除，口径有效；"
              "仍显著<1 → 还有未排除因素，应收手不上报。")
