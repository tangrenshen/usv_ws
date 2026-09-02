#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""轮廓/尺寸误差 —— 按赛会2026-09-02口径：**长宽高误差绝对值之和**。

与旧版 contour_xy_diagnostic.py 的三点差异：
  1. 旧版报的是**相对百分比且带符号**（如 0-20m 为 -4.9%~+9.0%）。新口径要求
     绝对值之和，正负不再相互抵消——这使得"20-40m符号不稳定"的后果比旧版
     估计的更严重：旧版里一次 -25.9% 和一次 +10.7% 在平均时部分抵消，
     绝对值口径下两者都是纯损失。
  2. 旧版**只算XY**，z轴推后。新口径明确含高度，必须一并算。
  3. 尺寸真值取自**世界SDF逐实例**，不取自GT话题——GT话题只发位置、
     完全不带尺寸字段（立柱SDF里pose的z恒为0，连高度都读不到），
     且立柱高度逐个随机（5.11~11.68m实测），不能用常数代替。
     见 world_gt_dimensions.py。

**z轴口径的已知问题，必须连同数字一起呈现**：检测侧的 dz 来自点云包围盒，
只反映**雷达实际打到的部分**；而SDF给的是物体完整几何。两者基准不同：
  - 立柱：雷达只打到中段（实测z范围约 -3.1~+3.8m），SDF全高5.11~11.68m；
  - 浮块：SDF碰撞体高0.4m，但模型摆在z=0.2、网格z范围-0.407~-0.001，
    水面以上仅约0.2m；而实测点云dz为0.48~0.71m（含水面附近回波）。
因此**高度项的绝对误差不能简单解读为"高度估计不准"**，其中包含"可见部分
与完整几何之间的基准差"。本工具如实输出，并在报告中标注该口径限制。

用法:
    python3 contour_error_v2.py <log.jsonl> --world <world.sdf>
"""

import argparse
import math
import statistics
import sys

sys.path.insert(0, "/home/lyf040817/usv_ws")
import four_category_evaluator_v2 as ev
import world_gt_dimensions as wgd

CATS = [("boat", "gt", "det"), ("buoy", "gt_buoy", "det_buoy"),
        ("pillar", "gt_pillar", "det_pillar"), ("block", "gt_block", "det_block")]
BUCKETS = [(0.0, 20.0), (20.0, 40.0), (40.0, math.inf)]


def bucket(d):
    for lo, hi in BUCKETS:
        if lo <= d < hi:
            return f"{lo:.0f}-{'inf' if hi == math.inf else f'{hi:.0f}'}m"
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--world", required=True)
    ap.add_argument("--match-tol", type=float, default=2.0)
    args = ap.parse_args()

    items = wgd.parse_world(args.world)
    # 立柱逐实例（按世界坐标索引）；其余类别同一模型、用canonical尺寸
    pillars = [(i["x"], i["y"], i["dx"], i["dy"], i["dz"])
               for i in items if i["cls"] == "pillar"]
    canon = {}
    for c in ("buoy", "block", "boat"):
        v = [i for i in items if i["cls"] == c]
        if v:
            canon[c] = (v[0]["dx"], v[0]["dy"], v[0]["dz"])

    records = ev.load_records(args.log)
    odom_t = [r["stamp"] for r in records["odom"]]
    o0, o1 = min(odom_t), max(odom_t)

    def remap(keys):                    # 与 v3 同一套线性速率对齐
        ts = [r["stamp"] for k in keys for r in records[k] if "stamp" in r]
        if not ts:
            return
        a, b = min(ts), max(ts)
        if b - a < 1e-6:
            return
        sc = (o1 - o0) / (b - a)
        for k in keys:
            for r in records[k]:
                r["stamp"] = o0 + (r["stamp"] - a) * sc

    remap([g for _, g, _ in CATS])
    remap([d for _, _, d in CATS])
    interp_odom = ev.make_odom_interpolator(records["odom"])

    out = {}
    for name, gk, dk in CATS:
        gi = ev.make_gt_interpolator(records[gk])
        dets, _, _ = ev.deduplicate_frames(records[dk])
        rows = {}
        for frame in dets:
            odom = interp_odom(frame["stamp"])
            gt = gi(frame["stamp"])
            if odom is None or gt is None:
                continue
            tw = [(p["x"], p["y"]) for p in gt["poses"]]          # 世界坐标
            tb = [ev.world_to_boat(p, odom) for p in gt["poses"]]  # 船体系
            pts = [(p["x"], p["y"]) for p in frame["poses"]]
            pairs, _ = ev.maximum_cardinality_matches(pts, tb, args.match_tol)
            for di, ti in pairs:
                m = frame["poses"][di]
                if name == "pillar":
                    gx, gy = tw[ti]
                    best, bd = None, 1e9
                    for px, py, ddx, ddy, ddz in pillars:
                        d2 = (px - gx) ** 2 + (py - gy) ** 2
                        if d2 < bd:
                            bd, best = d2, (ddx, ddy, ddz)
                    if best is None or bd > 9.0:      # 3m内才认为是同一根
                        continue
                    tdx, tdy, tdz = best
                else:
                    if name not in canon:
                        continue
                    tdx, tdy, tdz = canon[name]
                dist = math.hypot(*tb[ti])
                b = bucket(dist)
                if b is None:
                    continue
                e = (abs(m["dx"] - tdx), abs(m["dy"] - tdy), abs(m["dz"] - tdz))
                rows.setdefault(b, []).append(e)
        out[name] = rows

    print(f"世界文件: {args.world.split('/')[-1]}")
    print("口径: 长宽高误差**绝对值之和**（赛会2026-09-02），尺寸真值取自世界SDF\n")
    print(f"{'类别':<8}{'距离段':<12}{'样本':>7}{'|Δ长|':>9}{'|Δ宽|':>9}"
          f"{'|Δ高|':>9}{'三轴绝对值之和':>15}")
    for name, _, _ in CATS:
        for b in ("0-20m", "20-40m", "40-infm"):
            v = out[name].get(b)
            if not v or len(v) < 5:
                continue
            mx = statistics.median(e[0] for e in v)
            my = statistics.median(e[1] for e in v)
            mz = statistics.median(e[2] for e in v)
            tot = statistics.median(e[0] + e[1] + e[2] for e in v)
            print(f"{name:<8}{b:<12}{len(v):>7}{mx:>8.2f}m{my:>8.2f}m"
                  f"{mz:>8.2f}m{tot:>14.2f}m")
    print("\n[口径限制] 高度项含'雷达可见部分 vs 完整几何'的基准差，"
          "不能单独解读为高度估计误差，见本文件头部说明。")


if __name__ == "__main__":
    main()
