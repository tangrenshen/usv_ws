#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""立柱误检率按赛会新口径重算（2026-08-31）。

赛会裁决：「只要激光雷达能检测到的，都会认为是立柱，/world/obstacles/pillars
只是参考」。因此匹配不上GT的检出，如果对应的是场景中真实存在的静态结构，
不应计为误检。

难点是怎么把「真实结构」和「噪声」分开，而不循环论证。本脚本用的判据是
**跨run的世界坐标一致性**：

  - 五次运行的本船世界位置互不相同（实测 (-702,333)/(-711,350)/(-713,345)/
    (-720,356)/(-713,333)），所以任何与船体相关的噪声（水面反射、尾流、
    近场杂波）在不同run里会落到**不同的世界坐标**；
  - 而环境模型自带的静态结构（码头桩等，属 sydney_regatta 网格，不是
    generator 摆放的 pillar_* 模型）在世界坐标下是**固定**的，会在多个run
    的同一世界位置重复出现。

注意方向：这里用的是"世界坐标跨run重复"，不是"相对本船跨run重复"。后者
不成立——世界生成器是相对本船出生点摆放障碍物的，相对坐标本来就跨run一致
（这个坑在 report/stitching_reprojection_chapter.md §6.1.3 有记录）。

另注：generator 摆放的36个 pillar_* 在世界固定多边形内随机取位，各run不同，
因此不会被本判据误判为环境结构；何况它们本来就在GT里。

用法:
  python3 pillar_fp_recompute.py run1.jsonl run2.jsonl ... [--match-tol 2.0]
"""

import argparse
import json
import math
import sys
from collections import defaultdict

sys.path.insert(0, "/home/lyf040817/usv_ws")
import four_category_evaluator_v2 as ev

CLUSTER_R = 2.5      # 同一结构的世界坐标聚类半径(米)
CROSS_RUN_R = 3.0    # 跨run认定为同一结构的世界坐标容差(米)
MIN_RUNS = 2         # 至少在几个独立run里出现才判为真实环境结构


def boat_to_world(px, py, odom):
    """world_to_boat 的逆变换（与 ev.world_to_boat 保持同一约定）。"""
    yaw = ev.quat_to_yaw(odom)
    c, s = math.cos(yaw), math.sin(yaw)
    return odom["x"] + px * c - py * s, odom["y"] + px * s + py * c


def cluster_points(points, radius):
    """简单单链聚类，返回 [(cx, cy, n_points, n_frames)]。"""
    clusters = []          # [[sumx, sumy, n, set(frame_idx)]]
    for x, y, fi in points:
        placed = False
        for c in clusters:
            if math.hypot(c[0] / c[2] - x, c[1] / c[2] - y) <= radius:
                c[0] += x; c[1] += y; c[2] += 1; c[3].add(fi)
                placed = True
                break
        if not placed:
            clusters.append([x, y, 1, {fi}])
    return [(c[0] / c[2], c[1] / c[2], c[2], len(c[3])) for c in clusters]


def analyse_run(path, match_tol, sensor_clock_gap=0.0):
    """严格复刻 four_category_evaluator_v2.main() 的预处理，否则时间对不齐。

    真值话题(epoch域)、检测输出(发布时挂钟域)、本船位姿(仿真时间域)分属三个
    时钟域，必须先 resolve_epoch_offset + align_to_odom 才能做位置匹配。
    """
    records = ev.load_records(path)
    if not records["odom"]:
        raise ValueError("日志里没有 odom 记录")
    offset, offset_source = ev.resolve_epoch_offset(records, None, True)
    odom_mid = records["odom"][len(records["odom"]) // 2]["stamp"]
    for record_type, items in records.items():
        if record_type != "odom":
            ev.align_to_odom(items, offset, odom_mid)
    if sensor_clock_gap:
        for item in records["det_pillar"]:
            item["stamp"] -= sensor_clock_gap

    interp_odom = ev.make_odom_interpolator(records["odom"])
    interp_gt = ev.make_gt_interpolator(records["gt_pillar"])
    dets, _, _ = ev.deduplicate_frames(records["det_pillar"])

    hit = miss = fp = 0
    unmatched = []                       # (world_x, world_y, frame_index)
    for fi, frame in enumerate(dets):
        t = frame["stamp"]
        odom = interp_odom(t)
        gt = interp_gt(t)
        if odom is None or gt is None:
            continue
        targets = [ev.world_to_boat(p, odom) for p in gt["poses"]]
        points = [(p["x"], p["y"]) for p in frame["poses"]]
        pairs, _ = ev.maximum_cardinality_matches(points, targets, match_tol)
        matched_det = {a for a, _ in pairs} if pairs and isinstance(pairs[0], tuple) else set()
        hit += len(pairs)
        miss += len(targets) - len(pairs)
        fp += len(points) - len(pairs)
        for i, (px, py) in enumerate(points):
            if i not in matched_det:
                unmatched.append(boat_to_world(px, py, odom) + (fi,))
    return {
        "path": path, "hit": hit, "miss": miss, "fp": fp,
        "offset_source": offset_source,
        "unmatched": unmatched,
        "clusters": cluster_points(unmatched, CLUSTER_R),
        "n_frames": len(dets),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--match-tol", type=float, default=2.0)
    ap.add_argument("--min-runs", type=int, default=MIN_RUNS)
    ap.add_argument("--sensor-clock-gap", type=float, default=0.0,
                    help="与 four_category_evaluator_v2 同义，只作用于 det_pillar")
    args = ap.parse_args()

    runs = []
    for p in args.logs:
        try:
            runs.append(analyse_run(p, args.match_tol, args.sensor_clock_gap))
        except Exception as exc:                     # noqa: BLE001
            print(f"[跳过] {p}: {exc}")
    if not runs:
        print("没有可用的日志"); return

    print(f"{'run':<28}{'命中':>7}{'遗漏':>7}{'误检':>8}{'旧误检率':>10}{'未匹配簇':>10}")
    for r in runs:
        old_fpr = 100 * r["fp"] / (r["hit"] + r["fp"]) if r["hit"] + r["fp"] else 0
        print(f"{r['path'].split('/')[-1]:<28}{r['hit']:>7}{r['miss']:>7}{r['fp']:>8}"
              f"{old_fpr:>9.1f}%{len(r['clusters']):>10}")

    # 跨run匹配：同一世界位置在≥min_runs个run里都出现 -> 真实环境结构
    allc = [(c, ri) for ri, r in enumerate(runs) for c in r["clusters"]]
    groups = []                                   # [[cx,cy,n,set(run_idx),pts]]
    for (cx, cy, npts, nfr), ri in allc:
        for g in groups:
            if math.hypot(g[0] / g[2] - cx, g[1] / g[2] - cy) <= CROSS_RUN_R:
                g[0] += cx; g[1] += cy; g[2] += 1; g[3].add(ri); g[4] += npts
                break
        else:
            groups.append([cx, cy, 1, {ri}, npts])

    persistent = [g for g in groups if len(g[3]) >= args.min_runs]
    print(f"\n跨run世界坐标聚合: 共{len(groups)}个位置, "
          f"其中出现在≥{args.min_runs}个独立run的有 **{len(persistent)}** 个"
          f"（判为真实环境结构，按新口径不计误检）")

    print(f"\n{'run':<28}{'旧误检率':>10}{'结构性检出':>12}{'新误检率':>10}{'变化':>9}")
    for ri, r in enumerate(runs):
        struct_pts = 0
        for cx, cy, npts, _ in r["clusters"]:
            for g in persistent:
                if math.hypot(g[0] / g[2] - cx, g[1] / g[2] - cy) <= CROSS_RUN_R:
                    struct_pts += npts
                    break
        new_fp = r["fp"] - struct_pts
        old_fpr = 100 * r["fp"] / (r["hit"] + r["fp"]) if r["hit"] + r["fp"] else 0
        new_fpr = 100 * new_fp / (r["hit"] + new_fp) if r["hit"] + new_fp else 0
        print(f"{r['path'].split('/')[-1]:<28}{old_fpr:>9.1f}%{struct_pts:>12}"
              f"{new_fpr:>9.1f}%{new_fpr - old_fpr:>+8.1f}pp")

    print("\n[口径说明] 召回率一侧未改动：那48个未标注簇是**已检出**的，"
          "把它们计入分母也同时计入分子，召回率只会变好；真正会拉低召回的是"
          "「存在但未被检出的未标注结构」，而这类东西无法从我们自己的检出结果里"
          "枚举，需要赛会明确遗漏率的分母口径。")


if __name__ == "__main__":
    main()
