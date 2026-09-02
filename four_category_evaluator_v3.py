#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""四类目标评估 v3 —— 按赛会 2026-09-02 明确的口径。

官方定义（原文）：
    遗漏率     = 漏检的真值个数 ÷ 范围内的真值总数
    误检率     = 误检的检测个数 ÷ **检测总数**
    分类错误率 = 类别配错的个数 ÷ 检测总数
    "如果没有发布在指定的话题则认为没有检测出来"

与 v2 的差异（v2 的两项口径其实已经正确，不要重算它们）：
  - 遗漏率、误检率的分母 v2 已经对：v2 里 hit+fp == len(points) == 检测总数，
    hit+miss == 真值总数。v3 保持一致，数值应当可复现。
  - **分类错误率 v2 完全没有实现**，这是 v3 新增的唯一实质内容。
  - 连带影响：一个检测若匹配上了**其它类别**的真值，v2 把它记成本类别的误检；
    按新口径它是"分类错误"，不是误检。因此 v3 的误检率会低于 v2。
  - 被判为分类错误的检测，**在其真实类别里仍然算漏检**——这正是
    "没有发布在指定话题就算没检测出来"那句话的含义：分类错误单独统计，
    但不豁免对应类别的遗漏。v3 因此不改动遗漏率的算法。

复用 v2 里已验证过的时钟对齐、odom插值、坐标变换与最大基数二分匹配，
不重新实现（三时钟域对齐是这条线上历史上最容易出错的部分）。

用法:
    python3 four_category_evaluator_v3.py boat_log.jsonl [--match-tol 2.0]
"""

import argparse
import statistics
import sys

sys.path.insert(0, "/home/lyf040817/usv_ws")
import four_category_evaluator_v2 as ev

# (显示名, GT记录类型, 检测记录类型)
CATS = [
    ("boat",   "gt",        "det"),
    ("buoy",   "gt_buoy",   "det_buoy"),
    ("pillar", "gt_pillar", "det_pillar"),
    ("block",  "gt_block",  "det_block"),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log_path")
    ap.add_argument("--match-tol", type=float, default=2.0)
    ap.add_argument("--sensor-clock-gap", type=float, default=0.0)
    args = ap.parse_args()

    records = ev.load_records(args.log_path)
    if not records["odom"]:
        raise SystemExit("日志里没有 odom 记录")

    # ── 时间对齐 ──────────────────────────────────────────────
    # 本日志里存在三个时钟域，且**不能只用偏移对齐**：
    #   odom : 仿真时间 0.104~56.680   (跨度 56.6s)
    #   gt   : 录制当时的挂钟epoch      (跨度 93.0s)
    #   det  : 回放当天的挂钟epoch      (跨度 85.1s)
    # odom的stamp按约0.61倍实时推进（录制时仿真跑不满实时，已在eval_history
    # 中独立确认），所以gt与odom之间除偏移外还差一个**比例**；v2的
    # align_to_odom只做纯偏移，两端会差到±18s，GT插值必然落空——实测v2在本
    # 日志上skipped=733/733、全部指标为0，就是这个原因。
    #
    # 这里改为按各流自身的首末时刻做线性映射到odom时间轴。odom/gt/det都始于
    # 同一次回放、终于同一次回放，端点在物理上对应，因此线性映射是合理的。
    # 对齐是否成立由下面的静止目标位置误差自校验：立柱静止，若对齐错误其
    # 位置误差会显著变大。
    odom_t = [r["stamp"] for r in records["odom"]]
    o0, o1 = min(odom_t), max(odom_t)

    def remap(keys):
        ts = [r["stamp"] for k in keys for r in records[k] if "stamp" in r]
        if not ts:
            return None
        a, b = min(ts), max(ts)
        if b - a < 1e-6:
            return None
        scale = (o1 - o0) / (b - a)
        for k in keys:
            for r in records[k]:
                r["stamp"] = o0 + (r["stamp"] - a) * scale
        return (a, b, scale)

    gt_map  = remap([gk for _, gk, _ in CATS])
    det_map = remap([dk for _, _, dk in CATS])
    offset_source = "linear rate-corrected map to odom timeline"
    offset = 0.0
    print(f"[对齐] odom {o0:.3f}~{o1:.3f}")
    if gt_map:  print(f"[对齐] gt  {gt_map[0]:.3f}~{gt_map[1]:.3f} scale={gt_map[2]:.4f}")
    if det_map: print(f"[对齐] det {det_map[0]:.3f}~{det_map[1]:.3f} scale={det_map[2]:.4f}")
    if args.sensor_clock_gap:
        for _, _, dk in CATS:
            if dk == "det":       # boat 是唯一动态类别，默认不施加该修正
                continue
            for it in records[dk]:
                it["stamp"] -= args.sensor_clock_gap

    interp_odom = ev.make_odom_interpolator(records["odom"])
    gt_interp = {gk: ev.make_gt_interpolator(records[gk]) for _, gk, _ in CATS}
    dets = {dk: ev.deduplicate_frames(records[dk])[0] for _, _, dk in CATS}

    print(f"对齐方式: {offset_source}")
    print("口径: 遗漏率=漏检/真值总数  误检率=误检/检测总数  分类错误率=配错/检测总数")
    print("匹配: 一对一最大基数; 同一时间戳视为一帧\n")

    res = {}
    for name, gk, dk in CATS:
        hit = miss = fp = mis = 0
        errs = []
        for frame in dets[dk]:
            t = frame["stamp"]
            odom = interp_odom(t)
            if odom is None:
                continue
            gt_self = gt_interp[gk](t)
            if gt_self is None:
                continue

            points = [(p["x"], p["y"]) for p in frame["poses"]]
            targets = [ev.world_to_boat(p, odom) for p in gt_self["poses"]]
            pairs, pe = ev.maximum_cardinality_matches(points, targets, args.match_tol)
            matched_det = {d for d, _ in pairs}
            hit += len(pairs)
            miss += len(targets) - len(pairs)
            errs.extend(pe)

            # 未匹配上本类真值的检测：先看是否匹配得上**其它类别**的真值。
            # 匹配得上 -> 分类错误（不是误检）；都匹配不上 -> 真误检。
            leftover = [p for i, p in enumerate(points) if i not in matched_det]
            if not leftover:
                continue
            other = []
            for name2, gk2, _ in CATS:
                if gk2 == gk:
                    continue
                g2 = gt_interp[gk2](t)
                if g2 is None:
                    continue
                other.extend(ev.world_to_boat(p, odom) for p in g2["poses"])
            if other:
                p2, _ = ev.maximum_cardinality_matches(leftover, other, args.match_tol)
                mis += len(p2)
                fp += len(leftover) - len(p2)
            else:
                fp += len(leftover)

        det_total = hit + fp + mis           # 该类别发布出去的检测总数
        gt_total = hit + miss
        res[name] = dict(hit=hit, miss=miss, fp=fp, mis=mis,
                         det_total=det_total, gt_total=gt_total,
                         err=statistics.fmean(errs) if errs else 0.0)

    hdr = (f"{'类别':<8}{'检测总数':>9}{'真值总数':>9}{'命中':>7}{'漏检':>7}"
           f"{'误检':>7}{'分类错':>8}{'遗漏率':>9}{'误检率':>9}{'分类错误率':>11}{'位置误差':>10}")
    print(hdr)
    T = dict(hit=0, miss=0, fp=0, mis=0, det_total=0, gt_total=0)
    for name, _, _ in CATS:
        r = res[name]
        for k in T:
            T[k] += r[k]
        mr = 100 * r["miss"] / r["gt_total"] if r["gt_total"] else 0.0
        fr = 100 * r["fp"] / r["det_total"] if r["det_total"] else 0.0
        cr = 100 * r["mis"] / r["det_total"] if r["det_total"] else 0.0
        print(f"{name:<8}{r['det_total']:>9}{r['gt_total']:>9}{r['hit']:>7}{r['miss']:>7}"
              f"{r['fp']:>7}{r['mis']:>8}{mr:>8.1f}%{fr:>8.1f}%{cr:>10.1f}%{r['err']:>9.2f}m")
    mr = 100 * T["miss"] / T["gt_total"] if T["gt_total"] else 0.0
    fr = 100 * T["fp"] / T["det_total"] if T["det_total"] else 0.0
    cr = 100 * T["mis"] / T["det_total"] if T["det_total"] else 0.0
    print(f"{'合计':<8}{T['det_total']:>9}{T['gt_total']:>9}{T['hit']:>7}{T['miss']:>7}"
          f"{T['fp']:>7}{T['mis']:>8}{mr:>8.1f}%{fr:>8.1f}%{cr:>10.1f}%")
    print("\n[提示] 分类错误的检测在其真实类别里仍计漏检——这是"
          "「没有发布在指定话题就算没检测出来」的直接后果，不是重复计数。")


if __name__ == "__main__":
    main()
