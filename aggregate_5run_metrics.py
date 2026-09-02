#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""跨5次独立运行汇总四类指标（赛会2026-09-02口径）。

评分细则原文：「测试进行5次不同初始化条件下的独立运行，取平均成绩作为最终
测试结果」——这一层点名要求**算术平均**，不套稳健统计（n=5太小，且原文
写法明确）。与拼接重投影误差那条线的层2处理保持一致。

逐run先各自算出遗漏率/误检率/分类错误率（分母是该run自己的真值总数/检测
总数），再对5个百分比取算术平均——而不是把5个run的计数先合并再算比率。
后者会让样本多的run隐性加权，不符合"取平均成绩"的字面含义。

用法:
    python3 aggregate_5run_metrics.py eval_logs/run*.jsonl
"""

import argparse
import io
import re
import statistics
import subprocess
import sys

CATS = ["boat", "buoy", "pillar", "block", "合计"]
ROW = re.compile(
    r"^(boat|buoy|pillar|block|合计)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+"
    r"([\d.]+)%\s+([\d.]+)%\s+([\d.]+)%(?:\s+([\d.]+)m)?"
)


def run_one(path, match_tol):
    out = subprocess.run(
        [sys.executable, "/home/lyf040817/usv_ws/four_category_evaluator_v3.py",
         path, "--match-tol", str(match_tol)],
        capture_output=True, text=True, timeout=900)
    res = {}
    for line in out.stdout.splitlines():
        m = ROW.match(line.strip())
        if m:
            g = m.groups()
            res[g[0]] = dict(det=int(g[1]), gt=int(g[2]), hit=int(g[3]),
                             miss=int(g[4]), fp=int(g[5]), mis=int(g[6]),
                             mr=float(g[7]), fr=float(g[8]), cr=float(g[9]),
                             err=float(g[10]) if g[10] else None)
    if not res:
        sys.stderr.write(out.stdout[-2000:] + out.stderr[-2000:])
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--match-tol", type=float, default=2.0)
    args = ap.parse_args()

    per = []
    for p in args.logs:
        r = run_one(p, args.match_tol)
        if r:
            per.append((p.split("/")[-1], r))
            print(f"  已处理 {p.split('/')[-1]}")
        else:
            print(f"  [跳过] {p.split('/')[-1]}：未解析出结果")
    if not per:
        raise SystemExit("没有可用结果")

    n = len(per)
    if n != 5:
        print(f"\n[口径警告] 本次只汇总了 {n} 次运行，评分细则要求 5 次独立运行。"
              f"这不是最终成绩，不要当作最终数字引用。")

    print(f"\n===== {n} 次独立运行的逐run结果 =====")
    for name in CATS:
        vals = [(tag, r[name]) for tag, r in per if name in r]
        if not vals:
            continue
        print(f"\n[{name}]")
        print(f"  {'run':<16}{'遗漏率':>9}{'误检率':>9}{'分类错误率':>11}{'位置误差':>10}")
        for tag, v in vals:
            e = f"{v['err']:.2f}m" if v["err"] is not None else "-"
            print(f"  {tag:<16}{v['mr']:>8.1f}%{v['fr']:>8.1f}%{v['cr']:>10.1f}%{e:>10}")
        for key, lbl in (("mr", "遗漏率"), ("fr", "误检率"), ("cr", "分类错误率")):
            xs = [v[key] for _, v in vals]
            print(f"  {lbl}: 均值={statistics.fmean(xs):.1f}%  "
                  f"区间={min(xs):.1f}~{max(xs):.1f}%"
                  + (f"  标准差={statistics.stdev(xs):.1f}pp" if len(xs) > 1 else ""))
        errs = [v["err"] for _, v in vals if v["err"] is not None]
        if errs:
            print(f"  位置误差: 均值={statistics.fmean(errs):.2f}m  "
                  f"区间={min(errs):.2f}~{max(errs):.2f}m")

    print("\n[呈现原则] 任何汇报的数字必须同时给出5次运行的区间，不能只写均值——"
          "已知跨场景方差很大，只写均值是不完整的呈现。")


if __name__ == "__main__":
    main()
