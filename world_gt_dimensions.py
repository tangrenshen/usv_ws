#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从世界SDF提取逐实例的尺寸真值表（2026-08-31）。

**为什么必须从SDF取，而不是从GT话题取**：四类GT话题
（`/world/obstacles/{buoys,boats,pillars,blocks}`）是 PoseArray，**只有位置、
没有任何尺寸字段**；立柱在SDF里的 pose z 恒为0，连高度都读不出来。而赛会新
口径要求"长宽高误差绝对值之和"，高度项没有真值就无从算起。

**为什么尺寸不能用一个常数代替**：立柱高度是逐个随机的
（p2区 `random.uniform(5,10)` 共35根，p3区 `random.uniform(8,15)` 共1根），
每个实例都不同，必须逐实例取。

各类真值来源：
  pillar : 世界SDF内联的 <cylinder><radius>0.8</radius><length>h</length>
           —— 半径固定、高度逐实例随机
  buoy   : include 的 mb_round_buoy_{orange,black}，两者是**同一种物理模型**
           （球，<radius>0.25</radius>，直径0.5m），只有颜色不同。
           注意：结论章节曾记载"round型与marker型直径差30%、无类型字段故
           无法验证"，那个阻塞在本赛题场景下不成立——场景里根本没有第二种。
  block  : include 的 dock_block_4x4
  boat   : include 的 roboboat01/02

用法:
    python3 world_gt_dimensions.py <world.sdf> [--json out.json]
    python3 world_gt_dimensions.py --from-launch-log bags_5run_v2/launch_run1.log
"""

import argparse
import json
import os
import re
import sys

# 场景内 include 模型的 canonical 尺寸 (dx, dy, dz)，单位米。
# 浮球：球体 radius=0.25（已核对 ~/.gz/fuel 本地模型缓存）
# 船只：roboboat01/02，水面以上约0.4m（浮体1.15x0.22，cpubox顶面z=0.4）
INCLUDE_DIMS = {
    "mb_round_buoy_orange": (0.50, 0.50, 0.50),
    "mb_round_buoy_black":  (0.50, 0.50, 0.50),
    "roboboat01":           (1.15, 0.85, 0.40),
    "roboboat02":           (1.15, 0.85, 0.40),
    "dock_block_4x4":       (2.00, 2.00, 0.40),
}
CLASS_OF = {
    "mb_round_buoy_orange": "buoy",
    "mb_round_buoy_black":  "buoy",
    "roboboat01":           "boat",
    "roboboat02":           "boat",
    "dock_block_4x4":       "block",
}

PILLAR_RE = re.compile(
    r'<model\s+name="(pillar_[^"]+)">\s*<static>true</static>\s*'
    r'<pose>([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)[^<]*</pose>.*?'
    r'<cylinder><radius>([\d.]+)</radius><length>([\d.]+)</length>',
    re.S,
)
INCLUDE_RE = re.compile(
    r'<include>(?:<name>([^<]*)</name>)?<pose>([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)'
    r'[^<]*</pose><uri>([^<]+)</uri>'
)
# 浮块是内联 model，不是 include：碰撞体为 box，视觉是 dock_block_4x4.dae 网格
BLOCK_RE = re.compile(
    r'<model\s+name="(block_[^"]+)">\s*<pose>([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)[^<]*</pose>.*?'
    r'<box><size>([\d.]+)\s+([\d.]+)\s+([\d.]+)</size></box>',
    re.S,
)


def parse_world(path):
    text = open(path, encoding="utf-8", errors="ignore").read()
    items = []

    for m in PILLAR_RE.finditer(text):
        name, x, y, z, r, h = m.groups()
        d = 2 * float(r)
        items.append({
            "name": name, "cls": "pillar",
            "x": float(x), "y": float(y), "z": float(z),
            "dx": d, "dy": d, "dz": float(h),
            "source": "sdf_inline_cylinder",
        })

    for m in BLOCK_RE.finditer(text):
        name, x, y, z, sx, sy, sz = m.groups()
        items.append({
            "name": name, "cls": "block",
            "x": float(x), "y": float(y), "z": float(z),
            "dx": float(sx), "dy": float(sy), "dz": float(sz),
            # 注意：这里取的是**碰撞体** box 尺寸。视觉侧是
            # dock_block_4x4.dae 网格且带 (-0.25, 0.25) 偏移，而 Gazebo 的
            # GPU 雷达打的是视觉几何。两者若不一致，雷达实际看到的尺寸就不是
            # 这个 box —— 用本表算浮块轮廓误差前需要先核实这一点。
            "source": "sdf_inline_box_collision",
            "caveat": "visual mesh dock_block_4x4.dae may differ from collision box",
        })

    for m in INCLUDE_RE.finditer(text):
        name, x, y, z, uri = m.groups()
        model = uri.rstrip("/").split("/")[-1].replace(".dae", "")
        if model not in INCLUDE_DIMS:
            continue
        dx, dy, dz = INCLUDE_DIMS[model]
        items.append({
            "name": name or model, "cls": CLASS_OF[model],
            "x": float(x), "y": float(y), "z": float(z),
            "dx": dx, "dy": dy, "dz": dz,
            "model": model, "source": "include_canonical",
        })
    return items


def world_from_launch_log(path):
    """从launch日志里回溯出该次运行实际使用的世界SDF路径。"""
    text = open(path, encoding="utf-8", errors="ignore").read()
    m = re.search(r"(perception_user_random_\d+\.sdf)", text)
    if not m:
        return None
    fname = m.group(1)
    for base in (
        "/home/lyf040817/2026-challenge-cup/install/vrx_gz/share/vrx_gz/worlds",
        "/home/lyf040817/2026-challenge-cup/src/vrx/vrx_gz/worlds",
    ):
        p = os.path.join(base, fname)
        if os.path.exists(p):
            return p
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("world", nargs="?")
    ap.add_argument("--from-launch-log")
    ap.add_argument("--json")
    args = ap.parse_args()

    path = args.world
    if args.from_launch_log:
        path = world_from_launch_log(args.from_launch_log)
        if not path:
            sys.exit(f"从 {args.from_launch_log} 里找不到对应的世界SDF")
        print(f"世界文件: {path}")
    if not path or not os.path.exists(path):
        sys.exit("需要给出存在的世界SDF路径")

    items = parse_world(path)
    by_cls = {}
    for it in items:
        by_cls.setdefault(it["cls"], []).append(it)

    print(f"\n{'类别':<8}{'实例数':>7}{'dx':>18}{'dy':>18}{'dz(高)':>22}")
    for cls in ("buoy", "boat", "pillar", "block"):
        v = by_cls.get(cls, [])
        if not v:
            print(f"{cls:<8}{0:>7}")
            continue
        def rng(k):
            a = [i[k] for i in v]
            return f"{min(a):.2f}" if min(a) == max(a) else f"{min(a):.2f}~{max(a):.2f}"
        print(f"{cls:<8}{len(v):>7}{rng('dx'):>18}{rng('dy'):>18}{rng('dz'):>22}")

    pil = by_cls.get("pillar", [])
    if pil:
        hs = sorted(i["dz"] for i in pil)
        print(f"\n立柱高度逐实例分布: 最小={hs[0]:.2f}m 中位={hs[len(hs)//2]:.2f}m "
              f"最大={hs[-1]:.2f}m —— 逐实例随机，不能用单一常数代替")

    if args.json:
        json.dump(items, open(args.json, "w"), ensure_ascii=False, indent=1)
        print(f"\n已写出 {args.json}（{len(items)} 个实例）")


if __name__ == "__main__":
    main()
