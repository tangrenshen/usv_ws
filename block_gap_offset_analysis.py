#!/usr/bin/env python3
"""Block candidate-formation deep-dive: for each (block, frame) sample with
raw points within 4m of the GT position, compute:
  - centroid offset from GT center (magnitude + direction relative to the
    own-ship bearing, to test "GT-center vs point-cloud-centroid offset,
    systematically toward own-ship" hypothesis A vs "these points belong to
    a different object, scattered directions" hypothesis B)
  - max gap along the principal axis (same method as the boat cluster-glue
    check) and total extent
  - point count
split into "cluster formed within 2m" (control group, uses the same
cluster_diagnostic cross-reference as block_formation_rate.py) vs "no
cluster formed", to see what actually differs between the two groups.
"""
import math
import re
import sys
from collections import defaultdict

diag_path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/block_diag_run3.log"

BLOCK_RE = re.compile(r"\[BlockRawPts\] t=([\d.]+) block=(\d+) bx=([-\d.eE]+) by=([-\d.eE]+)(.*)")
PPAT = re.compile(r"p=([-\d.eE]+),([-\d.eE]+)")
CLUSTER_RE = re.compile(
    r"\[cluster_diagnostic\] t=[-\d.eE]+ center=\(([-\d.eE]+),([-\d.eE]+),[-\d.eE]+\) "
    r"size=\([-\d.eE]+,[-\d.eE]+,[-\d.eE]+\) fp_max=[-\d.eE]+ fp_min=[-\d.eE]+ "
    r"square=[-\d.eE]+ pts=\d+ measurement_t=([\d.]+) -> classification pending"
)

rows = []
with open(diag_path, encoding="utf-8", errors="replace") as f:
    for line in f:
        m = BLOCK_RE.search(line)
        if not m:
            continue
        try:
            t = float(m.group(1))
            idx = int(m.group(2))
            bx = float(m.group(3))
            by = float(m.group(4))
        except ValueError:
            continue
        pts = []
        bad = False
        for a, b in PPAT.findall(m.group(5)):
            try:
                pts.append((float(a), float(b)))
            except ValueError:
                bad = True
                break
        if bad:
            continue
        rows.append((t, idx, bx, by, pts))

print(f"parsed {len(rows)} BlockRawPts rows")

clusters_by_t = defaultdict(list)
with open(diag_path, encoding="utf-8", errors="replace") as f:
    for line in f:
        m = CLUSTER_RE.search(line)
        if m:
            try:
                cx, cy = float(m.group(1)), float(m.group(2))
                mt = round(float(m.group(3)), 3)
            except ValueError:
                continue
            clusters_by_t[mt].append((cx, cy))

print(f"parsed cluster_diagnostic across {len(clusters_by_t)} frame timestamps")


def analyze_points(pts):
    n = len(pts)
    if n == 0:
        return None
    mx = sum(p[0] for p in pts) / n
    my = sum(p[1] for p in pts) / n
    if n < 2:
        return {"n": n, "cx": mx, "cy": my, "max_gap": 0.0, "extent": 0.0}
    cxx = sum((p[0] - mx) ** 2 for p in pts) / n
    cyy = sum((p[1] - my) ** 2 for p in pts) / n
    cxy = sum((p[0] - mx) * (p[1] - my) for p in pts) / n
    tr = cxx + cyy
    d = math.sqrt(max(0.0, (cxx - cyy) ** 2 + 4 * cxy * cxy))
    l1 = (tr + d) / 2.0
    vx, vy = l1 - cyy, cxy
    if abs(vx) + abs(vy) < 1e-9:
        vx, vy = 1.0, 0.0
    norm = math.hypot(vx, vy)
    vx, vy = vx / norm, vy / norm
    projs = sorted((p[0] - mx) * vx + (p[1] - my) * vy for p in pts)
    max_gap = max((projs[i] - projs[i - 1] for i in range(1, len(projs))), default=0.0)
    extent = projs[-1] - projs[0]
    return {"n": n, "cx": mx, "cy": my, "max_gap": max_gap, "extent": extent}


def quantile(vals, q):
    vals = sorted(vals)
    if not vals:
        return float("nan")
    idx = min(int(len(vals) * q), len(vals) - 1)
    return vals[idx]


formed_group = []
not_formed_group = []
offset_angles_rel_bearing = []  # angle between (GT->centroid) and (own-ship->GT), degrees

for t, idx, bx, by, pts in rows:
    tk = round(t, 3)
    candidates = clusters_by_t.get(tk, [])
    has_nearby = any(math.hypot(cx - bx, cy - by) < 2.0 for cx, cy in candidates)

    stats = analyze_points(pts)
    entry = {"t": t, "idx": idx, "bx": bx, "by": by, "stats": stats}
    if has_nearby:
        formed_group.append(entry)
    else:
        not_formed_group.append(entry)

    if stats is not None and stats["n"] >= 3:
        # own-ship -> GT bearing (own-ship is origin in base_link)
        bearing_angle = math.atan2(by, bx)
        # GT -> centroid offset direction
        offset_x = stats["cx"] - bx
        offset_y = stats["cy"] - by
        offset_mag = math.hypot(offset_x, offset_y)
        if offset_mag > 0.3:  # only meaningful offsets
            offset_angle = math.atan2(offset_y, offset_x)
            rel = math.degrees(offset_angle - bearing_angle)
            rel = (rel + 180) % 360 - 180
            offset_angles_rel_bearing.append((rel, offset_mag, math.hypot(bx, by)))


def summarize_group(name, group):
    with_pts = [e for e in group if e["stats"] is not None and e["stats"]["n"] >= 3]
    print(f"\n{name}: {len(group)} total, {len(with_pts)} with >=3 raw points")
    if not with_pts:
        return
    gaps = [e["stats"]["max_gap"] for e in with_pts]
    extents = [e["stats"]["extent"] for e in with_pts]
    ns = [e["stats"]["n"] for e in with_pts]
    print(f"  max_gap:  p25={quantile(gaps,0.25):.2f} median={quantile(gaps,0.5):.2f} "
          f"p75={quantile(gaps,0.75):.2f} p90={quantile(gaps,0.9):.2f}")
    print(f"  extent:   p25={quantile(extents,0.25):.2f} median={quantile(extents,0.5):.2f} "
          f"p75={quantile(extents,0.75):.2f} p90={quantile(extents,0.9):.2f}")
    print(f"  n_points: p25={quantile(ns,0.25):.0f} median={quantile(ns,0.5):.0f} "
          f"p75={quantile(ns,0.75):.0f} p90={quantile(ns,0.9):.0f}")


summarize_group("formed (cluster within 2m of GT) - control group", formed_group)
summarize_group("not formed (no cluster within 2m of GT)", not_formed_group)

print(f"\n--- offset direction analysis (n={len(offset_angles_rel_bearing)}, offset_mag>0.3m) ---")
print("angle = (GT->point-centroid direction) - (own-ship->GT bearing), degrees")
print("  0 deg = offset points AWAY from own-ship (centroid beyond GT)")
print("180 deg = offset points TOWARD own-ship (centroid between own-ship and GT, i.e. near face)")
if offset_angles_rel_bearing:
    angles = [a for a, m, d in offset_angles_rel_bearing]
    mags = [m for a, m, d in offset_angles_rel_bearing]
    near_180 = sum(1 for a in angles if abs(abs(a) - 180) < 45)
    near_0 = sum(1 for a in angles if abs(a) < 45)
    scattered = len(angles) - near_180 - near_0
    n = len(angles)
    print(f"  near 180deg (toward own-ship, 'near face' hypothesis A): {near_180}/{n} = {100.0*near_180/n:.1f}%")
    print(f"  near 0deg (away from own-ship):                          {near_0}/{n} = {100.0*near_0/n:.1f}%")
    print(f"  scattered (neither):                                     {scattered}/{n} = {100.0*scattered/n:.1f}%")
    print(f"  offset magnitude: median={quantile(mags,0.5):.2f}m p90={quantile(mags,0.9):.2f}m")
