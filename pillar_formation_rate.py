#!/usr/bin/env python3
"""Pillar candidate-formation-rate diagnosis, distance-bucketed.

Cross-references [PillarRawPts] (per-pillar, per-frame raw point count within
1m/3m, computed from known GT positions transformed to base_link at runtime)
against [cluster_diagnostic] first-prints (fires for every formed cluster
regardless of eventual classification) to separate:
  - candidate formed nearby (2m) -> classification-stage question, already
    answered clean by pillar_predicate_diagnosis.py
  - no candidate formed -> bucket by raw point count:
      0 points          -> physical blind spot, unfixable
      1-3 points         -> below min_cluster_pts=4, a threshold question
      >=4 points, no cluster -> clustering-parameter question (the one
                                 actually worth investigating)
"""
import re
import sys
from collections import defaultdict

diag_path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/pillar_diag_run.log"

PILLAR_RE = re.compile(
    r"\[PillarRawPts\] t=([\d.]+) pillar=(\d+) bx=([-\d.eE]+) by=([-\d.eE]+) "
    r"n1m=(\d+) n3m=(\d+)"
)
CLUSTER_RE = re.compile(
    r"\[cluster_diagnostic\] t=[-\d.eE]+ center=\(([-\d.eE]+),([-\d.eE]+),[-\d.eE]+\) "
    r"size=\([-\d.eE]+,[-\d.eE]+,[-\d.eE]+\) fp_max=[-\d.eE]+ fp_min=[-\d.eE]+ "
    r"square=[-\d.eE]+ pts=\d+ measurement_t=([\d.]+) -> classification pending"
)

pillar_rows = []
with open(diag_path, encoding="utf-8", errors="replace") as f:
    for line in f:
        m = PILLAR_RE.search(line)
        if m:
            try:
                t = float(m.group(1))
                idx = int(m.group(2))
                bx = float(m.group(3))
                by = float(m.group(4))
                n1 = int(m.group(5))
                n3 = int(m.group(6))
            except ValueError:
                continue
            pillar_rows.append((t, idx, bx, by, n1, n3))

print(f"parsed {len(pillar_rows)} PillarRawPts rows")

clusters_by_t = defaultdict(list)
skipped = 0
with open(diag_path, encoding="utf-8", errors="replace") as f:
    for line in f:
        m = CLUSTER_RE.search(line)
        if m:
            try:
                cx, cy = float(m.group(1)), float(m.group(2))
                mt = round(float(m.group(3)), 3)
            except ValueError:
                skipped += 1
                continue
            clusters_by_t[mt].append((cx, cy))

print(f"parsed {sum(len(v) for v in clusters_by_t.values())} cluster_diagnostic "
      f"first-prints across {len(clusters_by_t)} distinct frame timestamps "
      f"(skipped {skipped} corrupted)")


def bucket(dist):
    if dist < 20:
        return "0-20m"
    if dist < 40:
        return "20-40m"
    if dist < 60:
        return "40-60m"
    return "60m+"


import math

formed = defaultdict(int)
total = defaultdict(int)
no_candidate_n1 = defaultdict(list)
no_candidate_n3 = defaultdict(list)

for t, idx, bx, by, n1, n3 in pillar_rows:
    dist = math.hypot(bx, by)
    b = bucket(dist)
    total[b] += 1
    tk = round(t, 3)
    candidates = clusters_by_t.get(tk, [])
    has_nearby = any(math.hypot(cx - bx, cy - by) < 2.0 for cx, cy in candidates)
    if has_nearby:
        formed[b] += 1
    else:
        no_candidate_n1[b].append(n1)
        no_candidate_n3[b].append(n3)

print(f"\n{'bucket':>8} {'total':>7} {'candidate_formed':>18} {'formation_rate':>16}")
for b in ("0-20m", "20-40m", "40-60m", "60m+"):
    t_, f_ = total[b], formed[b]
    if t_ == 0:
        continue
    print(f"{b:>8} {t_:>7} {f_:>18} {100.0*f_/t_:>15.1f}%")

print("\n--- no-candidate cases: raw point count breakdown (1m radius) ---")
print(f"{'bucket':>8} {'no_cand_n':>10} {'0pt':>8} {'1-3pt':>8} {'>=4pt':>8}")
for b in ("0-20m", "20-40m", "40-60m", "60m+"):
    vals = no_candidate_n1[b]
    if not vals:
        continue
    n0 = sum(1 for v in vals if v == 0)
    n13 = sum(1 for v in vals if 1 <= v <= 3)
    n4 = sum(1 for v in vals if v >= 4)
    n = len(vals)
    print(f"{b:>8} {n:>10} {100.0*n0/n:>6.1f}% {100.0*n13/n:>6.1f}% {100.0*n4/n:>6.1f}%")

print("\n--- same breakdown at 3m radius (checking if points exist just outside cluster_tol) ---")
print(f"{'bucket':>8} {'no_cand_n':>10} {'0pt':>8} {'1-3pt':>8} {'>=4pt':>8}")
for b in ("0-20m", "20-40m", "40-60m", "60m+"):
    vals = no_candidate_n3[b]
    if not vals:
        continue
    n0 = sum(1 for v in vals if v == 0)
    n13 = sum(1 for v in vals if 1 <= v <= 3)
    n4 = sum(1 for v in vals if v >= 4)
    n = len(vals)
    print(f"{b:>8} {n:>10} {100.0*n0/n:>6.1f}% {100.0*n13/n:>6.1f}% {100.0*n4/n:>6.1f}%")

overall_total = sum(total.values())
overall_formed = sum(formed.values())
print(f"\noverall candidate-formation rate: {overall_formed}/{overall_total} = "
      f"{100.0*overall_formed/overall_total:.1f}%")
