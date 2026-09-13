# -*- coding: utf-8 -*-
"""逐目标明细（口径B，周期=一次完整测试）：每个真值目标的中位距离、是否被识别到、是否在覆盖内。
输出 pickle 供绘图使用。"""
import sys, math, pickle, statistics as st
sys.path.insert(0, '/home/lyf040817/usv_ws')
import four_category_evaluator_v2 as ev
TAG = sys.argv[1] if len(sys.argv) > 1 else 'combo'
CATS = [('浮球', 'gt_buoy', 'det_buoy'), ('浮块', 'gt_block', 'det_block'),
        ('立柱', 'gt_pillar', 'det_pillar'), ('船只', 'gt', 'det')]
def covered(bx, by):
    if math.hypot(bx, by) <= 60.0: return True
    dx, dy = bx - 7.0, by
    return math.hypot(dx, dy) <= 200.0 and abs(math.degrees(math.atan2(dy, dx))) <= 60.0
rows = []
for i in range(1, 6):
    R = ev.load_records(f'/home/lyf040817/boatmiss/{TAG}/run{i}.jsonl')
    io = ev.make_odom_interpolator(R['odom'])
    for name, gk, dk in CATS:
        ig = ev.make_gt_interpolator(R[gk]); D = {}; seen = set(); cov = set()
        for fr in ev.deduplicate_frames(R[dk])[0]:
            t = fr['stamp']; od = io(t); g = ig(t)
            if od is None or g is None: continue
            tg = [ev.world_to_boat(p, od) for p in g['poses']]
            for j, (bx, by) in enumerate(tg):
                D.setdefault(j, []).append(math.hypot(bx, by))
                if covered(bx, by): cov.add(j)
            pairs, _ = ev.maximum_cardinality_matches([(p['x'], p['y']) for p in fr['poses']], tg, 2.0)
            seen |= {j for _, j in pairs}
        for j, d in D.items():
            rows.append(dict(run=i, cat=name, idx=j, dist=st.median(d),
                             dmin=min(d), seen=j in seen, cov=j in cov))
pickle.dump(rows, open(f'/home/lyf040817/boatmiss/{TAG}/obj_detail.pkl', 'wb'))
n = len(rows)
print(f'{TAG}: 目标实例 {n}（5次×四类）  覆盖内 {sum(r["cov"] for r in rows)}  识别到 {sum(r["seen"] for r in rows)}')
for name, _, _ in CATS:
    s = [r for r in rows if r['cat'] == name and r['cov']]
    ms = [r for r in s if not r['seen']]
    hs = [r for r in s if r['seen']]
    f = lambda v: f'{st.median(v):.1f}' if v else '—'
    print(f'  {name}: 覆盖内 {len(s)}  漏检 {len(ms)}  漏检中位距离 {f([r["dist"] for r in ms])} m'
          f'  检出中位距离 {f([r["dist"] for r in hs])} m'
          f'  漏检最近距离 {f([r["dmin"] for r in ms])} m')
