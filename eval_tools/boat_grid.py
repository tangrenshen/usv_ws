# -*- coding: utf-8 -*-
"""船只判据网格搜索（进程内快速评估，口径同 v4：一对一最大基数、2m、误检=既不匹配船也不匹配他类真值）。
船只分支位于级联末端，放宽只会接走原本被丢弃/兜底的簇；浮球兜底在船只之后，
故另报 buoy_fallback 数量变化以确认对浮球的影响可忽略。"""
import sys, math, pickle, itertools
from collections import defaultdict
sys.path.insert(0, '/home/lyf040817/usv_ws'); sys.path.insert(0, '/home/lyf040817/cascade_exp')
import four_category_evaluator_v2 as ev
from cascade import classify, BASE
TAG = sys.argv[1] if len(sys.argv) > 1 else 'base'
D = f'/home/lyf040817/boatmiss/{TAG}'
CL = pickle.load(open(f'{D}/clusters.pkl', 'rb'))
FR = {}   # run -> list of (clusters, boat_targets, other_targets)
for i in range(1, 6):
    R = ev.load_records(f'{D}/run{i}.jsonl')
    io = ev.make_odom_interpolator(R['odom'])
    ig = {k: ev.make_gt_interpolator(R[k]) for k in ('gt', 'gt_buoy', 'gt_pillar', 'gt_block')}
    byt = defaultdict(list)
    for c in CL[i]: byt[round(c['t'], 6)].append(c)
    L = []
    for fr in ev.deduplicate_frames(R['det'])[0]:
        if fr.get('hstamp') is None: continue
        t = fr['stamp']; od = io(t); g = ig['gt'](t)
        if od is None or g is None: continue
        oth = []
        for k in ('gt_buoy', 'gt_pillar', 'gt_block'):
            g2 = ig[k](t)
            if g2: oth += [ev.world_to_boat(p, od) for p in g2['poses']]
        L.append((byt.get(round(fr['hstamp'], 6), []), [ev.world_to_boat(p, od) for p in g['poses']], oth))
    FR[i] = L
def score(P):
    out = []
    for i in range(1, 6):
        hit = miss = fp = mis = bf = 0
        for cls, tg, oth in FR[i]:
            pts = []
            for c in cls:
                lab = classify(c, P)
                if lab in ('boat', 'boat_fallback'): pts.append((c['cx'], c['cy']))
                elif lab == 'buoy_fallback': bf += 1
            pairs, _ = ev.maximum_cardinality_matches(pts, tg, 2.0)
            hit += len(pairs); miss += len(tg) - len(pairs)
            md = {d for d, _ in pairs}; left = [p for k, p in enumerate(pts) if k not in md]
            if left and oth:
                p2, _ = ev.maximum_cardinality_matches(left, oth, 2.0); mis += len(p2); fp += len(left) - len(p2)
            else: fp += len(left)
        n = hit + fp + mis
        out.append((100 * miss / (hit + miss), 100 * fp / n if n else 0, 100 * mis / n if n else 0, bf))
    return out
def fmt(name, r):
    m = [x[0] for x in r]; f = [x[1] for x in r]; c = [x[2] for x in r]
    return (f'{name:44s} 遗漏 {sum(m)/5:5.1f} [{min(m):4.1f}-{max(m):4.1f}]  误检 {sum(f)/5:4.1f} [{min(f):4.1f}-{max(f):4.1f}]'
            f'  分类错 {sum(c)/5:4.1f}  浮球兜底 {sum(x[3] for x in r)}')
if __name__ == '__main__':
    print(fmt('基线', score(BASE)))
    grid = dict(boat_min_pts=[40, 25, 20, 15, 12, 10], boat_fp_max_sane=[2.0, 2.5, 3.0],
                boat_h_max=[1.8, 2.5], boat_fp_min=[0.9, 0.6], far_reject=[True, False],
                boat_fallback_min_pts=[25, 12])
    keys = list(grid); rows = []
    for vals in itertools.product(*grid.values()):
        P = dict(BASE); P.update(zip(keys, vals))
        r = score(P); rows.append((vals, r))
    pickle.dump((keys, rows), open(f'{D}/boat_grid.pkl', 'wb'))
    # Pareto：按平均误检分档，取遗漏最低
    for cap in (1, 2, 3, 5, 8):
        ok = [(v, r) for v, r in rows if sum(x[1] for x in r) / 5 <= cap]
        v, r = min(ok, key=lambda vr: sum(x[0] for x in vr[1]))
        print(fmt(f'误检≤{cap}%: ' + ' '.join(f'{k.replace("boat_","")}={x}' for k, x in zip(keys, v) if BASE[k] != x), r))
