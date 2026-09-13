# -*- coding: utf-8 -*-
"""修正对齐：真值(录制挂钟)与检测(回放挂钟)均为 1 倍速挂钟 → 二者只差常数偏移。
偏移由 jsonl 的接收顺序估计：每条检测记录写入时，最近一条已写入真值的 stamp 为 g_last，
则 d - g_last ∈ [off, off + 真值周期]。取中位数减半个真值周期。
随后 真值→odom 沿用 v3 的线性映射（二者都覆盖整个 bag，端点物理对应），检测经同一映射。
输出：四类遗漏/误检，以及修正后船只的偏移扫描（应在 Δ=0 取峰）。"""
import sys, json, statistics
sys.path.insert(0, '/home/lyf040817/usv_ws')
import four_category_evaluator_v2 as ev
run = sys.argv[1]
path = f'/home/lyf040817/eval_logs_0902/run{run}.jsonl'
recs = [json.loads(l) for l in open(path)]
g_last = None; diffs = []; gper = []
prev_g = None
for r in recs:
    if r['type'] == 'gt':
        if prev_g is not None: gper.append(r['stamp'] - prev_g)
        prev_g = g_last = r['stamp']
    elif r['type'] == 'det' and g_last is not None:
        diffs.append(r['stamp'] - g_last)
per = statistics.median(gper)
off = statistics.median(diffs) - per / 2
q = sorted(diffs)
print(f'run{run}: 真值周期 {per:.3f}s  d-g_last 分位 P5 {q[len(q)//20]-q[len(q)//2]:+.3f} P50 0 P95 {q[len(q)*19//20]-q[len(q)//2]:+.3f} (相对中位)  偏移 {off:.3f}s')

R = ev.load_records(path)
o = [r['stamp'] for r in R['odom']]; o0, o1 = min(o), max(o)
GK = ['gt', 'gt_buoy', 'gt_pillar', 'gt_block']; DK = ['det', 'det_buoy', 'det_pillar', 'det_block']
ts = [r['stamp'] for k in GK for r in R[k]]; a, b = min(ts), max(ts); s = (o1 - o0) / (b - a)
f = lambda t: o0 + (t - a) * s
for k in GK:
    for r in R[k]: r['stamp'] = f(r['stamp'])
for k in DK:
    for r in R[k]: r['stamp'] = f(r['stamp'] - off)
io = ev.make_odom_interpolator(R['odom'])
def score(gk, dk, sh=0.0):
    ig = ev.make_gt_interpolator([{'stamp': g['stamp'] + sh, 'poses': g['poses']} for g in R[gk]])
    hit = tot = nd = 0
    for fr in ev.deduplicate_frames(R[dk])[0]:
        od = io(fr['stamp']); g = ig(fr['stamp'])
        if od is None or g is None: continue
        tg = [ev.world_to_boat(p, od) for p in g['poses']]
        pts = [(p['x'], p['y']) for p in fr['poses']]
        pairs, _ = ev.maximum_cardinality_matches(pts, tg, 2.0)
        hit += len(pairs); tot += len(tg); nd += len(pts)
    return hit, tot, nd
for gk, dk in zip(GK, DK):
    h, t, n = score(gk, dk)
    print(f'  {gk:9s} 真值 {t:6d} 检测 {n:6d} 命中 {h:6d}  遗漏率 {100*(1-h/t):5.1f}%')
print('  船只偏移扫描(odom秒): ' + '  '.join(f'{sh:+.1f}:{100*(1-(lambda x: x[0]/x[1])(score("gt","det",sh))):.1f}'
                                          for sh in (-3, -2, -1, -0.5, 0, 0.5, 1, 2, 3)))
