# -*- coding: utf-8 -*-
"""按"周期"口径算遗漏率：在一个周期窗口内，某个真值目标只要被识别到一次即算识别到（0/1）。
分子=周期内未被识别到的目标数，分母=该周期内存在的真值目标数；逐周期累加，再逐run平均。
逐帧匹配沿用赛会口径：同类别一对一最大基数匹配、容差2m；判错类别者在本类仍算未识别。
用法: python3 miss_window.py <tag> [窗口秒,逗号分隔]"""
import sys, json
sys.path.insert(0, '/home/lyf040817/usv_ws')
import four_category_evaluator_v2 as ev
TAG = sys.argv[1]
WINS = [float(x) for x in sys.argv[2].split(',')] if len(sys.argv) > 2 else [0, 0.5, 1, 2, 5, 10, 1e9]
CATS = [('浮球', 'gt_buoy', 'det_buoy'), ('浮块', 'gt_block', 'det_block'),
        ('立柱', 'gt_pillar', 'det_pillar'), ('船只', 'gt', 'det')]
res = {}
for i in range(1, 6):
    R = ev.load_records(f'/home/lyf040817/boatmiss/{TAG}/run{i}.jsonl')
    io = ev.make_odom_interpolator(R['odom'])
    for name, gk, dk in CATS:
        ig = ev.make_gt_interpolator(R[gk])
        obs = []           # (t, 目标总数, 被匹配上的目标索引集合)
        for fr in ev.deduplicate_frames(R[dk])[0]:
            t = fr['stamp']; od = io(t); g = ig(t)
            if od is None or g is None: continue
            tg = [ev.world_to_boat(p, od) for p in g['poses']]
            pairs, _ = ev.maximum_cardinality_matches([(p['x'], p['y']) for p in fr['poses']], tg, 2.0)
            obs.append((t, len(tg), {j for _, j in pairs}))
        if not obs: continue
        t0 = obs[0][0]
        for w in WINS:
            buckets = {}
            for t, n, hit in obs:
                b = 0 if w >= 1e8 else (len(buckets) if w == 0 else int((t - t0) // w))
                k = (b, t) if w == 0 else b
                cur = buckets.setdefault(k, [n, set()])
                cur[0] = max(cur[0], n); cur[1] |= hit
            miss = tot = 0
            for n, hit in buckets.values():
                tot += n; miss += n - len(hit)
            res.setdefault((name, w), []).append((miss, tot))
hdr = '窗口'.ljust(10) + ''.join(f'{n:>18s}' for n, _, _ in CATS) + f'{"合计(加权)":>18s}'
print(f'数据: {TAG}   口径: 周期内识别到一次即算识别到\n' + hdr)
for w in WINS:
    lab = '逐帧(现口径)' if w == 0 else ('整段(92s)' if w >= 1e8 else f'{w:g} s')
    cells = []
    for n, _, _ in CATS:
        v = [100 * m / t for m, t in res[(n, w)]]
        cells.append(f'{sum(v)/len(v):5.1f} [{min(v):4.1f}-{max(v):4.1f}]')
    # 合计：逐run把四类的分子分母各自相加后再算比值，最后对5次取平均
    per_run = []
    for k in range(5):
        m = sum(res[(n, w)][k][0] for n, _, _ in CATS)
        t = sum(res[(n, w)][k][1] for n, _, _ in CATS)
        per_run.append(100 * m / t)
    cells.append(f'{sum(per_run)/5:5.1f} [{min(per_run):4.1f}-{max(per_run):4.1f}]')
    print(lab.ljust(10) + ''.join(f'{c:>18s}' for c in cells))
