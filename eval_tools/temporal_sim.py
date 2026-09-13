# -*- coding: utf-8 -*-
"""CandidateTemporalClassifier 的忠实 Python 移植 + 船只/浮球评估（口径同 v4）。
在线标签取自节点日志（簇的 assigned=），逐帧按发布时刻 t 顺序喂入。"""
import sys, math, pickle, itertools
from collections import defaultdict
sys.path.insert(0, '/home/lyf040817/usv_ws')
import four_category_evaluator_v2 as ev
TAG = sys.argv[1] if len(sys.argv) > 1 else 'base'
D = f'/home/lyf040817/boatmiss/{TAG}'
CL = pickle.load(open(f'{D}/clusters.pkl', 'rb'))
GK = ('gt', 'gt_buoy', 'gt_pillar', 'gt_block')
RUNS = {}
for i in range(1, 6):
    R = ev.load_records(f'{D}/run{i}.jsonl')
    io = ev.make_odom_interpolator(R['odom']); ig = {k: ev.make_gt_interpolator(R[k]) for k in GK}
    byt = defaultdict(list)
    for c in CL[i]: byt[round(c['t'], 6)].append(c)
    L = []
    for fr in ev.deduplicate_frames(R['det'])[0]:
        if fr.get('hstamp') is None: continue
        t = fr['stamp']; od = io(t)
        if od is None: continue
        gts = {}
        for k in GK:
            g = ig[k](t); gts[k] = [ev.world_to_boat(p, od) for p in g['poses']] if g else None
        if gts['gt'] is None: continue
        L.append((fr['hstamp'], byt.get(round(fr['hstamp'], 6), []), gts))
    L.sort(key=lambda x: x[0]); RUNS[i] = L

PROMOTABLE = {'buoy', 'discarded_shape_implausible', 'discarded_low_confidence'}
def temporal(frames, gate=0.5, max_gap=0.35, window=1.0, rng=20.0, enabled=True, suppress_fallback=True):
    """返回每帧 (boats, buoys) 位置列表"""
    out = []; tracks = []; last = None
    for t, cls, _ in frames:
        if not enabled:
            out.append(([(c['cx'], c['cy']) for c in cls if c['online'] in ('boat', 'boat_fallback')],
                        [(c['cx'], c['cy']) for c in cls if c['online'] in ('buoy', 'buoy_fallback')])); continue
        if last is not None and (t <= last or t - last > max_gap): tracks = []
        last = t
        tracks = [tr for tr in tracks if t - tr['last'] <= max_gap]
        edges = sorted((math.hypot(c['cx'] - tr['x'], c['cy'] - tr['y']), ci, ti)
                       for ci, c in enumerate(cls) for ti, tr in enumerate(tracks)
                       if math.hypot(c['cx'] - tr['x'], c['cy'] - tr['y']) <= gate)
        asg = [None] * len(cls); used = set()
        for d, ci, ti in edges:
            if asg[ci] is None and ti not in used: asg[ci] = ti; used.add(ti)
        for ci in range(len(cls)):
            if asg[ci] is None:
                tracks.append({'x': cls[ci]['cx'], 'y': cls[ci]['cy'], 'last': t, 'ev': []}); asg[ci] = len(tracks) - 1
        boats, buoys = [], []
        for ci, c in enumerate(cls):
            tr = tracks[asg[ci]]; lab = c['online']
            tr['ev'] = [s for s in tr['ev'] if s >= t - window]
            prior = bool(tr['ev'])
            promote = lab in PROMOTABLE and math.hypot(c['cx'], c['cy']) <= rng and prior
            fb_ok = prior or not suppress_fallback
            if lab == 'boat' or (lab == 'boat_fallback' and fb_ok) or promote: boats.append((c['cx'], c['cy']))
            if (lab == 'buoy' and not promote) or lab == 'buoy_fallback': buoys.append((c['cx'], c['cy']))
            tr['x'], tr['y'], tr['last'] = c['cx'], c['cy'], t
            if lab in ('boat', 'boat_fallback'): tr['ev'].append(t)
        out.append((boats, buoys))
    return out

def evaluate(**kw):
    res = {'boat': [], 'buoy': []}
    for i in range(1, 6):
        fr = RUNS[i]; det = temporal(fr, **kw)
        for cat, gk, j in (('boat', 'gt', 0), ('buoy', 'gt_buoy', 1)):
            hit = miss = fp = mis = 0
            for (t, cls, gts), dd in zip(fr, det):
                pts = dd[j]; tg = gts[gk]
                if tg is None: continue
                pairs, _ = ev.maximum_cardinality_matches(pts, tg, 2.0)
                hit += len(pairs); miss += len(tg) - len(pairs)
                md = {d for d, _ in pairs}; left = [p for k, p in enumerate(pts) if k not in md]
                oth = [p for k in GK if k != gk and gts[k] for p in gts[k]]
                if left and oth:
                    p2, _ = ev.maximum_cardinality_matches(left, oth, 2.0); mis += len(p2); fp += len(left) - len(p2)
                else: fp += len(left)
            n = hit + fp + mis
            res[cat].append((100 * miss / (hit + miss), 100 * fp / n if n else 0, 100 * mis / n if n else 0))
    return res
def fmt(name, r):
    s = f'{name:40s}'
    for cat in ('boat', 'buoy'):
        a = [sum(x[j] for x in r[cat]) / 5 for j in range(3)]
        s += f' │{cat} 遗漏 {a[0]:5.1f} 误检 {a[1]:4.1f} 分类错 {a[2]:4.1f}'
    return s + '  船逐run ' + ' '.join(f'{x[0]:.1f}' for x in r['boat'])
if __name__ == '__main__':
    print(fmt('关闭(=基线)', evaluate(enabled=False)))
    print(fmt('开启,默认参数', evaluate()))
    for gate, window, rng, sup in itertools.product((0.5, 1.0, 1.5), (1.0, 3.0, 6.0), (20.0, 40.0, 60.0), (True, False)):
        print(fmt(f'gate={gate} win={window} rng={rng} 压兜底={sup}', evaluate(gate=gate, window=window, rng=rng, suppress_fallback=sup)), flush=True)
