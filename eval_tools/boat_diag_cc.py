# -*- coding: utf-8 -*-
"""单一时钟数据：每个真值船实例 → 2m 内最近簇（节点日志）→ 在线标签与未通过的船只判据。"""
import sys, math, pickle, statistics as st
from collections import Counter, defaultdict
sys.path.insert(0, '/home/lyf040817/usv_ws'); sys.path.insert(0, '/home/lyf040817/cascade_exp')
import four_category_evaluator_v2 as ev
from cascade import BASE as P
TAG = sys.argv[1] if len(sys.argv) > 1 else 'base'
D = f'/home/lyf040817/boatmiss/{TAG}'
CL = pickle.load(open(f'{D}/clusters.pkl', 'rb'))
lab = Counter(); fails = Counter(); nocl = N = 0; F = defaultdict(list); bydist = defaultdict(Counter)
for i in range(1, 6):
    R = ev.load_records(f'{D}/run{i}.jsonl')
    io = ev.make_odom_interpolator(R['odom']); ig = ev.make_gt_interpolator(R['gt'])
    byt = defaultdict(list)
    for c in CL[i]: byt[round(c['t'], 6)].append(c)
    for fr in ev.deduplicate_frames(R['det'])[0]:
        if fr.get('hstamp') is None: continue
        od = io(fr['stamp']); g = ig(fr['stamp'])
        if od is None or g is None: continue
        cls = byt.get(round(fr['hstamp'], 6), [])
        dets = [(p['x'], p['y']) for p in fr['poses']]
        for p in g['poses']:
            bx, by = ev.world_to_boat(p, od); N += 1
            d0 = math.hypot(bx, by); bucket = min(int(d0 // 10) * 10, 50)
            hit = any(math.hypot(x - bx, y - by) < 2 for x, y in dets)
            best = min(cls, key=lambda c: math.hypot(c['cx'] - bx, c['cy'] - by), default=None)
            if best is None or math.hypot(best['cx'] - bx, best['cy'] - by) >= 2:
                nocl += 1; bydist[bucket]['无簇'] += 1; continue
            c = best; lab[c['online']] += 1; bydist[bucket][c['online']] += 1
            dist = math.hypot(c['cx'], c['cy'])
            eff = P['boat_fp_max_sane_near'] if dist <= P['near_range_threshold'] else P['boat_fp_max_sane']
            if c['online'] in ('boat', 'boat_fallback'): continue
            f = []
            if c['fp_max'] < P['boat_fp_min']: f.append('fp_max<0.9')
            if c['fp_max'] > eff: f.append('fp_max>上限')
            if c['dz'] < P['boat_h_min']: f.append('dz<0.3')
            if c['dz'] > P['boat_h_max']: f.append('dz>1.8')
            if c['pts'] < P['boat_min_pts']: f.append('pts<40')
            if c['have_src'] and dist > 40 and c['fwd'] >= 0.5: f.append('远距前向拒绝')
            for x in (f or ['判据全过但被前序分支截走']): fails[x] += 1
            for k in ('fp_max', 'dz', 'pts'): F[k].append(c[k])
            F['dist'].append(dist)
print(f'真值船实例 N={N}   2m内无簇 {nocl} ({nocl/N*100:.1f}%)')
print('最近簇在线标签:', {k: f'{v} ({v/N*100:.1f}%)' for k, v in lab.most_common()})
M = sum(v for k, v in lab.items() if k not in ('boat', 'boat_fallback'))
print(f'\n有簇但未判为船的 {M} 个，未通过的船只判据（可多条）:')
for k, v in fails.most_common(): print(f'   {k:16s} {v:6d} ({v/M*100:5.1f}%)')
for k in ('fp_max', 'dz', 'pts', 'dist'):
    v = sorted(F[k]); print(f'   {k:6s} 中位 {st.median(v):7.2f}  P10 {v[len(v)//10]:7.2f}  P90 {v[len(v)*9//10]:7.2f}')
print('\n按距离（最近簇标签占比%）:')
for b in sorted(bydist):
    c = bydist[b]; n = sum(c.values())
    print(f'  {b:2d}-{b+10}m n={n:5d}  ' + '  '.join(f'{k}:{v/n*100:.0f}' for k, v in c.most_common(6)))
