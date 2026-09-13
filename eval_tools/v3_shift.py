# -*- coding: utf-8 -*-
"""在 v3 的线性重映射时间轴上，对船只真值再加偏移 Δ 扫描。
若 v3 对齐正确，命中应在 Δ=0 取峰；若峰在别处，说明 v3 对动态目标错位。
静态类（浮球）作对照：它们对小时间错位不敏感，应基本平坦。"""
import sys
sys.path.insert(0, '/home/lyf040817/usv_ws')
import four_category_evaluator_v2 as ev
run = sys.argv[1]
SH = [float(x) for x in sys.argv[2].split(',')] if len(sys.argv) > 2 else [-6, -4, -3, -2, -1, 0, 1, 2, 3, 4, 6]
R = ev.load_records(f'/home/lyf040817/eval_logs_0902/run{run}.jsonl')
o = [r['stamp'] for r in R['odom']]; o0, o1 = min(o), max(o)
def remap(keys):
    ts = [r['stamp'] for k in keys for r in R[k]]
    a, b = min(ts), max(ts); s = (o1 - o0) / (b - a)
    for k in keys:
        for r in R[k]: r['stamp'] = o0 + (r['stamp'] - a) * s
    return s
sg = remap(['gt', 'gt_buoy', 'gt_pillar', 'gt_block']); sd = remap(['det', 'det_buoy', 'det_pillar', 'det_block'])
print(f'run{run}: gt scale {sg:.4f}  det scale {sd:.4f}  (odom 时间轴秒)')
io = ev.make_odom_interpolator(R['odom'])
for gk, dk in (('gt', 'det'), ('gt_buoy', 'det_buoy')):
    dets = ev.deduplicate_frames(R[dk])[0]
    out = []
    for sh in SH:
        ig = ev.make_gt_interpolator([{'stamp': g['stamp'] + sh, 'poses': g['poses']} for g in R[gk]])
        hit = tot = 0
        for fr in dets:
            od = io(fr['stamp']); g = ig(fr['stamp'])
            if od is None or g is None: continue
            tg = [ev.world_to_boat(p, od) for p in g['poses']]
            pairs, _ = ev.maximum_cardinality_matches([(p['x'], p['y']) for p in fr['poses']], tg, 2.0)
            hit += len(pairs); tot += len(tg)
        out.append(f'{sh:+.1f}:{100*(1-hit/tot):.1f}')
    print(f'  {gk:8s} 遗漏率 vs Δ(odom秒)  ' + '  '.join(out))
