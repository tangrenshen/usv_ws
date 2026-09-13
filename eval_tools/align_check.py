# -*- coding: utf-8 -*-
"""单一时钟 jsonl 的对齐自校验（可失败的检验）：
  船只：给船只真值加 Δ，命中应在 Δ≈0 取峰；
  静止三类：给 odom 加 Δ，位置误差应在 Δ≈0 取谷。
用法: python3 align_check.py run.jsonl"""
import sys, statistics
sys.path.insert(0, '/home/lyf040817/usv_ws')
import four_category_evaluator_v2 as ev
R = ev.load_records(sys.argv[1])
SH = (-2, -1, -0.5, -0.25, 0, 0.25, 0.5, 1, 2)
def score(gk, dk, sh_gt=0.0, sh_od=0.0):
    io = ev.make_odom_interpolator([dict(o, stamp=o['stamp'] + sh_od) for o in R['odom']])
    ig = ev.make_gt_interpolator([{'stamp': g['stamp'] + sh_gt, 'poses': g['poses']} for g in R[gk]])
    hit = tot = 0; errs = []
    for fr in ev.deduplicate_frames(R[dk])[0]:
        od = io(fr['stamp']); g = ig(fr['stamp'])
        if od is None or g is None: continue
        tg = [ev.world_to_boat(p, od) for p in g['poses']]
        pairs, pe = ev.maximum_cardinality_matches([(p['x'], p['y']) for p in fr['poses']], tg, 2.0)
        hit += len(pairs); tot += len(tg); errs += pe
    return 100 * (1 - hit / tot), statistics.fmean(errs) if errs else float('nan')
print('船只 遗漏率 vs 真值Δ: ' + '  '.join(f'{s:+.2f}:{score("gt", "det", sh_gt=s)[0]:.1f}' for s in SH))
for gk, dk in (('gt_buoy', 'det_buoy'), ('gt_pillar', 'det_pillar'), ('gt_block', 'det_block')):
    print(f'{gk:9s} 位置误差 vs odomΔ: ' + '  '.join(f'{s:+.2f}:{score(gk, dk, sh_od=s)[1]:.3f}' for s in SH))
