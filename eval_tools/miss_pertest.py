# -*- coding: utf-8 -*-
"""周期=一次完整测试：每个真值目标在整次运行中只要被识别到一次即算识别到。
真值只取四个真值话题发布的目标（立柱即话题里的36个，未标注结构不进分母）。
两种分母：
  A 全部目标（不设距离豁免，赛会"雷达能测到的都算"的最严读法）
  B 仅计入"运行中至少有一帧落在传感器覆盖内"的目标
     覆盖 = Airy 水平 60 m 内，或 EMX（装在 x=7.0）前向 ±60°、200 m 内
"""
import sys, math
sys.path.insert(0, '/home/lyf040817/usv_ws')
import four_category_evaluator_v2 as ev
TAG = sys.argv[1]
CATS = [('浮球', 'gt_buoy', 'det_buoy'), ('浮块', 'gt_block', 'det_block'),
        ('立柱', 'gt_pillar', 'det_pillar'), ('船只', 'gt', 'det')]
def covered(bx, by):
    if math.hypot(bx, by) <= 60.0: return True
    dx, dy = bx - 7.0, by
    return math.hypot(dx, dy) <= 200.0 and abs(math.degrees(math.atan2(dy, dx))) <= 60.0
out = {}
for i in range(1, 6):
    R = ev.load_records(f'/home/lyf040817/boatmiss/{TAG}/run{i}.jsonl')
    io = ev.make_odom_interpolator(R['odom'])
    for name, gk, dk in CATS:
        ig = ev.make_gt_interpolator(R[gk])
        seen, cov, tot = set(), set(), 0
        for fr in ev.deduplicate_frames(R[dk])[0]:
            t = fr['stamp']; od = io(t); g = ig(t)
            if od is None or g is None: continue
            tg = [ev.world_to_boat(p, od) for p in g['poses']]
            tot = max(tot, len(tg))
            for j, (bx, by) in enumerate(tg):
                if covered(bx, by): cov.add(j)
            pairs, _ = ev.maximum_cardinality_matches([(p['x'], p['y']) for p in fr['poses']], tg, 2.0)
            seen |= {j for _, j in pairs}
        out.setdefault((name, 'A'), []).append((tot - len(seen), tot))
        out.setdefault((name, 'B'), []).append((len(cov - seen), len(cov)))
print(f'数据 {TAG}   周期 = 一次完整测试（约92 s）   真值仅取四个真值话题')
for kind, desc in (('A', '分母=话题内全部目标'), ('B', '分母=运行中曾落入传感器覆盖内的目标')):
    print(f'\n【{kind}】{desc}')
    print('类别      遗漏率(5次均值)   逐次 分子/分母')
    ms = ts = 0
    for name, _, _ in CATS:
        v = out[(name, kind)]
        r = [100 * m / t if t else 0 for m, t in v]
        ms += sum(m for m, _ in v); ts += sum(t for _, t in v)
        print(f'{name:6s}  {sum(r)/5:5.1f}%  [{min(r):4.1f}-{max(r):4.1f}]   ' + '  '.join(f'{m}/{t}' for m, t in v))
    per_run = [100 * sum(out[(n, kind)][k][0] for n, _, _ in CATS) / sum(out[(n, kind)][k][1] for n, _, _ in CATS) for k in range(5)]
    print(f'合计    {sum(per_run)/5:5.1f}%  [{min(per_run):4.1f}-{max(per_run):4.1f}]   总计 {ms}/{ts}')
