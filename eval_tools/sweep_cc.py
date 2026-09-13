# -*- coding: utf-8 -*-
"""单一时钟数据上的离线级联扫描：用节点日志里的簇特征按变体参数重新分类，替换 jsonl 的检测帧
（按 hstamp == 簇 t 对应），再用 v4 --common-clock 评估。
用法: python3 sweep_cc.py <capture_tag>   (读 /home/lyf040817/boatmiss/<tag>/runN_node.log 与 runN.jsonl)"""
import sys, json, pickle, os, subprocess, re
from collections import defaultdict
sys.path.insert(0, '/home/lyf040817/cascade_exp')
from cascade import parse, classify, BASE
TAG = sys.argv[1]; D = f'/home/lyf040817/boatmiss/{TAG}'
PUB = {'block': 'det_block', 'buoy': 'det_buoy', 'buoy_fallback': 'det_buoy',
       'pillar': 'det_pillar', 'boat': 'det', 'boat_fallback': 'det'}
CACHE = f'{D}/clusters.pkl'
if os.path.exists(CACHE): CL = pickle.load(open(CACHE, 'rb'))
else:
    CL = {i: parse(f'{D}/run{i}_node.log') for i in range(1, 6)}
    pickle.dump(CL, open(CACHE, 'wb'))
EV = '/home/lyf040817/usv_ws/four_category_evaluator_v4.py'
LINE = re.compile(r'^(boat|buoy|pillar|block|合计)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+([\d.]+)%\s+([\d.]+)%\s+([\d.]+)%')
def evaluate(path):
    o = subprocess.run(['python3', EV, path, '--common-clock'], capture_output=True, text=True).stdout
    return {m.group(1): (float(m.group(8)), float(m.group(9)), float(m.group(10)))
            for m in map(LINE.match, o.splitlines()) if m}
def run_variant(P, tag, online=False):
    res = defaultdict(list)
    for i in range(1, 6):
        by = defaultdict(lambda: defaultdict(list)); miss_key = 0
        for c in CL[i]:
            lab = c['online'] if online else classify(c, P)
            if lab in PUB: by[round(c['t'], 6)][PUB[lab]].append({'x': c['cx'], 'y': c['cy'], 'z': c['cz']})
        out = f'/tmp/cc_{TAG}_{tag}_{i}.jsonl'
        with open(f'{D}/run{i}.jsonl') as f, open(out, 'w') as g:
            for line in f:
                r = json.loads(line)
                if r['type'] in PUB.values() and r.get('hstamp') is not None:
                    k = round(r['hstamp'], 6)
                    if k not in by: miss_key += 1
                    r['poses'] = by[k][r['type']] if k in by else []
                g.write(json.dumps(r) + '\n')
        for k, v in evaluate(out).items(): res[k].append(v)
        os.remove(out)
    return res
if __name__ == '__main__':
    # 自检：在线标签重建必须逐位复现直接评估
    direct = defaultdict(list)
    for i in range(1, 6):
        for k, v in evaluate(f'{D}/run{i}.jsonl').items(): direct[k].append(v)
    rebuilt = run_variant(BASE, 'online', online=True)
    print('自检(遗漏率 直接 vs 在线标签重建):', {k: (direct[k], rebuilt[k]) for k in ('boat', '合计')})
    VARIANTS = [
        ('V0 基线', {}),
        ('V1 浮球上界 1.0→0.85', {'buoy_fp_max': 0.85}),
        ('V5 船只提到浮球之前', {'order': ('block', 'boat', 'buoy', 'pillar')}),
        ('V6 船只提到最前', {'order': ('boat', 'block', 'buoy', 'pillar')}),
        ('V7 船只点数门槛 40→25', {'boat_min_pts': 25}),
        ('V8 船只点数门槛 40→15', {'boat_min_pts': 15}),
        ('V9 船只下界 0.9→0.7', {'boat_fp_min': 0.7}),
    ]
    for name, ov in VARIANTS:
        P = dict(BASE); P.update(ov)
        r = run_variant(P, str(abs(hash(name)) % 10000))
        a = {k: [sum(x[j] for x in v) / len(v) for j in range(3)] for k, v in r.items()}
        print(f"{name:22s} 遗漏 船{a['boat'][0]:5.1f} 球{a['buoy'][0]:5.1f} 块{a['block'][0]:5.1f} 柱{a['pillar'][0]:5.1f} 合{a['合计'][0]:5.1f}"
              f" │误检 船{a['boat'][1]:5.1f} 球{a['buoy'][1]:5.1f} 块{a['block'][1]:5.1f} 柱{a['pillar'][1]:5.1f} 合{a['合计'][1]:5.1f}"
              f" │分类错 合{a['合计'][2]:4.1f}", flush=True)
