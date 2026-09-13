# -*- coding: utf-8 -*-
"""级联分类离线复现：解析节点 cluster_diagnostic 日志，按 ObstacleDetector.cpp 判据重新分类"""
import re, math
PEND=re.compile(r'\[cluster_diagnostic\] t=([\d.]+) center=\(([-\d.e]+),([-\d.e]+),([-\d.e]+)\) '
                r'size=\(([-\d.e]+),([-\d.e]+),([-\d.e]+)\) fp_max=([-\d.e]+) fp_min=([-\d.e]+) '
                r'square=([-\d.e]+) pts=(\d+)(?: forward_pts=(\d+) forward_ratio=([-\d.e]+))?')
ASG=re.compile(r'\[cluster_diagnostic\].*assigned=(\w+)')
def parse(path):
    out=[]; cur=None
    with open(path,errors='ignore') as f:
        for line in f:
            if 'classification pending' in line:
                m=PEND.search(line)
                if not m: cur=None; continue
                g=m.groups()
                cur=dict(t=float(g[0]),cx=float(g[1]),cy=float(g[2]),cz=float(g[3]),
                         dz=float(g[6]),fp_max=float(g[7]),fp_min=float(g[8]),square=float(g[9]),
                         pts=int(g[10]),have_src=g[11] is not None,
                         fwd=float(g[12]) if g[12] else 0.0)
            elif cur is not None and line.startswith('[cluster_diagnostic]') and 'assigned=' in line:
                cur['online']=ASG.search(line).group(1); out.append(cur); cur=None
    return out

BASE=dict(min_confident_pts=8, block_fp_min=1.65, block_fp_max=3.2, block_h_max=1.5, block_min_pts=10,
          block_square_min=0.55, block_center_z_max=1.0, pillar_fp_max=2.0, pillar_h_min=1.5,
          pillar_slenderness_min=2.5, pillar_max_pts=3000, buoy_fp_max=1.0, buoy_h_max=1.5,
          boat_fp_min=0.9, boat_min_pts=40, boat_fp_max_sane=2.0, near_range_threshold=20.0,
          boat_fp_max_sane_near=3.0, boat_fallback_square_min=0.35, boat_fallback_min_pts=25,
          boat_h_min=0.3, boat_h_max=1.8, far_reject=True, far_reject_min_dist=40.0,
          far_reject_ratio_min=0.5, order=('block','buoy','pillar','boat'))

def classify(c, P):
    fp,dz,sq,n=c['fp_max'],c['dz'],c['square'],c['pts']
    dist=math.hypot(c['cx'],c['cy'])
    eff=P['boat_fp_max_sane_near'] if dist<=P['near_range_threshold'] else P['boat_fp_max_sane']
    far_rej=P['far_reject'] and c['have_src'] and dist>P['far_reject_min_dist'] and c['fwd']>=P['far_reject_ratio_min']
    test={
      'block': lambda: P['block_fp_min']<=fp<=P['block_fp_max'] and dz<=P['block_h_max'] and sq>=P['block_square_min']
                       and n>=P['block_min_pts'] and abs(c['cz'])<=P['block_center_z_max'],
      'buoy':  lambda: fp<=P['buoy_fp_max'] and dz<=P['buoy_h_max'],
      'pillar':lambda: fp<=P['pillar_fp_max'] and dz>=P['pillar_h_min'] and dz/max(fp,0.05)>=P['pillar_slenderness_min']
                       and n<=P['pillar_max_pts'],
      'boat':  lambda: P['boat_fp_min']<=fp<=eff and P['boat_h_min']<=dz<=P['boat_h_max'] and n>=P['boat_min_pts'] and not far_rej,
    }
    for k in P['order']:
        if test[k](): return k
    if n<P['min_confident_pts']: return 'discarded_low_confidence'
    if fp<=P['buoy_fp_max']: return 'buoy_fallback'
    if sq>=P['boat_fallback_square_min'] and n>=P['boat_fallback_min_pts'] and fp<=eff \
       and P['boat_h_min']<=dz<=P['boat_h_max'] and not far_rej: return 'boat_fallback'
    return 'discarded_shape_implausible'
