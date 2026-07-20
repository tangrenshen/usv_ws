#!/usr/bin/env python3
"""
可行性测试：用cluster_diagnostic日志(几何特征) + 四类真值(位置匹配自动打标签)，
训练一个决策树，看纯几何特征本身的可分性上限有多高。
"""
import re
import json
import math
import bisect
import numpy as np
from sklearn.tree import DecisionTreeClassifier
from sklearn.model_selection import train_test_split
from sklearn.metrics import classification_report, confusion_matrix

LOG_PATH = '/home/lyf040817/usv_ws/ml_train_log.txt'
JSONL_PATH = '/home/lyf040817/usv_ws/boat_log.jsonl'
MATCH_TOL = 2.0
MAX_TIME_GAP = 0.5

# ---------- 1. 解析特征日志(每个簇一行) ----------
pattern = re.compile(
    r'\[cluster_diagnostic\] t=([\d.]+) center=\(([-\d.]+),([-\d.]+),([-\d.]+)\) '
    r'size=\(([-\d.]+),([-\d.]+),([-\d.]+)\) fp_max=([-\d.]+) fp_min=([-\d.]+) '
    r'square=([-\d.]+) pts=(\d+)'
)
clusters = []
with open(LOG_PATH) as f:
    for line in f:
        m = pattern.search(line)
        if m:
            clusters.append({
                'stamp': float(m.group(1)),
                'x': float(m.group(2)), 'y': float(m.group(3)), 'z': float(m.group(4)),
                'dx': float(m.group(5)), 'dy': float(m.group(6)), 'dz': float(m.group(7)),
                'fp_max': float(m.group(8)), 'fp_min': float(m.group(9)),
                'square': float(m.group(10)), 'pts': int(m.group(11)),
            })
print(f"解析到候选簇特征: {len(clusters)}")

# ---------- 2. 加载四类真值+位姿 ----------
records = {'gt': [], 'gt_pillar': [], 'gt_buoy': [], 'gt_block': [], 'odom': []}
with open(JSONL_PATH) as f:
    for line in f:
        rec = json.loads(line)
        if rec['type'] in records:
            records[rec['type']].append(rec)

if records['gt'] and records['odom']:
    _offset = records['gt'][0]['stamp'] - records['odom'][0]['stamp']
    for key in ['gt', 'gt_pillar', 'gt_buoy', 'gt_block']:
        for r in records[key]:
            r['stamp'] -= _offset
    print(f"[时间基准校正] 偏移量: {_offset:.4f}s")

for key in records:
    records[key].sort(key=lambda r: r['stamp'])
stamps = {key: [r['stamp'] for r in records[key]] for key in records}

def find_nearest(key, t):
    stamp_list = stamps[key]; rec_list = records[key]
    idx = bisect.bisect_left(stamp_list, t)
    candidates = []
    if idx < len(rec_list): candidates.append(rec_list[idx])
    if idx > 0: candidates.append(rec_list[idx-1])
    if not candidates: return None, None
    best = min(candidates, key=lambda r: abs(r['stamp'] - t))
    return best, abs(best['stamp'] - t)

def world_to_boat(wx, wy, bx, by, yaw):
    dx, dy = wx - bx, wy - by
    return (dx*math.cos(-yaw) - dy*math.sin(-yaw), dx*math.sin(-yaw) + dy*math.cos(-yaw))

def quat_to_yaw(qx, qy, qz, qw):
    return math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))

# ---------- 3. 给每个簇按位置匹配打标签 ----------
X, y = [], []
label_count = {}
odom_cache = {}

for c in clusters:
    t = c['stamp']
    if t not in odom_cache:
        odom, odom_gap = find_nearest('odom', t)
        if odom is None or odom_gap > MAX_TIME_GAP:
            odom_cache[t] = None
        else:
            odom_cache[t] = odom
    odom = odom_cache[t]
    if odom is None: continue

    yaw = quat_to_yaw(odom['qx'], odom['qy'], odom['qz'], odom['qw'])

    best_label, best_d = 'none', MATCH_TOL
    for key, label in [('gt', 'boat'), ('gt_pillar', 'pillar'), ('gt_buoy', 'buoy'), ('gt_block', 'block')]:
        rec, gap = find_nearest(key, t)
        if rec is None or gap > MAX_TIME_GAP: continue
        for p in rec['poses']:
            bx, by = world_to_boat(p['x'], p['y'], odom['x'], odom['y'], yaw)
            d = math.hypot(c['x']-bx, c['y']-by)
            if d < best_d:
                best_d = d; best_label = label

    if best_label == 'none': continue  # 只用能打上标签的样本训练

    X.append([c['dx'], c['dy'], c['dz'], c['fp_max'], c['fp_min'], c['square'], c['pts']])
    y.append(best_label)
    label_count[best_label] = label_count.get(best_label, 0) + 1

print(f"\n有效带标签样本数: {len(X)}")
print(f"标签分布: {label_count}")

# ---------- 4. 训练决策树，看可分性上限 ----------
X = np.array(X)
y = np.array(y)
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.3, random_state=42, stratify=y)

clf = DecisionTreeClassifier(max_depth=6, random_state=42)
clf.fit(X_train, y_train)
y_pred = clf.predict(X_test)

print(f"\n=== 决策树可行性测试结果(仅用dx,dy,dz,fp_max,fp_min,square,pts这7个几何特征) ===")
print(classification_report(y_test, y_pred))
print("混淆矩阵(行=真实标签, 列=预测标签):")
labels = sorted(set(y))
print("labels:", labels)
print(confusion_matrix(y_test, y_pred, labels=labels))

feature_names = ['dx','dy','dz','fp_max','fp_min','square','pts']
print(f"\n特征重要性:")
for name, imp in sorted(zip(feature_names, clf.feature_importances_), key=lambda x: -x[1]):
    print(f"  {name}: {imp:.3f}")

# ---------- 5. 打印决策树完整分裂规则 ----------
from sklearn.tree import export_text
print("\n\n=== 决策树完整分裂规则 ===")
rules = export_text(clf, feature_names=feature_names, class_names=clf.classes_.tolist())
print(rules)

# ---------- 6. 导出决策树结构为C++可用的数组，用于程序化生成推理代码 ----------
tree_ = clf.tree_
class_names_sorted = clf.classes_.tolist()

print("\n\n=== 导出C++推理代码 ===")
lines = []
lines.append("// 本文件由 ml_feasibility.py 自动生成，请勿手工编辑")
lines.append("// 决策树分类器：交叉核验boat/boat_fallback候选，纠正与pillar/block/buoy重叠的误判")
lines.append("#pragma once")
lines.append("#include <string>")
lines.append("#include <array>")
lines.append("")
lines.append("namespace usv_perception {")
lines.append("")
n_nodes = tree_.node_count
lines.append(f"static constexpr int kTreeNodeCount = {n_nodes};")
lines.append(f"static constexpr int kTreeFeature[{n_nodes}] = {{" + ",".join(str(int(f)) for f in tree_.feature) + "};")
lines.append(f"static constexpr double kTreeThreshold[{n_nodes}] = {{" + ",".join(f"{t:.6f}" for t in tree_.threshold) + "};")
lines.append(f"static constexpr int kTreeChildLeft[{n_nodes}] = {{" + ",".join(str(int(c)) for c in tree_.children_left) + "};")
lines.append(f"static constexpr int kTreeChildRight[{n_nodes}] = {{" + ",".join(str(int(c)) for c in tree_.children_right) + "};")

# 每个叶子节点对应的类别(取value最大的那一类)
leaf_class = []
for i in range(n_nodes):
    if tree_.children_left[i] == -1:  # 叶子节点
        class_idx = tree_.value[i][0].argmax()
        leaf_class.append(class_names_sorted[class_idx])
    else:
        leaf_class.append("")
leaf_class_escaped = ",".join(f'"{c}"' for c in leaf_class)
lines.append(f'static const std::array<std::string, {n_nodes}> kTreeLeafClass = {{{leaf_class_escaped}}};')
lines.append("")
lines.append("// 特征顺序: [dx, dy, dz, fp_max, fp_min, square, pts]")
lines.append("inline std::string classifyByDecisionTree(double dx, double dy, double dz,")
lines.append("    double fp_max, double fp_min, double square, double pts) {")
lines.append("    double feat[7] = {dx, dy, dz, fp_max, fp_min, square, pts};")
lines.append("    int node = 0;")
lines.append("    while (kTreeChildLeft[node] != -1) {")
lines.append("        int f = kTreeFeature[node];")
lines.append("        if (feat[f] <= kTreeThreshold[node]) node = kTreeChildLeft[node];")
lines.append("        else node = kTreeChildRight[node];")
lines.append("    }")
lines.append("    return kTreeLeafClass[node];")
lines.append("}")
lines.append("")
lines.append("}  // namespace usv_perception")

with open('/home/lyf040817/usv_ws/src/usv_perception/include/usv_perception/BoatDecisionTree.hpp', 'w') as f:
    f.write("\n".join(lines))
print("已导出到 include/usv_perception/BoatDecisionTree.hpp")
print(f"树节点总数: {n_nodes}")
