#!/usr/bin/env python3
import json

with open('boat_log.jsonl', 'r') as f:
    records = [json.loads(line) for line in f]

det_records = [r for r in records if r['type'] == 'det']
gt_records = [r for r in records if r['type'] == 'gt']
odom_records = [r for r in records if r['type'] == 'odom']

print(f"det记录数: {len(det_records)}")
print(f"gt记录数: {len(gt_records)}")
print(f"odom记录数: {len(odom_records)}")

if det_records:
    print("\n=== det位置统计 ===")
    all_x = []
    all_y = []
    for r in det_records:
        for p in r.get('poses', []):
            all_x.append(p['x'])
            all_y.append(p['y'])
    print(f"检测到的障碍物总数: {len(all_x)}")
    if all_x:
        print(f"x范围: [{min(all_x):.2f}, {max(all_x):.2f}]")
        print(f"y范围: [{min(all_y):.2f}, {max(all_y):.2f}]")
        print(f"前10个检测位置:")
        for i in range(min(10, len(all_x))):
            print(f"  ({all_x[i]:.2f}, {all_y[i]:.2f})")

if gt_records:
    print("\n=== gt位置统计 ===")
    all_x = []
    all_y = []
    for r in gt_records:
        for p in r.get('poses', []):
            all_x.append(p['x'])
            all_y.append(p['y'])
    print(f"真值船只总数(去重前): {len(all_x)}")
    if all_x:
        print(f"x范围: [{min(all_x):.2f}, {max(all_x):.2f}]")
        print(f"y范围: [{min(all_y):.2f}, {max(all_y):.2f}]")

if odom_records:
    print("\n=== odom时间范围 ===")
    stamps = [r['stamp'] for r in odom_records]
    print(f"odom时间范围: [{min(stamps):.2f}, {max(stamps):.2f}]")

print("\n=== 时间基准检查 ===")
if gt_records and odom_records:
    gt_first = gt_records[0]['stamp']
    odom_first = odom_records[0]['stamp']
    offset = gt_first - odom_first
    print(f"gt[0].stamp: {gt_first:.6f}")
    print(f"odom[0].stamp: {odom_first:.6f}")
    print(f"偏移量: {offset:.6f}")
