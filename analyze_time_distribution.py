import json

def analyze_time_distribution(log_file):
    with open(log_file, 'r') as f:
        lines = f.readlines()
    
    det_records = []
    gt_records = []
    
    for line in lines:
        try:
            record = json.loads(line)
            if record.get('type') == 'det':
                det_records.append(record)
            elif record.get('type') == 'gt':
                gt_records.append(record)
        except:
            continue
    
    if not det_records or not gt_records:
        print(f"  无有效数据")
        return
    
    min_stamp = min(d['stamp'] for d in det_records + gt_records)
    max_stamp = max(d['stamp'] for d in det_records + gt_records)
    duration = max_stamp - min_stamp
    
    print(f"  时间范围: {duration:.1f}s")
    print(f"  det总数: {len(det_records)}, gt总数: {len(gt_records)}")
    
    bins = [(0, 5), (5, 10), (10, 20), (20, 30), (30, 45), (45, 60), (60, 75), (75, 90)]
    det_counts = []
    
    for start, end in bins:
        count = sum(1 for d in det_records if min_stamp + start <= d['stamp'] < min_stamp + end)
        det_counts.append(count)
        print(f"    [{start}-{end}s]: {count}条")
    
    return det_counts

for run in range(1, 6):
    print(f"=== 第{run}次测试 ===")
    analyze_time_distribution(f"boat_log_run{run}.jsonl")
    print()
