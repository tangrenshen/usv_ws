#!/bin/bash
cd "$(dirname "${BASH_SOURCE[0]}")"
echo "=== 5次连续验证 ==="
echo ""

for i in $(seq 1 5); do
  echo "--- 第 $i/5 次 ---"
  bash run_clean_baseline.sh
  echo ""
done

echo "=== 汇总 ==="
grep "平均召回率" perception_log.log 2>/dev/null || echo "未找到召回率数据"