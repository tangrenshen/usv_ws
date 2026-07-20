#!/bin/bash
cd "$(dirname "${BASH_SOURCE[0]}")"
echo "[1/6] 清理进程..."
pkill -9 -f "perception_node" 2>/dev/null || true
pkill -9 -f "boat_logger" 2>/dev/null || true
pkill -9 -f "ros2 bag play" 2>/dev/null || true
sleep 2

echo "[2/6] 清理文件..."
rm -f boat_log.jsonl perception_log.log boat_logger.out

echo "[3/6] 启动感知节点..."
source /opt/ros/jazzy/setup.bash
source install/setup.bash
nohup ros2 run usv_perception perception_node > perception_log.log 2>&1 &
PERCEIVE_PID=$!
echo "  PID: $PERCEIVE_PID"

for i in $(seq 1 30); do
  if grep -q "初始化完毕" perception_log.log 2>/dev/null; then
    echo "  ✅ 启动完成"
    break
  fi
  sleep 1
done

echo "[4/6] 启动logger..."
nohup python3 boat_logger.py > boat_logger.out 2>&1 &
sleep 2

echo "[5/6] 播放bag..."
ros2 bag play ~/bags/bag_09_03_17/ --clock --rate 1.0
echo "  ✅ bag完成"

echo "[6/6] 等待处理..."
sleep 10

echo "[清理] 停止进程..."
kill $PERCEIVE_PID 2>/dev/null || true
pkill -9 -f "perception_node" 2>/dev/null || true
pkill -9 -f "boat_logger" 2>/dev/null || true
sleep 2

echo ""
echo "=== 标定结果 ==="
grep -E "标定核查-真实修正量|第.*次尝试" perception_log.log

echo ""
echo "=== 召回率 ==="
python3 boat_evaluator.py 2>&1 | grep "平均召回率"
