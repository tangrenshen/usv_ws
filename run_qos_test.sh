#!/bin/bash

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
cd "$SCRIPT_DIR"

echo "=== QoS丢包检测测试 ==="

echo "[1/5] 清理所有进程..."
pkill -9 -f "perception_node" 2>/dev/null || true
pkill -9 -f "boat_logger" 2>/dev/null || true
pkill -9 -f "ros2 bag play" 2>/dev/null || true
pkill -9 -f "check_topic_hz" 2>/dev/null || true
sleep 3

echo "[2/5] 检查残留进程..."
ps aux | grep -E "perception_node|boat_logger|ros2 bag play" | grep -v grep || echo "  ✅ 无残留进程"

echo "[3/5] 清理旧文件..."
rm -f boat_log.jsonl perception_log.log topic_hz_result.log

echo "[4/5] 启动感知节点..."
source /opt/ros/jazzy/setup.bash
source install/setup.bash
nohup ros2 run usv_perception perception_node > perception_log.log 2>&1 &
PERCEPTION_PID=$!
echo "  perception_node PID: $PERCEPTION_PID"

echo "[4b/5] 等待节点初始化..."
sleep 3

echo "[4c/5] 启动日志记录器..."
nohup python3 boat_logger.py > boat_logger.out 2>&1 &
LOGGER_PID=$!
echo "  boat_logger PID: $LOGGER_PID"
sleep 1

echo "[4d/5] 启动话题频率监控..."
nohup bash check_topic_hz.sh > check_topic_hz.out 2>&1 &
HZ_PID=$!
echo "  check_topic_hz PID: $HZ_PID"
sleep 2

echo "[5/5] 播放bag..."
ros2 bag play ~/bags/bag_09_03_17/ --clock
echo "  ✅ bag播放完成"

echo "[清理] 等待频率监控完成..."
sleep 5

echo "[清理] 停止进程..."
kill $PERCEPTION_PID 2>/dev/null || true
kill $LOGGER_PID 2>/dev/null || true
kill $HZ_PID 2>/dev/null || true
sleep 2

echo "[清理] 确保进程已停止..."
pkill -9 -f "perception_node" 2>/dev/null || true
pkill -9 -f "boat_logger" 2>/dev/null || true
pkill -9 -f "check_topic_hz" 2>/dev/null || true
sleep 1

echo ""
echo "=== QoS检测完成 ==="
echo ""
echo "--- 话题频率结果 ---"
cat topic_hz_result.log

echo ""
echo "--- 检测统计 ---"
gt_count=$(grep -c '"type": "gt"' boat_log.jsonl 2>/dev/null || echo "0")
det_count=$(grep -c '"type": "det"' boat_log.jsonl 2>/dev/null || echo "0")
echo "gt_records: $gt_count"
echo "det_records: $det_count"

echo ""
echo "--- 评估结果 ---"
python3 boat_evaluator.py
