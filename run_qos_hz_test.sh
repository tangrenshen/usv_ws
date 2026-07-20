#!/bin/bash

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
cd "$SCRIPT_DIR"

echo "[1/4] 清理所有进程..."
pkill -9 -f "perception_node" 2>/dev/null || true
pkill -9 -f "ros2 bag play" 2>/dev/null || true
sleep 3

echo "[2/4] 启动感知节点..."
source /opt/ros/jazzy/setup.bash
source install/setup.bash
nohup ros2 run usv_perception perception_node > perception_log.log 2>&1 &
PERCEPTION_PID=$!
echo "  perception_node PID: $PERCEPTION_PID"
sleep 3

echo "[3/4] 启动topic hz监控..."
ros2 topic hz /wamv/sensors/lidars/front_lidar_sensor/points > hz_output.txt 2>&1 &
HZ_PID=$!
echo "  ros2 topic hz PID: $HZ_PID"
sleep 1

echo "[4/4] 播放bag..."
ros2 bag play ~/bags/bag_09_03_17/ --clock
echo "  ✅ bag播放完成"

echo "[清理] 等待处理完剩余数据..."
sleep 5

echo "[清理] 停止进程..."
kill $PERCEPTION_PID 2>/dev/null || true
kill $HZ_PID 2>/dev/null || true
sleep 2

echo ""
echo "=== ros2 topic hz 结果 ==="
cat hz_output.txt

echo ""
echo "[perception_node] front_lidar消息数:"
grep "front_lidar" perception_log.log | grep "条"
