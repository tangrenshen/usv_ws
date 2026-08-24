#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source /opt/ros/jazzy/setup.bash

echo "[1/5] 清理所有进程..."
pkill -9 -f "perception_node" 2>/dev/null || true
pkill -9 -f "boat_logger" 2>/dev/null || true
pkill -9 -f "ros2 bag play" 2>/dev/null || true
sleep 3

echo "[2/5] 清理旧文件..."
rm -f boat_log.jsonl perception_log.log

echo "[3/5] 启动感知节点(boat_tracker_enabled:=true)..."
source install/setup.bash
nohup ros2 run usv_perception perception_node --ros-args -p use_sim_time:=true -p boat_tracker_enabled:=true > perception_log.log 2>&1 &
PERCEPTION_PID=$!
sleep 3

echo "[4/5] 启动日志记录器..."
nohup python3 boat_logger.py > boat_logger.out 2>&1 &
LOGGER_PID=$!
sleep 1

echo "[5/5] 播放bag..."
ros2 bag play ~/bags/bag_09_03_17/ --clock > /dev/null 2>&1
echo "  bag播放完成"

echo "[清理] 等待处理完剩余数据..."
sleep 8

echo "[清理] 停止进程..."
kill $PERCEPTION_PID 2>/dev/null || true
kill $LOGGER_PID 2>/dev/null || true
sleep 2

echo "[清理] 确保进程已停止..."
pkill -9 -f "perception_node" 2>/dev/null || true
pkill -9 -f "boat_logger" 2>/dev/null || true
sleep 1

echo "done"
