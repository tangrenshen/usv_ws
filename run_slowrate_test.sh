#!/bin/bash
# 降速+同比放大定时器实验
# bag --rate 0.1, perception_node所有定时器周期×10, stale_sec×10
# 目的：验证"拖影根因链路"是否成立。如果丢包率大幅下降、召回率回升，
# 则证明整条链路成立，同时给算法调优期间提供"慢速但正确"的测试环境

cd "$(dirname "${BASH_SOURCE[0]}")"

echo "[1/5] 清理所有进程..."
pkill -9 -f "perception_node" 2>/dev/null || true
pkill -9 -f "boat_logger" 2>/dev/null || true
pkill -9 -f "ros2 bag play" 2>/dev/null || true
sleep 3

echo "[2/5] 检查残留进程..."
ps aux | grep -E "perception_node|boat_logger|ros2 bag play" | grep -v grep || echo "  ✅ 无残留进程"

echo "[3/5] 清理旧文件..."
rm -f boat_log.jsonl perception_log.log

echo "[4/5] 启动感知节点(定时器周期×10, stale_sec×10)..."
source install/setup.bash
nohup ros2 run usv_perception perception_node \
  --ros-args \
  -p image_timer_period_sec:=2.0 \
  -p lidar_timer_period_sec:=1.0 \
  -p calib_timer_period_sec:=10.0 \
  -p image_stale_sec:=5.0 \
  -p lidar_stale_sec:=5.0 \
  > perception_log.log 2>&1 &
PERCEPTION_PID=$!
echo "  perception_node PID: $PERCEPTION_PID"
echo "  定时器周期: image=2.0s, lidar=1.0s, calib=10.0s (×10)"
echo "  stale_sec: image=5.0s, lidar=5.0s (×10)"

echo "[4b/5] 等待节点初始化..."
sleep 5

echo "[4c/5] 启动日志记录器..."
nohup python3 boat_logger.py > boat_logger.out 2>&1 &
LOGGER_PID=$!
echo "  boat_logger PID: $LOGGER_PID"
sleep 1

echo "[5/5] 播放bag(--rate 0.1, 速度降为1/10)..."
ros2 bag play ~/bags/bag_09_03_17/ --clock --rate 0.1
echo "  ✅ bag播放完成(耗时约15分钟)"

echo "[清理] 等待处理完剩余数据..."
sleep 30

echo "[清理] 停止进程..."
kill $PERCEPTION_PID 2>/dev/null || true
kill $LOGGER_PID 2>/dev/null || true
sleep 3

echo "[清理] 确保进程已停止..."
pkill -9 -f "perception_node" 2>/dev/null || true
pkill -9 -f "boat_logger" 2>/dev/null || true
sleep 1

echo ""
echo "========================================"
echo "[统计] QoS消息计数(期望768条)..."
grep "QoS" perception_log.log

echo ""
echo "[统计] 检查odom记录数..."
grep '"type":"odom"' boat_log.jsonl | wc -l

echo ""
echo "[评估] 运行boat_evaluator..."
python3 boat_evaluator.py

echo ""
echo "[分析] 运行boat簇分析(看拖影是否消失)..."
python3 analyze_boat_clusters.py 2>&1 | head -30
