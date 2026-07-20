#!/bin/bash
cd "$(dirname "${BASH_SOURCE[0]}")"

PX="${1:-0.0}"
PY="${2:-0.0}"
PZ="${3:-0.0}"
PR="${4:-0.0}"
PP="${5:-0.0}"
PYAW="${6:-0.0}"
PONLY="${7:-}"

echo "[1/6] 清理进程..."
pkill -9 -f "perception_node" 2>/dev/null || true
pkill -9 -f "boat_logger" 2>/dev/null || true
pkill -9 -f "ros2 bag play" 2>/dev/null || true
sleep 2

echo "[2/6] 清理文件..."
rm -f boat_log.jsonl perception_log.log boat_logger.out

echo "[3/6] 启动感知节点 (扰动: x=$PX, y=$PY, z=$PZ, r=$PR, p=$PP, yaw=$PYAW, only=$PONLY)..."
source /opt/ros/jazzy/setup.bash
source install/setup.bash

PERCEIVE_ARGS=""
PERCEIVE_ARGS="$PERCEIVE_ARGS -p nominal_perturb_x:=$PX"
PERCEIVE_ARGS="$PERCEIVE_ARGS -p nominal_perturb_y:=$PY"
PERCEIVE_ARGS="$PERCEIVE_ARGS -p nominal_perturb_z:=$PZ"
PERCEIVE_ARGS="$PERCEIVE_ARGS -p nominal_perturb_roll:=$PR"
PERCEIVE_ARGS="$PERCEIVE_ARGS -p nominal_perturb_pitch:=$PP"
PERCEIVE_ARGS="$PERCEIVE_ARGS -p nominal_perturb_yaw:=$PYAW"
if [ -n "$PONLY" ]; then
  PERCEIVE_ARGS="$PERCEIVE_ARGS -p nominal_perturb_only:=$PONLY"
fi

nohup ros2 run usv_perception perception_node \
  --ros-args \
  $PERCEIVE_ARGS \
  > perception_log.log 2>&1 &
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
grep -E "鲁棒性测试|标定核查-与URDF真值偏差|标定核查-相对nominal修正|第.*次尝试" perception_log.log

echo ""
echo "=== 召回率 ==="
python3 boat_evaluator.py 2>&1 | grep "平均召回率"