#!/bin/bash
set -e

echo "=== QoS交叉验证测试 ==="
echo "目标: 对比独立节点和perception_node的消息接收数"

source /opt/ros/jazzy/setup.bash
source install/setup.bash

rm -f /tmp/qos_verify_count.txt

echo "[1/5] 清理进程..."
pkill -9 -f "perception_node" 2>/dev/null || true
pkill -9 -f "qos_verify_node" 2>/dev/null || true
pkill -9 -f "ros2 bag play" 2>/dev/null || true
sleep 2

echo "[2/5] 启动独立验证节点..."
python3 qos_verify_node.py > /tmp/qos_verify.log 2>&1 &
VERIFY_PID=$!
echo "  qos_verify_node PID: $VERIFY_PID"
sleep 2

echo "[3/5] 启动perception_node..."
ros2 run usv_perception perception_node > /tmp/perception_verify.log 2>&1 &
PERCEPTION_PID=$!
echo "  perception_node PID: $PERCEPTION_PID"
sleep 5

echo "[4/5] 播放bag..."
ros2 bag play ~/bags/bag_09_03_17/ --clock > /dev/null 2>&1 &
BAG_PID=$!
wait $BAG_PID
echo "  ✅ bag播放完成"

sleep 3

echo "[5/5] 停止进程..."
kill $VERIFY_PID 2>/dev/null || true
kill $PERCEPTION_PID 2>/dev/null || true
sleep 2

echo ""
echo "=== 结果对比 ==="
echo "期望消息数: 768"
echo ""

if [ -f /tmp/qos_verify_count.txt ]; then
    VERIFY_COUNT=$(cat /tmp/qos_verify_count.txt)
    VERIFY_LOSS=$(( (768 - VERIFY_COUNT) * 100 / 768 ))
    echo "[独立节点] front_lidar: ${VERIFY_COUNT}条 (丢失${VERIFY_LOSS}%)"
else
    echo "[独立节点] 未获取到计数"
fi

PERCEPTION_COUNT=$(grep "front_lidar" /tmp/perception_verify.log | grep "条" | head -1 | awk '{print $4}' | tr -d '条')
if [ ! -z "$PERCEPTION_COUNT" ]; then
    PERCEPTION_LOSS=$(( (768 - PERCEPTION_COUNT) * 100 / 768 ))
    echo "[perception_node] front_lidar: ${PERCEPTION_COUNT}条 (丢失${PERCEPTION_LOSS}%)"
else
    echo "[perception_node] 未获取到计数"
    grep "QoS消息计数" /tmp/perception_verify.log -A 5
fi

echo ""
echo "=== 结论 ==="
if [ ! -z "$VERIFY_COUNT" ] && [ ! -z "$PERCEPTION_COUNT" ]; then
    if [ $VERIFY_LOSS -ge 80 ]; then
        echo "独立节点也丢失80%以上 → 问题在传输层(DDS/bag play)，建议切Reliable QoS"
    elif [ $PERCEPTION_LOSS -ge 80 ] && [ $VERIFY_LOSS -lt 30 ]; then
        echo "独立节点几乎不丢，perception_node丢80%以上 → 问题在perception_node内部调度"
    else
        echo "两者丢包率相近 → 需要进一步分析"
    fi
fi
