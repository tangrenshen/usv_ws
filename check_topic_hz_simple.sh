#!/bin/bash

source /opt/ros/jazzy/setup.bash

echo "=== 简单QoS丢包检测 ==="
echo ""

TOPICS=(
    "/wamv/sensors/lidars/front_lidar_sensor/points"
    "/wamv/sensors/lidars/right_lidar_sensor/points"
    "/wamv/sensors/lidars/back_lidar_sensor/points"
    "/wamv/sensors/lidars/left_lidar_sensor/points"
)

EXPECTED_HZ=8.9
DURATION=86.23

echo "期望频率: ${EXPECTED_HZ} Hz"
echo "持续时间: ${DURATION} s"
echo "期望消息数: $(echo "${EXPECTED_HZ} * ${DURATION}" | bc)"
echo ""

echo "=== 启动频率监控 (后台运行) ==="
for idx in "${!TOPICS[@]}"; do
    topic="${TOPICS[$idx]}"
    echo "启动监控: ${topic}"
    {
        ros2 topic hz "$topic" --window 100 2>&1 | grep "average rate" | tail -20
    } > "/tmp/hz_${idx}.txt" &
    sleep 0.2
done

echo ""
echo "等待测量完成 (90秒)..."
sleep 90

echo ""
echo "=== 测量结果 ==="
for idx in "${!TOPICS[@]}"; do
    topic="${TOPICS[$idx]}"
    echo "--- ${topic} ---"
    if [ -f "/tmp/hz_${idx}.txt" ]; then
        cat "/tmp/hz_${idx}.txt" | tail -5
    else
        echo "  无数据"
    fi
    echo ""
done

rm -f /tmp/hz_*.txt
