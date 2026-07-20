#!/bin/bash

source /opt/ros/jazzy/setup.bash

OUTPUT_FILE="/tmp/topic_hz_output_$$.txt"
> "$OUTPUT_FILE"

CAMERA_TOPICS=(
    "/wamv/sensors/cameras/front_camera_sensor/optical/image_raw"
    "/wamv/sensors/cameras/right_camera_sensor/optical/image_raw"
    "/wamv/sensors/cameras/right2_camera_sensor/optical/image_raw"
    "/wamv/sensors/cameras/back_camera_sensor/optical/image_raw"
    "/wamv/sensors/cameras/left2_camera_sensor/optical/image_raw"
    "/wamv/sensors/cameras/left_camera_sensor/optical/image_raw"
)

LIDAR_TOPICS=(
    "/wamv/sensors/lidars/front_lidar_sensor/points"
    "/wamv/sensors/lidars/right_lidar_sensor/points"
    "/wamv/sensors/lidars/right2_lidar_sensor/points"
    "/wamv/sensors/lidars/back_lidar_sensor/points"
    "/wamv/sensors/lidars/left2_lidar_sensor/points"
    "/wamv/sensors/lidars/left_lidar_sensor/points"
    "/wamv/sensors/lidars/forward_lidar_sensor/points"
)

BAG_DURATION=86.23
EXPECTED_CAMERA_HZ=4.45
EXPECTED_LIDAR_HZ=8.9

echo "=== 话题频率检查脚本 ===" | tee -a "$OUTPUT_FILE"
echo "Bag持续时间: $BAG_DURATION s" | tee -a "$OUTPUT_FILE"
echo "期望相机频率: $EXPECTED_CAMERA_HZ Hz" | tee -a "$OUTPUT_FILE"
echo "期望雷达频率: $EXPECTED_LIDAR_HZ Hz" | tee -a "$OUTPUT_FILE"
echo "" | tee -a "$OUTPUT_FILE"

echo "=== 启动所有话题频率监控 (后台运行) ===" | tee -a "$OUTPUT_FILE"

for topic in "${CAMERA_TOPICS[@]}"; do
    echo "启动监控: $topic" | tee -a "$OUTPUT_FILE"
    {
        timeout 100 ros2 topic hz "$topic" --window 50 2>&1 | grep -E "average rate|WARNING" | tail -10
        echo "--- $topic 监控结束 ---"
    } > "/tmp/hz_$(echo "$topic" | tr '/' '_').txt" &
    sleep 0.1
done

for topic in "${LIDAR_TOPICS[@]}"; do
    echo "启动监控: $topic" | tee -a "$OUTPUT_FILE"
    {
        timeout 100 ros2 topic hz "$topic" --window 50 2>&1 | grep -E "average rate|WARNING" | tail -10
        echo "--- $topic 监控结束 ---"
    } > "/tmp/hz_$(echo "$topic" | tr '/' '_').txt" &
    sleep 0.1
done

echo "" | tee -a "$OUTPUT_FILE"
echo "等待测量完成 (90秒)..." | tee -a "$OUTPUT_FILE"
sleep 90

echo "" | tee -a "$OUTPUT_FILE"
echo "=== 收集测量结果 ===" | tee -a "$OUTPUT_FILE"

echo "" | tee -a "$OUTPUT_FILE"
echo "--- 相机话题 ---" | tee -a "$OUTPUT_FILE"
for topic in "${CAMERA_TOPICS[@]}"; do
    file="/tmp/hz_$(echo "$topic" | tr '/' '_').txt"
    echo "[$(basename "$topic")]:" | tee -a "$OUTPUT_FILE"
    if [ -f "$file" ]; then
        cat "$file" | tee -a "$OUTPUT_FILE"
    else
        echo "  无数据" | tee -a "$OUTPUT_FILE"
    fi
    echo "" | tee -a "$OUTPUT_FILE"
done

echo "--- 雷达话题 ---" | tee -a "$OUTPUT_FILE"
for topic in "${LIDAR_TOPICS[@]}"; do
    file="/tmp/hz_$(echo "$topic" | tr '/' '_').txt"
    echo "[$(basename "$topic")]:" | tee -a "$OUTPUT_FILE"
    if [ -f "$file" ]; then
        cat "$file" | tee -a "$OUTPUT_FILE"
    else
        echo "  无数据" | tee -a "$OUTPUT_FILE"
    fi
    echo "" | tee -a "$OUTPUT_FILE"
done

cp "$OUTPUT_FILE" /home/lyf040817/usv_ws/topic_hz_result.log
echo "结果已保存到: topic_hz_result.log"

rm -f /tmp/hz_*.txt
rm -f "$OUTPUT_FILE"
