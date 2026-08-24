#!/bin/bash
# 端到端合规演练：只播放比测现场真实会有的话题（7路点云+6路鱼眼图像+/clock），
# 显式排除GT/odom/overhead_camera，外参注入扰动，实时回放（--rate 1.0，不降速）。
# 检查：节点能否正常启动、四个_check话题频率是否正常、有无隐藏依赖导致的崩溃
# 或卡死，并记录各分支耗时分布。
cd "$(dirname "${BASH_SOURCE[0]}")"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "[1/6] 清理进程..."
pkill -9 -f "perception_node" 2>/dev/null || true
pkill -9 -f "ros2 bag play" 2>/dev/null || true
sleep 2

echo "[2/6] 清理旧文件..."
rm -f compliance_drill_perception.log compliance_drill_topics.log

echo "[3/6] 启动感知节点（外参扰动seed=42，±0.5m/±10°）..."
nohup ros2 run usv_perception perception_node --ros-args -p use_sim_time:=true \
  -p test_extrinsic_perturb_seed:=42 \
  -p test_extrinsic_perturb_translation_m:=0.5 \
  -p test_extrinsic_perturb_rotation_deg:=10.0 \
  > compliance_drill_perception.log 2>&1 &
PERCEPTION_PID=$!
sleep 3

echo "[4/6] 后台监控四个_check话题频率..."
timeout 40 ros2 topic hz /world/obstacles/buoys_check > compliance_drill_hz_buoys.log 2>&1 &
timeout 40 ros2 topic hz /world/obstacles/boats_check > compliance_drill_hz_boats.log 2>&1 &
timeout 40 ros2 topic hz /world/obstacles/pillars_check > compliance_drill_hz_pillars.log 2>&1 &
timeout 40 ros2 topic hz /world/obstacles/blocks_check > compliance_drill_hz_blocks.log 2>&1 &
timeout 40 ros2 topic hz /world/obstacles/lidar_check > compliance_drill_hz_lidar.log 2>&1 &
timeout 40 ros2 topic hz /world/obstacles/bev_check > compliance_drill_hz_bev.log 2>&1 &

echo "[5/6] 只播放比测现场真实存在的话题，实时速率（--rate 1.0）..."
timeout 35 ros2 bag play ~/bags/bag_09_03_17 --clock --rate 1.0 --topics \
  /wamv/sensors/lidars/front_lidar_sensor/points \
  /wamv/sensors/lidars/back_lidar_sensor/points \
  /wamv/sensors/lidars/left_lidar_sensor/points \
  /wamv/sensors/lidars/left2_lidar_sensor/points \
  /wamv/sensors/lidars/right_lidar_sensor/points \
  /wamv/sensors/lidars/right2_lidar_sensor/points \
  /wamv/sensors/lidars/forward_lidar_sensor/points \
  /wamv/sensors/cameras/front_camera_sensor/image_raw \
  /wamv/sensors/cameras/back_camera_sensor/image_raw \
  /wamv/sensors/cameras/left_camera_sensor/image_raw \
  /wamv/sensors/cameras/left2_camera_sensor/image_raw \
  /wamv/sensors/cameras/right_camera_sensor/image_raw \
  /wamv/sensors/cameras/right2_camera_sensor/image_raw \
  > compliance_drill_bag.log 2>&1
echo "  bag播放完成"

echo "[6/6] 等待收尾..."
sleep 8
kill "${PERCEPTION_PID}" 2>/dev/null || true
sleep 2
pkill -9 -f "perception_node" 2>/dev/null || true
echo "done"
