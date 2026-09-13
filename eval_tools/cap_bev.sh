#!/bin/bash
# $1=object_render_mode  $2=输出包目录。回放 bags_full40/run1（含俯视相机+七路雷达），
# 同一录制器接收鸟瞰图与俯视图，按接收时刻配对，避免跨时钟域的固定偏移假设。
MODE=$1; OUT=$2
source /opt/ros/jazzy/setup.bash
source /home/lyf040817/usv_ws/install/setup.bash
export ROS_DOMAIN_ID=94
NODE=/home/lyf040817/usv_ws/install/usv_perception/lib/usv_perception/perception_node
rm -rf $OUT
$NODE --ros-args -p use_sim_time:=true -p object_render_mode:=$MODE > ${OUT}_node.log 2>&1 &
NP=$!
ros2 bag record -o $OUT /world/obstacles/bev_check \
  /wamv/sensors/cameras/overhead_camera_sensor/image_raw \
  /wamv/sensors/position/ground_truth_odometry \
  /world/obstacles/buoys /world/obstacles/blocks /world/obstacles/boats /world/obstacles/pillars \
  > ${OUT}_rec.log 2>&1 &
RP=$!
cleanup() { kill -9 $NP 2>/dev/null; kill -TERM $RP 2>/dev/null; }
trap cleanup EXIT
sleep 8
ros2 bag play /home/lyf040817/bags_full40/run1 --clock -r 1.0 > /dev/null 2>&1
sleep 3
kill -9 $NP 2>/dev/null
kill -TERM $RP; timeout 60 tail --pid=$RP -f /dev/null
if kill -0 $RP 2>/dev/null; then echo "!! 录制器未退出，强杀"; kill -9 $RP; fi
[ -f $OUT/metadata.yaml ] || ros2 bag reindex $OUT > /dev/null 2>&1
echo "mode=$MODE 完成：$(ros2 bag info $OUT | grep -c Topic) ；节点日志尾："
grep -c "renderObjects\|object_render" ${OUT}_node.log 2>/dev/null
