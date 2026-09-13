#!/bin/bash
# $1=run号 $2=输出目录  其余=节点参数。回放 bags_5run/run$1，同一录制器接收四类真值/四类检测/odom/融合点云
RUN=$1; OUT=$2; shift 2
source /opt/ros/jazzy/setup.bash
source /home/lyf040817/usv_ws/install/setup.bash
export ROS_DOMAIN_ID=92
NODE=/home/lyf040817/usv_ws/install/usv_perception/lib/usv_perception/perception_node
rm -rf $OUT
$NODE --ros-args -p use_sim_time:=true "$@" > ${OUT}_node.log 2>&1 &
NP=$!
T=/world/obstacles
ros2 bag record -o $OUT $T/lidar_check $T/boats $T/buoys $T/pillars $T/blocks \
  $T/boats_check $T/buoys_check $T/pillars_check $T/blocks_check \
  /wamv/sensors/position/ground_truth_odometry > ${OUT}_rec.log 2>&1 &
RP=$!
cleanup() { kill -9 $NP 2>/dev/null; kill -TERM $RP 2>/dev/null; }
trap cleanup EXIT
sleep 8
ros2 bag play /home/lyf040817/bags_5run/run$RUN --clock -r 1.0 > /dev/null 2>&1
sleep 3
kill -9 $NP 2>/dev/null
# 后台进程在非交互 shell 里屏蔽 SIGINT，必须用 SIGTERM；并核实确已退出
kill -TERM $RP; timeout 60 tail --pid=$RP -f /dev/null
if kill -0 $RP 2>/dev/null; then echo "!! 录制器未退出 PID=$RP，强杀"; kill -9 $RP; fi
[ -f $OUT/metadata.yaml ] || ros2 bag reindex $OUT > /dev/null 2>&1
ros2 bag info $OUT | grep -E "Duration|Topic:" | sed 's/Serialization.*//'
