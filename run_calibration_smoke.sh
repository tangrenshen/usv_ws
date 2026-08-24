#!/usr/bin/env bash
set -eo pipefail

project_dir=/home/lyf040817/usv_ws
output_dir=${1:?usage: run_calibration_smoke.sh OUTPUT_DIR SEED}
seed=${2:--1}
mkdir -p "${output_dir}"

source /opt/ros/jazzy/setup.bash
source "${project_dir}/install/setup.bash"

cleanup() {
  if [[ -n "${node_pid:-}" ]]; then
    kill "${node_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

ros2 run usv_perception perception_node \
  --ros-args \
  -p use_sim_time:=true \
  -p test_extrinsic_perturb_seed:="${seed}" \
  >"${output_dir}/perception.log" 2>&1 &
node_pid=$!
sleep 2

timeout 20 ros2 bag play /home/lyf040817/bags/bag_09_03_17 \
  --clock --rate 1.0 >"${output_dir}/bag.log" 2>&1 || true
sleep 2

kill "${node_pid}" 2>/dev/null || true
node_pid=

grep -E '\[ICP\]|\[.*标定|TestExtrinsicResidual' \
  "${output_dir}/perception.log" || true
