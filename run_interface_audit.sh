#!/usr/bin/env bash

cd /home/lyf040817/usv_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u

audit_pid=""
node_pid=""

cleanup() {
  if [[ -n "${audit_pid}" ]]; then
    kill "${audit_pid}" 2>/dev/null || true
  fi
  if [[ -n "${node_pid}" ]]; then
    kill "${node_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

python3 interface_audit.py 94 \
  > interface_audit_result.json \
  2> interface_audit.err &
audit_pid=$!

ros2 run usv_perception perception_node \
  --ros-args -p use_sim_time:=true \
  > interface_audit_perception.log \
  2>&1 &
node_pid=$!

sleep 3
ros2 bag play /home/lyf040817/bags/bag_09_03_17 \
  --clock --rate 1.0 \
  > interface_audit_bag.log \
  2>&1

wait "${audit_pid}"
audit_pid=""
cleanup
trap - EXIT INT TERM

printf '%s\n' '--- AUDIT ---'
cat interface_audit_result.json
printf '%s\n' '--- AUDIT ERRORS ---'
cat interface_audit.err
printf '%s\n' '--- NODE ERRORS ---'
grep -E '\[ERROR\]|terminate|what\(\)|exception' \
  interface_audit_perception.log | tail -n 80 || true
