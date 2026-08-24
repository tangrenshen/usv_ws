#!/usr/bin/env bash
set -eo pipefail

project_dir=/home/lyf040817/usv_ws
output_dir=${1:?usage: run_clean_forward_support_trial.sh OUTPUT_DIR}

mkdir -p "${output_dir}"
source /opt/ros/jazzy/setup.bash
source "${project_dir}/install/setup.bash"

perception_log="${output_dir}/perception.log"
boat_log="${output_dir}/boat_log.jsonl"
logger_log="${output_dir}/boat_logger.log"
bag_log="${output_dir}/bag.log"
evaluation_log="${output_dir}/four_category_v2.txt"

cleanup() {
  if [[ -n "${perception_pid:-}" ]]; then
    kill "${perception_pid}" 2>/dev/null || true
  fi
  if [[ -n "${logger_pid:-}" ]]; then
    kill "${logger_pid}" 2>/dev/null || true
  fi
  if [[ -n "${bag_pid:-}" ]]; then
    kill "${bag_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

ros2 run usv_perception perception_node \
  --ros-args \
  -p use_sim_time:=true \
  -p boat_tracker_enabled:=false \
  -p candidate_temporal_enabled:=false \
  -p test_extrinsic_perturb_seed:=-1 \
  -p forward_support_diagnostics_enabled:=true \
  -p calib_timer_period_sec:=1000.0 \
  >"${perception_log}" 2>&1 &
perception_pid=$!

for _ in $(seq 1 15); do
  if grep -q "ImageStitcher" "${perception_log}" 2>/dev/null; then
    break
  fi
  if ! kill -0 "${perception_pid}" 2>/dev/null; then
    echo "perception node exited during initialization" >&2
    exit 1
  fi
  sleep 1
done

BOAT_LOG_PATH="${boat_log}" \
CLUSTER_LOG_PATH="${perception_log}" \
python3 "${project_dir}/boat_logger.py" >"${logger_log}" 2>&1 &
logger_pid=$!
sleep 2

ros2 bag play /home/lyf040817/bags/bag_09_03_17 \
  --clock --rate 1.0 >"${bag_log}" 2>&1 &
bag_pid=$!
wait "${bag_pid}"
bag_pid=
sleep 10

kill "${logger_pid}" 2>/dev/null || true
logger_pid=
kill "${perception_pid}" 2>/dev/null || true
perception_pid=
sleep 2

python3 "${project_dir}/four_category_evaluator_v2.py" \
  "${boat_log}" \
  --epoch-offset 1782954197.752339 \
  >"${evaluation_log}"

cat "${evaluation_log}"
