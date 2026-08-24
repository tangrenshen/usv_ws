#!/usr/bin/env bash
set -eo pipefail

project_dir=/home/lyf040817/usv_ws
output_dir=${1:?usage: run_forward_consistency_trial.sh OUTPUT_DIR SCENARIO [DURATION_SEC]}
scenario=${2:?scenario must be clean, clean_forced_filter, forward_bad, all_bad, or yaw_fixed}
duration_sec=${3:-0}
monitor_bad_ratio=0.5
monitor_good_ratio=0.8

case "${scenario}" in
  clean)
    perturb_seed=-1
    restore_airy=false
    forward_mode=none
    ;;
  clean_forced_filter)
    perturb_seed=-1
    restore_airy=false
    forward_mode=none
    # Test-only forcing: a geometrically perfect ratio of 1.0 is classified
    # BAD, so the normal five-frame path activates the real online consumer.
    monitor_bad_ratio=1.1
    monitor_good_ratio=1.2
    ;;
  forward_bad)
    perturb_seed=0
    restore_airy=true
    forward_mode=none
    ;;
  all_bad)
    perturb_seed=0
    restore_airy=false
    forward_mode=none
    ;;
  yaw_fixed)
    perturb_seed=0
    restore_airy=true
    forward_mode=yaw
    ;;
  *)
    echo "unknown scenario: ${scenario}" >&2
    exit 2
    ;;
esac

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
  -p test_extrinsic_perturb_seed:="${perturb_seed}" \
  -p test_extrinsic_perturb_translation_m:=0.5 \
  -p test_extrinsic_perturb_rotation_deg:=10.0 \
  -p test_restore_airy_after_perturb:="${restore_airy}" \
  -p test_forward_correction_mode:="${forward_mode}" \
  -p forward_support_diagnostics_enabled:=false \
  -p forward_consistency_enabled:=true \
  -p forward_degraded_filter_enabled:=true \
  -p forward_consistency_bad_ratio:="${monitor_bad_ratio}" \
  -p forward_consistency_good_ratio:="${monitor_good_ratio}" \
  -p calib_timer_period_sec:=1000.0 \
  >"${perception_log}" 2>&1 &
perception_pid=$!

sleep 3
if ! kill -0 "${perception_pid}" 2>/dev/null; then
  echo "perception node exited during initialization" >&2
  exit 1
fi

BOAT_LOG_PATH="${boat_log}" \
CLUSTER_LOG_PATH="${perception_log}" \
python3 "${project_dir}/boat_logger.py" >"${logger_log}" 2>&1 &
logger_pid=$!
sleep 2

bag_args=(
  ros2 bag play /home/lyf040817/bags/bag_09_03_17
  --clock
  --rate 1.0
)
if [[ "${duration_sec}" != "0" ]]; then
  bag_args+=(--playback-duration "${duration_sec}")
fi
"${bag_args[@]}" >"${bag_log}" 2>&1 &
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

{
  echo "--- scenario ${scenario} ---"
  grep "ForwardConsistency" "${perception_log}" | tail -20 || true
  grep "ForwardDegradedFilter" "${perception_log}" | tail -5 || true
  echo "--- evaluation ---"
  cat "${evaluation_log}"
} | tee "${output_dir}/summary.txt"
