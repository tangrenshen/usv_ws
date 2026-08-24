#!/usr/bin/env bash
set -eo pipefail

project_dir=/home/lyf040817/usv_ws
output_dir=${1:?usage: run_partial_correction_trial.sh OUTPUT_DIR}
forward_mode=${2:-none}
restore_airy=${3:-true}

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
  -p test_extrinsic_perturb_seed:=0 \
  -p test_extrinsic_perturb_translation_m:=0.5 \
  -p test_extrinsic_perturb_rotation_deg:=10.0 \
  -p test_restore_airy_after_perturb:="${restore_airy}" \
  -p test_forward_correction_mode:="${forward_mode}" \
  -p forward_support_diagnostics_enabled:=true \
  -p calib_timer_period_sec:=1000.0 \
  >"${perception_log}" 2>&1 &
perception_pid=$!

for _ in $(seq 1 15); do
  if grep -q "TestExtrinsicPerturbation.*sensor=forward_lidar" "${perception_log}" 2>/dev/null; then
    break
  fi
  if ! kill -0 "${perception_pid}" 2>/dev/null; then
    echo "perception node exited during initialization" >&2
    exit 1
  fi
  sleep 1
done

if [[ "$(grep -c 'TestExtrinsicPerturbation.*sensor=' "${perception_log}")" -ne 7 ]]; then
  echo "expected seven deterministic perturbations" >&2
  exit 1
fi
if [[ "${restore_airy}" == "true" ]]; then
  if [[ "$(grep -c 'TestExtrinsicPartialCorrection.*restored_to_reference' "${perception_log}")" -ne 6 ]]; then
    echo "expected six restored Airy lidars" >&2
    exit 1
  fi
elif grep -q 'TestExtrinsicPartialCorrection.*restored_to_reference' "${perception_log}"; then
  echo "Airy restoration unexpectedly active" >&2
  exit 1
fi
if [[ "${forward_mode}" != "none" ]] &&
   ! grep -q "TestExtrinsicForwardCorrection.*mode=${forward_mode}" "${perception_log}"; then
  echo "expected forward correction mode ${forward_mode}" >&2
  exit 1
fi

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

printf '%s\n' "--- seed 0 partial-correction setup ---"
grep "TestExtrinsicPerturbation" "${perception_log}"
grep "TestExtrinsicPartialCorrection" "${perception_log}"
grep "TestExtrinsicForwardCorrection" "${perception_log}" || true
printf '%s\n' "--- evaluation ---"
cat "${evaluation_log}"
