#!/usr/bin/env bash
set -eo pipefail

project_dir=/home/lyf040817/usv_ws
output_dir=${1:?usage: run_candidate_measurement_log.sh OUTPUT_DIR}
candidate_temporal_enabled=${2:-false}
perception_log="${output_dir}/perception_log.log"
boat_log="${output_dir}/boat_log.jsonl"
logger_log="${output_dir}/boat_logger.out"

mkdir -p "${output_dir}"
source /opt/ros/jazzy/setup.bash
source "${project_dir}/install/setup.bash"

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
  -p candidate_temporal_enabled:="${candidate_temporal_enabled}" \
  >"${perception_log}" 2>&1 &
perception_pid=$!

# Calibration/initialization normally takes about 10-15 seconds.
for _ in $(seq 1 30); do
  if grep -q "初始化完毕" "${perception_log}" 2>/dev/null; then
    break
  fi
  if ! kill -0 "${perception_pid}" 2>/dev/null; then
    echo "perception_node exited during initialization" >&2
    exit 1
  fi
  sleep 1
done

BOAT_LOG_PATH="${boat_log}" \
CLUSTER_LOG_PATH="${perception_log}" \
python3 "${project_dir}/boat_logger.py" >"${logger_log}" 2>&1 &
logger_pid=$!
sleep 2

ros2 bag play /home/lyf040817/bags/bag_09_03_17/ --clock --rate 1.0 &
bag_pid=$!
wait "${bag_pid}"
bag_pid=
sleep 10

kill "${logger_pid}" 2>/dev/null || true
sleep 2

python3 - "${boat_log}" <<'PY'
import collections
import json
import sys

counts = collections.Counter()
measurement = 0
missing = 0
for line in open(sys.argv[1], encoding="utf-8"):
    record = json.loads(line)
    counts[record["type"]] += 1
    if record["type"] == "cluster":
        if record.get("measurement_t") is None:
            missing += 1
        else:
            measurement += 1
print("record_counts", dict(counts))
print("cluster_measurement_t", measurement, "missing", missing)
if not counts["det"] or not counts["cluster"] or missing:
    raise SystemExit(2)
PY
