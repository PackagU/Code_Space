#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

RECORD_DOMAIN_ID="${RECORD_DOMAIN_ID:-224}"
REPLAY_DOMAIN_ID="${REPLAY_DOMAIN_ID:-225}"
OUTPUT_ROOT="${1:-/tmp/packagu_p07_rosbag_$(date +%s)}"
BAG_DIR="${OUTPUT_ROOT}/bag"
VERIFY_JSON="${OUTPUT_ROOT}/replay_counts.json"
VERIFY_RESTART_JSON="${OUTPUT_ROOT}/replay_counts_restart.json"
[[ "${OUTPUT_ROOT}" =~ ^(/[A-Za-z0-9_.-]+)+$ ]] || { echo "error: unsafe output path" >&2; exit 2; }
[[ ! -e "${OUTPUT_ROOT}" ]] || { echo "error: refusing to overwrite ${OUTPUT_ROOT}" >&2; exit 1; }
mkdir -p "${OUTPUT_ROOT}"

cleanup() {
  [[ -n "${publisher_pid:-}" ]] && kill "${publisher_pid}" 2>/dev/null || true
  [[ -n "${verifier_pid:-}" ]] && kill "${verifier_pid}" 2>/dev/null || true
}
trap cleanup EXIT

ROS_DOMAIN_ID="${RECORD_DOMAIN_ID}" python3 scripts/p07_rosbag_fixture.py publish --duration-sec 6 --publish-command > "${OUTPUT_ROOT}/publisher.log" 2>&1 &
publisher_pid=$!
sleep 1
set +e
ROS_DOMAIN_ID="${RECORD_DOMAIN_ID}" timeout --signal=INT --kill-after=3 4 ros2 bag record \
  --qos-profile-overrides-path scripts/rosbag_qos_overrides.yaml \
  -o "${BAG_DIR}" /scan /odom /imu /tf /tf_static /cmd_vel > "${OUTPUT_ROOT}/record.log" 2>&1
record_rc=$?
set -e
[[ ${record_rc} -eq 0 || ${record_rc} -eq 124 || ${record_rc} -eq 130 ]] || {
  echo "error: recorder exit ${record_rc}" >&2
  exit 1
}
wait "${publisher_pid}"
unset publisher_pid

python3 scripts/bag_contract.py inspect "${BAG_DIR}" \
  --require /scan --require /odom --require /imu --require /tf --require /tf_static \
  > "${OUTPUT_ROOT}/contract.json"
ros2 bag info "${BAG_DIR}" > "${OUTPUT_ROOT}/bag_info.txt"
grep -Eq 'Topic: /tf_static.*Count: [1-9]' "${OUTPUT_ROOT}/bag_info.txt" || {
  echo "error: late recorder missed transient-local /tf_static" >&2
  exit 1
}

safe_topics="$(python3 scripts/bag_contract.py replay-topics "${BAG_DIR}" --profile reslam)"
grep -Eq '(^| )/cmd_vel( |$)' <<<"${safe_topics}" && { echo "error: cmd_vel selected for replay" >&2; exit 1; }
ROS_DOMAIN_ID="${REPLAY_DOMAIN_ID}" python3 scripts/p07_rosbag_fixture.py verify \
  --duration-sec 5 --output "${VERIFY_JSON}" > "${OUTPUT_ROOT}/verifier.log" 2>&1 &
verifier_pid=$!
sleep 0.5
# The playback-side delay is DDS discovery time, not a drive/gate/watchdog timeout.
ROS_DOMAIN_ID="${REPLAY_DOMAIN_ID}" ros2 bag play "${BAG_DIR}" --delay 1.0 \
  --clock 100 --rate 2.0 --topics ${safe_topics} \
  > "${OUTPUT_ROOT}/replay.log" 2>&1
wait "${verifier_pid}"
unset verifier_pid

ROS_DOMAIN_ID="${REPLAY_DOMAIN_ID}" python3 scripts/p07_rosbag_fixture.py verify \
  --duration-sec 5 --output "${VERIFY_RESTART_JSON}" > "${OUTPUT_ROOT}/verifier_restart.log" 2>&1 &
verifier_pid=$!
sleep 0.5
ROS_DOMAIN_ID="${REPLAY_DOMAIN_ID}" ros2 bag play "${BAG_DIR}" --delay 1.0 \
  --clock 100 --rate 2.0 --topics ${safe_topics} \
  > "${OUTPUT_ROOT}/replay_restart.log" 2>&1
wait "${verifier_pid}"
unset verifier_pid

python3 - "${VERIFY_JSON}" "${VERIFY_RESTART_JSON}" <<'PY'
import json
from pathlib import Path
import sys

for path_arg in sys.argv[1:]:
    counts = json.loads(Path(path_arg).read_text(encoding="utf-8"))
    assert counts["scan"] > 0, counts
    assert counts["tf_static"] > 0, counts
    assert counts["cmd_vel"] == 0, counts
    print("safe replay counts", Path(path_arg).name, counts)
PY
sha256sum "${BAG_DIR}/metadata.yaml" "${BAG_DIR}"/*.db3 > "${OUTPUT_ROOT}/SHA256SUMS"
echo "P07 rosbag late-join, inspect, restart, and motion-safe replay PASS"
