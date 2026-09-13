#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "${ROOT_DIR}/install/setup.bash"
set -u
# Isolated domain only: the fixture publishes synthetic /cmd_vel* samples.
export ROS_DOMAIN_ID="${PROBE_TEST_DOMAIN_ID:-229}"
OUT="$(mktemp -d /tmp/packagu_probe_test_XXXXXX)"
probe_pid=""
cleanup() {
  [[ -n "${probe_pid}" ]] && kill -INT "${probe_pid}" 2>/dev/null || true
  rm -rf -- "${OUT}"
}
trap cleanup EXIT

python3 "${ROOT_DIR}/scripts/field_cmd_chain_probe.py" record \
  --output-dir "${OUT}/run" --duration-sec 14 --no-sys > "${OUT}/probe.log" 2>&1 &
probe_pid=$!
sleep 1.0
PACKAGU_ISOLATED_TEST_DOMAIN=1 python3 "${ROOT_DIR}/scripts/field_cmd_chain_fixture.py" --duration-sec 3.5
set +e
wait "${probe_pid}"
probe_rc=$?
set -e
probe_pid=""
if [[ ${probe_rc} -ne 0 ]]; then
  cat "${OUT}/probe.log" >&2
  exit 1
fi
python3 "${ROOT_DIR}/scripts/field_cmd_chain_probe.py" analyze "${OUT}/run" > "${OUT}/report.txt"
python3 - "${OUT}/run/summary.json" <<'PY'
import json
import sys

summary = json.load(open(sys.argv[1], encoding="utf-8"))
topics = summary["topics"]
assert topics["/cmd_vel_safe"]["count"] >= 40, topics
assert topics["/odom"]["count"] >= 100, topics
classes = summary["stutter_by_class"]
assert "drive_not_ready:feedback rpm over limit" in classes, (classes, summary["stutters"])
motion = set(summary["probe_publishers"] or []) & {"/cmd_vel", "/cmd_vel_safe", "/cmd_vel_nav", "/nav_safety/stop"}
assert not motion, summary["probe_publishers"]
print("field cmd chain probe runtime PASS", classes)
PY
