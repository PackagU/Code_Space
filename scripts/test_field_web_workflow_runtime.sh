#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

TEST_DOMAIN_ID="${TEST_DOMAIN_ID:-231}"
TEST_PORT="${TEST_PORT:-18081}"
OUTPUT_ROOT="${1:-/tmp/packagu_field_web_workflow_$(date +%s)}"
MAP_ROOT="${OUTPUT_ROOT}/maps"
LOG_ROOT="${OUTPUT_ROOT}/process_logs"
[[ "${OUTPUT_ROOT}" =~ ^(/[A-Za-z0-9_.-]+)+$ ]] || { echo "error: unsafe output path" >&2; exit 2; }
[[ ! -e "${OUTPUT_ROOT}" ]] || { echo "error: refusing to overwrite ${OUTPUT_ROOT}" >&2; exit 1; }
mkdir -p "${OUTPUT_ROOT}"

cleanup() {
  [[ -n "${web_pid:-}" ]] && kill -INT -- "-${web_pid}" 2>/dev/null || true
  [[ -n "${base_pid:-}" ]] && kill -INT -- "-${base_pid}" 2>/dev/null || true
}
trap cleanup EXIT

setsid env ROS_DOMAIN_ID="${TEST_DOMAIN_ID}" python3 scripts/synthetic_stationary_base.py \
  > "${OUTPUT_ROOT}/base.log" 2>&1 &
base_pid=$!
setsid env ROS_DOMAIN_ID="${TEST_DOMAIN_ID}" ros2 run slam_pkg field_web_ui --ros-args \
  -p port:="${TEST_PORT}" -p map_root:="${MAP_ROOT}" -p log_root:="${LOG_ROOT}" \
  > "${OUTPUT_ROOT}/web.log" 2>&1 &
web_pid=$!

for _ in $(seq 1 40); do
  curl -fsS "http://127.0.0.1:${TEST_PORT}/" > "${OUTPUT_ROOT}/index.html" 2>/dev/null && break
  sleep 0.2
done
[[ -s "${OUTPUT_ROOT}/index.html" ]] || { echo "error: web UI did not start" >&2; exit 1; }
token="$(sed -n 's/.*name="packagu-token" content="\([^"]*\)".*/\1/p' "${OUTPUT_ROOT}/index.html")"
[[ -n "${token}" && "${token}" != "__PACKAGU_TOKEN__" ]] || { echo "error: missing UI token" >&2; exit 1; }
api="http://127.0.0.1:${TEST_PORT}/api"
headers=(-H "X-Packagu-Token: ${token}" -H 'Content-Type: application/json')

for _ in $(seq 1 30); do
  curl -fsS "${headers[@]}" "${api}/status" > "${OUTPUT_ROOT}/status_ready.json"
  python3 - "${OUTPUT_ROOT}/status_ready.json" <<'PY' && break || true
import json, sys
raise SystemExit(0 if json.load(open(sys.argv[1]))["base_ready"] else 1)
PY
  sleep 0.2
done
python3 - "${OUTPUT_ROOT}/status_ready.json" <<'PY'
import json, sys
assert json.load(open(sys.argv[1]))["base_ready"] is True
PY

curl -fsS "${headers[@]}" -d '{"floor":"F1"}' "${api}/mapping/start" > "${OUTPUT_ROOT}/mapping_start.json"
map_ready=0
for _ in $(seq 1 50); do
  curl -fsS "${headers[@]}" "${api}/map" > "${OUTPUT_ROOT}/live_map.json"
  if python3 - "${OUTPUT_ROOT}/live_map.json" <<'PY'
import json, sys
data = json.load(open(sys.argv[1]))
raise SystemExit(0 if data.get("available") and data.get("width", 0) > 0 else 1)
PY
  then
    map_ready=1
    break
  fi
  sleep 0.2
done
[[ "${map_ready}" == "1" ]] || { echo "error: live map was not published" >&2; exit 1; }

curl -fsS "${headers[@]}" -d '{"floor":"F1","name":"ui_workflow_smoke"}' \
  "${api}/mapping/stop" > "${OUTPUT_ROOT}/mapping_stop.json"
map_path="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["map"])' "${OUTPUT_ROOT}/mapping_stop.json")"
python3 -m slam_pkg.map_contract "${map_path}" > "${OUTPUT_ROOT}/map_contract.txt"

python3 - "${map_path}" > "${OUTPUT_ROOT}/navigation_request.json" <<'PY'
import json, sys
print(json.dumps({"map": sys.argv[1]}))
PY
curl -fsS "${headers[@]}" --data-binary "@${OUTPUT_ROOT}/navigation_request.json" \
  "${api}/navigation/start" > "${OUTPUT_ROOT}/navigation_start.json"
sleep 3
curl -fsS "${headers[@]}" -d '{"x":0.0,"y":0.0,"yaw":0.0}' \
  "${api}/navigation/initial-pose" > "${OUTPUT_ROOT}/initial_pose.json"
curl -fsS "${headers[@]}" -d '{"x":0.0,"y":0.0,"yaw":0.0}' \
  "${api}/navigation/goal" > "${OUTPUT_ROOT}/goal.json"
sleep 1
curl -fsS "${headers[@]}" "${api}/status" > "${OUTPUT_ROOT}/status_navigation.json"
curl -fsS "${headers[@]}" -d '{}' "${api}/navigation/stop" > "${OUTPUT_ROOT}/navigation_stop.json"

python3 - "${OUTPUT_ROOT}" <<'PY'
import json
from pathlib import Path
import sys

root = Path(sys.argv[1])
live = json.loads((root / "live_map.json").read_text())
nav = json.loads((root / "status_navigation.json").read_text())
saved = json.loads((root / "mapping_stop.json").read_text())
assert live["available"] and live["width"] > 0 and live["height"] > 0, live
assert nav["navigation_running"] is True, nav
assert Path(saved["map"]).is_file(), saved
print("field web workflow map", live["width"], live["height"], saved["map"])
PY

kill -INT -- "-${web_pid}" 2>/dev/null || true
kill -INT -- "-${base_pid}" 2>/dev/null || true
for pid in "${web_pid}" "${base_pid}"; do
  for _ in $(seq 1 30); do kill -0 "${pid}" 2>/dev/null || break; sleep 0.1; done
done
kill -TERM -- "-${web_pid}" "-${base_pid}" 2>/dev/null || true
wait "${web_pid}" || true
wait "${base_pid}" || true
unset web_pid base_pid
sha256sum "${OUTPUT_ROOT}"/*.json "${OUTPUT_ROOT}"/maps/f1/ui_workflow_smoke.* > "${OUTPUT_ROOT}/SHA256SUMS"
echo "field web mapping, save, navigation, pose, goal, and shutdown workflow PASS"
