#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

TEST_DOMAIN_ID="${TEST_DOMAIN_ID:-230}"
TEST_PORT="${TEST_PORT:-18080}"
OUTPUT_ROOT="${1:-/tmp/packagu_field_web_ui_$(date +%s)}"
COUNTS="${OUTPUT_ROOT}/cmd_counts.json"
[[ "${OUTPUT_ROOT}" =~ ^(/[A-Za-z0-9_.-]+)+$ ]] || { echo "error: unsafe output path" >&2; exit 2; }
[[ ! -e "${OUTPUT_ROOT}" ]] || { echo "error: refusing to overwrite ${OUTPUT_ROOT}" >&2; exit 1; }
mkdir -p "${OUTPUT_ROOT}"

cleanup() {
  [[ -n "${web_pid:-}" ]] && kill -INT -- "-${web_pid}" 2>/dev/null || true
  [[ -n "${fixture_pid:-}" ]] && kill -INT -- "-${fixture_pid}" 2>/dev/null || true
}
trap cleanup EXIT

setsid env ROS_DOMAIN_ID="${TEST_DOMAIN_ID}" python3 scripts/field_web_ui_fixture.py \
  --duration-sec 8 --output "${COUNTS}" > "${OUTPUT_ROOT}/fixture.log" 2>&1 &
fixture_pid=$!
setsid env ROS_DOMAIN_ID="${TEST_DOMAIN_ID}" ros2 run slam_pkg field_web_ui --ros-args \
  -p port:="${TEST_PORT}" > "${OUTPUT_ROOT}/web.log" 2>&1 &
web_pid=$!

for _ in $(seq 1 30); do
  curl -fsS "http://127.0.0.1:${TEST_PORT}/" > "${OUTPUT_ROOT}/index.html" 2>/dev/null && break
  sleep 0.2
done
[[ -s "${OUTPUT_ROOT}/index.html" ]] || { echo "error: web UI did not start" >&2; exit 1; }
token="$(sed -n 's/.*name="packagu-token" content="\([^"]*\)".*/\1/p' "${OUTPUT_ROOT}/index.html")"
[[ -n "${token}" && "${token}" != "__PACKAGU_TOKEN__" ]] || { echo "error: missing UI token" >&2; exit 1; }
sleep 1
curl -fsS -H "X-Packagu-Token: ${token}" "http://127.0.0.1:${TEST_PORT}/api/status" > "${OUTPUT_ROOT}/status.json"
curl -fsS -H "X-Packagu-Token: ${token}" "http://127.0.0.1:${TEST_PORT}/api/map" > "${OUTPUT_ROOT}/map.json"
curl -fsS -H "X-Packagu-Token: ${token}" -H 'Content-Type: application/json' \
  -d '{"direction":"forward"}' "http://127.0.0.1:${TEST_PORT}/api/cmd" > "${OUTPUT_ROOT}/forward.json"
sleep 0.7
curl -fsS -H "X-Packagu-Token: ${token}" -H 'Content-Type: application/json' \
  -d '{}' "http://127.0.0.1:${TEST_PORT}/api/stop" > "${OUTPUT_ROOT}/stop.json"
curl -fsS -H "X-Packagu-Token: ${token}" -H 'Content-Type: application/json' \
  -d '{}' "http://127.0.0.1:${TEST_PORT}/api/stop/reset" > "${OUTPUT_ROOT}/reset.json"

wait "${fixture_pid}"
unset fixture_pid
kill -INT -- "-${web_pid}" 2>/dev/null || true
for _ in $(seq 1 30); do
  kill -0 "${web_pid}" 2>/dev/null || break
  sleep 0.1
done
kill -TERM -- "-${web_pid}" 2>/dev/null || true
wait "${web_pid}" || true
unset web_pid

python3 - "${OUTPUT_ROOT}" <<'PY'
import json
from pathlib import Path
import sys

root = Path(sys.argv[1])
status = json.loads((root / "status.json").read_text())
grid = json.loads((root / "map.json").read_text())
counts = json.loads((root / "cmd_counts.json").read_text())
assert status["base_ready"] is True, status
assert grid["available"] is True and grid["width"] == 80, grid
assert counts["nonzero"] > 0, counts
assert counts["zero_after_nonzero"] > 0, counts
assert counts["max_abs_linear"] <= 0.100001, counts
assert counts["max_abs_angular"] <= 0.350001, counts
print("field web UI runtime counts", counts)
PY
sha256sum "${OUTPUT_ROOT}"/*.json "${OUTPUT_ROOT}/index.html" > "${OUTPUT_ROOT}/SHA256SUMS"
echo "field web UI loopback, token, live map, command, deadman, and stop/reset PASS"
