#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTAINER_NAME="${PACKAGU_CONTAINER_NAME:-ros2_humble}"
DURATION_SEC="${DURATION_SEC:-1800}"
INTERVAL_SEC="${INTERVAL_SEC:-5}"
INCLUDE_CAMERA="${INCLUDE_CAMERA:-0}"
MONITOR_DOMAIN_ID="${MONITOR_DOMAIN_ID:-0}"
LABEL="${1:-field_load_$(date '+%Y%m%d_%H%M%S')}"

[[ "${DURATION_SEC}" =~ ^[1-9][0-9]*$ && "${INTERVAL_SEC}" =~ ^[1-9][0-9]*$ ]] || {
  echo "error: durations must be positive integers" >&2
  exit 2
}
[[ "${INCLUDE_CAMERA}" == "0" || "${INCLUDE_CAMERA}" == "1" ]] || { echo "error: INCLUDE_CAMERA must be 0 or 1" >&2; exit 2; }
[[ "${MONITOR_DOMAIN_ID}" =~ ^([1-9]|[1-9][0-9]|1[0-9][0-9]|2[0-2][0-9]|23[0-2]|0)$ ]] || {
  echo "error: MONITOR_DOMAIN_ID must be 0..232" >&2
  exit 2
}
[[ "${LABEL}" =~ ^[A-Za-z0-9_-]+$ ]] || { echo "error: invalid label" >&2; exit 2; }
docker ps --format '{{.Names}}' | grep -Fxq "${CONTAINER_NAME}" || { echo "error: container not running" >&2; exit 1; }

output_dir="${ROOT_DIR}/logs/field_load/${LABEL}"
[[ ! -e "${output_dir}" ]] || { echo "error: refusing to overwrite ${output_dir}" >&2; exit 1; }
mkdir -p "${output_dir}"
container_output="/ros2_ws/logs/field_load/${LABEL}/topic_metrics.json"
topics="/scan,/odom"
topic_list="$(docker exec -i "${CONTAINER_NAME}" bash -lc "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && export ROS_DOMAIN_ID='${MONITOR_DOMAIN_ID}' && ros2 topic list")"
grep -Fxq /scan <<<"${topic_list}" || { echo "error: /scan missing" >&2; exit 1; }
grep -Fxq /odom <<<"${topic_list}" || { echo "error: /odom missing" >&2; exit 1; }
grep -Fxq /imu <<<"${topic_list}" && topics+=",/imu"
if [[ "${INCLUDE_CAMERA}" == "1" ]]; then
  grep -Fxq /camera/image_raw <<<"${topic_list}" || { echo "error: camera topic missing" >&2; exit 1; }
  topics+=",/camera/image_raw"
fi

docker exec -d -w /ros2_ws "${CONTAINER_NAME}" bash -lc \
  "source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID='${MONITOR_DOMAIN_ID}' && python3 scripts/field_topic_metrics.py --duration-sec '${DURATION_SEC}' --topics '${topics}' --output '${container_output}' > '/ros2_ws/logs/field_load/${LABEL}/topic_metrics.stdout' 2>&1"

printf 'timestamp_epoch,max_temp_millic,load1,mem_available_kib,root_used_kib,container_cpu,container_mem\n' > "${output_dir}/resources.csv"
start_epoch="$(date +%s)"
while (( $(date +%s) - start_epoch < DURATION_SEC )); do
  max_temp=0
  for zone in /sys/class/thermal/thermal_zone*/temp; do
    [[ -r "${zone}" ]] || continue
    value="$(tr -dc '0-9' < "${zone}")"
    (( value > max_temp )) && max_temp="${value}"
  done
  load1="$(awk '{print $1}' /proc/loadavg)"
  mem_kib="$(awk '/MemAvailable:/ {print $2}' /proc/meminfo)"
  read -r _ _ root_used_kib _ _ _ < <(df -Pk / | awk 'NR==2')
  stats="$(docker stats --no-stream --format '{{.CPUPerc}},{{.MemUsage}}' "${CONTAINER_NAME}" | tr -d ' ')"
  printf '%s,%s,%s,%s,%s,%s\n' "$(date +%s)" "${max_temp}" "${load1}" "${mem_kib}" "${root_used_kib}" "${stats}" >> "${output_dir}/resources.csv"
  if command -v tegrastats >/dev/null 2>&1; then
    timeout 2 tegrastats --interval 1000 >> "${output_dir}/tegrastats.log" 2>&1 || true
  fi
  sleep "${INTERVAL_SEC}"
done

for _ in $(seq 1 20); do
  [[ -s "${output_dir}/topic_metrics.json" ]] && break
  sleep 1
done
[[ -s "${output_dir}/topic_metrics.json" ]] || { echo "error: topic metrics did not finish" >&2; exit 1; }
sha256sum "${output_dir}"/* > "${output_dir}/SHA256SUMS"
echo "[field-load] evidence=${output_dir}"
echo "[field-load] 30-minute thresholds remain proposed until H02/C02 field data exists"
