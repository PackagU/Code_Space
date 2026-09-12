#!/usr/bin/env bash
set -u

# User-space overnight thermal guard for the Jetson.
# It never starts robot hardware. On a sustained critical temperature it stops
# ROS processes/container, records the incident, and attempts a clean poweroff.

WARN_MILLIC="${WARN_MILLIC:-70000}"
STOP_MILLIC="${STOP_MILLIC:-80000}"
SAMPLE_SECONDS="${SAMPLE_SECONDS:-30}"
STOP_AFTER_SAMPLES="${STOP_AFTER_SAMPLES:-2}"
END_EPOCH="${END_EPOCH:-0}"
LOG_DIR="${LOG_DIR:?LOG_DIR must be set}"
LOG_FILE="${LOG_DIR}/thermal_watchdog.log"
INCIDENT_FILE="${LOG_DIR}/incident.md"

mkdir -p -- "${LOG_DIR}"
touch -- "${LOG_FILE}"

log_line() {
  printf '%s %s\n' "$(date '+%F %T %Z')" "$*" | tee -a "${LOG_FILE}"
}

max_temperature() {
  local zone value max=0
  for zone in /sys/class/thermal/thermal_zone*/temp; do
    [[ -r "${zone}" ]] || continue
    read -r value < "${zone}" || continue
    [[ "${value}" =~ ^[0-9]+$ ]] || continue
    (( value > max )) && max="${value}"
  done
  printf '%s\n' "${max}"
}

record_snapshot() {
  local zone value type
  while IFS= read -r zone; do
    [[ -r "${zone}" ]] || continue
    read -r value < "${zone}" || value="unreadable"
    type="unknown"
    [[ -r "${zone%/temp}/type" ]] && read -r type < "${zone%/temp}/type"
    printf '%s=%s ' "${type}" "${value}"
  done < <(printf '%s\n' /sys/class/thermal/thermal_zone*/temp)
  printf '\n'
}

emergency_stop() {
  local measured="$1"
  log_line "CRITICAL max_millic=${measured}; starting emergency stop"

  if docker ps --format '{{.Names}}' | grep -Fxq 'ros2_humble'; then
    docker exec ros2_humble bash -lc \
      "pkill -INT -f 'rplidar|slam_toolbox|nav2|opencr_bridge|velocity_smoother|controller_server' || true" \
      >>"${LOG_FILE}" 2>&1 || true
    docker stop -t 20 ros2_humble >>"${LOG_FILE}" 2>&1 || true
  fi

  cat >"${INCIDENT_FILE}" <<EOF
# Jetson thermal watchdog incident

- Time: $(date '+%F %T %Z')
- Maximum thermal-zone reading: ${measured} millidegrees Celsius
- Action: sent interrupt to ROS navigation, SLAM, LiDAR, and drive-bridge processes; stopped the \`ros2_humble\` container if it was running.
- Poweroff: attempted below; success depends on the account's system power authorization.
- Robot motion: this watchdog did not start any motor or arm command.

Full log: \`thermal_watchdog.log\`
EOF
  sync

  if systemctl poweroff --no-wall >>"${LOG_FILE}" 2>&1; then
    log_line "poweroff request accepted"
  else
    log_line "poweroff request rejected; ROS container remains stopped"
  fi
}

consecutive=0
log_line "START warn_millic=${WARN_MILLIC} stop_millic=${STOP_MILLIC} stop_after_samples=${STOP_AFTER_SAMPLES} interval_seconds=${SAMPLE_SECONDS} end_epoch=${END_EPOCH}"

while true; do
  now_epoch="$(date +%s)"
  if (( END_EPOCH > 0 && now_epoch >= END_EPOCH )); then
    log_line "END scheduled monitoring window reached"
    exit 0
  fi

  max_millic="$(max_temperature)"
  snapshot="$(record_snapshot)"
  load="$(cut -d' ' -f1-3 /proc/loadavg)"
  root_usage="$(df -P / | awk 'NR==2 {print $5}')"
  log_line "SAMPLE max_millic=${max_millic} load=${load} root_usage=${root_usage} zones=${snapshot}"

  if (( max_millic >= STOP_MILLIC )); then
    ((consecutive += 1))
  else
    consecutive=0
  fi

  if (( max_millic >= WARN_MILLIC )); then
    log_line "WARNING max_millic=${max_millic} consecutive_critical=${consecutive}"
  fi

  if (( consecutive >= STOP_AFTER_SAMPLES )); then
    emergency_stop "${max_millic}"
    exit 2
  fi

  sleep "${SAMPLE_SECONDS}"
done
