#!/usr/bin/env bash
# Start script가 남긴 프로세스 그룹에 SIGINT를 보내 LiDAR를 정상 정리한다.
set -eo pipefail
PID_FILE=/ros2_ws/logs/handheld_mapping.pid
if [[ ! -r "${PID_FILE}" ]]; then
    echo "handheld_mapping_not_running"
    exit 0
fi
PID="$(tr -dc '0-9' <"${PID_FILE}")"
if [[ -z "${PID}" ]] || ! kill -0 "${PID}" 2>/dev/null; then
    rm -f "${PID_FILE}"
    echo "handheld_mapping_not_running"
    exit 0
fi
kill -INT -- "-${PID}"
for _ in $(seq 1 20); do
    if ! kill -0 "${PID}" 2>/dev/null; then
        rm -f "${PID_FILE}"
        echo "handheld_mapping_stopped_cleanly"
        exit 0
    fi
    sleep 1
done
echo "ERROR: handheld mapping did not stop after SIGINT" >&2
exit 1
