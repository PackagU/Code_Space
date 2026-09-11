#!/usr/bin/env bash
# 컨테이너 안에서 실행: RPLiDAR + RSP + fake odom + SLAM Toolbox만 시작한다.
set -eo pipefail

source /opt/ros/humble/setup.bash
set -u
cd /ros2_ws
set +u
source install/setup.bash
set -u

if [[ ! -c /dev/rplidar ]]; then
    echo "ERROR: /dev/rplidar character device missing" >&2
    exit 1
fi
for target in /dev/opencr /dev/arm_servo /dev/motor_nano; do
    if [[ ! -e "${target}" ]]; then
        continue
    fi
    if [[ "$(stat -Lc '%t:%T' "${target}")" != "$(stat -Lc '%t:%T' /dev/null)" ]]; then
        echo "ERROR: motion device unexpectedly exposed: ${target}" >&2
        exit 1
    fi
done

if pgrep -f '[h]andheld_mapping.launch.py' >/dev/null || pgrep -f '[r]plidar_composition' >/dev/null; then
    echo "ERROR: handheld mapping or RPLiDAR process already running" >&2
    exit 1
fi

STAMP="$(date +%Y%m%d-%H%M%S)"
LOG="/ros2_ws/logs/handheld_mapping_${STAMP}.log"
PID_FILE="/ros2_ws/logs/handheld_mapping.pid"

nohup ros2 launch slam_pkg handheld_mapping.launch.py serial_port:=/dev/rplidar \
    >"${LOG}" 2>&1 &
PID=$!
echo "${PID}" >"${PID_FILE}"
sleep 8
if ! kill -0 "${PID}" 2>/dev/null; then
    echo "ERROR: handheld mapping exited during startup" >&2
    tail -80 "${LOG}" >&2
    exit 1
fi

echo "handheld_mapping_started pid=${PID} log=${LOG}"
