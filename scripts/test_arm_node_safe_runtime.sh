#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-226}"

if [[ "$(stat -Lc '%t:%T' /dev/arm_servo)" != "$(stat -Lc '%t:%T' /dev/null)" ]]; then
    echo "ERROR: safe runtime requires /dev/arm_servo mapped to /dev/null" >&2
    exit 1
fi

LOG=/tmp/p05_arm_safe_runtime.log
setsid ros2 launch robot_arm_pkg arm_sequence.launch.py >"${LOG}" 2>&1 &
PID=$!
cleanup() {
    kill -INT -- "-${PID}" 2>/dev/null || true
    wait "${PID}" 2>/dev/null || true
}
trap cleanup EXIT
sleep 2
python3 /ros2_ws/scripts/arm_node_safe_probe.py
cleanup
trap - EXIT

grep -q 'driver=none simulation=False home_on_start=False legacy_trigger=False' "${LOG}"
if grep -Eq 'startup-home|self-test-home|arm serial open:' "${LOG}"; then
    echo "ERROR: unsafe startup activity found" >&2
    cat "${LOG}" >&2
    exit 1
fi
grep -q 'hardware_unavailable' "${LOG}"
echo 'arm_node_safe_runtime=PASS'
