#!/usr/bin/env bash
# Running handheld mapping graph verification. Read-only: no actuator topics/services.
set -eo pipefail
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

OUT=/tmp/handheld_mapping_check
mkdir -p "${OUT}"
ros2 node list >"${OUT}/nodes.txt"
if grep -Eiq '/(opencr|arm|lift|motor)' "${OUT}/nodes.txt"; then
    echo "ERROR: actuator node found" >&2
    cat "${OUT}/nodes.txt" >&2
    exit 1
fi

set +e
timeout -s INT 8 ros2 topic hz /scan --window 30 >"${OUT}/scan_hz.txt" 2>&1
set -e
if ! grep -q 'average rate:' "${OUT}/scan_hz.txt"; then
    echo "ERROR: /scan rate not measured" >&2
    cat "${OUT}/scan_hz.txt" >&2
    exit 1
fi

timeout 15 ros2 topic echo /scan --once --qos-reliability best_effort --field header \
    >"${OUT}/scan_header.txt"
timeout 20 ros2 topic echo /map --once --qos-durability transient_local \
    --qos-reliability reliable --field info >"${OUT}/map_info.txt"
timeout 8 ros2 run tf2_ros tf2_echo base_link laser >"${OUT}/tf_base_laser.txt" 2>&1 || true
timeout 8 ros2 run tf2_ros tf2_echo map odom >"${OUT}/tf_map_odom.txt" 2>&1 || true
if ! grep -q 'Translation:' "${OUT}/tf_base_laser.txt"; then
    echo "ERROR: base_link -> laser TF unavailable" >&2
    cat "${OUT}/tf_base_laser.txt" >&2
    exit 1
fi
if ! grep -q 'Translation:' "${OUT}/tf_map_odom.txt"; then
    echo "ERROR: map -> odom TF unavailable" >&2
    cat "${OUT}/tf_map_odom.txt" >&2
    exit 1
fi

echo 'NODES'
cat "${OUT}/nodes.txt"
echo 'SCAN_RATE'
tail -4 "${OUT}/scan_hz.txt"
echo 'SCAN_HEADER'
cat "${OUT}/scan_header.txt"
echo 'MAP_INFO'
cat "${OUT}/map_info.txt"
echo 'TF_BASE_LASER'
grep -m1 -A1 'Translation:' "${OUT}/tf_base_laser.txt"
echo 'TF_MAP_ODOM'
grep -m1 -A1 'Translation:' "${OUT}/tf_map_odom.txt"
echo 'handheld_mapping_check=PASS'
