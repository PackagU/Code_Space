#!/usr/bin/env bash
# 장치 없는 임시 컨테이너로 P03 DDS 계약을 검증한다.
set -euo pipefail

IMAGE="${PACKAGU_JETSON_IMAGE:-packagu/ros2-humble-slam:humble-jetson-p02}"
DOMAIN_ID="${P03_ROS_DOMAIN_ID:-223}"
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DONOR="packagu_p03_donor_$$"
UDP_DONOR="packagu_p03_udp_donor_$$"

cleanup() {
    docker rm -f "${DONOR}" "${UDP_DONOR}" >/dev/null 2>&1 || true
}
trap cleanup EXIT

run_shared_donor() {
    local token="$1"
    docker run -d --rm --name "${DONOR}" \
        --network host --ipc shareable \
        --env ROS_DOMAIN_ID="${DOMAIN_ID}" \
        --volume "${ROOT_DIR}/scripts:/ros2_ws/scripts:ro" \
        "${IMAGE}" python3 /ros2_ws/scripts/dds_contract_probe.py publisher --token "${token}" >/dev/null
    sleep 2
    timeout 25 docker run --rm \
        --network host --ipc="container:${DONOR}" \
        --env ROS_DOMAIN_ID="${DOMAIN_ID}" \
        --volume "${ROOT_DIR}/scripts:/ros2_ws/scripts:ro" \
        "${IMAGE}" python3 /ros2_ws/scripts/dds_contract_probe.py subscriber \
        --token "${token}" --timeout-sec 15
}

echo "[P03] shared IPC late-subscriber/service/QoS/transient test"
run_shared_donor SHARED_A

echo "[P03] donor recreate and sidecar rejoin test"
docker rm -f "${DONOR}" >/dev/null
sleep 1
run_shared_donor SHARED_B
docker rm -f "${DONOR}" >/dev/null

echo "[P03] UDP-only private IPC test"
docker run -d --rm --name "${UDP_DONOR}" \
    --network host --ipc private \
    --env ROS_DOMAIN_ID="${DOMAIN_ID}" \
    --env ROS_LOCALHOST_ONLY=0 \
    --env FASTRTPS_DEFAULT_PROFILES_FILE=/ros2_ws/scripts/fastdds_udp_only.xml \
    --volume "${ROOT_DIR}/scripts:/ros2_ws/scripts:ro" \
    "${IMAGE}" python3 /ros2_ws/scripts/dds_contract_probe.py publisher --token UDP_A >/dev/null
sleep 2
timeout 25 docker run --rm \
    --network host --ipc private \
    --env ROS_DOMAIN_ID="${DOMAIN_ID}" \
    --env ROS_LOCALHOST_ONLY=0 \
    --env FASTRTPS_DEFAULT_PROFILES_FILE=/ros2_ws/scripts/fastdds_udp_only.xml \
    --volume "${ROOT_DIR}/scripts:/ros2_ws/scripts:ro" \
    "${IMAGE}" python3 /ros2_ws/scripts/dds_contract_probe.py subscriber \
    --token UDP_A --timeout-sec 15

echo "jetson_dds_contract=PASS domain=${DOMAIN_ID}"
