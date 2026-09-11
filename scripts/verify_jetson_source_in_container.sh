#!/usr/bin/env bash
# 새 Jetson 이미지에서 현재 소스를 빌드하고 오프라인 계약을 검증한다.
# 하드웨어 노드는 시작하지 않으며 직렬 장치를 컨테이너에 전달하지 않는다.
set -euo pipefail

VERIFY_ROOT="/tmp/packagu_p02_verify"
rm -rf "${VERIFY_ROOT}"
mkdir -p "${VERIFY_ROOT}/build" "${VERIFY_ROOT}/install" "${VERIFY_ROOT}/log"

cd /ros2_ws
# 호스트 사용자 소유 저장소를 root 컨테이너에서 읽을 때 Git의 소유권 보호를
# 이 검증 프로세스에만 한정해 해제한다. 전역 Git 설정은 변경하지 않는다.
export GIT_CONFIG_COUNT=1
export GIT_CONFIG_KEY_0=safe.directory
export GIT_CONFIG_VALUE_0=/ros2_ws

colcon --log-base "${VERIFY_ROOT}/log" build \
    --base-paths src \
    --build-base "${VERIFY_ROOT}/build" \
    --install-base "${VERIFY_ROOT}/install" \
    --merge-install

# 일부 colcon 생성 스크립트는 설정되지 않은 추적 변수를 직접 참조한다.
# shellcheck disable=SC1091
set +u
source "${VERIFY_ROOT}/install/setup.bash"
set -u

for package in common_pkg drive_pkg robot_arm_pkg slam_pkg; do
    ros2 pkg prefix "${package}"
done

ros2 pkg executables drive_pkg | grep -q 'drive_pkg keyboard_teleop'
ros2 pkg executables drive_pkg | grep -q 'drive_pkg opencr_bridge'
ros2 pkg executables robot_arm_pkg | grep -q 'robot_arm_pkg arm_sequence'

test -r "$(ros2 pkg prefix slam_pkg)/share/slam_pkg/config/slam_toolbox_params.yaml"
test -r "$(ros2 pkg prefix slam_pkg)/share/slam_pkg/config/slam_toolbox_handheld_params.yaml"
test -r "$(ros2 pkg prefix slam_pkg)/share/slam_pkg/launch/handheld_mapping.launch.py"

bash scripts/run_offline_tests.sh
echo "jetson_source_container=PASS"
