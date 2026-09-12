#!/usr/bin/env bash
# 컨테이너 안에서 실행: 구동/팔 노드를 시작하지 않고 매핑 패키지만 빌드한다.
set -eo pipefail
source /opt/ros/humble/setup.bash
set -u
cd /ros2_ws
colcon build --symlink-install --base-paths src --packages-select common_pkg slam_pkg
set +u
source install/setup.bash
set -u
ros2 pkg prefix common_pkg
ros2 pkg prefix slam_pkg
test -r "$(ros2 pkg prefix slam_pkg)/share/slam_pkg/launch/handheld_mapping.launch.py"
test -r "$(ros2 pkg prefix slam_pkg)/share/slam_pkg/config/slam_toolbox_handheld_params.yaml"
echo "handheld_mapping_build=PASS"
