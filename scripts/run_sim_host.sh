#!/usr/bin/env bash
# 분산 시뮬 호스트(데스크톱) 원커맨드: Gazebo 물리/센서만 띄운다.
# Jetson 이 같은 LAN 에서 GAZEBO_REMOTE=1 smoke 로 실전 스택(Nav2/미션/팔)을 실행한다.
# 절차 문서: docs/deployment/01_portability_policy.md §4.5
#
# 사용:
#   bash scripts/run_sim_host.sh [F1]          # 헤드리스 (기본)
#   GAZEBO_GUI=true bash scripts/run_sim_host.sh F1   # GUI 관찰
# 종료: Ctrl+C (컨테이너 안 gazebo 프로세스 정리까지 수행)
set -euo pipefail

FLOOR="${1:-F1}"
GAZEBO_GUI="${GAZEBO_GUI:-false}"
COMPOSE="docker/compose/docker-compose.linux.yml"

cd "$(dirname "$0")/.."

# 월드 파일이 없으면 생성 (git 미포함 산출물)
[[ -f "src/common_pkg/worlds/kku_$(echo "$FLOOR" | tr 'A-Z' 'a-z').world" ]] || python3 scripts/generate_kku_worlds.py

xhost +local:docker >/dev/null 2>&1 || true
docker compose -f "$COMPOSE" up -d

cleanup() {
  docker exec ros2_humble bash -c "pkill -f 'gazebo.launch.py' 2>/dev/null; pkill gzserver 2>/dev/null; pkill gzclient 2>/dev/null" || true
  echo "[run_sim_host] gazebo 정리 완료"
}
trap cleanup EXIT

echo "[run_sim_host] floor=$FLOOR gui=$GAZEBO_GUI — Ctrl+C 로 종료"
docker exec ros2_humble bash -c \
  "source /opt/ros/humble/setup.bash && cd /ros2_ws \
   && colcon build --symlink-install --packages-select common_pkg >/dev/null \
   && source install/setup.bash \
   && ros2 launch common_pkg gazebo.launch.py floor:=${FLOOR} spawn_point:=charge_station use_sim_time:=true gui:=${GAZEBO_GUI}"
