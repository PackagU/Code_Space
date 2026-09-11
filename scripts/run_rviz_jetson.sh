#!/usr/bin/env bash
# 젯슨 로컬 화면(:0)에 RViz2를 띄운다. 2026-09-10 작성.
#
# 배경: 배포 이미지 humble-jetson 에는 rviz2 실행파일이 없어 현장에서 지도가
# 그려지는 것을 볼 수단이 없었다. ros-humble-rviz2 를 설치해 커밋한 이미지가
# humble-jetson-rviz 이며, 이 스크립트는 그 이미지로 별도 컨테이너를 띄운다.
# 주 컨테이너(ros2_humble)는 GUI 없이 그대로 두는 설계를 유지한다.
#
# 사용:
#   bash scripts/run_rviz_jetson.sh                 # 빈 RViz
#   bash scripts/run_rviz_jetson.sh <설정.rviz>     # 설정 파일 지정
#
# 종료: 창을 닫거나  docker stop packagu_rviz

set -euo pipefail

IMAGE="${PACKAGU_RVIZ_IMAGE:-ghcr.io/packagu/ros2-humble-slam:humble-jetson-rviz}"
NAME="packagu_rviz"
CONFIG="${1:-}"

# 로컬 X 세션이 있어야 한다 (젯슨에 모니터가 붙어 있고 로그인된 상태)
if [ ! -S /tmp/.X11-unix/X0 ]; then
  echo "오류: /tmp/.X11-unix/X0 가 없다. 젯슨 로컬 화면에 로그인되어 있는지 확인할 것." >&2
  exit 1
fi

if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  echo "오류: 이미지 $IMAGE 가 없다." >&2
  echo "      humble-jetson 컨테이너에서 'apt-get install -y ros-humble-rviz2' 후" >&2
  echo "      'docker commit ros2_humble $IMAGE' 로 다시 만들 것." >&2
  exit 1
fi

# Fast DDS 공유 메모리(SHM) 전송 대응 (2026-09-10).
#
# 이 컨테이너와 ros2_humble 은 둘 다 host 네트워크라 호스트명이 같다. Fast DDS 는
# 이를 보고 "같은 기계" 로 판단해 SHM 전송을 고르는데, Docker 는 컨테이너마다
# /dev/shm 을 따로 준다. 그래서 상대가 만든 세그먼트를 열지 못한다.
# 증상: ros2 node list 에는 상대 노드가 다 보이는데 (탐색은 UDP 멀티캐스트라 되니까)
#       토픽/TF 는 한 건도 안 들어온다. RViz 가 "No tf data" 로 멈춘다.
#
# 해결: ros2_humble 의 IPC 네임스페이스에 합류해 /dev/shm 을 공유한다.
#       단 기증 컨테이너가 ipc: shareable 이어야 도커가 허용한다
#       (docker-compose.jetson.yml 에 설정됨. 기본값 private 로는 거부당한다).
# 폴백: shareable 이 아니면 이 컨테이너만 UDPv4 전용으로 돌린다. 느리지만 동작하고
#       ros2_humble 을 재생성할 필요가 없다 (진행 중인 매핑 세션 보존).
IPC_ARGS=()
DDS_ARGS=()
DONOR_IPC="$(docker inspect -f '{{.HostConfig.IpcMode}}' ros2_humble 2>/dev/null || true)"
if [ "$DONOR_IPC" = "shareable" ]; then
  IPC_ARGS=(--ipc=container:ros2_humble)
  echo "DDS 전송: 공유 메모리 (ros2_humble IPC 합류)"
else
  if [ ! -f "$HOME/Code_Space/scripts/fastdds_udp_only.xml" ]; then
    echo "오류: ros2_humble 이 shareable 이 아닌데 폴백 프로파일도 없다." >&2
    echo "      scripts/fastdds_udp_only.xml 을 확인할 것." >&2
    exit 1
  fi
  DDS_ARGS=(-e FASTRTPS_DEFAULT_PROFILES_FILE=/ros2_ws/scripts/fastdds_udp_only.xml)
  echo "DDS 전송: UDP 전용 폴백 (ros2_humble ipc=${DONOR_IPC:-없음})"
fi

# 컨테이너가 X 소켓에 붙을 수 있게 허용 (로컬 연결만, 네트워크 노출 아님)
DISPLAY=:0 xhost +local: >/dev/null

docker rm -f "$NAME" >/dev/null 2>&1 || true

ARGS=()
if [ -n "$CONFIG" ]; then
  ARGS=(-d "$CONFIG")
  echo "설정 파일: $CONFIG"
fi

echo "RViz2 기동 중... (이미지: $IMAGE)"
docker run -d --rm --name "$NAME" \
  --network host \
  "${IPC_ARGS[@]}" "${DDS_ARGS[@]}" \
  -v "$HOME/Code_Space/scripts:/ros2_ws/scripts:ro" \
  -e DISPLAY=:0 \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
  -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}" \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -e XDG_RUNTIME_DIR=/tmp/runtime-root \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$HOME/Code_Space/src:/ros2_ws/src:ro" \
  "$IMAGE" \
  bash -lc "source /opt/ros/humble/setup.bash && rviz2 ${ARGS[*]:-}" >/dev/null

sleep 8
if docker ps --filter "name=$NAME" --format '{{.Names}}' | grep -q "$NAME"; then
  echo "기동됨. 젯슨 화면을 확인할 것."
  echo "로그:  docker logs -f $NAME"
  echo "종료:  docker stop $NAME"
else
  echo "기동 실패. 로그:" >&2
  docker logs "$NAME" 2>&1 | tail -20 >&2
  exit 1
fi
