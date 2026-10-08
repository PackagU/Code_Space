#!/usr/bin/env bash
# Jetson 읽기 전용 상태 수집 (2026-10-08 초안).
# PC에서: ssh hsm@192.168.0.7 'bash -s' < plan/final_demo/tools/jetson_readonly_snapshot.sh > out.txt
# 하지 않는 것: 시리얼 장치 열기, 노드·컨테이너 시작/정지, git 상태 변경, 환경변수·원격 URL·비밀 파일 출력.
# 한 항목이 실패해도 다음 항목으로 넘어간다(set -e 쓰지 않음).
set -u
CS="${CS:-$HOME/Code_Space}"
C="${CONTAINER:-ros2_humble}"
sec() { printf '\n===== %s =====\n' "$1"; }

sec "A 기본"
date -Is
hostname
hostname -I
uptime
timedatectl 2>/dev/null | grep -E "Local time|synchronized|NTP service" || echo "timedatectl 없음"
df -h / | tail -1
free -h

sec "B 컨테이너"
docker ps -a --format '{{.Names}} {{.Image}} {{.Status}}' 2>&1
docker inspect "$C" --format '{{range .Mounts}}{{.Source}} -> {{.Destination}}{{"\n"}}{{end}}' 2>&1
docker inspect "$C" --format 'devices: {{range .HostConfig.Devices}}{{.PathOnHost}}->{{.PathInContainer}} {{end}}' 2>&1
docker inspect "$C" --format 'privileged={{.HostConfig.Privileged}} network={{.HostConfig.NetworkMode}} created={{.Created}}' 2>&1

sec "C Git"
if cd "$CS" 2>/dev/null; then
  git rev-parse HEAD
  git status -sb | head -1
  git log --oneline -8
  git branch -a --format='%(refname:short) %(objectname:short)' | head -30
  echo "remotes(이름만): $(git remote | tr '\n' ' ')"
  echo "stash 수: $(git stash list | wc -l)"
  echo "미커밋·untracked 항목 수: $(git status --porcelain | wc -l)"
  git status --porcelain | head -60
else
  echo "$CS 없음"
fi

sec "D 핵심 파일 SHA256 (앞 12자리)"
if cd "$CS" 2>/dev/null; then
  for f in \
    src/slam_pkg/config/nav2_params.yaml \
    src/slam_pkg/config/nav2_params_pre_wallpush_20260915.yaml \
    src/slam_pkg/config/nav2_params_wall_push_v1.yaml \
    src/slam_pkg/config/nav2_params_speed_v011.yaml \
    src/slam_pkg/config/nav2_params_speed_v012.yaml \
    src/drive_pkg/config/nav_safety.yaml \
    src/drive_pkg/config/drive_calib.yaml \
    src/slam_pkg/maps/field/waypoints.json \
    src/slam_pkg/maps/field/map_pins.json \
    src/slam_pkg/maps/field/f1/latest_map.txt \
    src/slam_pkg/maps/field/f2/latest_map.txt \
    src/slam_pkg/maps/field/f1/f1_manual_clean_v3.yaml \
    src/slam_pkg/maps/field/f1/f1_manual_clean_v3.pgm \
    src/slam_pkg/maps/field/f2/f2_nav_unknown_v1.yaml \
    src/slam_pkg/maps/field/f2/f2_raw_20260914.pgm \
    src/robot_arm_pkg/config/camera_views.json; do
    if [ -f "$f" ]; then
      printf '%s %s\n' "$(sha256sum "$f" | cut -c1-12)" "$f"
    else
      printf '%-12s %s\n' "없음" "$f"
    fi
  done
  echo "--- latest_map 내용"
  cat src/slam_pkg/maps/field/f1/latest_map.txt src/slam_pkg/maps/field/f2/latest_map.txt 2>&1
  echo "--- 지도 폴더 파일 목록"
  find src/slam_pkg/maps/field -maxdepth 2 -type f -printf '%TY-%Tm-%Td %TH:%TM %s %p\n' 2>/dev/null | sort -k3 | head -60
fi

sec "E 2026-10-06 흔적"
if cd "$CS" 2>/dev/null; then
  for f in \
    scripts/floor_arrival_probe.py \
    test_workspace/elevator_mission/src/elevator_mission_pkg/elevator_mission_pkg/elevator_camera.py \
    test_workspace/elevator_mission/src/elevator_mission_pkg/elevator_mission_pkg/camera_behaviors.py \
    test_workspace/elevator_mission/src/elevator_mission_pkg/elevator_mission_pkg/floor_reader_bridge.py \
    test_workspace/elevator_mission/src/elevator_mission_pkg/launch/elevator_camera_mission.launch.py; do
    if [ -f "$f" ]; then echo "있음 $(sha256sum "$f" | cut -c1-12) $f"; else echo "없음 $f"; fi
  done
  cat src/robot_arm_pkg/config/camera_views.json 2>&1
fi

sec "F 장치 (열지 않음)"
ls -l /dev/rplidar /dev/opencr /dev/arm_servo /dev/motor_nano 2>&1
ls -l /dev/video* 2>&1
ls -l /dev/serial/by-id/ 2>&1
lsusb 2>&1
docker exec "$C" sh -c 'ls -l /dev/rplidar /dev/opencr /dev/arm_servo /dev/video* 2>&1' 2>&1

sec "G 프로세스·포트"
echo "--- 호스트"
pgrep -af 'floor_reader|button_arm_test|app\.py|ros2|field_web' 2>/dev/null | head -30 || true
echo "--- 컨테이너"
docker exec "$C" pgrep -af 'ros2 launch|ros2 run|python3' 2>&1 | head -40
echo "--- 수신 포트 (8765 층수 인식기 등)"
ss -ltn 2>/dev/null | awk 'NR==1 || /:(8765|8766|8080|8000|5000|11311)[[:space:]]/' || echo "ss 없음"
echo "--- 층수 인식기 API (떠 있을 때만 응답)"
curl -sS --max-time 3 http://127.0.0.1:8765/api/state 2>&1 | head -c 600; echo
echo "--- ROS 노드"
docker exec "$C" bash -lc 'source /opt/ros/humble/setup.bash; timeout 8 ros2 node list' 2>&1 | sort | uniq -c | head -50

sec "H 빌드"
docker exec "$C" sh -c 'python3 --version; ls /ros2_ws/install 2>/dev/null | tr "\n" " "; echo; ls -ld --time-style=+%F_%T /ros2_ws/install/*/ 2>/dev/null | head -40' 2>&1

sec "I 9/17 이후 기록"
find "$CS/logs" -maxdepth 3 -type d -newermt 2026-09-17 -printf '%TY-%Tm-%Td %TH:%TM %p\n' 2>/dev/null | sort | tail -60
find "$HOME" -maxdepth 2 -type d -newermt 2026-09-17 -not -path '*/.*' -printf '%TY-%Tm-%Td %TH:%TM %p\n' 2>/dev/null | sort | tail -30

sec "J 팔 노드 시작 동작 (코드 읽기)"
grep -n -E 'home_on_start|simulation_mode|require_homed|serial_port' "$CS/src/robot_arm_pkg/robot_arm_pkg/arm_sequence_node.py" 2>&1 | head -20

sec "K 앱 데이터"
for d in "$HOME/button_arm_test_20260915/data" "$HOME/floor_reader/data" "$CS/tools/floor_reader/data" "$CS/tools/button_arm_test/data"; do
  if [ -d "$d" ]; then
    echo "--- $d"
    find "$d" -maxdepth 1 -type f -printf '%TY-%Tm-%Td %TH:%TM %f\n' | sort | head -40
    sha256sum "$d"/config.json 2>/dev/null | cut -c1-12
  else
    echo "없음 $d"
  fi
done

sec "L GitHub 도달 (브랜치 머리만)"
if cd "$CS" 2>/dev/null; then
  # 실패 메시지에 원격 URL이 찍힐 수 있어 사용자·토큰 부분을 가린다.
  GIT_TERMINAL_PROMPT=0 timeout 8 git ls-remote --heads origin 2>&1 \
    | sed -E 's#//[^/@[:space:]]+@#//***@#g' | awk '{print substr($1,1,12), $2}' | head -20
fi

sec "끝"
date -Is
