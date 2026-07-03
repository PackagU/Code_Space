# KKU Pre-Simulation Map — 완료 보고서

작성일: 2026-05-27
담당: Claude Code (Lee)
관련 SSOT: [01_kku_pre_simulation_plan.md](01_kku_pre_simulation_plan.md) · [src/common_pkg/config/kku_pre_simulation_map.yaml](../../../src/common_pkg/config/kku_pre_simulation_map.yaml)

## 1. 무엇이 완성되었는가

`ros2 launch slam_pkg kku_simulation.launch.py floor:=F1|F2|F3` 한 줄로 Gazebo + 로봇 + SLAM Toolbox + RViz2 가 같이 올라와 KKU 신공학관 가상 맵에서 SLAM을 돌릴 수 있는 상태.

### 1.1 생성/수정된 파일

| 파일 | 역할 | 비고 |
|------|------|------|
| [scripts/generate_kku_worlds.py](../../../scripts/generate_kku_worlds.py) | YAML→.world 생성기 | YAML 바뀌면 재실행 |
| [src/common_pkg/worlds/kku_f1.world](../../../src/common_pkg/worlds/kku_f1.world) | F1 가상 건물 | 18 walls — 엘베 + 오른쪽 15m + 뒤쪽 8m form-only + 택배 alcove |
| [src/common_pkg/worlds/kku_f2.world](../../../src/common_pkg/worlds/kku_f2.world) | F2 가상 건물 | 46 walls — 엘베 + 오른쪽 22m + 뒤쪽 14m + 8개 방 (201~208) |
| [src/common_pkg/worlds/kku_f3.world](../../../src/common_pkg/worlds/kku_f3.world) | F3 가상 건물 | 46 walls — F2와 동형, 호수 301~308 |
| [src/common_pkg/launch/gazebo.launch.py](../../../src/common_pkg/launch/gazebo.launch.py) | Gazebo + 로봇 spawn | `floor:=F1\|F2\|F3` 인자 |
| [src/slam_pkg/launch/kku_simulation.launch.py](../../../src/slam_pkg/launch/kku_simulation.launch.py) | 통합 launch | Gazebo + SLAM Toolbox + RViz |
| [src/slam_pkg/config/slam_view.rviz](../../../src/slam_pkg/config/slam_view.rviz) | RViz 기본 표시 | Grid / TF / RobotModel / LaserScan / Map |
| [src/slam_pkg/package.xml](../../../src/slam_pkg/package.xml) | 의존성 | `common_pkg`, `gazebo_ros` exec_depend 추가 |

### 1.2 핵심 설계 결정

- 좌표계: 층별 원점=엘리베이터 중앙, +x=오른쪽 복도, +y=뒤쪽 복도. M1 단계에선 SLAM frame은 단일 `map` 사용 (논리적 층 전환은 respawn 기반)
- 벽 두께 0.12m / 높이 2.4m. 단일 static model 내 `<link name="walls">`에 collision/visual 박스로 통합 (Gazebo가 한 번에 로드)
- 엘베-복도 코너에 shoulder 벽 2개 추가 — LiDAR가 코너 갭을 통해 무한 거리로 빠지는 것 방지
- 모든 복도 끝에 dead-end 끝벽
- F1 오른쪽 복도 남쪽 0.9m 문 + 택배 alcove 외곽 3벽 (북쪽은 복도 남벽과 공유, 문으로 연결)
- F2/F3 뒤쪽 복도 동/서 벽은 방 문 위치마다 5세그먼트로 분할, 각 방은 3벽(south/north/far) 박스
- 로봇 spawn pose: 세 층 모두 `(1.6, 0, yaw=0)` — 엘베 동측 문 바로 바깥, 오른쪽 복도 시작점

### 1.3 검증한 것 / 안 한 것

- ✅ Python launch 파일 syntax (`ast.parse`)
- ✅ URDF xacro 처리 (`xacro robot.urdf.xacro` 성공)
- ✅ `.world` 파일 XML well-formedness (`xml.etree.ElementTree.parse`)
- ✅ RViz config YAML 유효성
- ⚠️ 실제 Gazebo 로드 / SLAM 동작: **호스트에서 못 함** — Docker 컨테이너 안에서 `colcon build` 후 실행 필요. 다음 단계 §2.1에 안내.

## 2. 다음에 무엇을 해야 하는가

### 2.1 즉시 — 시뮬레이션 첫 실행 (≤30분)

**선결조건**: `docker_env` 컨테이너 + VcXsrv(또는 X11) 동작 상태.

1. **권한 정리** (호스트, 한 번만)
   ```bash
   sudo chown -R $USER:$USER src/slam_pkg/maps
   mkdir -p src/slam_pkg/maps/kku_virtual/f{1,2,3}
   ```
   이전 컨테이너 빌드 때 root 소유로 남은 maps/ 디렉토리 정리.

2. **빌드 + launch** (컨테이너 내부)
   ```bash
   docker exec -it ros2_humble bash
   cd /ros2_ws
   colcon build --symlink-install --packages-select common_pkg slam_pkg drive_pkg
   source install/setup.bash
   ros2 launch slam_pkg kku_simulation.launch.py floor:=F1
   ```
   Gazebo + RViz가 같은 화면에 떠야 하며, RViz의 LaserScan에 코리도 형태가 보여야 함.

3. **수동 주행** (별 터미널)
   ```bash
   ros2 run drive_pkg keyboard_teleop
   ```
   `w,a,s,d` 로 복도 일주. `q/z`는 선속도, `e/c`는 각속도 조절. SLAM Toolbox가 `/map` 토픽을 채우는 걸 RViz로 확인.

4. **맵 저장**
   ```bash
   ros2 run nav2_map_server map_saver_cli -f \
     /ros2_ws/src/slam_pkg/maps/kku_virtual/f1/kku_f1
   ```
   `.pgm` + `.yaml` 한 쌍 생성. F2, F3 동일 절차로 반복.

### 2.2 단기 (M1 마감 2026-05-31 이전)

| 항목 | 무엇을 | 어디에 |
|------|--------|--------|
| 엘리베이터 상태머신 초안 | F1→F2/F3 respawn 기반 층 전환 노드 (`GO_TO_PARCEL_ZONE → PICKUP → ENTER_ELEVATOR → TRANSFER_FLOOR → EXIT → GO_TO_ROOM → DELIVER`) | `src/drive_pkg/` 또는 새 `elevator_pkg/` (Han 협의) |
| Nav2 waypoint 변환 | YAML의 `approach_pose`/`pickup_pose`를 Nav2 PoseStamped 시퀀스로 변환하는 스크립트 | `scripts/yaml_to_nav2_waypoints.py` + `src/common_pkg/config/waypoints/f{2,3}.yaml` |
| Nav2 localization 테스트 | 저장한 `.pgm`/`.yaml` 맵으로 amcl 띄워 위치 추정 | `src/slam_pkg/launch/nav2_localization.launch.py` |
| 시연 시나리오 2개 고정 | 잠정안 `1F 택배존 → 2F 208호`, `1F 택배존 → 3F 307호` 회의에서 확정 | `plan/integration.md` 갱신 |

### 2.3 중기 (M1 이후)

- 엘리베이터 cabin 3D model + 문 열림/닫힘 애니메이션 (발표 영상용)
- 층 표시 topic (`/current_floor` std_msgs/String) — 상태머신 디버깅
- 복도 벽에 약간의 텍스처/명도 변동 → SLAM 루프 클로저 신뢰도 향상
- 실측 LiDAR 데이터 도착 후 가상 맵과 비교 평가 (실측이 가능해지면)

### 2.4 알려진 트레이드오프 / 제한

1. **SLAM frame은 단일 `map`** — 층별 `map_f1/f2/f3`는 YAML에는 표기됐지만 실제 SLAM 실행 시엔 한 번에 한 층만 mapping하고 저장 시 파일명으로 구분. 층간 TF 연결은 상태머신이 respawn으로 처리.
2. **방 사이 0.5m "pocket"** — 인접 방 사이에 0.5m 빈 공간이 있으나 로봇이 들어갈 수 없는 enclosed 구간이라 SLAM에는 무관.
3. **택배 alcove 깊이가 YAML 명세보다 0.1m 짧음** — alcove 북벽을 복도 남벽과 공유시켜 실측 1.7m 깊이 (YAML 1.8m). 물리적으로 자연스러운 모델을 우선.
4. **로봇 spawn yaw=0 (동향)** — 엘베에서 나오는 자세. 회전 시뮬은 teleop으로 직접 돌려야 함.

## 3. 재현성

YAML이 변경되면:
```bash
python3 scripts/generate_kku_worlds.py
# 그 다음 컨테이너에서
colcon build --packages-select common_pkg
```

생성기는 idempotent — 같은 입력에 대해 항상 동일한 출력. 수동 편집한 `.world` 변경은 다음 실행 때 덮어쓰임.

## 4. 완료 기준 충족 여부

[01_kku_pre_simulation_plan.md](01_kku_pre_simulation_plan.md) §완료 기준: "Codex에게 시킬 일 6항목이 명령대로 수행되면 완료".

| # | 항목 | 상태 |
|---|------|------|
| 1 | YAML 기반 Gazebo world 3개 생성 | ✅ |
| 2 | 층별 launch 인자 `floor:=F1\|F2\|F3` | ✅ |
| 3 | SLAM 테스트 통합 launch | ✅ (`kku_simulation.launch.py`) |
| 4 | 층별 맵 저장 경로 표준화 | ✅ (경로 컨벤션 + 보고서 §2.1 명령) — 실제 디렉토리는 sudo 권한 정리 후 생성 |
| 5 | 엘리베이터 층 이동 상태머신 초안 | ⏳ §2.2 |
| 6 | Nav2 waypoint 파일 | ⏳ §2.2 |

§1~4가 완료되어 **시뮬레이션 자체는 바로 돌릴 수 있는 상태**. §5~6은 SLAM 동작 검증 후의 다음 단계.
