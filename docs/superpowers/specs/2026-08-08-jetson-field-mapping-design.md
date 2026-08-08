# Jetson 실측 세션 설계 — 신공학관 LiDAR 맵핑

> 작성: 2026-08-08 · 담당: Lee · 상태: 설계 확정 (구현 계획 전 단계)
> 전제: HW 완성 가정 (섀시 + 바퀴 + 리프트 + 팔 장착, 배터리 탑재)
> 관련: `Roadmap/03_real_map_capture.md`, `Roadmap/07_jetson_deployment.md`,
> `docs/deployment/01_portability_policy.md`, `docs/hardware_spec.md` §3,
> `docs/improvement_report.md` §1.8/§1.16/§1.17

## 1. 목표와 범위

신공학관에서 로봇 teleop SLAM으로 실측 맵을 확보한다 (Roadmap 07 점진 투입 기준 R2 수준).
Jetson Xavier NX를 실기 투입 가능한 상태로 세팅하는 것까지 포함한다.

### 1.1 확정 결정사항

| 질문 | 결정 |
|------|------|
| 첫 현장 세션 목표 | teleop SLAM 맵핑까지 (R2). Nav2 자율주행(R3)은 다음 방문 |
| Jetson 현재 상태 | JetPack만 설치됨 (문서 기록 0건 — Stage 1에서 인벤토리 필수) |
| 구동부 SW | OpenCR 브링업(펌웨어·odom 캘리브레이션)도 이 계획에 포함 |
| 현장 모니터링 | 노트북 + 폰 핫스팟, SSH teleop + 원격 RViz |
| 맵핑 범위 | F1 로비 + F2 + 엘베 내부(수동 실측 중심) |
| 팔 통합 | 같은 이미지·같은 colcon ws. 실측 경로에는 장치 규약/스캔면 체크만 포함 |

### 1.2 범위 제외

- Nav2 자율주행 / AMCL 튜닝 (다음 방문, 실맵 확보 후)
- 로봇팔 제어 SW (팔은 장착 + 서보 전원 off 상태로 맵핑)
- 엘베 내부 로봇 SLAM 맵핑 (수동 실측이 주, 로봇 스캔은 보조 1회)
- F3 (F2와 동일 레이아웃 가정 유지, 시간 여유 시에만)

## 2. 전체 구조 — 3-스테이지 + 게이트

각 스테이지 끝에 통과 기준(게이트)이 있고, 게이트를 못 넘으면 다음 스테이지로 가지 않는다.
특히 Stage 1 → Stage 2 (현장행)는 게이트 통과 없이 진행 금지.

```text
Stage 0 (데스크톱, 지금 시작)      Stage 1 (벤치/랩)                Stage 2 (신공학관 현장)
aarch64 이미지 빌드+push    →     Jetson 세팅 + 브링업       →     F1+F2 teleop SLAM
                                  R1 벤치 게이트 + 랩 리허설         + 엘베 수동 실측
게이트: Jetson pull+기동 OK        게이트: R1 전항목 + 리허설 맵 OK   게이트: 맵 품질 + 실측표
```

산출물: F1/F2 실맵 + rosbag + 실측값 표 + 재현 가능한 현장 런북.

## 3. Stage 0 — 데스크톱: aarch64 이미지 준비

Roadmap 07 1단계. 절차는 `docs/deployment/01_portability_policy.md` §4.1 그대로.
다른 준비물과 무관하게 병렬로 지금 시작 가능하며, 가장 먼저 막힐 항목이라 최우선.

1. `docker manifest inspect ghcr.io/packagu/ros2-humble-slam:humble` — 현 이미지가 amd64뿐인지 확인
2. `docker buildx build --platform linux/arm64 -f docker/Dockerfile.jetson -t ghcr.io/packagu/ros2-humble-slam:humble-jetson --push docker/`
3. secret 미포함 검증: `.dockerignore` 확인 + `docker history` 점검
4. Kim 팔 의존성 트랙 (비블로킹): 팔 apt/pip 의존성 목록 수령 → `docker/Dockerfile`(dev)과
   `docker/Dockerfile.jetson` 양쪽 반영 → 재빌드. 목록이 늦어도 첫 push를 막지 않는다.

게이트: Jetson에서 pull → 컨테이너 기동 → `ros2 launch` dry-run(노드 기동만) 성공.

## 4. 로봇팔 연계 트랙 (실측 크리티컬 패스 외)

코드 통합 자리는 이미 있다: `src/robot_arm_pkg/`(뼈대) + `kim/arm-*` → `dev` PR 전략 +
같은 colcon ws라 빌드 통합은 추가 작업 0. 통합의 실체는 아래 3가지.

### 4.1 URDF 병합과 그 파급

- 팔 URDF는 독립 파일이 아니라 `src/common_pkg/urdf/delivery_robot.urdf.xacro`에
  xacro include로 병합한다 (TF 트리 일체화 — `base_link` 아래).
- LiDAR 스캔면 간섭: LiDAR는 z=0.750m 평면을 360° 스캔한다. 팔(리치 60cm)·리프트가
  이 평면을 지나면 `/scan`에 자기 몸이 찍혀 실맵에 유령 장애물이 박힌다 (§1.18의 팔 버전).
  대응: R1 벤치에서 팔 장착 + 홈포지션으로 `/scan` self-hit 확인 → 있으면 laser filter
  각도 마스크 또는 홈포지션 변경. Stage 1 게이트 항목.
- 질량/관성: 팔 질량이 COM을 바꾼다 (§1.15 전방 전복 연동). URDF inertial 반영 +
  footprint 재확인은 실측과 독립 트랙으로 진행.

### 4.2 실측 계획에 포함되는 팔 항목 (3개만)

1. udev 장치명 규약에 팔 장치 포함 (`/dev/arm_nano`, 웹캠)
2. Dockerfile 의존성 반영 트랙 (§3-4)
3. 팔 장착 + 서보 전원 off 상태로 맵핑 (실주행 무게중심·풋프린트 재현)

### 4.3 회의 안건으로 분리 (이 계획 범위 외)

- 팔 토픽 인터페이스 계약 확정 (`/arm/*` 네임스페이스, 주행 토픽 불가침, `packagu_` prefix)
- Kim 코드 이식 PR 일정, 팔 질량 실측 → URDF inertial 반영 일정

## 5. Stage 1 — Jetson 세팅 + 브링업 + R1 게이트 + 랩 리허설

### 5.1 B-0: 젯슨 인벤토리 + 기록 (첫 항목)

SSH 접속 → `cat /etc/nv_tegra_release`(JetPack/L4T 버전), 디스크 여유, docker 유무 확인.
결과를 세션 일지에 기록한다. 문서 기록이 0건인 현 상태를 반복하지 않기 위한 30분 항목.
JetPack 버전이 Docker 설치 방법과 호환성을 결정하므로 어차피 첫 단추다.

### 5.2 B-1: 기본 세팅

- 전원 모드 15W로 시작 (맵핑은 CPU 부하 낮음. 20W는 Nav2 투입 시 재검토)
- SSH 키 로그인 + 비번 로그인 off, 고정 IP/호스트명 (08 하드닝 체크리스트)
- NTP 동기화 + 현장 오프라인 대비 "노트북 기준 수동 sync" 절차를 런북에 포함
  (시계가 어긋나면 TF 타임스탬프로 SLAM이 조용히 망가진다)
- `git clone Code_Space` → `scripts/bootstrap_workspace.sh` → GHCR pull
- GHCR 로그인은 read-only PAT(read:packages 전용)로 — 개인 풀권한 토큰을 로봇에 두지
  않는다 (08 게이트: 로봇은 도난·분실 가능 기기)

### 5.3 B-2: 핫스팟 네트워크 리허설 (현장 전에 집에서 완료)

- 폰 핫스팟에 Jetson + 노트북 연결 → 상호 ping 확인. 일부 폰은 AP isolation(기기 간
  통신 차단)이 있어 이게 안 되면 현장 전체가 무너진다. 실패 시 대안: 노트북을 AP로.
- `ROS_DOMAIN_ID`를 0에서 팀 고유값(예: 42 — 회의에서 확정, 전 장비 동일 적용)으로 변경
  (캠퍼스 내 타 ROS 장비와 충돌 방지. TODO.md의 "0 유지" 메모는 시뮬 시절 값이라 대체)
- 노트북에서 `ros2 topic list` 원격 discovery + RViz `/scan` 원격 표시 확인
- fallback 검증: 이더넷 직결(노트북-Jetson)로도 동일 확인

### 5.4 B-3: 센서/구동부 브링업

- udev rule 세트: `/dev/rplidar`, `/dev/opencr`, `/dev/arm_nano`, 웹캠 — by-id 기반,
  규칙 파일 + 설치 스크립트를 repo에 커밋 (`scripts/udev/`)
- LiDAR: `/scan` 주파수·범위 확인 + 장착 yaw가 URDF와 일치하는지 확인
- OpenCR (Han 협업, 인터페이스 계약 `/cmd_vel`·`/odom`·`/imu`) — 캘리브레이션 3종:
  1. 부호: 전진 명령 → 실제 전진, 좌회전 → 반시계 (시뮬 rpy 반전의 실기 버전)
  2. 직진: 1m 주행 실측 vs odom 보고값 (바퀴 지름/트랙폭 파라미터 보정)
  3. 회전: 제자리 360° vs odom yaw
- 팔: 장착 + 서보 전원 off + `/scan` self-hit 확인 (§4.1)
- 신규 코드 산출물: 실기 매핑 launch (`src/slam_pkg/launch/`) — 기존 launch는 Gazebo
  통합(시뮬 전용)이라, `rplidar + robot_state_publisher + slam_toolbox`,
  `use_sim_time:=false`, 시리얼 포트 인자화(이식성 R3) 구성의 실기 launch가 필요.
  이 계획의 유일한 실질 코드 작업.

### 5.5 B-4: R1 벤치 게이트 (바퀴 들린 상태)

`Roadmap/08_safety_security.md` 체크리스트(물리 E-stop 포함) 통과가 선행 조건.

| 항목 | 통과 기준 |
|------|----------|
| 토픽 4종 | scan/odom/imu/cmd_vel 정상 주파수 |
| E-stop | 물리 버튼으로 즉시 정지 |
| 전원 강건성 | 모터 급가속 시 Jetson 리부팅 없음 (브라운아웃 체크) |
| 팔 간섭 | `/scan`에 self-hit 없음 (또는 필터 적용 확인) |

### 5.6 B-5: 랩 리허설 (미니 R2)

집/랩 복도에서 현장 절차를 그대로 1회 완주: 핫스팟 + SSH teleop + 실맵 SLAM +
`save_kku_map.sh` 저장 + 노트북 RViz 확인 + rosbag 기록.

게이트: 저장 맵에 벽 이중선·유령 장애물 없음 + 30분 연속 세션 안정 (가능하면 배터리 구동).

## 6. Stage 2 — 현장 런북 (신공학관)

### 6.1 출발 게이트 + 준비물

R1 + 랩 리허설 게이트 통과가 출발 조건. 2인 1조, 비혼잡 시간대.

준비물: 로봇(배터리 만충) / 노트북(만충, RViz 확인 완료 상태) / 폰(핫스팟) /
레이저 거리계 / E-stop 확인 / 이더넷 케이블 1개(핫스팟 실패 시 직결 fallback) /
테이프(원점 표시) / 실측값 기입표 (hardware_spec §3 양식).

### 6.2 현장 절차 (층당 약 40분 목표)

1. 원점 규약: 엘베 문 중앙 앞 — 바닥 테이프 표시 후 그 지점에서 SLAM 시작.
   F1/F2 동일 규약이라야 시뮬 맵과 원점 정합 비교 가능 (Roadmap 03 리스크 대응)
2. F1: 엘베 홀 → 오른쪽 복도 → 택배존 alcove → 시작점 복귀(루프 클로저). 저속 0.2~0.3m/s
3. 저장: `kku_f1_real` (시뮬 맵과 구분되는 네이밍) → 노트북 RViz 즉석 품질 확인
   (벽 이중선·유령·기울어짐) → 불합격 시 그 자리 재시도 1회
4. F2 동일 반복 (층간 이동은 사람이 로봇 동반)
5. 엘베 내부 — 거리계 수동 실측이 주 (2D LiDAR는 z=0.75m 평면만 스캔하므로 바닥
   디테일은 수동 실측이 유일한 방법):
   칸 내부 가로/세로/깊이, 문 폭, 문턱 높이, 승강로 틈새 폭 (§1.16 바퀴/캐스터 발주
   선행 조건), 버튼 높이 (§1.17 팔 장착 위치 결정) + 가능하면 로봇 칸 내 스캔 1회
   (Roadmap 05 모델링용)
6. 보조 실측: 복도 폭 2~3지점 (LiDAR 맵 교차 검증), 방 문 폭, 유리벽/반사면 위치 기록
   (실기 AMCL 튜닝 자료)

### 6.3 데이터 수집 규약

- 현장 내내 rosbag 기록: `/scan`, `/odom`, `/tf` — 현장에서 맵이 불만족스러워도 bag이
  있으면 집에서 파라미터를 바꿔 오프라인 재SLAM 가능 (재방문 1회 절약 장치)
- 맵 백업: 기존 규칙대로 `SLAM/` 경유 `setup/linux-slam-workspace` 브랜치
- 사진: 개인정보 포함 사진은 repo/Notion 업로드 금지 (Roadmap 03 보안 게이트)

### 6.4 철수 기준 (현장 디버깅 금지)

- 네트워크/DDS 15분 내 복구 불가 → 수동 실측만 수행 후 철수
- `/scan`·`/odom` 이상, 배터리 경고, 브라운아웃 → 즉시 수동 실측 전환
- 어떤 경우든 거리계 실측표는 최소 산출물로 확보 — 빈손 철수 없음
  (실측표만 있어도 Roadmap 03 YAML 갱신 트랙은 진행 가능)

### 6.5 산출물 + 사후 처리

- 산출물: `kku_f{1,2}_real` 맵 + rosbag + 실측값 표 + 유리벽 기록 + 세션 일지
- 사후 순서: `hardware_spec.md` §3 실측값 승격 → `kku_pre_simulation_map.yaml` 갱신 →
  world/맵 재생성 → 회귀 재실행 (§1.8 절차) → improvement_report §1.8/§1.16/§1.17 갱신
  → Roadmap 03/07 상태 마커 갱신
- 다음 방문 예고: 실맵 기반 AMCL 튜닝 후 R3 (Nav2 자율주행)

## 7. 신규 파일/변경 목록

| 파일 | 내용 |
|------|------|
| `src/slam_pkg/launch/` 실기 매핑 launch 1개 | rplidar + robot_state_publisher + slam_toolbox, `use_sim_time:=false`, 포트 인자화 |
| `scripts/udev/` | udev rule + 설치 스크립트 (rplidar/opencr/arm_nano/웹캠) |
| `docker/Dockerfile` + `Dockerfile.jetson` | 팔 의존성 추가 (Kim 목록 수령 후, 양쪽 동시) |
| 문서 갱신 | hardware_spec §3, improvement_report §1.8/1.16/1.17, Roadmap 03/07 |

새 폴더 체계는 없다. Jetson에는 같은 repo를 clone하고 git pull로 동기화한다.

## 8. 실패 대응 요약

| 증상 | 현장 대응 | fallback |
|------|----------|----------|
| 핫스팟 AP isolation | 노트북을 AP로 전환 | 이더넷 직결 |
| DDS discovery 실패 | ROS_DOMAIN_ID·동일망 확인 | 이더넷 직결 + peer 고정 |
| 맵 품질 불량 | 그 자리 재시도 1회 | rosbag으로 오프라인 재SLAM |
| odom 이상 | 수동 실측 전환 | scan-only 재SLAM 시도(오프라인) |
| 전원 문제 | 즉시 종료 | 수동 실측만 확보 후 철수 |

## 9. 검증

게이트 3개(Stage 0/1/2 각 통과 기준)가 곧 검증 절차다. 코드 산출물(실기 launch,
udev 스크립트)은 커밋 전 `python3 scripts/check_portability.py` 통과 필수 (CI 동일).
사후 회귀는 §6.5 순서를 따른다.

## 10. 미해결 / 회의 안건

- Kim: 팔 의존성 목록, 코드 이식 PR 일정, 팔 토픽 계약 (§4.3)
- Han: OpenCR 펌웨어 준비 일정, 물리 E-stop 구현 확인, 배터리 → Jetson 전원 계통
  (브라운아웃 대비 — 승압/강압 구성)
- 팀: 신공학관 실측 방문 일정 (비혼잡 시간대), 건물 협조 필요 여부
