# Jetson 실측 세션 설계 — 신공학관 LiDAR 맵핑

> 작성: 2026-08-08 · 담당: Lee · 상태: 설계 v2 (Codex 리뷰 반영) — 구현 계획 전 단계
> 전제: HW 완성 가정 (섀시 + 바퀴 + 리프트 + 팔 장착, 배터리 탑재)
> 관련: `Roadmap/03_real_map_capture.md`, `Roadmap/07_jetson_deployment.md`,
> `docs/deployment/01_portability_policy.md`, `docs/hardware_spec.md` §3,
> `docs/improvement_report.md` §1.8/§1.16/§1.17, `docs/opencr_dynamixel_wheel_test.md` §14

## 1. 목표와 범위

신공학관에서 로봇 teleop SLAM으로 실측 맵을 확보한다 (Roadmap 07 점진 투입 기준 R2 수준).
Jetson Xavier NX를 실기 투입 가능한 상태로 세팅하는 것과, 그에 필요한 구동부 SW
(OpenCR 펌웨어 + Jetson 측 브리지)를 정식 산출물로 포함한다.

### 1.1 확정 결정사항

| 질문 | 결정 |
|------|------|
| 첫 현장 세션 목표 | teleop SLAM 맵핑까지 (R2). Nav2 자율주행(R3)은 다음 방문 |
| Jetson 현재 상태 | JetPack만 설치됨 (문서 기록 0건 — B-0 인벤토리 필수) |
| 구동부 SW | 이 계획에 정식 포함. OpenCR 펌웨어 = Han, Jetson 브리지 = Lee, 시리얼 프로토콜 계약 합의 선행 |
| 현장 모니터링 | 노트북 + 폰 핫스팟, SSH teleop + 원격 RViz |
| 맵핑 범위 | F1 로비 + F2 + 엘베 내부(수동 실측 중심) |
| 팔 통합 | 같은 이미지·같은 colcon ws. 실측 경로에는 장치 규약/스캔면 체크만 포함 |
| 팔 서보 구동 | ZX 시리얼 버스 서보 + 드라이버 보드 HPRO-0098 (USB to TTL) — udev 별칭 `/dev/arm_servo` |

### 1.2 범위 제외

- Nav2 자율주행 / AMCL 튜닝 (다음 방문, 실맵 확보 후)
- 로봇팔 제어 SW (팔은 장착 + 홈포지션 기계적 고정 상태로 맵핑 — §4.1)
- 엘베 내부 로봇 SLAM 맵핑 (수동 실측이 주, 로봇 스캔은 보조 1회)
- F3 실맵 (후속 방문. F2 실맵을 F3로 복제하는 것 금지 — §6.5)

## 2. 전체 구조 — 3-스테이지 + 게이트

각 스테이지 끝에 통과 기준(게이트)이 있고, 게이트를 못 넘으면 다음 스테이지로 가지 않는다.
특히 Stage 1 → Stage 2 (현장행)는 게이트 통과 없이 진행 금지.

```text
Stage 0 (데스크톱, 지금 시작)     Stage 1 (벤치/랩)                  Stage 2 (신공학관 현장)
aarch64 이미지 빌드+push    →    Jetson 세팅 + 구동/센서 SW     →    F1+F2 teleop SLAM
                                 R1A 벤치 → R1B 지면 → 랩 리허설      + 엘베 수동 실측
게이트: manifest arm64            게이트: R1A/R1B 전항목             게이트: 맵 품질 수치
       + QEMU smoke                     + bag 재생 재SLAM 통과              + 실측표 확보
```

산출물: F1/F2 실맵(+posegraph) + rosbag + 실측값 표 + 재현 가능한 현장 런북.

## 3. Stage 0 — 데스크톱: aarch64 이미지 준비

Roadmap 07 1단계. 절차는 `docs/deployment/01_portability_policy.md` §4.1 그대로.
다른 준비물과 무관하게 병렬로 지금 시작 가능하며, 가장 먼저 막힐 항목이라 최우선.

1. `docker manifest inspect ghcr.io/packagu/ros2-humble-slam:humble` — 현 이미지가
   amd64뿐임을 확인 (Roadmap 07 체크박스 소화)
2. `docker buildx build --platform linux/arm64 -f docker/Dockerfile.jetson -t ghcr.io/packagu/ros2-humble-slam:humble-jetson --push docker/`
3. 빌드 후 `docker manifest inspect ...:humble-jetson`으로 arm64 포함 확인 + **image
   digest와 git commit SHA를 기록** — 현장에서는 이 digest/commit을 고정 사용하고
   현장 git pull·재빌드는 금지 (재현성)
4. secret 미포함 검증: `.dockerignore` 확인 + `docker history` 점검
5. Kim 팔 의존성 트랙 (비블로킹): 팔 apt/pip 의존성 목록 수령 → `docker/Dockerfile`(dev)과
   `docker/Dockerfile.jetson` 양쪽 반영 → 재빌드. 목록이 늦어도 첫 push를 막지 않는다.

게이트: 데스크톱 QEMU 에뮬 smoke —
`docker run --rm --platform linux/arm64 <이미지> bash -lc "ros2 pkg list"` 출력에
`slam_toolbox`·`rplidar_ros` 포함. (Jetson 실기 pull/기동은 Stage 1 B-1의 첫 항목 —
스테이지 경계를 장비 기준으로 분리)

## 4. 로봇팔 연계 트랙 (실측 크리티컬 패스 외)

코드 통합 자리는 이미 있다: `src/robot_arm_pkg/`(뼈대) + `kim/arm-*` → `dev` PR 전략 +
같은 colcon ws라 빌드 통합은 추가 작업 0. 통합의 실체는 아래 3가지.

### 4.1 URDF 병합과 그 파급

- 팔 URDF는 독립 파일이 아니라 `src/common_pkg/urdf/delivery_robot.urdf.xacro`에
  xacro include로 병합한다 (TF 트리 일체화 — `base_link` 아래).
- LiDAR 스캔면 간섭: LiDAR는 z=0.750m 평면을 360° 스캔한다. 팔(리치 60cm)·리프트가
  이 평면을 지나면 `/scan`에 자기 몸이 찍혀 실맵에 유령 장애물이 박힌다 (§1.18의 팔 버전).
  대응: R1A 벤치에서 팔 장착 + 고정 상태로 `/scan` self-hit 확인 → 있으면 laser filter
  각도 마스크(이 경우 `/scan_raw`→`/scan` 구성) 또는 홈포지션 변경.
- 맵핑 중 팔 고정: 서보 전원 off만으로는 중력에 의해 처질 수 있다(스캔면 침범·요동).
  홈포지션에서 기계적 고정(스트랩/브래킷) 또는 브레이크 유무 확인을 준비물에 포함.
- 질량/관성: 팔 질량이 COM을 바꾼다 (§1.15 전방 전복 연동). URDF inertial 반영 +
  footprint 재확인은 실측과 독립 트랙으로 진행.

### 4.2 실측 계획에 포함되는 팔 항목 (3개만)

1. udev 장치명 규약에 팔 드라이버 포함: `/dev/arm_servo` (HPRO-0098, USB to TTL) + 웹캠
2. Dockerfile 의존성 반영 트랙 (§3-5)
3. 팔 장착 + 홈포지션 기계적 고정 상태로 맵핑 (실주행 무게중심·풋프린트 재현)

### 4.3 회의 안건으로 분리 (이 계획 범위 외)

- 팔 토픽 인터페이스 계약 확정 (`/arm/*` 네임스페이스, 주행 토픽 불가침, `packagu_` prefix)
- Kim 코드 이식 PR 일정, 팔 질량 실측 → URDF inertial 반영 일정
- HPRO-0098 연결·전원 사양 확인(Kim) + `hardware_spec.md`에 팔 서보 시스템 등재

## 5. Stage 1 — Jetson 세팅 + 구동/센서 SW + R1 게이트 + 랩 리허설

### 5.1 B-0: 젯슨 인벤토리 + 기록 (첫 항목)

SSH 접속 → `cat /etc/nv_tegra_release`(JetPack/L4T 버전), 디스크 여유, docker 유무,
`nvpmodel -q`(지원 전원 모드 목록) 확인. 결과를 세션 일지에 기록한다.
전원 모드는 여기서 결정: Xavier NX는 같은 전력 예산에도 코어 수가 다른 모드가 여럿이므로
6코어 모드 우선 선택, 이후 `tegrastats` 실측으로 재검토 (특정 W 수치를 미리 고정하지 않음).

### 5.2 B-1: 기본 세팅 + 이미지 반입 (Stage 0 게이트의 실기 확인 포함)

- GHCR 로그인은 read-only PAT(read:packages 전용)로 — 개인 풀권한 토큰을 로봇에 두지
  않는다 (08 게이트: 로봇은 도난·분실 가능 기기)
- `git clone Code_Space`(Stage 0에서 기록한 commit으로 checkout) →
  `scripts/bootstrap_workspace.sh` → 기록된 digest로 pull → 컨테이너 기동 →
  `ros2 launch` dry-run(노드 기동만) 성공 확인
- SSH 키 로그인 + 비번 로그인 off, 고정 IP/호스트명 (08 하드닝 체크리스트)
- NTP 동기화 + 현장 오프라인 대비 "노트북 기준 수동 sync" 절차를 런북에 포함
  (시계가 어긋나면 TF 타임스탬프로 SLAM이 조용히 망가진다)

### 5.3 B-2: 핫스팟 네트워크 리허설 (현장 전에 집에서 완료)

- 폰 핫스팟에 Jetson + 노트북 연결 → 상호 ping 확인. 일부 폰은 AP isolation(기기 간
  통신 차단)이 있어 이게 안 되면 현장 전체가 무너진다. 실패 시 대안: 노트북을 AP로.
- ping만으로는 불충분 — DDS discovery는 멀티캐스트 의존:
  `ros2 multicast send`/`receive`를 양방향으로 통과시킬 것. 멀티캐스트가 막히면
  Fast DDS discovery server 또는 initial peers 고정으로 fallback 구성을 미리 검증.
- `RMW_IMPLEMENTATION` 을 Jetson/노트북 동일하게 명시 (기본 `rmw_fastrtps_cpp`),
  노트북 방화벽을 실사용 상태(켠 채)로 두고 RViz `/scan` 원격 표시까지 확인.
- `ROS_DOMAIN_ID`를 0에서 팀 고유값(예: 42 — 회의에서 확정, 전 장비 동일 적용)으로 변경.
  주의: DOMAIN_ID는 충돌 감소 수단일 뿐 인증·접근통제가 아니다 (보안은 08 문서 영역).
- fallback 검증: 이더넷 직결(노트북-Jetson)로도 동일 확인.

### 5.4 B-3: 구동/센서 SW 산출물 (이 계획의 실제 코드 작업 범위)

현재 repo에는 teleop 외 구동부 SW가 없다 (`src/drive_pkg/` = keyboard_teleop 1개,
`docs/opencr_dynamixel_wheel_test.md` §14가 아래 항목들을 "다음 단계"로 명시).
SLAM Toolbox가 요구하는 `map→odom→base→laser` TF 트리를 만들려면 전부 필요하다.

| 산출물 | 내용 | 담당 |
|--------|------|------|
| 시리얼 프로토콜 계약 | Jetson↔OpenCR 명령/피드백 프레임·주기·단위 문서. 펌웨어·브리지 착수 전 합의 | Han+Lee |
| OpenCR 펌웨어 | Dynamixel2Arduino 기반: RPM 명령 수신, 엔코더/IMU 피드백 송신, 명령 두절 0.5s watchdog(자동 정지) | Han |
| drive_pkg 브리지 노드 | `/cmd_vel`→시리얼 RPM, 피드백→`/odom` + odom→base TF, `/imu` 발행, calib YAML(wheel_radius/separation/sign) | Lee |
| teleop 안전화 | 현 keyboard_teleop은 latch(키 안 눌러도 마지막 명령 계속 발행) — 키 미수신 0.5s 자동 정지(dead-man) 추가 | Lee |
| 매핑 launch 정비 | 기존 `slam_toolbox.launch.py`가 이미 rplidar+`serial_port` 인자 포함(시뮬·실기 공용). 리팩터링: robot_state_publisher+URDF 추가, RViz 조건부화(`rviz:=false` 기본 — Jetson 이미지에 rviz2 없음), `enable_lidar`/`enable_drive` dry-run 인자, 포트 기본값 udev 별칭 | Lee |
| 실맵 저장 | `save_kku_map.sh`는 `kku_f{1,2,3}` 하드코딩 — 실맵 모드(`kku_f?_real`, `maps/kku_real/`) 추가 + `slam_toolbox/serialize_map`으로 posegraph 동시 저장 | Lee |
| field 원커맨드 | 층별 start→launch→rosbag→맵/posegraph 저장→검증→cleanup 스크립트 + 오프라인 계약 테스트 (AGENTS E2E 정책) | Lee |

- udev rule 세트: `/dev/rplidar`, `/dev/opencr`, `/dev/motor_nano`(SSOT 표기 준수),
  `/dev/arm_servo`, 웹캠 — by-id 기반, 규칙 + 설치 스크립트를 `scripts/udev/`에 커밋
- LiDAR: `/scan` 주파수·범위 확인 + 장착 yaw가 URDF와 일치하는지 확인
- 모든 코드 산출물은 커밋 전 `python3 scripts/check_portability.py` 통과 (R1~R4 규칙)

### 5.5 B-4: R1 게이트 — 벤치(R1A) → 지면(R1B) 순서 고정

지면 주행은 R1A(E-stop·watchdog 검증) 통과 전 금지.
`Roadmap/08_safety_security.md` 체크리스트(물리 E-stop 포함)가 R1A 선행 조건.

R1A 벤치 (바퀴 들린 상태):

| 항목 | 통과 기준 (초기 제안값 — 리허설에서 조정 가능, 근거 없는 완화 금지) |
|------|------|
| 방향 부호 | 전진 명령→전진 회전방향, 좌회전→반시계 (시뮬 rpy 반전의 실기 버전) |
| 토픽 주파수 | scan ≥ 5Hz, odom ≥ 20Hz, imu ≥ 50Hz (목표치) |
| E-stop | 물리 버튼 → 0.5s 이내 정지 |
| watchdog 3종 | teleop 프로세스 kill / SSH 강제 종료 / 핫스팟 차단 — 각각 1s 이내 자동 정지 |
| 팔 간섭 | `/scan` self-hit 0 (또는 필터 적용 후 0) |

R1B 지면 (통제 구역, R1A 통과 후):

| 항목 | 통과 기준 |
|------|------|
| 직진 캘리브레이션 | 저속 1m 주행, 실측 vs odom 오차 ≤ 5% |
| 회전 캘리브레이션 | 제자리 360°, odom yaw 오차 ≤ 10° |
| 전원 강건성 | 적재 상태 급가속 시 Jetson 리부팅 없음 (브라운아웃 체크) |

### 5.6 B-5: 랩 리허설 (미니 R2) + bag 재생 게이트

집/랩 복도에서 현장 절차를 그대로 1회 완주: 핫스팟 + SSH teleop + 실맵 SLAM +
실맵 저장(맵+posegraph) + 노트북 RViz 확인 + rosbag 기록.

게이트 (전부 통과해야 현장행):

1. 맵 품질: 벽 이중선·유령 장애물 0, 시작점 복귀 시 맵상 위치 오차 ≤ 0.3m,
   실측 복도 폭 vs 맵 오차 ≤ ±5cm
2. 30분 연속 세션 안정 (가능하면 배터리 구동)
3. **bag 재생 재SLAM**: 리허설 bag을 오프라인 재생 → 재SLAM → 맵 저장까지 실제 통과.
   `/tf`에 라이브 SLAM의 `map→odom`이 섞여 있으므로 재생 시 해당 TF 제외/필터 절차를
   여기서 확립한다. 이 게이트를 통과한 절차만 "현장 bag = 보험"으로 신뢰.

## 6. Stage 2 — 현장 런북 (신공학관)

### 6.1 출발 게이트 + 준비물

R1A/R1B + 랩 리허설 게이트 통과가 출발 조건. 2인 1조, 비혼잡 시간대.
SW는 Stage 0에서 기록한 commit/digest 고정 — 현장에서 git pull·재빌드 금지.

준비물: 로봇(배터리 만충, 팔 기계적 고정) / 노트북(만충, RViz 확인 완료 상태) /
폰(핫스팟) / 레이저 거리계 / E-stop 확인 / 이더넷 케이블 1개(핫스팟 실패 시 직결) /
테이프(원점 표시) / 실측값 기입표 (hardware_spec §3 양식) /
Jetson 디스크 여유 확인(예상 rosbag 용량의 3배 이상).

### 6.2 현장 절차 (층당 약 40분 목표)

1. 원점 규약 (위치 + 방향): 엘베 문 중심선 연장선상, 문에서 복도 쪽으로 1.0m 지점.
   로봇 전방(+x)을 주 복도 진행 방향으로 정렬. 바닥 테이프에 위치와 방향 화살표를
   함께 표시. F1/F2 동일 규약이라야 시뮬 맵과 원점·yaw 정합 비교 가능.
2. F1: 엘베 홀 → 오른쪽 복도 → 택배존 alcove → 시작점 복귀(루프 클로저). 저속 0.2~0.3m/s
3. 층별 원커맨드 스크립트로 저장: `kku_f1_real` (맵 + posegraph) → 노트북 RViz 즉석
   품질 확인(벽 이중선·유령·기울어짐) → 불합격 시 그 자리 재시도 1회
4. F2 동일 반복 (층간 이동은 사람이 로봇 동반)
5. 엘베 내부 — 거리계 수동 실측이 주 (2D LiDAR는 z=0.75m 평면만 스캔하므로 바닥
   디테일은 수동 실측이 유일한 방법):
   칸 내부 가로/세로/깊이, 문 폭, 문턱 높이, 승강로 틈새 폭 (§1.16 바퀴/캐스터 발주
   선행 조건), 버튼 높이 (§1.17 팔 장착 위치 결정) + 가능하면 로봇 칸 내 스캔 1회
   (Roadmap 05 모델링용).
   안전 수칙: 문을 강제로 잡아두지 않기, 승강로 틈에 신체·도구를 넣지 않기, 문턱·틈새는
   문이 정상 개방된 상태에서 바닥면만 측정, 가능하면 시설 담당자 협조 하에 진행.
6. 보조 실측: 복도 폭 2~3지점 (LiDAR 맵 교차 검증), 방 문 폭, 유리벽/반사면 위치 기록
   (실기 AMCL 튜닝 자료)

### 6.3 데이터 수집 규약

- 현장 내내 rosbag 기록: `/scan`(laser filter 도입 시 `/scan_raw` 포함), `/odom`,
  `/imu`, `/cmd_vel`, `/tf`, `/tf_static` (+`/diagnostics` 있으면).
  오프라인 재SLAM은 B-5 bag 재생 게이트에서 확립한 절차로만 수행한다.
- rosbag은 로컬 전용: 대용량이므로 GitHub/Notion 등 외부 업로드 금지 (AGENTS #14),
  보관은 `logs/` 또는 로컬 디스크, 백업은 팀 물리 저장소로만.
- 맵/posegraph 백업: 기존 규칙대로 `SLAM/` 경유 `setup/linux-slam-workspace` 브랜치
- 사진: 개인정보 포함 사진은 repo/Notion 업로드 금지 (Roadmap 03 보안 게이트)

### 6.4 철수 기준 (현장 디버깅 금지)

- 네트워크: 핫스팟 실패 → 노트북 AP → 이더넷 직결 순으로 15분 내 복구 불가 시
  수동 실측만 수행 후 철수 (discovery만 실패하면 peers 고정 fallback 먼저)
- `/scan`·`/odom` 이상, 배터리 경고, 브라운아웃 → 즉시 수동 실측 전환
- 어떤 경우든 거리계 실측표는 최소 산출물로 확보 — 빈손 철수 없음
  (실측표만 있어도 Roadmap 03 YAML 갱신 트랙은 진행 가능)

### 6.5 산출물 + 사후 처리

- 산출물: `kku_f{1,2}_real` 맵+posegraph + rosbag + 실측값 표 + 유리벽 기록 + 세션 일지
- 사후 순서: `hardware_spec.md` §3 실측값 승격 → `kku_pre_simulation_map.yaml` 갱신 →
  world/맵 재생성 → 회귀 재실행 (§1.8 절차) → improvement_report §1.8/§1.16/§1.17 갱신
  → Roadmap 03/07 상태 마커 갱신
- Roadmap 03은 이 세션 후 **부분 완료**로 기록한다 (F3 실맵 미확보). F2 실맵을 F3
  실맵으로 복제하지 않는다 — 시뮬의 "F3 = F2 복제" 가정은 시뮬 한정으로 유지.
- 다음 방문 예고: 실맵 기반 AMCL 튜닝 후 R3 (Nav2 자율주행) + F3 실맵

## 7. 신규 파일/변경 목록

| 파일/산출물 | 내용 | 담당 |
|------|------|------|
| `docs/deployment/02_opencr_serial_protocol.md` | Jetson↔OpenCR 프레임·주기·단위 계약 | Han+Lee |
| `src/drive_pkg/firmware/opencr/` (colcon 설치 제외 대상) | OpenCR 펌웨어: RPM 제어 + 피드백 + 0.5s watchdog | Han |
| `src/drive_pkg/` 브리지 노드 + launch + calib YAML + 테스트 | cmd_vel→시리얼, /odom+TF, /imu | Lee |
| `src/drive_pkg/drive_pkg/keyboard_teleop.py` | dead-man(0.5s 자동 정지) 추가 | Lee |
| `src/slam_pkg/launch/slam_toolbox.launch.py` 리팩터링 | RSP+URDF, rviz 조건부, enable 인자, udev 포트 기본값 | Lee |
| `scripts/save_kku_map.sh` 또는 신규 field 저장 스크립트 | `_real` 모드 + posegraph 저장 | Lee |
| field 원커맨드 스크립트 + 계약 테스트 | start→launch→rosbag→저장→검증→cleanup | Lee |
| `scripts/udev/` | rplidar / opencr / motor_nano / arm_servo / 웹캠 규칙 + 설치 스크립트 | Lee |
| `docker/compose/docker-compose.jetson.yml` | udev 별칭 `devices:` 명시, `privileged` 제거 시도 + `cap_add: SYS_NICE`(chrt), 컨테이너 내 별칭 가시성 검증 | Lee |
| `docker/Dockerfile` + `Dockerfile.jetson` | 팔 의존성 추가 (Kim 목록 수령 후, 양쪽 동시) | Kim+Lee |
| `docs/hardware_spec.md` | 팔 서보 시스템 등재(ZX 시리얼 버스 서보 + HPRO-0098 USB-TTL), Nano="모터 제어 보조" 표기 재확인 | Kim 확인 |
| 문서 갱신 | hardware_spec §3, improvement_report §1.8/1.16/1.17, Roadmap 03(부분)/07 | Lee |

새 폴더 체계는 없다. Jetson에는 같은 repo를 clone하고, 현장 투입 시에는 검증된
commit/digest를 고정한다.

## 8. 실패 대응 요약

| 증상 | 현장 대응 | fallback |
|------|----------|----------|
| 핫스팟 AP isolation | 노트북을 AP로 전환 | 이더넷 직결 |
| DDS discovery 실패 (ping OK) | 멀티캐스트 차단 의심 → initial peers 고정 | discovery server / 이더넷 직결 |
| 맵 품질 불량 | 그 자리 재시도 1회 | rosbag으로 오프라인 재SLAM (B-5 확립 절차) |
| odom 이상 | 수동 실측 전환 | scan-only 재SLAM 시도(오프라인) |
| 통신 두절 중 주행 | watchdog 자동 정지 (R1A 검증됨) | 물리 E-stop |
| 전원 문제 | 즉시 종료 | 수동 실측만 확보 후 철수 |

## 9. 검증

게이트 4개(Stage 0 QEMU smoke / R1A 벤치 / R1B 지면 / B-5 리허설+bag 재생)가 곧 검증
절차다. 수치 기준은 §5.5~5.6 표를 따르고, 조정하려면 근거를 세션 일지에 남긴다.
코드 산출물은 커밋 전 `python3 scripts/check_portability.py` 통과 필수 (CI 동일).
사후 회귀는 §6.5 순서를 따른다.

## 10. 미해결 / 회의 안건

- Han+Lee: 시리얼 프로토콜 계약 합의 일정 (펌웨어·브리지 착수 전 선행)
- Han: OpenCR 펌웨어 일정, 물리 E-stop 구현 확인, 배터리 → Jetson 전원 계통
  (브라운아웃 대비 — 승압/강압 구성)
- Kim: HPRO-0098 연결·전원 사양 확인, 팔 의존성 목록, 코드 이식 PR 일정,
  팔 토픽 계약 (§4.3), hardware_spec 팔 서보 시스템 등재
- 팀: `ROS_DOMAIN_ID` 값 확정, 신공학관 실측 방문 일정(비혼잡 시간대), 시설 협조 여부
