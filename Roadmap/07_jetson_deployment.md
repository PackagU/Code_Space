# 07. Jetson 배포 — 시뮬에서 실기로

> 상태: [20%] 🔄 · 담당: Lee + Han(구동부 bringup 협의) · 선행: 06 (이미지 빌드는 병렬) · 갱신: 2026-07-03
> 실제 진행: `docker/Dockerfile.jetson`(aarch64, GUI 제외) + `docker/compose/docker-compose.jetson.yml`
> + 배포/부하 검증 절차(`docs/deployment/01_portability_policy.md`) + 공용 프로파일러 준비 완료.
> 남은 것: buildx 크로스 빌드/GHCR push, Jetson 실기 R1~R6 검증.

## 목표

검증된 스택을 Jetson Xavier NX에서 그대로 실행한다. 시뮬 전용 부분(Gazebo, elevator sim)과 실기 전용 부분(센서 드라이버, OpenCR)의 경계를 명확히 한 채 올린다.

## 작업 단계

### 1단계: aarch64 Docker 이미지 (최우선 병렬 — 가장 먼저 막힐 항목)

- [ ] 현재 GHCR 이미지의 아키텍처 확인: `docker manifest inspect ghcr.io/packagu/<image>` — amd64뿐이면 Jetson에서 실행 불가
- [ ] aarch64 빌드 경로 결정: (a) GitHub Actions buildx multi-arch, (b) Jetson에서 직접 빌드. JetPack L4T 베이스 이미지 필요 여부 검토 (GPU 가속 미사용 시 일반 arm64 ros:humble로 충분)
- [ ] Gazebo/RViz 등 시뮬 전용 패키지를 뺀 **런타임 이미지** 분리 (Jetson 자원 절약)
- [ ] 이미지에 secret 미포함 검증: `.dockerignore`에 `.env`, 키 파일 등재 + 빌드 후 `docker history`로 확인

### 2단계: Jetson 기본 세팅

- [ ] JetPack 설치, 전원 모드 결정(15W/20W — 배터리 용량과 협의), Docker + 컨테이너 자동 시작 설정
- [ ] 계정/SSH 하드닝 (08 문서 체크리스트), 고정 호스트명/IP
- [ ] 시계 동기화(NTP) — TF/로그 타임스탬프 어긋남 방지

### 3단계: 센서/구동부 브링업

- [ ] RPLiDAR A1m8: udev rule 고정 장치명, `rplidar_ros` 실행, `/scan` 주파수·범위 확인
- [ ] OpenCR: 펌웨어(IMU + 바퀴), 시리얼 연결, `/cmd_vel` → 모터, `/odom` 발행 확인 (Han 영역 — 인터페이스: `/cmd_vel`, `/odom`, `/imu` 토픽 계약 고정)
- [ ] teleop으로 바퀴 방향/속도 부호 검증 (시뮬에서 잡았던 rpy 반전 같은 문제의 실기 버전)

### 4단계: 성능 프로파일

- [ ] Nav2 + AMCL + mission 스택 실행 중 CPU/메모리 측정 (`tegrastats`), 여유 30% 이상 확보
- [ ] 부족 시: costmap 갱신 주기/해상도 하향, RViz는 데스크탑에서 원격으로만
- [ ] DDS 설정: `ROS_DOMAIN_ID` 고정, 원격 모니터링용 데스크탑과 동일 도메인, 필요 시 discovery peer 고정

### 5단계: 점진 투입 (각 단계 통과 후 다음으로)

| 순서 | 내용 | 통과 기준 |
|------|------|----------|
| R1 | 벤치: 바퀴 들린 상태 cmd_vel/odom/scan 확인 | 토픽 정상, E-stop 동작 |
| R2 | 빈 복도 teleop 주행 + 실맵 SLAM | 맵 품질 OK |
| R3 | 단일층 Nav2 자율주행 (사람 통제 구역) | point goal 3회 성공 |
| R4 | 단일층 미션 (픽업→배달, 엘베 제외) | 미션 1회 완주 |
| R5 | 엘베 포함 미션 — 호출/문은 수동 보조, 맵 전환은 자동 | 층 전환 자동 동작 |
| R6 | 최종: 시연 시나리오 전체 | 리허설 2회 연속 성공 |

## 완료 기준

- R4(단일층 미션 1회 완주)가 이 문서의 완료 기준. R5~R6은 시연 준비 단계로 별도 추적.

## 리스크 / 안전·보안 체크

| 리스크 | 대응 |
|--------|------|
| amd64 이미지라 Jetson에서 실행 불가 | 1단계를 지금 시작 — 마지막에 발견하면 일정 전체 지연 |
| Xavier NX 성능 부족 | 4단계 프로파일을 R3 전에 완료, 파라미터 하향 여지 확보 |
| 실기 odom 품질이 시뮬과 다름 | R2에서 odom 드리프트 측정, 필요 시 IMU 융합(EKF) 추가 결정 |
| 실제 엘베 버튼 — 로봇팔 연동 미완 | R5는 수동 보조로 진행 가능하게 시나리오 분리 (Kim 일정과 분리) |

[08_safety_security.md](./08_safety_security.md) 게이트: R1 전 전체 체크리스트(물리 E-stop 포함) 통과 필수.
