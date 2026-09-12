# Integration contract and field sheet

기준일: 2026-09-12 KST

이 문서는 P08의 통합 책임과 H01~H04 측정표다. 현재 복도 시험은 **drive-only**이며 팔·리프트 전원을 분리하고 엘리베이터 mission을 시작하지 않는다. full mission은 코드 존재 또는 mock 단계로 남아 있으며 실물 완료가 아니다.

## 1. 실행 모드 경계

| 모드 | 시작 조건 | 허용 출력 | 금지 출력 | 현재 판정 |
|---|---|---|---|---|
| drive-only | fresh scan·odom·drive-ready, 팔·리프트 전원 분리, H01 통과 | `/cmd_vel`을 safety gate로 전달, SLAM, map 저장, Nav2 | arm·lift·elevator call | 오프라인 검증 |
| full mission | 위 조건 + 측정된 arm stow·lift home·door open·elevator stopped·통과공간 + fresh camera event | Nav2·arm·lift·floor switch | 조건 없는 다음 상태 진행 | 코드 존재(PoC) / 실물 미확인 |

`request_id`가 없는 시작, 실행 중 중복 시작, scan·odom·drive-ready 중 하나의 0.5초 초과, 통신 단절, software stop은 모두 0속도와 active 작업 해제로 귀결한다. 정지 해제 뒤에도 새 센서와 drive-ready를 다시 받아야 한다.

## 2. 입력·출력 책임

| 입력/출력 | 생산자 | 소비자 | 시작·완료 | timeout·실패·재시도 |
|---|---|---|---|---|
| `/scan` | RPLiDAR | safety gate, SLAM/AMCL, costmap | frame `laser`, fresh | 0.5초 stale이면 주행 정지; USB 재연결 뒤 새 scan 확인 |
| `/odom`, `odom→base_footprint` | OpenCR bridge | safety gate, SLAM/AMCL/Nav2 | 실측 RPM feedback | 0.5초 stale이면 정지; 저장 명령 자동 재개 금지 |
| `/cmd_vel_safe` | nav safety gate | OpenCR bridge | fresh source와 명령 | 0.3초 명령 deadman, 30 rpm 초과는 bridge/firmware 거부 |
| map·posegraph | SLAM Toolbox | map server·AMCL | YAML·PGM·posegraph·data·SHA256 모두 저장 | 일부 저장 실패 시 SLAM 유지 후 재시도 |
| Nav2 goal | 웹 화면/operator | bt navigator | accepted가 완료가 아님; result 확인 | WASD·stop·stale source에서 cancel/정지 |
| floor event | camera/floor reader | full mission | 목표층·새 session·상태 진입 뒤 fresh frame | 1초 stale·session 불일치·문/정지 근거 없음이면 거부 |
| arm stow·lift home | 실제 driver/sensor | full mission gate | measured feedback만 인정 | timer·topic-only·command accepted는 완료 아님 |
| door open·elevator stopped·clearance | ⚠️미구현 센서 | full mission gate | 세 조건 모두 fresh | 하나라도 없으면 승하차 이동 금지 |

## 3. 오늘 H01 바퀴 들림 기록표

| 항목 | 기대 | 측정값·결과 |
|---|---|---|
| OpenCR 별칭 | `/dev/opencr`가 실제 ttyACM* | ⚠️미확인 |
| 펌웨어 | `HELLO opencr 0.2-minimal`, 30 rpm 상한 | ⚠️미확인 |
| 좌우 ID·부호 | ID 1/2, 전진에서 실제 양쪽 전진 | ⚠️미확인 |
| 정지 명령 | 화면 Space·소프트 정지 즉시 0 | ⚠️미확인 |
| 명령 단절 | USB/SSH 명령 중단 후 500 ms 이내 MCU 정지 | ⚠️미확인 |
| feedback 단절 | `/drive/ready=false`, 저장 명령 재개 없음 | ⚠️미확인 |
| E-Stop | 구동 전원 물리 차단 | ⚠️미확인 |
| 바퀴 반경·간격 | `[제안값]` 0.033 m·0.51324 m와 실측 비교 | ⚠️미확인 |

한 항목이라도 실패하면 바퀴를 지면에 내리지 않는다.

## 4. 오늘 H02 복도 매핑·Nav2 기록표

| 단계 | 합격 조건 | 측정값·결과 |
|---|---|---|
| LiDAR 장착 TF | 실제 x/y/z/yaw 측정·고정, 흔들림 없음 | ⚠️미확인 |
| 정지 odom | 30초 동안 의미 있는 이동·회전 drift 없음 | ⚠️미확인 |
| 직진 odom | 실측 2 m와 odom 거리·방향 기록 | ⚠️미확인 |
| 제자리 회전 | 실제 360°와 odom yaw 기록 | ⚠️미확인 |
| 5~10 m 왕복 map | 평행 벽이 한 줄, 복귀 위치 폐루프 정합 | ⚠️미확인 |
| 저장물 | YAML·PGM·posegraph·data·SHA256 PASS | ⚠️미확인 |
| AMCL | 초기 위치 후 scan과 map 벽이 지속 정합 | ⚠️미확인 |
| 0.5~1 m 목표 | 30 rpm 이하, 충돌 없음, 도착 후 10초 정지 | ⚠️미확인 |
| 2~3 m 목표 | 앞 단계 통과 뒤에만 시행 | ⚠️미확인 |
| 30분 부하 | scan/odom gap·age 상위값, CPU/RAM/온도/디스크 기록 | ⚠️미확인 |

긴 복도는 특징이 적다. 코너나 문틀이 있는 구간을 포함하고, 같은 경로를 천천히 되돌아 폐루프를 만든다. 사람이나 이동 물체가 많은 구간은 합격 지도에서 제외한다.

## 5. H03·H04 준비물과 미구현표

| 구분 | 필요한 실물·측정 | 현재 상태 |
|---|---|---|
| 팔 | servo 전원·driver, 원점/수납 센서, PWM/각도, 하중·전류·온도, 누름거리 | 동작 제외 / 실물 미확인 |
| 리프트 | 최종 MCU/driver/motor, home·upper/lower limit, 하중·낙하 방지 | 최종 방식 미확인 |
| 카메라 | 표시창 약 2 m·버튼 30~50 cm에서 초점·노출·프레임, ROI/중심 오차 | CSI 복구 필요, USB 없음 |
| 엘리베이터 | 문폭·틈새·문턱, 정지, 문 열림, 로봇 통과공간, 버튼 높이·크기·점등 | 센서/측정 미확인 |
| 안전 | 물리 E-Stop, 받침대, 보조자, 케이블 고정, 배터리·퓨즈 | 현장 확인 필요 |
| 복구 | 로컬 map·bag·로그, source/image SHA, USB 재연결, SSH 대체 연결 | 오프라인 절차 존재 |

H03은 팔·리프트 독립 measured feedback을 확보한 뒤, H04는 H02와 H03 및 fresh door/stopped/clearance를 모두 통과한 뒤에만 수행한다.
