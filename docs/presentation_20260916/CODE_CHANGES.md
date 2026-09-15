# 발표용 코드 설명

기준: 직전 GitHub `lee/jetson-live` 커밋 `e223b7532d98`와 2026-09-16 직접 회수한 Jetson 파일 비교.

## 이번에 갱신한 파일

| 파일 | 변경 내용 |
|---|---|
| `scripts/start_field_base.sh` | `GATE_MAX_LINEAR_SPEED`가 비어 있으면 launch 인자를 전달하지 않음. `gate_max_linear_speed:=` 빈 값으로 인한 시작 오류를 방지 |
| `src/slam_pkg/maps/field/waypoints.json` | `f2_delivery_left_room4`, `f1_initial_test` 추가. [측정값·로그 좌표] 목적지 `(-24.73,-17.98)`, 복귀 `(-4.52,2.18)`에 대응 |
| `tools/button_arm_test/` | 젯슨 독립 시험 폴더의 최신 코드·버튼 템플릿·좌표/PWM 설정 추가. 숫자 1은 메뉴 6, 숫자 4는 메뉴 7 자세 사용. ▲▼는 별도 자세 저장 |
| `tools/floor_reader/` | 젯슨 독립 층 인식기와 ROI·숫자 템플릿 추가 |
| `tools/field_tests/` | 현장 회전·RPM·주행 확인용 독립 스크립트 보존 |
| `src/slam_pkg/maps/field/`, `handheld/` | 기존 GitHub에서 빠져 있던 지도 PGM과 YAML 포함 |

## 현재 주행 구성

LiDAR `/scan`과 휠 `/odom`으로 위치·장애물 정보를 공급한다. AMCL이 저장 지도에서 위치를 추정하고 Nav2가 목표까지 경로를 계획·추종한다. 명령은 `/cmd_vel_nav` → `/cmd_vel` → 안전 게이트 `/cmd_vel_safe` → OpenCR bridge → 바퀴로 전달된다.

`fieldctl`은 지도 해시·선택 경로 확인, 초기 pose·waypoint·goal, bag 기록·분석을 지원한다. 지도 guard는 `latest_map.txt`, YAML·PGM SHA256, 실행 중 map_server 경로를 비교한다.

현재 코드 설정: [제안값] Nav2 선속도 상한 0.10 m/s·각속도 상한 0.25 rad/s, 주행 bridge 명령 상한 48 rpm. 이는 설정값이며 물리 속도 측정값이 아니다.

F1 기본 지도는 v3, F2는 회색 미탐색 영역을 unknown으로 해석하는 YAML이다. 벽과의 여유를 위한 전역 inflation 설정은 [제안값] 반경 1.0 m, cost scaling 2.0, planner cost travel multiplier 3.0이다.

## 버튼·팔·층 인식

버튼 인식은 OpenCV 원형 캡 검출, ▲▼ 기하 판정, 등록 숫자 템플릿 비교를 사용한다. 픽셀 좌표를 보정점의 PWM으로 바꾸고 `servo_test.py`의 접근·누르기·후퇴·home 사이클을 사용한다. 숫자·화살표별 최신 자세는 `data/config.json`과 `servo_test.py`에 보존했다.

층 인식은 지정 ROI를 변환한 뒤 숫자 템플릿과 비교하고, 안정된 목표층 관측을 이벤트로 제공한다. 버튼·팔·층 인식 코드는 **코드 존재**, 과거 합성 시험은 **오프라인 검증**이다. 이번 주행 로그만으로 버튼 접촉·팔·리프트의 자동 수행 결과를 판정하지 않는다.

## 식별자

Jetson 원본 Git HEAD: `c83d092cc32f36df20b97f29483a19f7dfa41ee7`. 최신 파일에는 미커밋·Git 외부 코드가 포함되므로 이 HEAD만으로 현재 코드를 나타낼 수 없다. 파일별 원본 SHA256과 정규화된 Git blob은 [code_snapshot.json](./code_snapshot.json)에 기록했다.
