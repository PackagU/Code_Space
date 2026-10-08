# 2026-09-14~15 자율주행 테스트 로그 수집·분석

## 수집 범위와 판정

- 수집 시각: **2026-09-15 00:50 KST 전후**
- 원본: Jetson `hsm@10.141.228.26`, Docker 컨테이너 `ros2_humble`
- 로봇 상태: 수집 당시 base·mapping·navigation 모두 stopped. 로그 수집 중 노드 시작이나 주행 명령은 실행하지 않았다.
- 수집 결과: ROS 2 로그 전체, 현장 실행 기록, F1/F2 지도와 포인터, waypoint registry, Nav2 설정, `field_nav_cli.py`, 컨테이너 stdout을 로컬에 보존했다.
- 판정: 파일 수집과 아래 로그 판독은 **오프라인 검증**이다. 사용자가 보고한 실제 주행 결과는 별도 현장 근거와 함께 **실물 검증**으로 취급한다.
- 한계: 이 시험의 rosbag은 생성되지 않았다. 따라서 `/scan`, `/odom`, `/tf`, `/amcl_pose`, global/local plan과 costmap의 시계열 재생은 불가능하다. 수집 시점에는 노드가 정지해 해당 토픽도 미수신이었다.

압축 원본은 `autonomy_test_logs_20260915_0050.tar.gz`, 해제본은 `extracted/`에 있다. Windows에서 해제할 때 Linux `colcon/latest*` 심볼릭 링크만 생성되지 않았으며 실제 ROS 로그와 지도·설정 파일은 정상 추출됐다.

## 현장 세션 시간순 정리

ROS 로그의 epoch를 Asia/Seoul(KST)로 변환했다. 자세한 좌표와 상태는 `session_summary.csv`에도 보존했다.

| 시각(KST) | 층/맵 | 초기 pose | 목표 | Nav2 결과 | 핵심 로그 |
|---|---|---|---|---|---|
| 23:04:08~23:07:15 | F1 `f1_manual_clean_v1.yaml` | `(-3.725, 2.075, -1.701)` | locker `(-5.125, -8.625)` | 23:05:35 시작, 100.6초 뒤 취소 | 충돌 예측 19회(23:06:49~52), 유효 경로 없음 1회 |
| 23:29:53~23:34:09 | F1 `f1_manual_clean_v2.yaml` | `(-3.725, 2.075, -1.701)` | locker `(-5.125, -8.625)` | 23:32:01 시작, 128.4초 뒤 취소 | 취소 직전/직후 시작점이 lethal space라는 계획 실패 4회 |
| 23:42:28~00:01:06 | F2 `f2_nav_unknown_v1.yaml` | `(-11.375, -2.525, -0.829)` | delivery `(-39.925, -32.625)` 왕복 | 가는 길 416.2초 성공, 복귀 512.5초 성공 | 복귀 중 충돌 예측 795회(23:54:14~00:00:16). 사용자 보고의 마지막 문 앞 장시간 정지와 시간상 부합 |
| 00:08:11~00:11:20 | F1 `f1_manual_clean_v1.yaml` | elevator 내부/하차점 `(-8.725, -0.175, 0.919)` | idle `(-3.725, 2.075)` | 117.2초 뒤 성공 | 충돌 예측 557회(00:09:33~00:10:01). Nav2는 성공을 반환했지만 사용자의 사진상 실제 정지 위치는 목표와 어긋남 |

## 확인된 결론

1. **F1 시험에서 v1과 v2가 섞였다.** 보관함으로 간 두 번째 세션은 v2를 명시해 로드했지만, 마지막 엘리베이터→idle 복귀 세션은 v1을 로드했다. 사후 스냅샷의 `/ros2_ws/maps/field/f1/latest_map.txt`도 v1을 가리킨다. 마지막 복귀 오차의 단독 원인이라고 단정할 수는 없지만, 그 주행이 v2였다는 전제는 로그와 맞지 않는다.
2. **F1 v2 보관함 주행은 정상 완료가 아니다.** action은 취소됐고, 끝부분에서 현재 위치가 global costmap의 lethal space라 계획할 수 없다는 경고가 4회 발생했다. 사용자가 보고한 초기 위치의 좌측 오차 및 막힌 문 접근과 일치하는 강한 로그 근거다.
3. **F2 action 성공만으로 벽 접근 문제가 사라진 것은 아니다.** 복귀 구간에서 controller가 6분 1.75초 동안 795회의 collision-ahead 경고를 냈다가 최종 성공했다. 마지막 문 부근 waypoint/경로 여유를 먼저 수정해야 한다.
4. **F1 마지막 복귀도 성공 판정과 실제 정확도가 다르다.** controller는 약 27.80초 동안 557회 충돌을 예측한 후 goal reached를 기록했다. AMCL의 map 좌표상 허용 오차 안에 들어간 것과 실제 건물 기준 정차점이 맞는 것은 별개이므로, 바닥 기준점에서 pose를 다시 측정해야 한다.
5. 모든 주요 세션에서 initial pose 직후 짧은 TF extrapolation 경고가 1회씩 있었고, 초기 pose 전에는 AMCL pose 미설정 경고가 있었다. pose는 이후 설정됐으므로 이것만으로 주행 실패 원인이라 단정하지 않는다.

## 원본 위치

- ROS 노드 로그: `extracted/root/.ros/log/`
- 현장 실행 스냅샷: `extracted/ros2_ws/logs/field_execution/`
- 지도·기본 포인터: `extracted/ros2_ws/maps/field/`
- Nav2 파라미터: `extracted/ros2_ws/src/slam_pkg/config/nav2_params.yaml`
- 컨테이너 stdout: `ros2_humble_docker_20260915_0050.log`
- 수집 당시 상태·지도 해시: `autonomy_test_inventory_20260915_0050.txt`

## 무결성

| 파일 | SHA256 |
|---|---|
| `autonomy_test_logs_20260915_0050.tar.gz` | `6eba80176b6bbc663cb354f8f01305ee0b94e3d43b2a0a937e9629eabeb257ae` |
| `ros2_humble_docker_20260915_0050.log` | `650e11e91dd841de32edd45670cd7f9c5692f48ce64fa15aecd49be7710a95bb` |
| `autonomy_test_inventory_20260915_0050.txt` | `77bab12817b2c150a0bb4eea1664af5637ccbf3772cec0db8fb6db7adf61298b` |

압축 원본은 12,741,918바이트이고, 해제된 일반 파일은 1,643개·65,422,293바이트다.

