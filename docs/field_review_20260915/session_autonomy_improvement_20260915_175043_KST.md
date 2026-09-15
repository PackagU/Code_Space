# autonomy_improvement_20260915_175043_KST

2026-09-15 17:48~18:20 KST, `ssh hsm@192.168.0.7`(자율주행 시험망 `10.141.228.26`은 연결 시간 초과), 컨테이너 `ros2_humble`.
판정: **오프라인 검증**. 로봇 구동·teleop·goal·Nav2/base 시작·펌웨어 업로드는 하지 않았다. 작업 전후 `fieldctl status`는 `base=stopped mapping=stopped navigation=stopped`였다.

현장 절차와 기록 양식: 이 폴더의 `FIELD_PROCEDURE.md` (= `docs/bringup_guide/2026-09-15_autonomy_field_procedure.md`).

## 폴더

- `before/`: 변경 전 `latest_map.txt`(F1/F2/F3), 지도 YAML, `waypoints.json`, Nav2·gate·bridge 설정, 수정 대상 스크립트와 `SHA256SUMS`, 지도 PGM 해시 `SHA256SUMS.map_images`
- `after/`: 변경 후 같은 항목과 신규 파일, `SHA256SUMS`
- `map_guard_records/`: 무구동 guard 실행 기록 5건(PASS 3, 의도한 FAIL 2)

## 실행 순서와 결과

1. 기준선: `fieldctl status` 전부 stopped. F1 v1/v2, F2 YAML/PGM, `waypoints.json` SHA256이 문서값과 일치. `latest_map.txt` F1은 v1이었다.
2. 백업: 위 `before/` 생성.
3. 로그 재현: 로컬 `logs/field_execution/20260915_autonomy_test_collection/extracted/root/.ros/log`을 bt_navigator·controller_server·planner_server·map_server 로그의 epoch로 다시 셌다. 5개 goal의 시작 시각, 결과, 소요 100.598/128.412/416.172/512.533/117.192 s, collision-ahead 19/0/0/795/557, 지속 2.948/–/–/361.750/27.800 s, no-valid-path 1, lethal-start 4, 세션별 로드 지도 v1/v2/F2/F2/v1이 `session_summary.csv`와 모두 일치했다. 스크립트는 로컬 `artifacts/autonomy_improvement_20260915/recount_logs.py`.
4. F1 pointer: v2 해시 `sha256sum -c` OK 후 교체. **실수 기록**: 첫 교체에서 셸 이스케이프 오류로 파일 끝이 `...v2.yamln`(줄바꿈 대신 문자 n)이 됐다. 다음 SSH 명령에서 `echo ... > tmp; mv`로 정상 47바이트(`8736d5ba…`)로 교정했다. 그 사이 Nav2는 정지 상태였고, 잘못된 값은 `nav start`의 `\.yaml$` 정규식에서 거부되므로 v1이 선택될 수는 없었다.
5. `fieldctl map validate .../f1_manual_clean_v2.yaml` → `VALID ... size=512x423 resolution=0.05`. `nav start F1`의 선택 로직(`cat latest_map.txt` + 경로 정규식)을 노드 없이 재현 → v2, regex PASS.
6. F2 staging 오프라인 검사 PASS 후 registry에 `f2_elevator_staging_v1` 추가(`b912376d…` → `ebcfb132…`, 기존 `f2_elevator_entry` 동일성 assert).
7. 신규·수정 스크립트 배포, 로컬/원격 SHA256 일치. `map_pins.json` 생성(F1/F2 기대 해시 assert).
8. 무구동 확인: `fieldctl map guard F1/F2` PASS, 명시 v1 지도 pre-nav FAIL, map_server 없는 goal 단계 FAIL, `record check` 레코더 없음 FAIL, `pose capture`가 guard FAIL로 차단됨.
9. 오프라인 시험: 호스트 `bash scripts/run_offline_tests.sh` 변경 전 PASS 35/SKIP 6/FAIL 4 → 변경 후 PASS 37/SKIP 7/FAIL 4. FAIL 4건은 변경 전과 같은 기존 실패다. 컨테이너에서 `test_field_map_guard`, `test_analyze_nav_bag`, `test_field_pose_capture`, `test_bag_contract`, `test_p07_scripts_contract`, `test_field_scripts_contract` PASS(노드 미기동).

## 사용자 답변 반영 (같은 날 20:00 KST 기록)

- F1 idle 시드는 로봇 오른쪽 0.8 m가 적당하다는 답을 받았다. 절차 문서에 반영했고 저장 좌표는 현장 capture 값으로 한다.
- F2에서 벽에 붙은 곳은 엘리베이터를 바라볼 때 오른쪽의 움푹 들어간 벽이다. 지도 추정 위치 `(-11.05, -1.85)`, entry까지 0.749 m, staging_v1까지 1.231 m다. 그림은 `artifacts/autonomy_improvement_20260915/f2_recess_candidates.png`.
- v012 단계 gate 0.13 승인: `start_field_base.sh`(`fa21474c…` → `27ccef20…`), `field_base.launch.py`(`c7d3d844…` → `f55a19d1…`)에 `GATE_MAX_LINEAR_SPEED` 명시 override를 추가했다. 변경 전 파일은 `before/scripts`, `before/launch`에 있다. setup 함수만 호출한 결과: 비움이면 YAML만, 0.13이면 YAML 뒤에 `max_linear_speed: 0.13` 추가, 0.20은 RuntimeError. 셸 스크립트는 `GATE_MAX_LINEAR_SPEED=0.2`를 exit 2로 거부했다. 호스트 시험은 PASS 37/SKIP 7/FAIL 4로 기존과 같고, 컨테이너 `field_scripts_contract`·`field_mapping_launch`·`opencr_imu_profile`은 PASS다.
- Humble SmacPlanner2D 소스를 GitHub humble 브랜치에서 대조했다. footprint를 반지름으로 설정하고(`setFootprint(..., true, 0.0)`), 중심 셀 cost가 `INSCRIBED` 이상일 때만 충돌로 본다.

## 추가 확인한 사실

- Nav2는 `use_composition: False`로 map_server가 단독 프로세스이며 로그 파일명이 `map_server_<PID>_*.log`다. goal 단계 guard는 이것에 의존한다.
- 설정 파일은 `install/.../config/*.yaml → src/...` symlink라 src 편집이 다음 launch부터 바로 반영된다. 기본 `nav2_params.yaml`은 바꾸지 않았다.
- bridge `send_command_tick`은 좌우 목표 RPM 중 하나라도 `max_wheel_rpm`을 넘으면 비율 축소 없이 0을 쓰고 drive ready를 끈다.
- 피드백 RPM 전용 토픽은 없다. `/odom` twist에서 역산한다.
- 펌웨어 인사말: 9/13 15:36~18:43 KST `HELLO opencr 0.2-minimal` 5회(일반 계열). 9/13 21:27 KST 피드백 `F 12.137 48.548`은 당시 30 rpm 초과 명령이 수락됐다는 근거다. 9/14 세션에는 인사말이 없어 당시 탑재본은 미확인이다.
- Nav2 footprint `x∈[-0.327, 0.033]`라 계산상 inscribed radius가 0.033 m다. costmap 253(INSCRIBED) 띠가 장애물에서 0.033 m뿐이라, 중심점 기준으로 충돌을 보는 2D planner는 차체 뒤·옆이 벽에 닿는 경로도 허용할 수 있다. 벽 근접·lethal-start의 유력 요인 후보다. 계획기가 중심 셀만 본다는 점은 위 사용자 답변 절에서 소스로 확인했다. bag으로 확인하기 전까지 설정은 바꾸지 않았다.
