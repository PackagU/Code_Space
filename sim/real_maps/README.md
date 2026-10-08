# 실측 지도 Gazebo 시뮬 검증

모든 판정은 시뮬레이션 검증이다. 현장 기본 지도·설정·스크립트·waypoint는 변경하지 않는다.

```bash
bash sim/real_maps/run_suite.sh
python3 sim/real_maps/summarize.py
```

AMCL 오차는 ROS message header 시각을 맞춰 지면 진실을 보간한 값으로 확인한다. 정규 조건 종료 후 native ROS Humble 환경에서 다음을 실행한다. ROS가 없으면 SKIP한다.

```bash
source /opt/ros/humble/setup.bash
python3 sim/real_maps/analyze_localization.py
python3 sim/real_maps/summarize.py
python3 sim/real_maps/write_report.py
```

기존 Docker 이미지에서 3패키지 build → 지도/월드/URDF 생성 및 정합 검사 → S1/S2/S3/S5 각 조건 3회 → bag 계측/검사 → action/지면 진실 거리/시간 기록 → field bag 분석 → 프로세스 정리까지 실행한다. Gazebo Classic 11/ROS Humble이 설치된 기존 이미지를 사용하며 패키지를 설치하지 않는다. `PACKAGU_SIM_IMAGE`로 이미지를 지정한다.

`--filter S1_`, `--repeats 1`, `--goal-timeout 900`, `--resume`를 runner 인자로 넘길 수 있다. bag은 `logs/real_map_sim/<조건>/bag`에, 작은 공유용 요약은 `sim/real_maps/results/`에 저장한다. 기존 bag을 덮어쓰지 않는다. `--resume`는 완료 결과가 있는 조건만 건너뛴다. 실행 중단/환경 오류 조건은 bag을 로컬의 별도 폴더로 보존한 뒤 재실행한다.

병렬 반복 실행은 먼저 한 번 build/generate를 끝낸 뒤 `SIM_SKIP_PREPARE=1`을 사용한다. 각 반복은 `PACKAGU_SIM_CONTAINER`, `PACKAGU_SIM_DOMAIN`을 서로 다르게 설정하고 `--filter _r1` 같은 조건 필터를 사용한다. build/generate와 같은 파일은 동시에 변경하지 않는다.

GUI를 함께 켜려면 `SIM_GUI=1`을 사용한다. 실행 중인 서버에는 `bash sim/real_maps/show_gui.sh packagu_real_map_sim 215`로 연결한다. 런처는 native Gazebo·RViz 창이 유지되도록 실행 세션을 유지한다. Gazebo master와 ROS domain은 해당 시뮬 컨테이너에 연결한다. 컨테이너로 GUI를 연결하려고 광범위한 X11 접근 권한을 추가할 필요는 없다. RViz에서 map, plan, costmap, scan 및 로봇을 확인한다. 자동 시험 중에는 GUI에서 pose/goal을 보내지 않는다.

F1 월드만 유리문이 없는 PGM을 사용한다. Nav2는 유리문을 포함한 원본 v3를 사용한다. 벽은 occupied 셀의 가로 run을 세로로 병합한 1 m 높이 box이며 F2 잡음을 보존한다. `generated/`는 재생성 가능한 로컬 산출물이다.

shim은 실제 `OpencrBridgeNode.send_command_tick`과 feedback 파서를 재사용하고 serial transport만 Gazebo 명령/odom에 연결한다. Gazebo가 odom/TF의 유일한 발행자이며 p3d가 `base_footprint`의 지면 진실 pose를 발행한다. bridge의 0.5초 watchdog, 48 rpm 초과 즉시 0/ready false, 60 rpm/s slew를 그대로 사용한다. 실제 펌웨어와 모터 응답은 검증하지 않는다.

`field_pose_capture_sim.py`는 원본 평가/출력 형식을 재사용하되 raw costmap을 구독한다. `--refresh-amcl`은 정지 상태 AMCL의 실제 갱신을 요청하는 S6 리허설 옵션이며 S3 초기 오차 재현 전에는 사용하지 않는다. 원본 field capture와 raw capture 결과를 모두 남긴다. 어느 시뮬 capture에서도 `--save`, `--replace`는 거부한다.

```bash
python3 sim/real_maps/test_geometry.py
python3 sim/real_maps/test_pose_capture_sim.py
```

두 번째 시험은 ROS 메시지가 없으면 SKIP를 출력하고 종료한다. 현장 파일 변경 방지를 위해 공용 offline runner에는 등록하지 않았다.

## 수동 배달 왕복

```bash
SIM_GUI=1 SIM_MODE=manual bash sim/real_maps/run_suite.sh --name M1_manual_roundtrip_r1
```

Nav2 → 실제 PTY WASD → Nav2 전환과 양쪽 문 위치 검증을 포함한다. 원본 F2 A/B 월드는 유지하고 수동 승하차 전용 월드만 1.0 m 개구부의 스캔 문 영역을 비운다. 자세한 명령은 `docs/sim/2026-09-15_manual_delivery_roundtrip.md`에 있다. 임의의 사용자 중단은 완주로 세지 않는다.

```bash
python3 sim/real_maps/run_operational_rehearsal.py --container packagu_real_map_sim --name sim_s6_nav_active
python3 sim/real_maps/write_report.py
```

S6는 Nav2가 활성화된 주행 구간에서 실행한다. 수동 정차 구간만 녹화하면 action status가 없어 원본 분석은 실패한다. 보정 후에도 metadata와 contract/analysis 결과를 확인한다.

원본 녹화의 hidden 토픽 누락·내부 recorder 종료 결함에 대한 시뮬 전용 보정 리허설은 다음과 같다. 원본 파일은 수정하지 않고 로컬 임시 복사본을 사용한다.

```bash
python3 sim/real_maps/record_rehearsal_sim.py --container packagu_real_map_sim --name sim_s6_fixed_review
```
