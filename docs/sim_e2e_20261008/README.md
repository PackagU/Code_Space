# 실측 지도 End-to-End 시뮬레이션 작업 브랜치 (2026-10-08)

브랜치: `lee/sim-e2e-20261008`. 목적: 192 데스크톱(리눅스)에서 Claude Code·Codex가 **우리가 만든 현장 지도(F1·F2)**로 Gazebo 시뮬레이션 환경을 만들고, 그 안에서 왕복 배달 흐름을 시험할 수 있게 한다. 이 브랜치 하나만 받으면 시작할 수 있게 코드·지도·설정·참고 문서를 모았다.

**데스크톱 세션은 [DESKTOP_SIM_PROMPT.md](DESKTOP_SIM_PROMPT.md)의 `---` 사이를 첫 메시지로 붙여 넣어 시작한다.** Codex 검토는 [CODEX_REVIEW_PROMPT.md](CODEX_REVIEW_PROMPT.md)를 쓴다.

표기: `아직 모름` = 사실 미확인, `미정` = 결정 안 됨, `[측정값]`·`[제안값]`. 판정 단계는 계획만 / 코드 존재 / 오프라인 검증 / 시뮬레이션 검증 / 실물 검증 / 미확인. 시뮬 결과는 실물 검증이 아니다.

## 1. 브랜치 구성

| 커밋 | 내용 |
|---|---|
| `43872e8` (`2026-10-06` 머리) | 이성덕 팔 카메라 자세·목표층 확인 미션(`79fdcb2`~`43872e8`). 부모 `9c5ee7e`(9/16 발표 정리본) = Jetson 현장 코드 |
| 병합 `lee/sim-real-maps`(`44e9fc8`) | `docs/field_review_20260915/`(F1 v3 생성기 `build_v3.py`·`geometry_v3.json`, 9/15 현장 검토, 옛 리눅스 지시문) |
| 이 문서 묶음 | `docs/sim_e2e_20261008/` |

`2026-10-06`·`lee/sim-real-maps`·`lee/jetson-live`·`main`·`dev`는 건드리지 않았다.

## 2. Jetson 현재 상태 (2026-10-08 20:13 KST 읽기 전용 점검)

원본 출력: [jetson_snapshot_20261008.txt](jetson_snapshot_20261008.txt). 구동·빌드·컨테이너 조작은 하지 않았다.

- Jetson `~/Code_Space` 작업 트리 = `9c5ee7e` 코드(gitignore·백업·지도 바이너리 제외 350개 파일 blob 대조, README·TODO만 다름). 이 브랜치에 그대로 들어 있다.
- GitHub 미러 `lee/jetson-live`를 `6026e79`로 갱신했다(9/15 이후 바뀐 `start_field_base.sh`·`waypoints.json` 반영).
- **Jetson `nav2_params.yaml` = `80787840cd76`(wall_push_v1).** 9/17 사용자 철회 결정은 Jetson에 아직 적용되지 않았다. 철회 기준 파일은 `src/slam_pkg/config/nav2_params_pre_wallpush_20260915.yaml`(`e8cf213b33fb`).
- 이성덕 코드는 Jetson의 별도 폴더 `~/Code_Space-2026-10-06`(`43872e8` checkout)에 있다. 현장에서 바뀐 것은 층수 인식기 ROI 하나다: [jetson_data/floor_reader_config_Code_Space-2026-10-06_20261007.json](jetson_data/floor_reader_config_Code_Space-2026-10-06_20261007.json). 버튼 앱·옛 층수 인식기 데이터는 브랜치의 `tools/*/data`와 같다(blob 대조).
- `ros2_humble` 컨테이너는 재부팅 뒤 꺼져 있다. 장치는 OpenCR만 연결, LiDAR·팔은 미연결.

핵심 파일 SHA256 앞 12자리(리눅스 checkout 기준, Jetson과 이 브랜치가 같음):

| 파일 | 해시 |
|---|---|
| `src/slam_pkg/config/nav2_params.yaml` | `80787840cd76` (wall_push) |
| `src/slam_pkg/config/nav2_params_pre_wallpush_20260915.yaml` | `e8cf213b33fb` |
| `src/drive_pkg/config/nav_safety.yaml` | `f2478064f359` |
| `src/drive_pkg/config/drive_calib.yaml` | `4248deb19979` |
| `src/slam_pkg/maps/field/waypoints.json` | `9694225dca40` (지점 13개) |
| `src/slam_pkg/maps/field/map_pins.json` | `eb5aaa2976b5` |
| `src/slam_pkg/maps/field/f1/f1_manual_clean_v3.yaml`·`.pgm` | `d701d2b6df57`·`d525759b07a8` |
| `src/slam_pkg/maps/field/f2/f2_nav_unknown_v1.yaml`, `f2_raw_20260914.pgm` | `b670cd2b8790`, `f89bfeb9f928` |

## 3. 들어 있는 것

| 위치 | 내용 |
|---|---|
| `src/slam_pkg/maps/field/` | 현장 지도 F1(v1~v3, raw)·F2(raw, nav_unknown)·F3, `waypoints.json`, `map_pins.json`. posegraph·data는 없다 |
| `src/slam_pkg/config/` | Nav2 기본(wall_push), 철회 기준(pre_wallpush), wall_push_v1, 증속 후보 2개 |
| `src/drive_pkg/` | 안전 게이트 `nav_safety_gate`, OpenCR bridge(`send_command_tick` 재현용) |
| `src/common_pkg/` | URDF `delivery_robot.urdf.xacro`, 가상 KKU 월드, `gazebo.launch.py` |
| `test_workspace/gazebo_world_swap/` | 층 이동 월드 교체, 보행자·정적 장애물(가상 KKU 기준) |
| `test_workspace/elevator_mission/` 외 | 미션 노드, 이성덕 카메라 미션, 자동 지도 전환 |
| `scripts/` | `fieldctl`, `field_map_guard.py`, `record_field_bag.sh`, `analyze_nav_bag.py`, `field_pose_capture.py`, `run_offline_tests.sh` 등 |
| `docker/compose/docker-compose.linux.yml` | 데스크톱 컨테이너(`ghcr.io/packagu/ros2-humble-slam:humble`, 이름 `ros2_humble`) |
| `docs/presentation_20260916/` | 9/16 현장 goal 결과·로그 근거 |
| `docs/field_review_20260915/` | F1 v3 생성기·기하, 9/15 현장 검토 |
| [reference/](reference/) | PC 저장소(`PackagU/2026_graduation_project`)의 계획 문서 사본. 아래 4절 |
| [jetson_snapshot_20261008.txt](jetson_snapshot_20261008.txt) | 10/8 Jetson 점검 원본 |

## 4. 참고 문서 (reference/)

PC 저장소에서 2026-10-08에 복사했다. 문서 안의 상대 링크는 PC 저장소 기준이라 여기서는 열리지 않을 수 있다.

| 파일 | 내용 |
|---|---|
| [2026-09-15_real_map_gazebo_report.md](reference/2026-09-15_real_map_gazebo_report.md) | 9/15 실측 지도 Gazebo 결과(7/33 조건 종료, 정합·shim·운용 스크립트 결함) |
| [2026-09-15_linux_desktop_sim_prompt.md](reference/2026-09-15_linux_desktop_sim_prompt.md) | 9/15 지시문(9/17 wall_push 철회 반영판). 브랜치의 `docs/field_review_20260915/LINUX_DESKTOP_SIM_PROMPT.md`는 철회 전 판이다 |
| [2026-09-15_manual_delivery_roundtrip.md](reference/2026-09-15_manual_delivery_roundtrip.md) | 9/16 수동 왕복 현장 절차, 부록 B PC 시뮬 사용법 |
| [final_demo/](reference/final_demo/README.md) | 최종 시연 계획: 시나리오(01), 이성덕 코드 검토 CR-01~31(02), 작업 목록(03), 단계별 예상 문제(04), 주말 시험(05), Jetson 점검(07), **데스크톱 Gazebo 시험(08)**, **현실적인 월드(09)**, **미정·아직 모름 목록(10)**. 06 검토 기록·Codex 원문은 넣지 않았다 |
| [TODO_pc_20261008.md](reference/TODO_pc_20261008.md) | PC 저장소 TODO |

## 5. 없는 것

- **9/15 데스크톱 시뮬 코드 `sim/real_maps/`**: GitHub 어디에도 없다. 데스크톱 로컬 checkout에만 있었다. 지금도 남아 있는지 아직 모름. 프롬프트 1단계에서 찾아 이 브랜치에 올린다.
- 현장 bag: Jetson `~/Code_Space/logs/field_bags/manual_roundtrip_test_f2_20260916_034617`(384 MB). git에 올리지 않았다. scan 비교가 필요하면 사용자가 따로 옮긴다.
- 지도 직렬화 파일(posegraph·data): 시뮬에는 필요 없다.

## 6. 규칙

- 현장 기본 파일(`nav2_params.yaml`, `nav_safety.yaml`, `drive_calib.yaml`, `fieldctl`, `start_field_*.sh`, `map_pins.json`, `latest_map.txt`, `waypoints.json`)은 고치지 않는다. 시뮬 전용 사본·프로필로 분리한다.
- 이 브랜치에만 커밋·push한다. force push·PR 생성·다른 브랜치 merge는 하지 않는다. Jetson에 반영하지 않는다.
- `.env`·토큰·키·인증 파일을 읽거나 커밋하지 않는다. 큰 bag·영상은 커밋하지 않는다.
- 팀원 표기는 이준형·한수민·이성덕.
