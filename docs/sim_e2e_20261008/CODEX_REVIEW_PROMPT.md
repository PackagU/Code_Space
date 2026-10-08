# Codex 검토 프롬프트 (2026-10-08)

[DESKTOP_SIM_PROMPT.md](DESKTOP_SIM_PROMPT.md)로 Claude Code가 한 단계(1~7)를 끝낼 때마다 Codex에게 읽기 전용 검토를 맡긴다. 아래 `---` 사이를 붙여 넣고 `<단계>`와 `<커밋 범위>`만 바꾼다. 같은 checkout에서 Claude Code가 파일을 고치는 중이면 검토를 시작하지 않는다.

---

`lee/sim-e2e-20261008` 브랜치 checkout에서 **읽기 전용**으로 검토해라. 파일을 고치지 말고, 빌드·시뮬 실행·git 상태 변경(commit, checkout, stash, reset, push)도 하지 마라. 검토 대상은 `<단계>`이고 범위는 `git log --oneline <커밋 범위>`와 `git diff <커밋 범위>`다. 기준 문서는 `docs/sim_e2e_20261008/DESKTOP_SIM_PROMPT.md`, `docs/sim_e2e_20261008/README.md`, `docs/sim_e2e_20261008/reference/final_demo/08_desktop_gazebo_selftest_plan.md`, `09_realistic_sim_world_plan.md`, 루트 `AGENTS.md`다.

아래 항목을 확인하고, 문제마다 `파일:줄`, 근거(직접 읽은 코드·출력), 영향, 수정 제안을 표로 답해라. 확인하지 못한 것은 "확인 못 함"으로 적어라. 추측으로 결함을 만들지 마라.

1. **실차 격리**: 시뮬 시작 스크립트가 `ROS_LOCALHOST_ONLY=1`·0이 아닌 `ROS_DOMAIN_ID` 또는 격리 네트워크 없이 뜰 수 있는가. `fastdds_lan_peers.xml`·`run_sim_host.sh`·하드웨어 장치 전달을 쓰는가.
2. **현장 파일 불변**: `git diff`에 `nav2_params.yaml`, `nav_safety.yaml`, `drive_calib.yaml`, `fieldctl`, `start_field_*.sh`, `map_pins.json`, `waypoints.json`, `latest_map.txt`, 이성덕 미션 코드 변경이 있는가.
3. **지도 정합**: 월드 생성기가 지도 origin·resolution·y축 방향을 맞추는가. 정합 검사가 실제로 실패를 잡는가(0.05 m 기준). F1 noglass 차이가 유리문 선분에만 있는가.
4. **현장과 같은 조건인가**: P0가 `nav2_params_pre_wallpush_20260915.yaml`이고 P1이 `nav2_params.yaml`인가. params·지도가 `src/...` 절대경로인가. footprint·바퀴 반경·윤거·LiDAR 프레임·주기·점 수가 문서 값과 같은가. shim이 0.5 s timeout, 48 rpm 초과 시 비율 축소 없이 0 + ready false, 60 rpm/s slew를 지키는가. 실제 `nav_safety_gate`를 거치는가. 지면 진실이 Nav2 입력으로 새지 않는가.
5. **판정의 정직성**: 결과 표가 실행하지 않은 조건을 성공으로 세지 않는가. RTF를 기록하는가. 임시 치수(`아직 모름`·`[제안값]`)로 만든 결과에 표시가 있는가. "시뮬레이션 검증"을 실물 검증처럼 쓰지 않는가.
6. **E2E 러너**: goal 결과를 action 최종 상태(SUCCEEDED)로 판정하는가. 제한 시간·실패 처리가 있는가. 층 이동 뒤 지도 전환·`/initialpose` 반영·AMCL 수렴을 확인하는가. 지점 좌표를 몰래 바꾸지 않았는가.
7. **재현성**: `sim/real_maps/README.md`만 보고 새 사람이 다시 돌릴 수 있는가. 하드코딩된 개인 경로·사용자명·IP가 있는가.
8. **커밋 위생**: CR 문자, 100 MB 초과 파일, bag·영상 커밋, 비밀 패턴(`ghp_`, `github_pat_`, `BEGIN .* PRIVATE KEY`, `password`).

마지막에 "다음 단계로 넘어가도 되는가"를 예/아니오와 막는 이유 목록으로 답해라.

---
