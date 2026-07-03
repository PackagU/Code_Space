# 01. PoC 마무리 — 자동 맵 전환 L3/L5 + 좌표 실측 캡처

> 상태: [80%] 🔄 · 담당: Lee · 선행: 없음 · 갱신: 2026-07-03
> 실제 진행: L3(맵+Gazebo 건물 동시 전환)는 world-swap smoke로 달성 — F1 충전소 출발 →
> 택배존 → 엘베 복귀 → F2 전환 → F2 복도 goal까지 8/8 SUCCEEDED
> (`test_workspace/gazebo_world_swap/scripts/run_l3_world_swap_smoke.sh`).
> 남은 것: 방 waypoint 좌표 실측 캡처/재검토 (improvement_report §1.1 — 208 goal ABORTED).

## 목표

dry-run까지 검증된 자동 맵 전환 PoC를 실제 Gazebo/Nav2 위에서 완주시키고, 설계 seed 좌표를 실측(AMCL 캡처) 좌표로 교체해 이후 모든 단계의 기반을 만든다.

## 작업 단계

### 1단계: L3 — 실제 Nav2 map load

- [ ] Gazebo F1 + Nav2 F1 실행 후 `bash test_workspace/elevator_auto_map_switch/scripts/probe_nav2_services.sh`로 load/clear 서비스명 확인
- [ ] `dry_run_map_load:=false`로 orchestrator 실행, 수동 elevator state 주입으로 맵 교체 확인

```bash
ros2 launch auto_floor_orchestrator_pkg auto_floor_orchestrator.launch.py \
  dry_run_map_load:=false target_floor:=F2 use_sim_time:=true
```

- [ ] RViz `/map`이 F2로 바뀌고 status에 `map_loaded=true` 확인

### 2단계: 좌표 실측 캡처

- [ ] 층별로 Nav2 실행 후 로봇을 각 지점에 teleop로 위치시키고 `capture_nav_point.py`로 캡처: `elevator_entry`, `elevator_inside`, `elevator_exit`, `parcel_pickup`(F1), 대표 방 2곳(F2/F3)
- [ ] `kku_nav_points.yaml`과 `floor_maps.yaml`의 points를 캡처 값으로 갱신
- [ ] 회귀 재실행: legacy 5종 + 신규 4종 전부 PASS

### 3단계: L5 — 무수정 legacy mission E2E

- [ ] legacy elevator_sim + 자동 orchestrator + legacy `delivery_mission_node` (`parcel_to_208`) 조합으로 실행 (터미널 절차: `test_workspace/elevator_auto_map_switch/docs/completion_report.md` 5절)
- [ ] ack 0회로 `MISSION COMPLETE` 로그 도달 확인
- [ ] 성공 화면 캡처를 Notion `2026-06-02 Elevator Auto Map Switch Planning` 페이지 상단 이미지 칸에 첨부

## 완료 기준

- L5 미션 1회 완주 (ack 0회).
- 좌표가 전부 캡처 값으로 교체되고 회귀 9종 PASS.

## 리스크 / 안전·보안 체크

| 리스크 | 대응 |
|--------|------|
| load_map 서비스명이 예상과 다름 | probe 스크립트 결과로 `load_map_service` 파라미터 교체 |
| 맵 전환 직후 경로가 이전 층 장애물에 걸림 | costmap clear 동작 로그 확인, 실패 시 `costmap_clear_services` 점검 |
| Relocalize(elevator_exit)로 인한 AMCL 떨림 | E2E에서 위치 추정 떨림 관찰 시 `FeedBack/05` 문서의 publish_initialpose 논의 참조 |

[08_safety_security.md](./08_safety_security.md) 게이트: 시뮬 전용 단계 — "실패 시 정지" 동작(load 실패 → mission 정지) 1회 고장 주입으로 확인.
