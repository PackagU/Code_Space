# Scripts

작성일: 2026-06-02 · 갱신: 2026-06-12 (구현 완료)

## 현재 파일

| 파일 | 검증 레벨 | 역할 |
|------|----------|------|
| `test_floor_map_registry.py` | L0 | floor_maps.yaml 파싱 + 3단계 경로 resolve 테스트 |
| `test_auto_switch_state_machine.py` | L1 | 자동 전환 상태머신 dry-run 테스트 (실패 시 pending 유지 포함) |
| `test_switch_floor_compatibility.py` | L4 (오프라인) | legacy SwitchFloor 게이트 호환 + legacy 소스 고정 검사 |
| `test_orchestrator_ros_smoke.py` | L2 | 인프로세스 ROS smoke — legacy와 같은 호출 순서로 ack 없이 ready 확인 |
| `probe_nav2_services.sh` | L3 사전 점검 | 실제 Nav2 load/clear 서비스명과 타입 출력 |

계획 단계 후보였던 `test_switch_floor_behavior.py`는 `test_switch_floor_compatibility.py`로, `probe_nav2_map_services.py`는 `probe_nav2_services.sh`로 이름이 정리됐다. 실행 명령과 결과는 `../docs/04_verification_plan.md`의 검증 결과 절과 `../docs/completion_report.md` 참조.
