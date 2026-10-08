# 최종 시연(완전 자율) 계획 묶음

작성일: 2026-10-07. 목표: 출발부터 복귀까지 사람 조작 없이 동작하는 시연. 현재 판정: **계획만**(복도 Nav2 일부 구간만 실물 검증).

## 읽는 순서

| 순서 | 문서 | 내용 |
|---|---|---|
| 1 | [01_scenario_and_architecture.md](01_scenario_and_architecture.md) | 목표 정의, 4개 정지 지점 + 복귀 단계(S0~S12, R1~R8), 시스템 구조, 필요한 지점 목록, 사전 준비 허용 범위 |
| 2 | [02_code_review_2026-10-06.md](02_code_review_2026-10-06.md) | 이성덕 `2026-10-06` 브랜치(`79fdcb2`) 검토, 지적 CR-01~CR-31(CR-22 이후는 Codex 1회차 추가), 오프라인 시험 결과 |
| 3 | [03_gap_and_work_breakdown.md](03_gap_and_work_breakdown.md) | 단계별 격차, 작업 W01~W16, 마일스톤 M0~M5, 결정 안건 D1~D12, 진행 순서 |
| 4 | [04_failure_modes.md](04_failure_modes.md) | 단계별 예상 문제와 대응(★ = 실제 발생 기록, ◆ = 코드·문서로 확인된 결함), 우선 확인 10개 |
| 5 | [05_weekend_field_test_plan.md](05_weekend_field_test_plan.md) | 이번 주말 현장 시험 T0~T9, 기록 양식 |
| 6 | [06_review_log.md](06_review_log.md) | 자체 피드백과 Codex 검사 5회 기록 |
| 7 | [07_jetson_ssh_readonly_check_2026-10-08.md](07_jetson_ssh_readonly_check_2026-10-08.md) | 10/8 Jetson SSH 읽기 전용 점검 절차, 기준 해시표, 수집 스크립트 [tools/jetson_readonly_snapshot.sh](tools/jetson_readonly_snapshot.sh) |
| 8 | [08_desktop_gazebo_selftest_plan.md](08_desktop_gazebo_selftest_plan.md) | 토요일 주행 시험 전 192 데스크톱 Gazebo 자체 시험(G1~G7) |
| 9 | [09_realistic_sim_world_plan.md](09_realistic_sim_world_plan.md) | 현실적인 시뮬 월드 계획(L1~L6, SW1~SW9) |
| 10 | [10_unknowns_register.md](10_unknowns_register.md) | 미정·아직 모름 목록(U-J·U-D·U-H·U-E·U-X). SSH·현장 확인 뒤 여기부터 갱신 |

## 한 장 요약

- **이성덕 커밋이 해결한 것**: 팔 카메라 자세 2개(`front_view`·`floor_view`), 캐빈 안 목표층 새 프레임 확인 → 지도 전환 → 정면 복귀 → 하차 로직, Nav2 ABORT 오판 수정. 로컬 오프라인 시험 13/13 통과.
- **아직 안 되는 것**: 택배 위치 인식·상하차(mock), 카메라로 버튼 찾아 누르기(고정 자세), 승강장 "1층 + 문 열림" 대기(없음), 캐빈 안 회전, 문 열림 기반 하차, 복귀 전 구간, 실측 좌표 연결.
- **지금 코드로 실차 카메라 미션을 돌리면 안 되는 이유**: 호출 직후 바로 탑승 주행(CR-01), 기본 설정이 가상 지도·가상 좌표(CR-03), 지도 전환 뒤 위치 확인 없음(CR-24), 팔 안전 범위 미측정(CR-19).
- **이번 주말에 할 것**: 팔 카메라 자세 실물 확인, 엘리베이터·버튼·표시창·문턱 측정, 층수 인식 실측, Nav2 실패 구간 반복, 지게팔 하중 시험.
- **가장 큰 위험**: 문턱 탑승 실패(9/15 기록), 문 열림 유지 시간 대비 느린 진입, 실제 버튼이 인식기 전제와 다를 가능성, 비상 버튼 오누름.

## 관련 문서

- 남은 작업 목록: [TODO.md](../../TODO.md)
- 마일스톤: [plan/roadmap.md](../roadmap.md)
- 시연 시나리오 요약: [plan/integration.md](../integration.md)
- 현장 명령: [RUNBOOK.md](../../docs/bringup_guide/RUNBOOK.md), [9/16 수동 왕복 가이드](../../docs/bringup_guide/2026-09-15_manual_delivery_roundtrip.md)

## 근거와 한계

- 코드 근거는 GitHub `PackagU/Code_Space` 브랜치 `2026-10-06`(`79fdcb215afc`)을 읽기 전용으로 받아 확인했다.
- Jetson은 2026-10-07 두 주소 모두 SSH 응답이 없었다. Jetson의 현재 작업 트리·설정은 ⚠️미확인이다.
- 실물 동작은 이번 작업에서 아무것도 실행하지 않았다.
- GitHub의 `Roadmap/`·`TODO.md`는 바꾸지 않았다. 이 묶음은 PC 로컬 저장소에만 있다.
