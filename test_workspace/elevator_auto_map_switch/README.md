# Elevator Auto Map Switch Test Workspace

작성일: 2026-06-02

## 목적

이 폴더는 엘리베이터가 목표층에 도착했을 때 Nav2 저장 맵을 자동으로 목표층 맵으로 전환하는 PoC를 준비하기 위한 독립 테스트 워크스페이스다.

기존 수동 엘리베이터 조작 PoC인 `test_workspace/elevator_mission/`은 legacy로 보존한다. 이 폴더의 작업은 기존 수동 PoC 코드를 수정하거나 덮어쓰지 않는다.

## 현재 범위

- 오늘(2026-06-02)은 코드 구현을 하지 않는다.
- 현재까지의 구조와 병목을 분석한다.
- 다음 구현자가 바로 시작할 수 있도록 시행 계획, 구현 계획, 검증 계획을 작성한다.
- 2026-06-01과 2026-06-02 작업 일지를 남긴다.

## 폴더 구조

```text
test_workspace/elevator_auto_map_switch/
├── README.md
├── config/
│   └── README.md
├── docs/
│   ├── 01_current_analysis.md
│   ├── 02_execution_plan.md
│   ├── 03_implementation_plan.md
│   ├── 04_verification_plan.md
│   ├── 05_journal_2026_06_01.md
│   └── 06_journal_2026_06_02.md
├── scripts/
│   └── README.md
└── src/
    └── README.md
```

## 핵심 결정

| 항목 | 결정 |
|------|------|
| 기존 수동 PoC | `test_workspace/elevator_mission/`을 legacy로 보존 |
| 신규 PoC 위치 | `test_workspace/elevator_auto_map_switch/` |
| 오늘 작업 범위 | 문서와 계획만 작성 |
| 자동 전환 트리거 | `/elevator/state`의 목표층 도착 및 문 열림 이벤트 |
| 맵 전환 방식 | Nav2 map server의 `load_map` 서비스 사용을 1순위로 검토 |
| 위치 초기화 | 맵 전환 뒤 `/initialpose`를 목표층 `elevator_exit` 기준으로 발행 |

## 읽는 순서

1. `docs/01_current_analysis.md`
2. `docs/02_execution_plan.md`
3. `docs/03_implementation_plan.md`
4. `docs/04_verification_plan.md`
5. `docs/05_journal_2026_06_01.md`
6. `docs/06_journal_2026_06_02.md`
