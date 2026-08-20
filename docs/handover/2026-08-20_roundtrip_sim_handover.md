# 왕복 배달 시뮬 인수인계서 (2026-08-20)

작성: 2026-08-17 밤 ~ 08-18 새벽 야간 자율 세션 결과 기준. 브랜치 `lee/hw-design-review`, HEAD `3fa76ad` (push 완료).
후속 작업자(사람 또는 AI 어시스턴트)가 이 문서만 읽고 현재 상태를 파악하고 이어갈 수 있도록 작성했다.

## 1. 무엇이 완성됐는가

한 층 왕복 배달 체인이 시뮬레이션에서 완성·검증됐다. 회의 결정(F3 확장 대신 로비↔F2 한 층 완성도 집중)을 구현한 것이다.

체인: `충전소 대기 → 택배함 픽업 → 로비 → 엘베 탑승 → F2 전환(맵+월드+팔 mock) → F2 배달 → 엘베 → F2→F1 역전환 → 로비 → 충전소 복귀`

| 검증 | 결과 | 증거 위치 |
|------|------|----------|
| 데스크톱 단독 연속 반복 | 10/10 PASS (OOM 0, 메모리 8.6~8.8GB 안정) | `test_workspace/gazebo_world_swap/verification/repeat_20260817_133709/repeat_summary.md` |
| Jetson 분산 (실전 스택) | 3/3 완주 (cpu 피크 67~87%/600%, 메모리 2.4~2.8GB) | Jetson `verification/run_20260817_160124`, `_174352`, `_181855` |
| 보행자 동적 장애물 | 전 회차 왕복 전 구간 활성 (F1 3명 + F2 2명 transit) | 각 run 디렉터리 `pedestrians.log` |
| 오프라인 게이트 | 26/26 + portability 105파일 + smoke contract | 매 커밋 실행 |

## 2. 핵심 산출물 (파일 지도)

| 파일 | 역할 |
|------|------|
| `test_workspace/gazebo_world_swap/scripts/run_l3_world_swap_smoke.sh` | E2E 스모크 본체. `WITH_RETURN=1`이 왕복 게이트 (기본 0 — 기존 결정적 스모크 보존) |
| `test_workspace/gazebo_world_swap/scripts/run_roundtrip_repeat.sh` | 연속 N회 판정기 (`REPEAT_N`, 기본 10) + OOM 가드 + 회차 집계 |
| `test_workspace/gazebo_world_swap/scripts/test_smoke_scripts_contract.py` | 회귀 가드 (옵션 기본값·왕복 마커·헬퍼 사용 강제) |
| `test_workspace/gazebo_world_swap/pedestrian/pedestrians.py` | 보행자 transit 모델 + 로봇 근접 0.7m 일시정지 + 프리즈 감지 |
| `src/common_pkg/urdf/delivery_robot.urdf.xacro` | 바퀴 마찰 방향 분리 mu1=100/mu2=1.0 (근거 주석 포함) |
| `scripts/generate_kku_worlds.py` + `src/common_pkg/config/kku_pre_simulation_map.yaml` | 택배존 문폭 1.0m (실측 전 가정치) |
| `docker/compose/docker-compose.jetson.yml` | 장치 매핑 env화 — 기본 `/dev/null`, 장착 시 `.env`에 실경로 |
| `docs/deployment/01_portability_policy.md` §4.5 | 분산 실행 절차 SSOT (왕복 원커맨드, retries=2, 성공 판정 기대값) |
| `docs/deployment/03_hw_update_checklist.md` | HW 재장착/확정 시 바꿀 파일·검증 명령 목록 |
| `docs/improvement_report.md` §1.23~1.25 | 결함 이력·근거·상태 |
| `docs/session_wiki/2026-08-18_overnight_roundtrip/briefing.md` | 세션 상세 브리핑 (로컬 전용) |

## 3. 스모크 내부 동작 (후속 디버깅에 필요한 수준만)

1. 매 런: stale 프로세스 정리 → `ros2 daemon stop`(오염 daemon 리셋) → colcon 빌드 → Gazebo/Nav2 기동(분산이면 `GAZEBO_REMOTE=1`로 Gazebo 생략)
2. F1 미션 7 goal → 픽업 후 도킹 재정위(initialpose 재발행) → footprint 노즈 0.35m 확장
3. F2 전환: `set_orchestrator_target_floor`(timeout 30s ×3) → `request_floor_switch`(응답 유실 시 orchestrator 로그의 armed 증거로 성공 판정) → ARRIVED_OPEN 발행 → verify → `align_robot_to_spawn`(텔레포트+costmap 클리어, §1.24 시뮬 부기)
4. 배달 후 `WITH_RETURN=1`이면 같은 기계로 F2→F1 역전환 (역방향은 orchestrator/world-swap이 방향 무관이라 신규 코드 최소)
5. goal 실패 시 회복 스택: `dump_world_state`(전 모델 참값+belief 스냅샷) → `realign_belief_if_drifted`(참값 대비 >0.3m면 initialpose 재발행 — 시뮬 오라클) → costmap 클리어 → 재시도(`NAV_GOAL_RETRIES`)

## 4. 이번에 격파한 산발 결함 6종 (재발 시 참조)

| # | 증상 | 확정 원인 | 수정 | 근거 |
|---|------|----------|------|------|
| 1 | `/clock` 대기 멈춤 | daemon rclpy 오염 (`!rclpy.ok()` fault) | 매 런 daemon stop | §1.23f |
| 2 | 팔 mock 노드 즉사 | 빈 `-p serial_port:=` rcl 파싱 에러 | 값 있을 때만 전달 | 커밋 7d8126e 이전 |
| 3 | request_switch 45s timeout | 서버 armed 완료 후 응답만 유실 (rmw) | armed 로그 증거 판정 헬퍼 | §1.23e |
| 4 | 문에서 450s collision-ahead | 0.9m 문에 로봇(0.52m) 물리 wedging | 문폭 1.0m | §1.25g, 스냅샷 좌표 |
| 5 | 로봇 90° 전복 (z=0.17~0.23) | 보행자 텔레포트 관통 충격량 + mu100 그립 | 근접 일시정지 + mu1/mu2 분리 | §1.25g, 쿼터니언 실측 |
| 6 | 엘베 진입 patience 초과 | footprint 노즈 0.40이 포켓 inflation과 간섭 | 노즈 0.35 | §1.25i |

## 5. 알려진 한계 · 기술 부채 (정직 공개)

- **시뮬 오라클 의존 2건**: `align_robot_to_spawn`(전환 후 텔레포트)과 `realign_belief_if_drifted`(참값 재정위)는 시뮬 전용 부기다. 실물 엘베는 물리 연속이라 전자는 불필요하지만, 후자에 해당하는 실기 재정위 수단(도킹 마커/QR 등)은 미설계.
- **mu2=1.0은 시뮬 물리값** — 실기 바퀴와 무관. 실기에서 전복은 별도 문제(§1.15 COM 이슈 참조).
- **문폭 1.0m·footprint 0.35m는 실측 전 가정치** — 신공학관 실측(03)에서 좁게 나오면 wedging·간섭 문제가 되돌아온다. 그 경우 Nav2 파라미터(inflation, RPP) 튜닝이 정공법.
- **분산 3회는 작은 표본** — 3회 중 첫 시도 기준 1회만 무결점, 2회는 수정 후 재실행으로 달성. 반복 신뢰도는 데스크톱 10/10이 근거.
- **`MAX_MISSED_RATE` 게이트 미설정** — missed 2~19 관측만 기록 중. 실기 임계 수립 과제.
- **WITH_F3 경로는 이번 수정들 이후 실행 검증 안 됨** (옵션 유지, contract 가드만).
- **초기 커밋 히스토리에 검증 로그 잔존** — `git add -A` 실수로 반복 런 로그가 몇 커밋에 포함됨(추적 해제 완료, 민감정보 없음, 히스토리 재작성은 안 함).
- **repeat 러너의 cpu_pct 지표 신뢰도 낮음** (데스크톱 회차 4.6~8.2%로 단독 런 87%와 불일치 — 프로파일러 샘플링 구간 문제 의심, 게이트엔 미사용).

## 6. 운영 방법 (원커맨드)

```bash
# 데스크톱 단독 왕복 1회 (컨테이너)
docker exec ros2_humble bash -c "source /opt/ros/humble/setup.bash && cd /ros2_ws && WITH_RETURN=1 WITH_ARM=1 WITH_PEDESTRIAN=1 WITH_PROFILE=1 bash test_workspace/gazebo_world_swap/scripts/run_l3_world_swap_smoke.sh"

# 연속 10회 판정
docker exec ros2_humble bash -c "source /opt/ros/humble/setup.bash && cd /ros2_ws && REPEAT_N=10 bash test_workspace/gazebo_world_swap/scripts/run_roundtrip_repeat.sh"

# 분산 (Jetson): docs/deployment/01_portability_policy.md §4.5 순서 엄수
# Jetson 접근: ssh hsm (10.42.0.250 유선), repo는 ~/Code_Space (데스크톱과 경로 다름!)
```

주의: Jetson 컨테이너 recreate 시 `ros-humble-gazebo-msgs` apt 재설치 + colcon 재빌드 필요 (이미지 미포함). 라이다·팔·HDMI는 현재 Jetson에서 제거된 상태 — 재장착 절차는 `03_hw_update_checklist.md`.

## 7. 다음 작업 (우선순위순)

1. 신공학관 실측 → 맵 재생성 — 문폭·footprint 가정치를 실측값으로 대체 (Roadmap 03)
2. RPLiDAR 재장착 + 실기 센서 주행 + `MAX_MISSED_RATE` 실기 임계 수립 (07 잔여)
3. 고장 주입 5종 완결 (06 잔여 10%)
4. 회의 안건: Fast DDS Discovery Server 전환(§1.23), 정렬/재정위 로직의 `gazebo_world_swap_pkg` 이동, mu2 시뮬 전용 값 공유, 실기 재정위 수단 설계
