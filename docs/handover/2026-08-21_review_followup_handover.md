# 리뷰 후속 인수인계서 (2026-08-21)

작성: 2026-08-20 밤 ~ 08-21 새벽 자율 세션 결과. 브랜치 `lee/hw-design-review`. 전 단계 인수인계서(`2026-08-20_roundtrip_sim_handover.md`)의 "10/10" 주장을 적대적 리뷰(`docs/session_wiki/2026-08-20_adversarial_review/review_report.md`, 29건)로 재검증한 뒤, 지적을 반영하고 **최종 스택으로 다시 증명**한 상태다. 실험 전수 기록은 `docs/session_wiki/2026-08-20_review_followup/journal.md`(로컬).

## 1. 지금 증명된 것 (최종 스택 기준)

| 검증 | 결과 | 증거 |
|------|------|------|
| 데스크톱 반복 (왕복 + 동적 보행자 5 + **경로 위 정적 장애물 2** + 리프트 mock + 팔 mock + 프로파일, retries=2, MAX_MISSED_RATE=30) | **10/10 연속 PASS, 전 회차 재시도 0·REALIGN 0·응답유실 0** (무결점 1차 통과), 278~348s | `verification/repeat_20260820_161023/repeat_summary.md` |
| 같은 조건 선행 캠페인 | 4/4 PASS 후 run_05 교착(보행자 일시정지↔collision-ahead) → yield 수정 후 위 캠페인 | `verification/repeat_20260820_153454/` |
| Jetson 분산 원커맨드 (`scripts/run_distributed_e2e.sh`) | **3/3 PASS**: 각 11 goal 1차 SUCCEEDED, LIFT/ARM/world swap 전부, retries 0. **Jetson mem_used_peak 2.5GB/6.8GB(여유 4.3GB), cpu avg 48%/peak 89~94%**(6코어 시스템 전체), missed 23/25/29 | `verification/dist_20260821_020614/`, `_022227/`, `_023101/` (`_021539/_021901` 은 stale gzserver 로 [1] 실패 — pkill 자기매칭 버그, 수정 2e79e2a) |
| 바퀴 물리 | check_idle_drift **0.0000m/90s** (이전 0.5425m), 주행 후 정지 활주 0, 정지 중 자발 회전 0 | journal §2, improvement_report §1.26 |
| 오프라인 게이트 | 28/28 (행위 테스트 2종 신규, contract 확장), portability 112 files | 매 커밋 |

주의: cpu_pct·mem 은 **시스템 전체**(프로세스 미분리, 리뷰 #24) — 데스크톱 값(10GB)은 다른 프로세스 포함. Jetson 값은 Jetson 전체.

## 2. 이번에 고친 것 (요약 — 상세는 improvement_report §1.26/§1.27, 커밋 ec83780~71a05c7)

1. **바퀴 물리 근본 원인 2건**: `fdir1 "1 0 0"` 은 collision 프레임과 함께 회전해 θ≈90° 퇴화(정지 중 자발 yaw, 전복 원인) → 차축 `0 0 1`; 무마찰 캐스터 + 차축 fdir1 = 6mm/s 등속 크리프(odom 불변 활주 → belief 드리프트·REALIGN 근원) → 캐스터 mu 0.5. contract 로 고정.
2. **스모크 핫픽스**: INT/TERM rc, RUN_DIR 계약, request_switch 본문 판정+armed 앵커, cancel 후 재시도, finalize 멱등·trap·pipefail 버그, missed-rate fail-closed, profiler flush, wait_for_* timeout, realign 수치 검증, 팔 완료 층별 쌍, 실패 attempt 로그 보존, SKIP_BUILD, gzserver 종료 대기, goal 응답 유실(서버 증거 판정 + 20s 내 미실행 시 빠른 재전송), F2 출구 스테이징 goal.
3. **보행자**: init 복원(a6a5115 회귀), 근접 판정 참값(model_states), yield(4s 막히면 0.8m 후퇴·최대 3회·그래도 가까우면 퇴장), 프리즈 감지 오탐 정정.
4. **정적 장애물**: 경로 위 배치 (3.3,0)/(2.5,7.5), static=true, 템플릿 pose 오프셋(+4m) 버그 수정, 레이아웃 단위 테스트.
5. **리프트 mock**: joint_pose_trajectory 플러그인 + `frame_id=world` + lift_joint 자기잠금(friction 20), /joint_states 검증(픽업·F2 하차).
6. **Nav2**: inflation 0.30/5.0 → 0.55/3.0 (+RPP 동기) — footprint 내접 0.27 정정과 함께 가야 했던 짝(리뷰 C).
7. **반복 러너**: retries/realign/resp_lost 열, cgroup v1/v2 OOM 가드, MAX_MISSED_RATE=30, SKIP_BUILD.
8. **분산 원커맨드** `scripts/run_distributed_e2e.sh`: 스폰 확인 자동, Jetson nohup, 아티팩트 수집 (§4.5 갱신).

## 3. 운영 (원커맨드)

```bash
# 데스크톱 단발 (컨테이너)
docker exec ros2_humble bash -c "source /opt/ros/humble/setup.bash && cd /ros2_ws && WITH_RETURN=1 WITH_PEDESTRIAN=1 WITH_STATIC_OBSTACLE=1 WITH_LIFT=1 WITH_ARM=1 WITH_PROFILE=1 NAV_GOAL_RETRIES=2 MAX_MISSED_RATE=30 bash test_workspace/gazebo_world_swap/scripts/run_l3_world_swap_smoke.sh"
# 데스크톱 연속 10회 (옵션 기본값이 위와 동일)
docker exec ros2_humble bash -c "source /opt/ros/humble/setup.bash && cd /ros2_ws && REPEAT_N=10 bash test_workspace/gazebo_world_swap/scripts/run_roundtrip_repeat.sh"
# Jetson 분산 (데스크톱 호스트에서)
bash scripts/run_distributed_e2e.sh
# 바퀴 물리 회귀 (마찰/스프링/캐스터 변경 시 필수)
docker exec ros2_humble bash -c "cd /ros2_ws && bash test_workspace/gazebo_world_swap/scripts/check_idle_drift.sh"
```

Jetson 코드 동기화: 데스크톱 `git bundle create /tmp/code_space.bundle lee/hw-design-review` → `scp /tmp/code_space.bundle hsm:/home/hsm/code_space.bundle` → `ssh hsm 'cd ~/Code_Space && git pull origin lee/hw-design-review'`.

## 4. 남은 것 · 정직 공개

- **회복 스택은 여전히 상설**(retries=2, REALIGN 시뮬 오라클) — 다만 이번 14회에서는 한 번도 발화하지 않았다(무결점 1차 통과). REALIGN 의 실기 대체 수단(도킹 마커/QR)은 미설계.
- **통계**: 14/14 무결점은 95% 신뢰로 실패율 상한 ~19%(Clopper-Pearson 0/14) — "완벽"이 아니라 "최종 스택에서 재현 가능" 수준. n=29 야간 배치(<10%@95%)가 다음 목표.
- **Jetson missed_rate 23~29**(3회): 데스크톱 0~3 대비 높고 게이트 30 에 근접 — 분산 원커맨드 기본 게이트는 60(폭주 회귀만 차단)으로 두었고, 실기 임계는 실센서 주행에서 수립. cpu/mem 는 시스템 전체 지표(#24).
- **rmw_fastrtps 응답 유실 계열**(goal 응답·request_switch): 스모크가 흡수하지만 근절은 CycloneDDS 전환(양쪽 컨테이너 미설치) 검토 과제.
- **가정치**: 문폭 1.0m(실측 대기, 리뷰는 0.9 복원 권고), 택배 노즈 0.40, 캐스터 mu 0.5(시뮬 전용 근사 — 실물 볼 캐스터는 구름).
- 미해결 MED: kill_matching 과잉(#21), latest/ 무잠금(#22), cpu_pct 시스템 전체(#24), fastdds IP 하드코딩(#25), 문서 사실오류(#28: 로봇 폭 0.5382/포켓 1.48/yaml 0.9 stale).

## 5. 다음 작업 (우선순위순)

1. 고장 주입 5종 완결(Roadmap 06 잔여) — 이번 회복 스택 계측(retries/realign/resp_lost 열)으로 원인 분류 가능
2. 신공학관 실측 → 맵 재생성(문폭·footprint 가정치 대체, Roadmap 03)
3. RPLiDAR 재장착 + 실기 센서 주행 + missed-rate 실기 임계(07 잔여)
4. 회의 안건: CycloneDDS 전환, 문폭 1.0 가정치, 실기 재정위 수단, Fast DDS Discovery Server(§1.23)
