# 02. URDF 실스펙화 — 시뮬 로봇을 실제 로봇과 일치시키기

> 상태: [50%] 🔄 · 담당: Lee + Han(스펙 제공, 협의 필요) · 선행: 01, HW 스펙 확정 · 갱신: 2026-07-03
> 실제 진행: HW팀 Fusion 실측 기반 `delivery_robot.urdf.xacro` 작성·시뮬 적용(2026-06-29),
> 실물 결함 2건(접지 클리어런스, 전방 COM) HW팀 역전달. `docs/hardware_spec.md` SSOT 신설.
> 남은 것: 모터/배터리 확정, 적재 질량/COM·리프트 inertial 반영 (improvement_report §1.7).

## 목표

URDF/Gazebo 물리/Nav2 파라미터를 실제 제작 로봇 사양과 일치시켜, 시뮬 통과가 실기 통과를 예측하게 만든다. 지금 시뮬 로봇과 실물이 다르면 이후 모든 튜닝이 헛수고가 된다.

## 작업 단계

### 1단계: 스펙 수집 (Han 협의)

- [ ] Fusion 모델/실측에서 수치 확보: 본체 가로·세로·높이, 총 질량(적재 20kg 포함/미포함), 바퀴 지름·간격, 무게중심
- [ ] 구동 한계: 최대 선속도/각속도, 가속도 (모터 스펙 확정 의존 — hardware_spec.md §2)
- [ ] 센서 장착 위치: LiDAR 높이/전후 오프셋, IMU 위치, 웹캠 위치
- [ ] Z-축 리프트/로봇팔 수납 상태의 외곽 치수 (footprint에 포함할 것)

### 2단계: URDF/Gazebo 갱신

- [ ] `src/common_pkg/urdf/robot.urdf.xacro` 치수·질량·관성·바퀴 파라미터 갱신
- [ ] diff_drive 플러그인 속도/가속 한계를 실측값으로
- [ ] LiDAR 플러그인을 RPLiDAR A1m8 실사양으로: 범위 0.15~12m, 360도, 스캔 주기와 샘플 수, 가우시안 노이즈 추가
- [ ] (선택) IMU 플러그인 추가 — 실기에서 odom 융합(EKF)을 쓸 계획이면 시뮬에도 동일 토픽 구성

### 3단계: Nav2 파라미터 동기화

- [ ] `nav2_params.yaml`: `robot_radius`(또는 footprint 다각형), `inflation_radius`, controller 최대 속도/가속을 URDF와 일치
- [ ] AMCL의 LiDAR 모델 파라미터(`z_hit` 등)는 기본 유지하되 `laser_max_range`를 실사양으로

### 4단계: 회귀

- [ ] 새 URDF로 SLAM 재맵핑이 필요한지 판단 (LiDAR 높이가 바뀌면 권장)
- [ ] L5 미션 재실행 + 엘베 문(1.0m) 통과 5회 연속 성공

## 완료 기준

- URDF 수치가 HW 스펙 문서와 일치 (스펙 SSOT는 `docs/hardware_spec.md`).
- 실스펙 URDF로 L5 재통과 + 좁은 문 5회 연속 통과.

## 리스크 / 안전·보안 체크

| 리스크 | 대응 |
|--------|------|
| 모터 미정으로 속도/가속 미확정 | 보수적 추정값으로 진행하되 yaml에 `# 추정값` 주석, 확정 시 교체 |
| footprint가 커져 엘베 문 통과 불가 판정 | inflation 축소 전에 실물 치수 재확인 — 시뮬을 속이지 말 것 |
| 리프트/팔 돌출 미반영 충돌 | 수납 상태 외곽 기준 footprint + 돌출 상태는 주행 금지 인터락(08 문서) |

[08_safety_security.md](./08_safety_security.md) 게이트: 속도 상한이 Nav2와 URDF 양쪽에 동일하게 박혀 있는지 확인.
