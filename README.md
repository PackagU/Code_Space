# PackagU — 엘리베이터 무인 배달 로봇 (종설 6조)

엘리베이터로 층간 이동까지 하는 실내 택배 배달 자율주행 로봇.
ROS2 Humble + SLAM Toolbox + Nav2, 실기는 Jetson Xavier NX(컨테이너 구동).

| 담당 | 역할 |
|------|------|
| 이준형 (JunhyungLee25) | SLAM · Nav2 · 시뮬레이션 |
| 한수민 (inonewater) | Fusion 모델링 · 구동부 · 리프트 |
| 이성덕 (DuckFrog123) | 4 DOF 로봇팔 · 비주얼 서보잉 |

## 퀵스타트 (Linux 데스크톱)

```bash
git clone https://github.com/PackagU/Code_Space.git && cd Code_Space
bash scripts/bootstrap_workspace.sh        # 월드/맵 생성 (최초 1회)
./scripts/run_kku_sim.sh F1                # 컨테이너 + Gazebo + SLAM + RViz
```

검증 원커맨드:

```bash
bash scripts/run_offline_tests.sh          # 오프라인 테스트 스위트 (host/CI)
# 컨테이너 안 E2E (F1 미션 → F2 층 전환 → 검증):
bash test_workspace/gazebo_world_swap/scripts/run_l3_world_swap_smoke.sh
```

## 저장소 안내

| 위치 | 내용 |
|------|------|
| [AGENTS.md](AGENTS.md) | 팀·AI 어시스턴트 공통 규칙 (필독) |
| [Roadmap/](Roadmap/) | 8단계 로드맵과 현재 상태 |
| [src/](src/) | 메인 ROS2 패키지 (slam / common / drive / robot_arm) |
| [test_workspace/](test_workspace/) | 층 전환 오케스트레이터 · Gazebo world swap · E2E smoke |
| [docker/](docker/) | 컨테이너 정의 SSOT — 개발(amd64) + Jetson(aarch64) |
| [docs/hardware_spec.md](docs/hardware_spec.md) | 하드웨어 SSOT |
| [docs/deployment/](docs/deployment/) | 이식성 정책 + Jetson 배포 절차 |
| [docs/improvement_report.md](docs/improvement_report.md) | 리스크/개선 추적기 |

브랜치: `main`(보호) ← `dev`(통합) ← `lee/* han/* kim/*` · SLAM 백업은 `setup/linux-slam-workspace`.
컨테이너 이미지: `ghcr.io/packagu/ros2-humble-slam` (GHCR private — [docker/README.md](docker/README.md) 참조).
