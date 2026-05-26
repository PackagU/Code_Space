# TODO — 종설_6조

> Claude Code와 Codex CLI가 공유하는 **세션 단위 휘발성 메모/lock 보드**.
> 영속 작업은 GitHub Issues 로, 큰 그림은 `plan/` 으로.
> 세션 시작 시 읽고, 세션 종료 시 업데이트 후 커밋.

---

## 🔒 현재 편집 중
(없음)

---

## 🗳 다음 회의 안건 (혼자 결정 불가)

## 메모
- 2026-05-04: `AGENTS.md`에 외부 업로드 금지 및 토큰/secret 접근 금지 공통 지침 추가
- 2026-05-13: 외부 반출 금지 범위를 민감정보(Notion API 값, token, secret, credential, `.env`, SSH key, auth/session 값, 개인정보 등)로 축소. 민감정보가 아닌 코드/문서/로그/설정은 사용자 허가 또는 팀 정책에 따라 GitHub/DockerHub/Notion 등에 공유 가능.
- 2026-05-13: `PackagU/ros2-humble-slam-docker` Docker 공유 repo 생성 및 GHCR publish workflow 성공. 팀원 전용 운영을 위해 repo/GHCR package는 private 유지가 맞음. 팀원은 GitHub 권한 + GHCR 로그인 후 pull.
- 2026-05-13: `docs/linux_docker_slam_setup.md`를 private GHCR + `docker_env` 기준으로 업데이트. Linux desktop 세팅은 이 문서의 `gh auth` → `docker login ghcr.io` → `docker compose pull/up` 순서로 진행.

### 자동 코드 리뷰 도구 도입 검토
PR 마다 자동으로 코드 리뷰 코멘트를 다는 도구가 있음. 도입 여부 / 어느 도구 / 비용 부담 주체를 회의에서 결정 필요.

**옵션**
| 도구 | 비용 | 비고 |
|------|------|------|
| Claude Code Action (anthropics/claude-code-action) | Anthropic API 종량제, 월 $5-40 | AGENTS.md 인식, 품질 최상 |
| GitHub Copilot Code Review | Copilot Pro 구독 (학생 무료 가능) | 가입 후 체크박스 한번 |
| CodeRabbit | Public repo 무료 / private 유료 | repo 가시성에 따라 결정 |
| 도입 안 함 (린터만) | 무료 | ament_lint + ruff + clang-format |

**결정 항목**
- [ ] 도입 여부
- [ ] 도입 시 도구 선택
- [ ] 비용 부담자 (개인 키 vs 팀 공용 계정)
- [ ] 모델/요금제 (Sonnet vs Haiku 등)
- [ ] 트리거 — 모든 PR vs 라벨 붙은 PR 만

**관련 자료**: 회의 후 결정되면 `docs/code_review_policy.md` 작성 + 워크플로우 추가.

### plan/ + GitHub Issues 운영 시작
신규 도입한 4-layer 체계(roadmap / plan / Issues / TODO) 의 운영 시작 일자와 첫 Issue 작성 책임자를 정해야 함.

**결정 항목**
- [ ] 운영 시작일 (당장 vs 다음 주부터)
- [ ] M1 Milestone 생성 + 5/31 마감 등록 책임자
- [ ] ROAD MAP Project 보드 초기 세팅 (`docs/project_board.md` §6 체크리스트) 책임자
- [ ] 라벨 동기화 (`.github/labels.yml` → repo) 책임자
- [ ] `plan/*.md` 의 각 영역 파일을 본인이 직접 검토 + 보강

---

## 🔴 우선순위 높음

- [ ] Linux desktop VSCode에 Codex CLI, Claude Code, Gemini CLI 설치
- [ ] GitHub 개선
- [ ] SLAM 시뮬레이션 + 자체 제작 테스트 지도 구성
      {실측 없이 일반적인 건물 구조로 먼저 테스트}
- [ ] Docker 컨테이너 빌드 테스트 (`docker compose up --build`)
      {docker\scripts\start_vcxsrv.bat 실행 → docker compose -f docker/docker-compose.yml up --build}
- [ ] VcXsrv + Docker GUI 연결 확인 (RViz2 또는 Gazebo 실행)
      {컨테이너 안에서 rviz2 실행 → GUI 창이 Windows에 뜨면 성공}
- [ ] `slam_pkg` 기본 launch 파일 작성 (SLAM Toolbox + RPLiDAR) — 1차 완료, 시뮬 검증 남음
      {ros2 launch slam_pkg slam_toolbox.launch.py use_sim_time:=true}
- [ ] GitHub 레포 초기 push (`git init` + remote 연결)
      {git init && git remote add origin https://github.com/PackagU/<repo명>}

## 🟡 진행 예정

- [ ] SLAM Toolbox 파라미터 초기 튜닝 (RPLiDAR A1m8 기준)
- [ ] URDF 로봇 모델 초안 (common_pkg) — 1차 작성됨, Fusion 결과 반영 필요
- [ ] drive_pkg 기본 구조 (Han)
- [ ] robot_arm_pkg 기본 구조 (Kim)
- [ ] Gazebo 시뮬레이션 환경 구성

## 🟢 완료

- [x] OpenCR + Dynamixel 2개 바퀴 구동 테스트 A to Z 문서 작성 — `docs/opencr_dynamixel_wheel_test.md`
- [x] Linux desktop Docker + ROS2 SLAM A to Z 가이드 작성 — `docs/linux_docker_slam_setup.md`
- [x] 워크스페이스 폴더 구조 생성
- [x] CLAUDE.md → AGENTS.md 분리 (Claude Code + Codex CLI 공통화)
- [x] Docker 파일 템플릿 작성
- [x] `plan/` 폴더 + 6개 영역별 계획 파일 작성
- [x] `.github/ISSUE_TEMPLATE/` task/bug/spike 템플릿 + `labels.yml`
- [x] `docs/project_board.md` ROAD MAP 자동화 가이드
- [x] `docs/improvement_report.md` 운영 지침 (AGENTS.md 에 등재)

---

## 메모

- ROS2 Domain ID: 0 (기본값 유지)
- DISPLAY: `host.docker.internal:0.0` (VcXsrv)
- Docker base image: `my_ros2_humble` (DockerHub, Kim 관리)
- M1 마감: **2026-05-31** — 하드웨어 스펙 확정 + Fusion 모델링 + SLAM/팔 시뮬 동작
# 2026-05-13 세션 업데이트

- Linux desktop Docker + ROS2 Humble SLAM 환경 구성 완료.
- `docker_env` 기준 compose 실행과 `ros2_humble` 컨테이너 실행 완료.
- Docker 권한 문제 해결: `hsm` 사용자를 `docker` group에 추가하고 재로그인.
- X11 GUI 권한 문제 해결: `xhost +local:docker`, `xhost +local:root` 적용.
- VSCode Dev Containers로 `ros2_humble` attach 성공.
- 컨테이너 VSCode에서 `/ros2_ws`를 열고 ROS2 작업 흐름 확인.
- `colcon build --symlink-install` 실패 원인 확인: GitHub가 빈 폴더를 저장하지 않아 `launch/`, `maps/` 폴더 누락.
- 빈 폴더 누락에도 빌드가 깨지지 않도록 `drive_pkg`, `robot_arm_pkg`, `slam_pkg` CMake install 조건 수정.
- Gazebo + SLAM Toolbox + RViz2 실행 흐름 확인.
- map 생성, 저장, RViz2 표시까지 완료.
- VSCode 기반 Docker SLAM 작업법 문서 추가: `docs/vscode_docker_slam_workflow.md`.
- 상세 시행착오 기록 추가: `obsidian_vault/SLAM/trials/2026-05-13.md`.

## 다음 작업

- 생성한 map 파일명과 저장 위치를 팀 기준으로 정리.
- `/scan`, `/odom`, `/tf` 확인 결과를 다음 실험 로그에 남김.
- SLAM Toolbox 파라미터 비교 실험 시작.
- `resolution`, `max_laser_range`, `loop_search_maximum_distance`별 map 품질 비교.
