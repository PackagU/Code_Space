# 종설_6조 (PackagU) — 엘리베이터 무인 배달 로봇

> 이 파일은 **모든 코드 어시스턴트(Claude Code, Codex CLI 등)** 의 공통 지침입니다.
> `CLAUDE.md` 는 이 파일을 import 하는 stub — 실제 규칙은 여기서 관리.

## ⚠️ 필수 규칙 (반드시 읽을 것)

1. **이 workspace에서 "Notion" = 반드시 `notion_guest` MCP** — `my_workspace` MCP 절대 사용 금지
2. **Claude Code + Codex CLI 동시 사용 시** → 같은 파일 동시 편집 금지. `TODO.md`에 "편집 중" 표시 후 작업
3. **세션 시작 전**: `git pull origin dev` + 이 파일 + `TODO.md` + `Roadmap/README.md` 읽기
4. **세션 종료 후**: `TODO.md` 업데이트 → `git commit` → (SLAM 작업 시) `python scripts/log_session.py` 실행
5. **취약점/개선점 발견 시**: 즉시 `docs/improvement_report.md` 갱신 (아래 §개선보고서 운영 지침 참조)
6. **하드웨어 스펙은 `docs/hardware_spec.md` 가 SSOT** — 이 파일에는 요약만 둠
7. **보안 최우선 / 모든 AI 어시스턴트 공통**: Claude Code, Codex CLI, Gemini 등 어떤 도구도 API token, secret, credential, `.env`, SSH key, auth 파일 값을 읽거나 추출하려고 시도하지 말 것
8. **민감정보 외부 반출 금지**: Notion API 값, API token, secret, credential, `.env`, SSH key, auth/session 파일 값, 개인정보, 비공개 계정 정보, 서비스 접속 정보는 GitHub, Notion, Docker Hub, Google Drive, Slack, 이메일 등 로컬이 아닌 외부 서비스로 업로드/동기화/붙여넣기/전송하지 말 것. 민감정보가 아닌 코드/문서/로그/설정 파일은 사용자의 명시적 허가 또는 팀 정책에 따라 외부 서비스에 업로드/동기화할 수 있다.
9. **규칙 충돌 시 보안 규칙 우선**: 아래 문서의 GitHub/Notion/PR/업로드 관련 안내보다 7~8번 규칙이 항상 우선한다. 외부 공유 전에는 민감정보 포함 여부를 먼저 확인한다.
10. **`docs/` 마크다운 작성 규칙**: `docs/` 하위 모든 `.md`는 **Notion + Obsidian 호환 형식**으로만 작성. 핵심 5줄 — (a) 헤딩 H1~H3만 (b) 이미지는 `./images/` 상대경로 (c) 코드블록 언어 태그 필수 (d) wikilink/HTML 태그/footnote 금지 (e) 한글 파일명 금지. **상세 규칙·매트릭스·예시는 [docs/_style/notion_markdown_style.md](docs/_style/notion_markdown_style.md) 참조** — `docs/` 편집 전 반드시 한 번 읽을 것. 동기화는 `python scripts/sync_docs_to_notion.py` 로 수행 (`docs/_style/`와 `docs/.notion_sync.json`은 동기화 제외).
11. **저장소 구조**: 팀 저장소는 `PackagU/Code_Space` 하나다. `main` 브랜치 = 메인 프로젝트 코드(이 workspace), `setup/linux-slam-workspace` 브랜치 = SLAM 작업 백업(workspace 안의 `SLAM/` 폴더가 이 브랜치의 체크아웃). SLAM 구현 코드·launch/config/world·민감정보 없는 map 백업은 `SLAM/`에 저장 후 해당 브랜치에 commit/push 한다. (구 `Road_MAP` repo는 `Code_Space`로 이름 변경됨.) 외부 업로드 전에는 7~9번 보안 규칙을 먼저 적용한다.
12. **E2E 원커맨드 정책**: 시연·검증·smoke·통합 테스트는 가능하면 단일 스크립트 한 줄로 실행되게 만든다. 스크립트는 build → launch → trigger/input → verify → log 저장 → cleanup까지 포함하고, GUI/RViz 같은 관찰 옵션은 환경변수로 켜고 끌 수 있게 둔다. 단일 스크립트가 불가능하면 이유와 수동 절차를 문서에 명시한다.
13. **이식성 정책 (Jetson 대비)**: `src/` 패키지는 컨테이너 절대경로(`/ros2_ws`)·사용자 홈 경로·시리얼 포트 하드코딩 금지 (장치 경로는 launch 인자화). 규칙 상세는 [docs/deployment/01_portability_policy.md](docs/deployment/01_portability_policy.md), 검사는 `python3 scripts/check_portability.py`. 커밋 전 통과 필수.
14. **로컬 전용 폴더**: `legacy/`(아카이브), `docs/session_wiki/`(세션 일지 — Notion으로만 공유), `logs/`, `obsidian_vault/`, `docker_env/`(이미지 publish repo 체크아웃)는 GitHub에 올리지 않는다(.gitignore로 강제). 대용량 영상(webm 등)은 어떤 경우에도 커밋 금지.

---

## 프로젝트 개요

| 항목 | 내용 |
|------|------|
| 목표 | 엘리베이터 탑승/하차 + 층간 이동 + 택배 배달 자율 로봇 |
| 팀 | Lee (SLAM), Han (fusion modeling + 구동부), Kim (로봇팔 제어) |
| GitHub | https://github.com/PackagU — 팀 저장소: `Code_Space` |
| IDs | Lee=JunhyungLee25, Han=inonewater, Kim=DuckFrog123 |
| 컨테이너 이미지 | `ghcr.io/packagu/ros2-humble-slam:humble` (GHCR, private) |
| ROS | ROS2 Humble (Docker 필수: Jetson JetPack은 Ubuntu 20.04라 직접 설치 불가) |
| SLAM | SLAM Toolbox |
| LiDAR | RPLiDAR A1m8 (360°, SLAM용) |
| 주 컴퓨터 | NVIDIA Jetson Xavier NX |
| MCU | OpenCR 1.0 (IMU + 바퀴 모터), Arduino Nano (모터 제어) |
| 카메라 | 웹캠 2개 |
| 네이밍 | 노드/실행자 prefix는 `packagu_` 권장 (예: `packagu_keyboard_teleop`) |

---

## 하드웨어 요약

> **상세 사양은 [docs/hardware_spec.md](docs/hardware_spec.md) 가 SSOT.** 변경은 그 파일에서만.

- 주행: 2륜 차동 + 볼롤러 / 20kg 하중 / 모터 미정 (Han)
- 로봇팔: 4 DOF / 60cm / 서보 + Visual Servoing (Kim)
- Z-축 리프트: 웜기어 + T스크류 / 70cm / 5kg (Han)
- 제어: Jetson Xavier NX + OpenCR 1.0 + Arduino Nano
- 센서: RPLiDAR A1m8, IMU 9축, 웹캠 2개

---

## Notion 구조 (notion_guest MCP page IDs)

> **주의**: Notion 접근 시 반드시 `notion_guest` MCP 사용. `mcp__notion_guest__*` 도구만 사용할 것.

| 페이지 | Page ID |
|--------|---------|
| 종설_6조 (루트) | `2e19c69b-0345-807c-9bdd-cfc401a76017` |
| 계획 | `2e19c69b-0345-80cf-816a-c3627c06ca26` |
| 스터디 | `30b9c69b-0345-80ad-bc95-f63aac4bd454` |
| 스터디 > SLAM | `30b9c69b-0345-80e0-8eef-cacfa54fa3e6` |
| ROS2 + Gazebo + RViz2 | `3339c69b-0345-802b-ab69-d1e831ab7b2c` |
| HW | `3289c69b-0345-80df-9cf7-dcfbe79f1299` |
| HW 세부 설계 사양서 | `3289c69b-0345-8053-a71d-c5f84ac68855` |
| docker + VcXsrv | `3519c69b-0345-8050-b483-c4e24b1eff31` |
| 주제_아이디어 | `2ec9c69b-0345-8007-a6c5-c6ae633dd7d5` |
| 회의 DB | `3249c69b-0345-80bc-9375-fbee05f0017a` |
| 아키텍처 | `3249c69b-0345-80ef-a900-f8a0fe98839f` |
| Prompt | `3289c69b-0345-802e-a3db-dacb1e0a9159` |

---

## 폴더 구조 (실제)

```text
2026_graduation_project/          (GitHub: PackagU/Code_Space, main)
├── AGENTS.md                     ← 모든 AI 어시스턴트 공통 지침 (이 파일, SSOT)
├── CLAUDE.md                     ← @AGENTS.md 한 줄 stub
├── TODO.md                       ← 세션 단위 휘발성 메모/lock
├── Roadmap/                      ← 8단계 로드맵 (01_poc_closure ~ 08_safety_security)
├── src/                          ← ROS2 colcon workspace (메인)
│   ├── slam_pkg/                 ← Lee — SLAM Toolbox + Nav2 launch/config/maps
│   ├── common_pkg/               ← 공용 — URDF, Gazebo worlds, 맵 스펙 YAML
│   ├── drive_pkg/                ← Han — 구동부 (현재 WASD teleop)
│   └── robot_arm_pkg/            ← Kim — 4 DOF 로봇팔 (뼈대)
├── test_workspace/               ← PoC/검증 전용 colcon workspace 3개
│   ├── elevator_mission/         ← 배달 미션 BT (waypoint 라우팅)
│   ├── elevator_auto_map_switch/ ← 자동 층 전환 오케스트레이터
│   └── gazebo_world_swap/        ← 층 전환 시 Gazebo 건물 교체 + E2E smoke
├── docker/                       ← 컨테이너 정의 SSOT (개발 amd64 + Jetson aarch64)
│   ├── Dockerfile                ← 개발용 (Gazebo/RViz 포함)
│   ├── Dockerfile.jetson         ← 실기용 (GUI 제외, arm64)
│   ├── compose/                  ← linux / windows / jetson compose
│   └── scripts/entrypoint.sh
├── docs/
│   ├── hardware_spec.md          ← 하드웨어 SSOT
│   ├── improvement_report.md     ← 취약점/개선점 추적기
│   ├── deployment/               ← 이식성 정책 + Jetson 배포 절차
│   ├── simulation_test/          ← 시뮬 실행 가이드 (01_environment ~ 99_reference)
│   ├── session_wiki/             ← 세션 일지 (로컬 전용, Notion으로만 공유)
│   └── _style/                   ← docs 마크다운 스타일 규칙
├── scripts/                      ← host 유틸 (시뮬 런처, 월드/맵 생성기, 검사기, Notion 동기화)
├── SLAM/                         ← Code_Space의 setup/linux-slam-workspace 브랜치 체크아웃 (gitignore)
├── legacy/                       ← 로컬 전용 아카이브 (gitignore — legacy/README.md 참조)
├── logs/                         ← SLAM 시행착오 로그 (로컬 전용)
├── obsidian_vault/SLAM/          ← Obsidian 로컬 일지 (로컬 전용)
└── .github/workflows/            ← CI (오프라인 테스트 + 이식성 검사)
```

---

## Docker 사용법

> 정의 파일과 상세 절차는 [docker/README.md](docker/README.md) 참조. 이미지는 GHCR private — `gh auth` 후 `docker login ghcr.io` 필요 (`docs/simulation_test/01_environment/` 가이드 참조).

### Linux 데스크톱 (개발)

```bash
# 원커맨드 (컨테이너 기동 + 빌드 + Gazebo/SLAM/RViz)
./scripts/run_kku_sim.sh F1

# 또는 수동으로
xhost +local:docker
docker compose -f docker/compose/docker-compose.linux.yml up -d
docker exec -it ros2_humble bash
# 컨테이너 안:
cd /ros2_ws && colcon build --symlink-install && source install/setup.bash
```

### Windows (개발)

1. VcXsrv 실행 (Display :0) 후 Docker Desktop 확인
2. `docker compose -f docker/compose/docker-compose.windows.yml up -d`
3. DISPLAY는 compose가 `host.docker.internal:0.0`으로 설정

### Jetson Xavier NX (실기)

```bash
docker compose -f docker/compose/docker-compose.jetson.yml up -d
```

이미지 크로스 빌드/배포와 부하 검증 절차는 [docs/deployment/01_portability_policy.md](docs/deployment/01_portability_policy.md) 참조.

---

## Claude Code + Codex CLI 병렬 사용 규칙

### 역할 분담

| 도구 | 담당 작업 |
|------|----------|
| **Claude Code** | Notion 연동, 복잡한 ROS2 패키지 설계, 디버깅, SLAM 파라미터 튜닝, GitHub PR 작성 |
| **Codex CLI** | 빠른 boilerplate 생성, Python/Bash 유틸 스크립트, 파라미터 yaml 생성 |

### CLI 전용 지침

- 웹 검색이 막히거나 일반 검색으로 충분한 결과를 얻지 못하면 `insane-search` 플러그인을 사용한다.

### 파일별 담당 (충돌 방지)

| 경로 | 기본 담당 |
|------|----------|
| `src/slam_pkg/`, `test_workspace/` | Claude Code (Lee) |
| `src/robot_arm_pkg/` | Codex CLI (Kim 지원) |
| `src/drive_pkg/` | Codex CLI (Han 지원) |
| `docker/` | 공용 — 동시 편집 금지, TODO.md에 lock 표시 |
| `AGENTS.md`, `TODO.md` | 세션 종료 시만 수정 |

### 세션 시작 체크리스트

```text
[ ] git pull origin dev
[ ] AGENTS.md 읽기 (이 파일)
[ ] TODO.md에서 현재 작업 확인
[ ] TODO.md에 "작업 중: [내 작업]" 표시
```

### 세션 종료 체크리스트

```text
[ ] TODO.md 업데이트 (완료/남은 작업)
[ ] python3 scripts/check_portability.py 통과 확인
[ ] git add -p && git commit -m "feat/fix: ..."
[ ] git push origin <브랜치>
[ ] SLAM 작업 시: python scripts/log_session.py --summary "..." --result success|fail
```

---

## SLAM 로그 자동화

```bash
# SLAM 세션 종료 시 실행
python scripts/log_session.py \
  --summary "오늘 시도한 내용" \
  --result success   # 또는 fail / partial
  --error "에러 메시지 (실패 시)"  \
  --next "다음에 시도할 것"

# 저장 위치
# logs/YYYY-MM-DD_HH-MM/summary.md  (로컬 상세 로그)
# obsidian_vault/SLAM/trials/YYYY-MM-DD.md  (Obsidian 일지)
```

보안 정책상 민감정보 포함 로그는 외부 업로드 금지:
> 세션 로그는 기본적으로 `logs/` 와 `obsidian_vault/` 등 로컬에 보관한다. 민감정보가 없는 요약 로그는 사용자의 허가 또는 팀 정책에 따라 Notion에 기록할 수 있으며, Notion 접근 시 반드시 `notion_guest` MCP만 사용한다.

---

## GitHub 워크플로우

### 저장소·브랜치 전략

```text
PackagU/Code_Space
├── main                        ← 항상 빌드 가능 상태 (직접 push 금지)
├── dev                         ← 통합 테스트 브랜치
├── lee/slam-*                  ← Lee의 SLAM/시뮬 feature 브랜치
├── han/drive-*                 ← Han의 구동부 브랜치
├── kim/arm-*                   ← Kim의 로봇팔 브랜치
└── setup/linux-slam-workspace  ← SLAM 백업 (workspace의 SLAM/ 폴더 전용)
```

### PR 규칙

- `feature → dev`: 셀프 리뷰 후 PR 생성
- `dev → main`: 팀원 1명 이상 리뷰 필수
- PR 본문에 `Closes #N` 또는 `Refs Roadmap/<파일>.md` 로 계획과 연결
- CI(`.github/workflows/check.yml`)가 오프라인 테스트 + 이식성 검사를 실행 — 실패 시 머지 금지

---

## 작업/계획 추적 — 4-layer 모델

| Layer | 어디서 | 무엇을 | 주기 |
|-------|--------|--------|------|
| 1. **로드맵 / 마일스톤** | `Roadmap/README.md` + `01~08_*.md` | 단계별 목표·완료 기준·상태 | 단계 완료/방향 변경 시 |
| 2. **개선점 추적** | `docs/improvement_report.md` | 리스크·갭·진행률 | 발견/해소 즉시 |
| 3. **실제 작업 단위** | GitHub Issues (Code_Space) | 1개 작업 = 1 Issue, 라벨 `area:slam\|drive\|arm\|docker\|common` | 매일 |
| 4. **세션 휘발성 메모** | `TODO.md` | 오늘 편집 중인 파일 lock, 한 줄 메모 | 세션마다 |

**원칙**:

- `Roadmap/*.md` 는 **변하는 큰 그림** — 상태 마커(✅/🔄/⏸/미착수)를 실제 진행에 맞춰 유지한다. 진행이 문서와 어긋난 채 방치 금지.
- 진행 중 작업은 GitHub Issue로 — 회의 안건은 `TODO.md` 🗳 절에 모은다.

---

## 개선 보고서 운영 지침 — `docs/improvement_report.md`

이 파일은 **취약점·리스크·개선점의 살아있는 추적기**. 회의나 작업 중 발견되면 즉시 추가, 해소되면 ✅ 표시.

### 항목 포맷

```markdown
### 1.9 🔴 [60%] 주행 구동부 — 2륜 + 볼롤러로 20kg + 문턱
> 상태: 🔄 in-progress · 담당: Han · 업데이트: 2026-07-03
> 진척: (a) 볼롤러 2개 변경 ✅ / (b) 모터 모델 검토 중 / (c) 회전반경 시뮬 미착수
```

### 상태 마커

| 마커 | 의미 | 헤더 표기 |
|------|------|----------|
| (없음) | 신규 발견 | `🔴 ...` |
| 🔄 | 진행 중 | `🔴 [40%] ...` |
| ⏸ | 보류 (외부 요인 대기) | `🔴 ⏸ ...` |
| ✅ | 완료 | `🔴 ✅ ...` |
| ❌ | 무효화/철회 (사유 기록) | `🔴 ❌ ...` |

### 갱신 룰

1. **추가**: 발견 즉시 해당 절에 새 항목. 우선순위 미정이면 🟡 기본.
2. **진척 갱신**: % 만 바꾸지 말고 `> 진척:` 줄에 무엇이 진행됐는지 명시.
3. **완료**: 헤더에 ✅ + `> 상태: ✅ done · 완료: YYYY-MM-DD` 로 변경. **삭제 금지** — 기록으로 남김.
4. **분기말 정리**: 분기 끝에 ✅ 항목들을 파일 하단 `## Completed Improvements` 절로 이동.
5. **커밋 메시지**: `report: §1.6 60%→80% (Jetson 실측 완료)` 형식.

### 트리거: 언제 항목을 추가/갱신해야 하는가

- **추가**: 회의·코드리뷰·SLAM 시행 중 새 리스크 식별 / 외부 조사 결과 우려 사항 발견
- **갱신**: 관련 PR 머지 / 회의 결정 / 부품 도착·테스트 결과 / 마일스톤 종료
- **완료**: DOD 충족 + 검증 완료 (시뮬 또는 실기 확인)

---

## 현재 포커스 (세션 후 업데이트)

**현재 단계**: 07(Jetson) 분산 E2E 완주 — 데스크톱 Gazebo + Jetson 실전 스택 + 실물 로봇팔 2회 층 전환, 부하 여유 확인 (cpu_peak 77.6%/600%). 남은 관문은 03(실측 맵)·06 완결(반복 10회)·07 잔여(실기 센서 주행·반복 안정성). `Roadmap/README.md` 참조.
**현재 작업**: 분산 스모크 재현성 정비 완료 (절차: [docs/deployment/01_portability_policy.md](docs/deployment/01_portability_policy.md) §4.5) — Jetson repo bundle 재동기화 대기
**블로커**: 모터/배터리/Depth 카메라 미정 + 복도/엘리베이터 실측 대기 ([docs/hardware_spec.md](docs/hardware_spec.md) §미정 항목, [improvement_report.md](docs/improvement_report.md) §1.8) + OpenCR 쇼트(§1.20)
**다음 할 일**: 신공학관 실측 → 맵 재생성 → Jetson 실기 센서(RPLiDAR) 주행 + missed-rate 실기 임계 수립
