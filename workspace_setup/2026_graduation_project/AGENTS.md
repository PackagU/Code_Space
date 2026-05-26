# 종설_6조 — 엘리베이터 무인 배달 로봇

> 이 파일은 **모든 코드 어시스턴트(Claude Code, Codex CLI 등)** 의 공통 지침입니다.
> `CLAUDE.md` 는 이 파일을 import 하는 stub — 실제 규칙은 여기서 관리.

## ⚠️ 필수 규칙 (반드시 읽을 것)

1. **이 workspace에서 "Notion" = 반드시 `notion_guest` MCP** — `my_workspace` MCP 절대 사용 금지
2. **Claude Code + Codex CLI 동시 사용 시** → 같은 파일 동시 편집 금지. `TODO.md`에 "편집 중" 표시 후 작업
3. **세션 시작 전**: `git pull origin dev` + 이 파일 + `TODO.md` + 해당 영역의 `plan/<area>.md` 읽기
4. **세션 종료 후**: `TODO.md` 업데이트 → `git commit` → (SLAM 작업 시) `python scripts/log_session.py` 실행
5. **취약점/개선점 발견 시**: 즉시 `docs/improvement_report.md` 갱신 (아래 §개선보고서 운영 지침 참조)
6. **하드웨어 스펙은 `docs/hardware_spec.md` 가 SSOT** — 이 파일에는 요약만 둠
7. **보안 최우선 / 모든 AI 어시스턴트 공통**: Claude Code, Codex CLI, Gemini 등 어떤 도구도 API token, secret, credential, `.env`, SSH key, auth 파일 값을 읽거나 추출하려고 시도하지 말 것
8. **민감정보 외부 반출 금지**: Notion API 값, API token, secret, credential, `.env`, SSH key, auth/session 파일 값, 개인정보, 비공개 계정 정보, 서비스 접속 정보는 GitHub, Notion, Docker Hub, Google Drive, Slack, 이메일 등 로컬이 아닌 외부 서비스로 업로드/동기화/붙여넣기/전송하지 말 것. 민감정보가 아닌 코드/문서/로그/설정 파일은 사용자의 명시적 허가 또는 팀 정책에 따라 외부 서비스에 업로드/동기화할 수 있다.
9. **규칙 충돌 시 보안 규칙 우선**: 아래 문서의 GitHub/Notion/PR/업로드 관련 안내보다 7~8번 규칙이 항상 우선한다. 외부 공유 전에는 민감정보 포함 여부를 먼저 확인한다.

---

## 프로젝트 개요

| 항목 | 내용 |
|------|------|
| 목표 | 엘리베이터 탑승/하차 + 층간 이동 + 택배 배달 자율 로봇 |
| 팀 | Lee (SLAM), Han (fusion modeling + 구동부), Kim (로봇팔 제어) |
| GitHub | https://github.com/PackagU |
| IDs | Lee=JunhyungLee25, Han=inonewater, Kim=DuckFrog123 |
| Docker Hub | `my_ros2_humble` (Kim 관리, 팀 공유) |
| ROS | ROS2 Humble (Docker 필수: Jetson Xavier NX 미지원) |
| SLAM | SLAM Toolbox |
| LiDAR | RPLiDAR A1m8 (360°, SLAM용) |
| 주 컴퓨터 | NVIDIA Jetson Xavier NX |
| MCU | OpenCR 1.0 (IMU + 바퀴 모터), Arduino Nano (모터 제어) |
| 카메라 | 웹캠 2개 |

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

## 폴더 구조

```
2026_graduation_project/
├── AGENTS.md                    ← 모든 AI 어시스턴트 공통 지침 (이 파일, SSOT)
├── CLAUDE.md                    ← @AGENTS.md 한 줄 stub
├── TODO.md                      ← 세션 단위 휘발성 메모/lock
├── .gitignore
├── plan/                        ← 큰 그림 (마일스톤·DOD·일정)
│   ├── roadmap.md               ← M1~Mn 전체 일정
│   ├── slam.md                  ← Lee
│   ├── drive.md                 ← Han
│   ├── arm.md                   ← Kim
│   ├── integration.md           ← 통합/시연 시나리오
│   └── risks.md                 ← 사전식별 리스크
├── docs/
│   ├── hardware_spec.md         ← 하드웨어 SSOT
│   ├── improvement_report.md    ← 취약점/개선점 추적기
│   └── project_board.md         ← ROAD MAP Project 자동화 가이드
├── docker/
│   ├── Dockerfile               ← FROM my_ros2_humble + SLAM Toolbox 추가
│   ├── docker-compose.yml       ← VcXsrv GUI 포함
│   └── scripts/
│       ├── start_vcxsrv.bat     ← Windows: 컨테이너 시작 전 먼저 실행
│       └── entrypoint.sh        ← 컨테이너 진입점
├── src/                         ← ROS2 colcon workspace
│   ├── slam_pkg/                ← Lee 담당 (SLAM Toolbox)
│   ├── robot_arm_pkg/           ← Kim 담당 (4 DOF 로봇팔)
│   ├── drive_pkg/               ← Han 담당 (구동부)
│   └── common_pkg/              ← 공용 (URDF, custom msgs)
├── logs/                        ← SLAM 시행착오 로그 (log_session.py 자동 생성)
├── obsidian_vault/SLAM/         ← Obsidian 로컬 일지
├── scripts/
│   └── log_session.py           ← SLAM 세션 로그 자동화
└── .github/
    ├── PULL_REQUEST_TEMPLATE.md
    ├── ISSUE_TEMPLATE/          ← task/bug/spike 템플릿
    ├── labels.yml               ← 라벨 컨벤션
    └── workflows/check.yml
```

---

## Docker 사용법

### 사전 준비 (Windows)
1. VcXsrv 실행: `docker\scripts\start_vcxsrv.bat` 더블클릭 (Display :0)
2. Docker Desktop 실행 확인

### 컨테이너 실행
```bash
# 최초 빌드
docker compose -f docker/docker-compose.yml up --build -d

# 이후 실행
docker compose -f docker/docker-compose.yml up -d

# 컨테이너 접속
docker exec -it ros2_humble bash

# 내부에서 ROS2 소스 빌드
cd /ros2_ws && colcon build --symlink-install
source install/setup.bash
```

### DISPLAY 설정
- Windows (Docker Desktop): `host.docker.internal:0.0`
- 컨테이너 내부 확인: `echo $DISPLAY`

---

## Claude Code + Codex CLI 병렬 사용 규칙

### 역할 분담
| 도구 | 담당 작업 |
|------|----------|
| **Claude Code** | Notion 연동, 복잡한 ROS2 패키지 설계, 디버깅, SLAM 파라미터 튜닝, GitHub PR 작성 |
| **Codex CLI** | 빠른 boilerplate 생성, Python/Bash 유틸 스크립트, 파라미터 yaml 생성 |

### 파일별 담당 (충돌 방지)
| 경로 | 기본 담당 |
|------|----------|
| `src/slam_pkg/` | Claude Code (Lee) |
| `src/robot_arm_pkg/` | Codex CLI (Kim 지원) |
| `src/drive_pkg/` | Codex CLI (Han 지원) |
| `docker/` | 공용 — 동시 편집 금지, TODO.md에 lock 표시 |
| `CLAUDE.md`, `TODO.md` | 세션 종료 시만 수정 |

### 세션 시작 체크리스트
```
[ ] git pull origin dev
[ ] CLAUDE.md 읽기 (이 파일)
[ ] TODO.md에서 현재 작업 확인
[ ] TODO.md에 "작업 중: [내 작업]" 표시
```

### 세션 종료 체크리스트
```
[ ] TODO.md 업데이트 (완료/남은 작업)
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

### 브랜치 전략
```
main     ← 항상 빌드 가능 상태 (직접 push 금지)
dev      ← 통합 테스트 브랜치
lee/slam-*    ← Lee의 SLAM feature 브랜치
han/drive-*   ← Han의 구동부 브랜치
kim/arm-*     ← Kim의 로봇팔 브랜치
```

### PR 규칙
- `feature → dev`: 셀프 리뷰 후 PR 생성
- `dev → main`: 팀원 1명 이상 리뷰 필수
- PR 템플릿: `.github/PULL_REQUEST_TEMPLATE.md` 자동 적용

---

## 작업/계획 추적 — 4-layer 모델

| Layer | 어디서 | 무엇을 | 주기 |
|-------|--------|--------|------|
| 1. **로드맵 / 마일스톤** | `plan/roadmap.md` | M1~Mn 의 목표·DOD·데드라인 | 월 단위, 거의 변경 없음 |
| 2. **영역별 계획** | `plan/{slam,drive,arm,integration,risks}.md` | 각 파트의 단계별 작업 + DOD | 주 단위 갱신 |
| 3. **실제 작업 단위** | GitHub Issues + ROAD MAP Project | 1개 작업 = 1 Issue, 보드 상태/진척 관리 | 매일 |
| 4. **세션 휘발성 메모** | `TODO.md` | 오늘 편집 중인 파일 lock, 한 줄 메모 | 세션마다 |

**원칙**:
- `plan/*.md` 는 **변하는 큰 그림** — 진척이 아니라 "무엇을·왜"를 적음
- 진행 중 작업은 모두 GitHub Issue 로 — 라벨 `area:slam|drive|arm|docker|common` + `priority:🔴🟡🟢`
- Issue 가 열리면 ROAD MAP Project (https://github.com/orgs/PackagU/projects/1) 에 자동 편입
- PR 본문에 `Closes #N` 또는 `Refs plan/<area>.md#섹션` 으로 plan 과 연결
- 자동화 가이드: `docs/project_board.md`

---

## 개선 보고서 운영 지침 — `docs/improvement_report.md`

이 파일은 **취약점·리스크·개선점의 살아있는 추적기**. 회의나 작업 중 발견되면 즉시 추가, 해소되면 ✅ 표시.

### 항목 포맷

```markdown
### 4.1 🔴 [60%] 주행 구동부 — 2륜 + 볼롤러 1개로 20kg + 문턱
> 상태: 🔄 in-progress · 담당: Han · 업데이트: 2026-05-03
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
4. **분기말 정리**: 분기 끝에 ✅ 항목들을 파일 하단 `## 완료된 개선 (Archive)` 절로 이동.
5. **커밋 메시지**: `report: §4.1 60%→80% (XH540 결정)` 형식.

### 트리거: 언제 항목을 추가/갱신해야 하는가

- **추가**: 회의·코드리뷰·SLAM 시행 중 새 리스크 식별 / 외부 조사 결과 우려 사항 발견
- **갱신**: 관련 PR 머지 / 회의 결정 / 부품 도착·테스트 결과 / 마일스톤 종료
- **완료**: DOD 충족 + 검증 완료 (시뮬 또는 실기 확인)

---

## 현재 포커스 (세션 후 업데이트)

**현재 마일스톤**: M1 (2026-05-31 마감) — `plan/roadmap.md` 참조
**현재 작업**: 워크스페이스 세팅 + 계획 폴더 구축
**블로커**: 모터/배터리/Depth 카메라 미정 ([improvement_report.md](docs/improvement_report.md) §4.1, §4.2)
**다음 할 일**: Docker 빌드 테스트 + SLAM Toolbox 기본 launch + 모델링 회의
