# `docs/_style/` — 동기화 설정 + 사용법

이 폴더는 **`docs/` 폴더의 Notion 위키 동기화를 위한 메타 자료**다. 동기화 대상이 아니므로 Notion에는 올라가지 않는다.

- [`notion_markdown_style.md`](./notion_markdown_style.md) — 모든 `.md`가 따라야 하는 작성 규칙 (Notion + Obsidian 호환)
- 이 `README.md` — 동기화 스크립트 setup + 실행 가이드

---

## 1. 한 번만 하는 setup (5분)

### (1) Notion 통합(Integration) 만들기

1. https://www.notion.so/profile/integrations 접속
2. **New integration** 클릭
3. 이름: `docs-sync` (또는 원하는 이름)
4. Type: **Internal**
5. Capabilities: **Read content, Update content, Insert content** 체크
6. 생성 후 **"Internal Integration Secret"** 복사 (한 번만 보이니까 안전한 곳에 저장)

### (2) 통합을 부모 페이지와 연결

1. Notion에서 **종설_6조** 루트 페이지 열기
2. 우상단 `...` → **Connections** → `Add connections`
3. 위에서 만든 `docs-sync` 추가
4. 하위 페이지는 자동 상속 → 한 번만 연결하면 됨

### (3) `.env` 만들기

```bash
cp .env.example .env
```

`.env`를 편집해서 `NOTION_TOKEN` 자리에 (1)에서 복사한 secret 붙여넣기.
`NOTION_PARENT_PAGE_ID`는 이미 종설_6조 루트로 채워져있음.

> ⚠️ `.env`는 `.gitignore`에 들어있으니 절대 커밋하지 말 것 — AGENTS.md 7번 규칙.

### (4) Python 의존성 설치

```bash
pip install -r scripts/requirements_notion_sync.txt
```

또는 venv 사용:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r scripts/requirements_notion_sync.txt
```

---

## 2. 매일 쓰는 명령

### 변경사항 미리보기 (Notion 안 건드림)

```bash
python scripts/sync_docs_to_notion.py --dry-run
```

### 실제 동기화

```bash
python scripts/sync_docs_to_notion.py
```

### 상세 로그

```bash
python scripts/sync_docs_to_notion.py --verbose
```

---

## 3. 동작 원리 요약

```
docs/                                    Notion
├── opencr_dynamixel_wheel_test.md  -->  📄 OpenCR Dynamixel Wheel Test
├── simulation_test/                -->  📁 simulation test
│   ├── README.md                   --> (부모 페이지 본문)
│   ├── 01_environment/             -->  📁 environment
│   │   ├── README.md               --> (부모 페이지 본문)
│   │   └── 01_linux_docker...md    -->  📄 linux docker slam setup
│   └── ...
├── session_wiki/                   -->  📁 session wiki
│   └── 2026-05-27_slam_debug/      -->  📁 2026-05-27 slam debug
│       └── ...
├── _style/                         -->  (제외 — 동기화 안 됨)
└── .notion_sync.json               -->  (제외 — 매니페스트)
```

규칙:
- **폴더** = Notion 부모 페이지. `README.md`가 있으면 그 내용이 부모 페이지의 본문이 된다.
- **`.md` 파일** = Notion 자식 페이지. 첫 H1이 페이지 제목, 나머지는 본문.
- **번호 prefix** (`01_`, `99_`)는 페이지 제목에서 자동 제거. 정렬은 Notion에서 직접 또는 알파벳 순.
- **언더스코어/점**으로 시작하는 파일/폴더는 동기화 안 됨 (`_style`, `.notion_sync.json` 등).
- **`README.md`**는 항상 부모 폴더의 본문으로 흡수 — 자식 페이지로 따로 안 만들어짐.

---

## 4. 동기화 후 시나리오

### 새 파일/폴더 추가
- 그냥 `docs/` 안에 만들고 → `python scripts/sync_docs_to_notion.py`
- 매니페스트에 자동으로 ID가 추가됨

### 기존 파일 수정
- `.md` 편집 → sync → Notion 페이지가 통째로 다시 그려짐
- ⚠️ Notion에서 직접 추가한 코멘트/하이라이트는 사라짐 (Notion = read-only view)

### 파일/폴더 이름 변경
- 스크립트 입장에선 "삭제 + 신규"로 보임 → 기존 Notion 페이지는 그대로(고아), 새 페이지가 생성됨
- **해결**: `docs/.notion_sync.json` 열어서 해당 `"paths"` key만 새 이름으로 바꾸고 sync 재실행 → 기존 페이지 그대로 유지됨

### 파일 삭제
- 스크립트는 **자동으로 Notion 페이지를 삭제하지 않는다** (안전장치)
- sync 끝에 `stale manifest entries` 경고가 뜸 → Notion에서 수동으로 페이지 휴지통 이동 + 매니페스트에서 해당 key 삭제

---

## 5. 자주 만나는 에러

| 증상 | 원인 | 해결 |
|---|---|---|
| `NOTION_TOKEN not set` | `.env` 없음 또는 placeholder 그대로 | `.env`에 실제 token 넣기 |
| `404 Not Found` | 통합이 부모 페이지에 연결 안 됨 | Notion 페이지 → Connections에서 `docs-sync` 추가 |
| `401 Unauthorized` | token 잘못됨 또는 만료 | Notion integrations 페이지에서 secret 재발급 |
| `400 Bad Request: body...validation_error` | 마크다운에 Notion이 안 받는 요소 (보통 HTML 태그) | 해당 `.md`에서 HTML 제거 — [`notion_markdown_style.md`](./notion_markdown_style.md) 참고 |
| 이미지가 깨져 보임 | 로컬 이미지가 GitHub에 push 안 된 상태 | `git push` 먼저, 그 다음 sync |
| 표가 이상하게 보임 | 병합셀이나 너무 긴 셀 | 단순 표로 변경 |

---

## 6. 참고 — 매니페스트 구조

`docs/.notion_sync.json` 은 git 추적 대상이다 (gitignore에서 화이트리스트). 팀원 모두 같은 페이지 ID를 공유해서 같은 페이지를 업데이트한다.

```json
{
  "_root_page_id": "abc123...",       // 📚 문서 위키 페이지 ID
  "_repo_url": "...",
  "_repo_branch": "main",             // 이미지 GitHub URL 빌딩에 사용
  "paths": {
    "opencr_dynamixel_wheel_test.md": "def456...",
    "simulation_test": "ghi789...",
    "simulation_test/01_environment": "jkl012...",
    ...
  }
}
```

수동 편집해도 되는 곳: `paths` 의 key (rename/move 시).
수동 편집 금지: ID 값들 (잘못 바꾸면 다른 페이지를 덮어쓰게 됨).
