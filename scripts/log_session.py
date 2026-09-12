#!/usr/bin/env python3
"""
SLAM Session Logger
SLAM 시행착오 로그를 로컬(logs/ + obsidian_vault/)에 저장하고 Notion 업로드 안내를 출력.

사용법:
  python scripts/log_session.py \\
    --summary "SLAM Toolbox 파라미터 resolution 조정" \\
    --result fail \\
    --error "map corruption at loop closure" \\
    --next "resolution 0.05 → 0.1로 변경 후 재시도"

결과:
  1. logs/YYYY-MM-DD_HH-MM/summary.md 생성
  2. obsidian_vault/SLAM/trials/YYYY-MM-DD.md 에 append
  3. Notion 업로드용 안내 메시지 출력
"""

import argparse
import os
import sys
from datetime import datetime
from pathlib import Path


NOTION_SLAM_PAGE = "30b9c69b-0345-80e0-8eef-cacfa54fa3e6"


def parse_args():
    parser = argparse.ArgumentParser(description="Log a SLAM simulation session")
    parser.add_argument("--summary", required=True, help="이번 세션에서 시도한 것")
    parser.add_argument(
        "--result",
        required=True,
        choices=["success", "fail", "partial"],
        help="세션 결과",
    )
    parser.add_argument("--error", default="", help="에러 메시지 또는 문제점 (실패 시)")
    parser.add_argument("--next", default="", help="다음에 시도할 것")
    parser.add_argument("--params", default="", help="사용한 파라미터 yaml 파일 경로")
    return parser.parse_args()


def main():
    args = parse_args()
    now = datetime.now()
    session_id = now.strftime("%Y-%m-%d_%H-%M")
    date_str = now.strftime("%Y-%m-%d")
    time_str = now.strftime("%H:%M")

    result_emoji = {"success": "✅", "fail": "❌", "partial": "🔶"}[args.result]

    root = Path(__file__).parent.parent

    # 1. logs/YYYY-MM-DD_HH-MM/summary.md
    log_dir = root / "logs" / session_id
    log_dir.mkdir(parents=True, exist_ok=True)

    params_section = f"`{args.params}`" if args.params else "(기록 없음)"
    error_section = args.error if args.error else "없음"
    next_section = args.next if args.next else "(미정)"

    summary_md = f"""# SLAM 세션 로그 — {session_id}

**결과**: {result_emoji} {args.result.upper()}
**시간**: {date_str} {time_str}

## 시도한 것
{args.summary}

## 사용한 파라미터
{params_section}

## 에러 / 문제점
{error_section}

## 다음 시도
{next_section}
"""

    summary_path = log_dir / "summary.md"
    summary_path.write_text(summary_md, encoding="utf-8")
    print(f"[log_session] ✅ 로그 저장: {summary_path}")

    # 2. obsidian_vault/SLAM/trials/YYYY-MM-DD.md (append)
    trials_dir = root / "obsidian_vault" / "SLAM" / "trials"
    trials_dir.mkdir(parents=True, exist_ok=True)

    obsidian_file = trials_dir / f"{date_str}.md"

    if not obsidian_file.exists():
        obsidian_file.write_text(
            f"# SLAM 시행착오 — {date_str}\n\n", encoding="utf-8"
        )

    append_block = f"""
---

## {time_str} — {result_emoji} {args.result.upper()}

**시도**: {args.summary}
**에러**: {error_section}
**다음**: {next_section}
**로그**: `logs/{session_id}/summary.md`
"""

    with obsidian_file.open("a", encoding="utf-8") as f:
        f.write(append_block)

    print(f"[log_session] ✅ Obsidian 업데이트: {obsidian_file}")

    # 3. Notion 업로드 안내
    print(f"""
[log_session] 📋 Notion 업로드 방법:
  Claude Code에서 아래 메시지 전송:

  "SLAM 세션 로그를 Notion의 SLAM 스터디 페이지에 업로드해줘.
   로그 파일: logs/{session_id}/summary.md
   Notion page ID: {NOTION_SLAM_PAGE}"
""")


if __name__ == "__main__":
    main()
