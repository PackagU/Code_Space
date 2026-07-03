#!/usr/bin/env python3
"""이식성 가드 — Jetson/컨테이너 이전을 막는 하드코딩을 잡는다.

정책 (docs/deployment/01_portability_policy.md 의 실행 가능한 요약):
  R1  /home/... 절대경로 금지         — src/, scripts/, test_workspace/ 전체
  R2  /ros2_ws 기능적 사용 금지        — src/ 한정 (배포 대상 패키지는
      컨테이너 레이아웃을 가정하지 않는다; 주석은 허용)
  R3  /dev/tty* 하드코딩 금지          — src/ launch 파일은 반드시
      DeclareLaunchArgument 로 포트를 인자화한다
  R4  map_file_name 절대경로 금지      — src/ yaml (localization 시
      launch 파라미터로 override)

test_workspace 러너/스크립트는 컨테이너 전용 도구라 ${ROOT:-/ros2_ws}
패턴을 허용한다. host 래퍼(scripts/*.sh)의 docker exec 내부 경로도 허용.

사용: python3 scripts/check_portability.py   (위반 시 exit 1)
"""
from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
CODE_SUFFIXES = {".py", ".sh", ".yaml", ".yml", ".xml"}
SCAN_PREFIXES = ("src/", "scripts/", "test_workspace/")


def tracked_files() -> list[Path]:
    out = subprocess.run(
        ["git", "ls-files", "--cached", "--others", "--exclude-standard", *SCAN_PREFIXES],
        cwd=REPO_ROOT, capture_output=True, text=True, check=True,
    ).stdout
    return [
        REPO_ROOT / line
        for line in out.splitlines()
        if Path(line).suffix in CODE_SUFFIXES
    ]


def strip_comment(line: str) -> str:
    """# 이후를 제거 (따옴표 안 # 은 드물어 단순 처리로 충분)."""
    return line.split("#", 1)[0]


def check_file(path: Path) -> list[str]:
    violations = []
    rel = path.relative_to(REPO_ROOT).as_posix()
    try:
        text = path.read_text(encoding="utf-8")
    except (UnicodeDecodeError, FileNotFoundError):
        return violations
    in_src = rel.startswith("src/")

    for lineno, raw in enumerate(text.splitlines(), start=1):
        code = strip_comment(raw)

        # R1: 사용자 홈 절대경로는 어디서든 금지
        if re.search(r"/home/\w+", code):
            violations.append(f"{rel}:{lineno}: R1 /home/ 절대경로 금지: {raw.strip()}")

        if in_src:
            # R2: 배포 패키지는 컨테이너 경로를 가정하지 않는다
            if "/ros2_ws" in code:
                violations.append(
                    f"{rel}:{lineno}: R2 src/ 에서 /ros2_ws 기능적 사용 금지: {raw.strip()}")
            # R4: yaml 의 map_file_name 절대경로
            if path.suffix in (".yaml", ".yml") and re.match(
                    r"\s*map_file_name:\s*/", code):
                violations.append(
                    f"{rel}:{lineno}: R4 map_file_name 절대경로 금지 (launch override 사용): {raw.strip()}")

    # R3: src/ launch 파일의 시리얼 포트는 인자화 필수
    if in_src and path.suffix == ".py" and "launch" in rel and "/dev/tty" in text:
        if "DeclareLaunchArgument" not in text or "serial_port" not in text:
            violations.append(
                f"{rel}: R3 /dev/tty* 는 DeclareLaunchArgument(serial_port) 로 인자화할 것")

    return violations


def main() -> int:
    all_violations: list[str] = []
    files = tracked_files()
    for path in files:
        all_violations.extend(check_file(path))

    if all_violations:
        print(f"FAIL portability check — {len(all_violations)} violation(s):")
        for v in all_violations:
            print(f"  {v}")
        return 1
    print(f"PASS portability check ({len(files)} files scanned)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
