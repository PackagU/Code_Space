#!/usr/bin/env python3
"""로봇팔 원커맨드 벤치 실행기의 하드웨어 없는 회귀 테스트."""

import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RUNNER = ROOT / "scripts" / "run_arm_press.py"


def invoke(*args):
    return subprocess.run(
        [sys.executable, str(RUNNER), *args],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=False,
    )


def main():
    for cycle_id in (1, 2, 3):
        result = invoke(str(cycle_id), "--dry-run", "--countdown", "0")
        assert result.returncode == 0, result.stderr
        assert f"press cycle #{cycle_id}" in result.stdout
        assert result.stdout.count("[dry-run]") == 7, "homing 1회 + 자세 명령 6회여야 함"
        assert "사이클 완료 — home 대기 자세" in result.stdout

    invalid = invoke("4", "--dry-run", "--countdown", "0")
    assert invalid.returncode != 0, "없는 사이클 번호는 실패해야 함"
    assert "알 수 없는 press_cycle" in invalid.stderr

    print("PASS arm press runner (cycles 1~3 dry-run + invalid cycle guard)")


if __name__ == "__main__":
    main()
