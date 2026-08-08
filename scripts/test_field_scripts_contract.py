#!/usr/bin/env python3
"""실맵 저장/field 스크립트 계약 검사 (스펙 §5.4 '실맵 저장', 'field 원커맨드')."""
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
SAVE = ROOT / "scripts/save_kku_map.sh"
FIELD = ROOT / "scripts/run_field_mapping.sh"


def require(condition, message):
    if not condition:
        print(f"FAIL: {message}", file=sys.stderr)
        sys.exit(1)


def bash_syntax_ok(path):
    return subprocess.run(["bash", "-n", str(path)], capture_output=True).returncode == 0


def main():
    require(SAVE.exists(), "missing save_kku_map.sh")
    require(bash_syntax_ok(SAVE), "save_kku_map.sh syntax error")
    save_src = SAVE.read_text(encoding="utf-8")
    for needle in [
        "--real",                      # 실맵 모드 플래그
        "kku_real",                    # 실맵 저장 루트
        "_real",                       # kku_f?_real 네이밍
        "serialize_map",               # posegraph 저장 (slam_toolbox 서비스)
        "map_saver_cli",               # 기존 pgm/yaml 저장 유지
    ]:
        require(needle in save_src, f"save_kku_map.sh missing: {needle}")
    require("kku_virtual" in save_src, "virtual map path must remain (regression)")

    require(FIELD.exists(), "missing run_field_mapping.sh")
    require(bash_syntax_ok(FIELD), "run_field_mapping.sh syntax error")
    field_src = FIELD.read_text(encoding="utf-8")
    for needle in [
        "set -euo pipefail",
        "ros2 bag record",
        "/scan", "/odom", "/imu", "/cmd_vel", "/tf", "/tf_static",
        "enable_drive:=true",
        "rviz:=",                      # 관찰 옵션 env 게이트 (E2E 정책)
        "use_sim_time:=false",
        "save_kku_map.sh",             # 저장 경로 단일화 (--real 재사용)
        "trap",                        # cleanup 보장
    ]:
        require(needle in field_src, f"run_field_mapping.sh missing: {needle}")

    print("field scripts contract passed")


if __name__ == "__main__":
    main()
