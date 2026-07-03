#!/usr/bin/env python3
"""층별 맵 기대값의 단일 소스 (single source of truth).

verifier / 계약 테스트가 맵 크기·원점을 하드코딩하지 않도록,
커밋된 맵 yaml + pgm 헤더에서 기대값을 직접 읽는다.
맵을 재생성하면 기대값도 자동으로 따라온다.

(복도 폭 회귀 가드 CORRIDOR_HALF=2.5 는 의도된 별도 상수로,
 test_smoke_scripts_contract.py 에 남아 있다.)

호스트/컨테이너 공용: 이 파일 기준 상대 경로로 repo root 를 찾는다
(호스트 = repo 체크아웃, 컨테이너 = /ros2_ws 마운트).
"""
from __future__ import annotations

from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[3]
MAPS_ROOT = REPO_ROOT / "src" / "slam_pkg" / "maps" / "kku_virtual"

KNOWN_FLOORS = ("F1", "F2", "F3")


def map_paths(floor: str) -> tuple[Path, Path]:
    """floor("F1"|"F2"|"F3") -> (yaml_path, pgm_path)."""
    key = floor.lower()
    floor_dir = MAPS_ROOT / key
    return floor_dir / f"kku_{key}.yaml", floor_dir / f"kku_{key}.pgm"


def _read_pgm_size(pgm_path: Path) -> tuple[int, int]:
    with pgm_path.open("rb") as fh:
        magic = fh.readline().strip()
        if magic not in (b"P2", b"P5"):
            raise ValueError(f"{pgm_path}: not a PGM header ({magic!r})")
        line = fh.readline()
        while line.startswith(b"#"):
            line = fh.readline()
        width, height = (int(v) for v in line.split()[:2])
    return width, height


def load_expectation(floor: str) -> dict:
    """맵 yaml/pgm 에서 {width, height, origin_x, origin_y, resolution} 반환."""
    yaml_path, pgm_path = map_paths(floor)
    if not yaml_path.exists():
        raise FileNotFoundError(
            f"{yaml_path} 없음 — scripts/generate_kku_maps.py 를 먼저 실행할 것")
    if not pgm_path.exists():
        raise FileNotFoundError(
            f"{pgm_path} 없음 (pgm 은 gitignore) — scripts/generate_kku_maps.py 로 생성할 것")
    meta = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    origin = meta["origin"]
    width, height = _read_pgm_size(pgm_path)
    return {
        "width": width,
        "height": height,
        "origin_x": float(origin[0]),
        "origin_y": float(origin[1]),
        "resolution": float(meta["resolution"]),
    }


if __name__ == "__main__":
    for name in KNOWN_FLOORS:
        try:
            print(name, load_expectation(name))
        except FileNotFoundError as exc:
            print(name, f"MISSING: {exc}")
