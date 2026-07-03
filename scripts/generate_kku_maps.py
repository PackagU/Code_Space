#!/usr/bin/env python3
"""Generate Nav2 occupancy maps (.pgm/.yaml) for the KKU pre-simulation floors.

벽 geometry(generate_kku_worlds 의 floor_walls)에서 직접 래스터화한다.
SLAM 주행이 필요 없고 드리프트가 없어 월드와 정확히 일치한다.
복도 폭(CORRIDOR_HALF) 등 월드 파라미터를 바꾸면 이 스크립트를 다시 돌려
저장맵을 월드와 동기화해야 AMCL/Nav2 localization 이 깨지지 않는다.

Source of truth: src/common_pkg/config/kku_pre_simulation_map.yaml
Outputs:         src/slam_pkg/maps/kku_virtual/f{1,2,3}/kku_f{1,2,3}.{pgm,yaml}

장애물(obstacle_*)은 의도적으로 맵에 넣지 않는다 -> Nav2 가 LiDAR 로 실시간 감지/회피.
"""
from __future__ import annotations

from collections import deque
from pathlib import Path

import yaml

import generate_kku_worlds as g

RES = 0.05          # m/cell (기존 맵과 동일)
PAD = 0.4           # 건물 외곽 여유 [m]
FREE, OCC, UNK = 254, 0, 205
# 내부 free 영역 flood fill 시드 (엘리베이터 내부, 모든 층 공통으로 빈 공간)
SEED_XY = (0.0, 0.0)


def rasterize(walls: list[g.Wall]) -> tuple[bytearray, int, int, float, float]:
    xs0 = [w.cx - w.sx / 2 for w in walls]
    xs1 = [w.cx + w.sx / 2 for w in walls]
    ys0 = [w.cy - w.sy / 2 for w in walls]
    ys1 = [w.cy + w.sy / 2 for w in walls]
    min_x, max_x = min(xs0) - PAD, max(xs1) + PAD
    min_y, max_y = min(ys0) - PAD, max(ys1) + PAD
    W = int(round((max_x - min_x) / RES))
    H = int(round((max_y - min_y) / RES))
    origin_x, origin_y = min_x, min_y

    def cell_center(c, r):
        # r: image row (0 = top = max y)
        x = origin_x + (c + 0.5) * RES
        y = origin_y + (H - 1 - r + 0.5) * RES
        return x, y

    # 1) 모두 unknown 으로 시작
    grid = bytearray([UNK]) * (W * H)

    # 2) 벽 cell -> occupied
    for w in walls:
        x0, x1 = w.cx - w.sx / 2, w.cx + w.sx / 2
        y0, y1 = w.cy - w.sy / 2, w.cy + w.sy / 2
        c0 = max(0, int((x0 - origin_x) / RES))
        c1 = min(W - 1, int((x1 - origin_x) / RES))
        r_lo = max(0, int((max_y - y1) / RES))
        r_hi = min(H - 1, int((max_y - y0) / RES))
        for r in range(r_lo, r_hi + 1):
            base = r * W
            for c in range(c0, c1 + 1):
                grid[base + c] = OCC

    # 3) 내부 free 영역 flood fill (4-연결, 벽을 넘지 못함 -> 외부는 unknown 유지)
    sx, sy = SEED_XY
    sc = int((sx - origin_x) / RES)
    sr = int((max_y - sy) / RES)
    if not (0 <= sc < W and 0 <= sr < H) or grid[sr * W + sc] == OCC:
        raise RuntimeError(f"seed {SEED_XY} is not in free space")
    q = deque([(sr, sc)])
    grid[sr * W + sc] = FREE
    while q:
        r, c = q.popleft()
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nr, nc = r + dr, c + dc
            if 0 <= nr < H and 0 <= nc < W:
                i = nr * W + nc
                if grid[i] == UNK:
                    grid[i] = FREE
                    q.append((nr, nc))
    return grid, W, H, origin_x, origin_y


def main() -> None:
    root = Path(__file__).resolve().parent.parent
    yaml_path = root / "src/common_pkg/config/kku_pre_simulation_map.yaml"
    out_root = root / "src/slam_pkg/maps/kku_virtual"
    data = yaml.safe_load(yaml_path.read_text())
    floors = {f["id"]: f for f in data["floors"]}

    for fid in ("F1", "F2", "F3"):
        walls = g.floor_walls(floors[fid])
        grid, W, H, ox, oy = rasterize(walls)
        fdir = fid.lower()
        out_dir = out_root / fdir
        out_dir.mkdir(parents=True, exist_ok=True)
        name = f"kku_{fdir}"
        pgm = out_dir / f"{name}.pgm"
        with pgm.open("wb") as fh:
            fh.write(f"P5\n{W} {H}\n255\n".encode("ascii"))
            fh.write(bytes(grid))
        yml = out_dir / f"{name}.yaml"
        yml.write_text(
            f"image: {name}.pgm\n"
            "mode: trinary\n"
            f"resolution: {RES}\n"
            f"origin: [{ox:.4g}, {oy:.4g}, 0]\n"
            "negate: 0\n"
            "occupied_thresh: 0.65\n"
            "free_thresh: 0.25\n"
        )
        free = grid.count(FREE)
        print(f"{name}: {W}x{H} origin=({ox:.3g},{oy:.3g}) free={free} occ={grid.count(OCC)}")


if __name__ == "__main__":
    main()
