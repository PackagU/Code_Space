#!/usr/bin/env python3
"""F1 manual map v3: v2 outline snapped to right angles, square elevator cabin, map_saver-style values.

Manual reconstruction, not a SLAM result and not field validated.
- Grid angle 21.75 deg = dominant wall direction of the raw F1 SLAM map (whole-map and hall-region
  histogram peak). Every outline edge is snapped to that grid.
- Each snapped edge keeps v2's position at its midpoint, except the wall on the robot's right at idle,
  which keeps v2's position next to the idle point (user confirmed the 0.8 m idle seed against it).
- Elevator cabin: user said it is a plain rectangle. The v2 door gap (u 234..256) is kept.
- Pixel values follow map_saver (free 254, occupied 0, unknown 205); the YAML uses free_thresh 0.19
  like F2 so 205 stays unknown. No artificial scan noise is added.
"""

import hashlib
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

HERE = Path(__file__).resolve().parent
V2 = HERE.parent / "f1_manual_clean_20260914_v2"
NAME = "f1_manual_clean_v3"
W, H, RES, ORIGIN = 512, 423, 0.05, (-18.5, -11.6)
THETA = math.radians(21.75)
C, S = math.cos(THETA), math.sin(THETA)
WALL_HALF_WIDTH_PX = 0.8

# Rectilinear outline in (u, v) raw-pixel grid coordinates, v2 vertex ids in comments.
OUTLINE_UV = [
    (298.8, -118.5),  # v00 top-left; x kept at v2 wall position beside idle (v18 side)
    (477.4, -118.5),  # v01
    (477.4, 204.5),   # v02 right wall snapped to midpoint u
    (410.0, 204.5),   # v03 neck right; neck line unified at v=204.5 (v2 202.6..206.0)
    (410.0, 268.5),   # v04 locker alcove right wall
    (357.3, 268.5),   # v05 (edge v04->v05 is the open entrance)
    (357.3, 204.5),   # v06
    (305.2, 204.5),   # v07
    (305.2, 129.0),   # v08 hall wall next to the elevator (v2 129..137)
    (256.0, 129.0),   # v09 elevator door right edge
    (256.0, 169.0),   # cabin right wall (replaces v10/v11 notch and slanted v11->v12)
    (219.0, 169.0),   # cabin far wall
    (219.0, 129.0),   # cabin left wall
    (207.9, 129.0),   # v15 hall wall left of the elevator (v2 120..129)
    (207.9, 40.0),    # v16 left wall snapped to midpoint u
    (298.8, 40.0),    # v17/v18 corner; v kept near the corner beside idle
]
OPEN_EDGES = {4}  # index of the edge starting at OUTLINE_UV[i]
INNER_WALLS_UV = [
    [(219.0, 129.0), (234.0, 129.0)],  # cabin wall left of the door gap (hall on the other side)
    [(357.3, 204.5), (383.6, 204.5)],  # locked half of the glass fixed door (v2 rule)
]
CHECKPOINTS_UV = {"main_hall": (400.0, 60.0), "elevator_interior": (240.0, 150.0), "locker_alcove": (383.6, 240.0)}


def to_raw(u, v):
    return u * C - v * S, u * S + v * C


def inside(x, y, poly):
    hit = False
    for i in range(len(poly)):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % len(poly)]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            hit = not hit
    return hit


def seg_dist(px, py, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    t = np.clip(((px - a[0]) * dx + (py - a[1]) * dy) / (dx * dx + dy * dy), 0.0, 1.0)
    return np.hypot(px - a[0] - t * dx, py - a[1] - t * dy)


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    outline = [to_raw(*p) for p in OUTLINE_UV]
    walls = [(outline[i], outline[(i + 1) % len(outline)]) for i in range(len(outline)) if i not in OPEN_EDGES]
    walls += [tuple(to_raw(*p) for p in seg) for seg in INNER_WALLS_UV]
    ys, xs = np.mgrid[0:H, 0:W]
    px, py = xs + 0.5, ys + 0.5  # cell centres (v2 used integer corners; half-cell shift is below wall width)
    grid = np.full((H, W), 205, np.uint8)
    mask = np.zeros((H, W), bool)
    flat_x, flat_y = px.ravel(), py.ravel()
    inside_flags = np.fromiter((inside(x, y, outline) for x, y in zip(flat_x, flat_y)), bool, count=W * H)
    grid.ravel()[inside_flags] = 254
    for a, b in walls:
        mask |= seg_dist(px, py, a, b) <= WALL_HALF_WIDTH_PX
    grid[mask] = 0

    occ = (255 - grid.astype(float)) / 255
    classes = {"occupied": int((occ > 0.65).sum()), "free": int((occ < 0.19).sum())}
    classes["unknown"] = W * H - classes["occupied"] - classes["free"]

    # 4-connected flood fill over free cells from the main hall.
    free = occ < 0.19
    start = tuple(int(round(t)) for t in to_raw(*CHECKPOINTS_UV["main_hall"]))
    seen = np.zeros_like(free)
    seen[start[1], start[0]] = True
    frontier = seen.copy()
    while frontier.any():
        grown = np.zeros_like(frontier)
        grown[1:, :] |= frontier[:-1, :]
        grown[:-1, :] |= frontier[1:, :]
        grown[:, 1:] |= frontier[:, :-1]
        grown[:, :-1] |= frontier[:, 1:]
        frontier = grown & free & ~seen
        seen |= frontier
    connectivity = {}
    for key, uv in CHECKPOINTS_UV.items():
        x, y = (int(round(t)) for t in to_raw(*uv))
        connectivity[key] = bool(seen[y, x])
    if not all(connectivity.values()):
        raise SystemExit(f"disconnected regions: {connectivity}")

    header = b"P5\n# F1 manual reconstruction v3 (right-angle outline). NOT field validated.\n512 423\n255\n"
    (HERE / f"{NAME}.pgm").write_bytes(header + grid.tobytes())
    yaml = f"image: {NAME}.pgm\nmode: trinary\nresolution: 0.05\norigin: [-18.5, -11.6, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.19\n"
    (HERE / f"{NAME}.yaml").write_text(yaml, encoding="utf-8", newline="\n")
    Image.fromarray(grid).save(HERE / f"{NAME}.png")

    v2 = np.asarray(Image.open(V2 / "f1_manual_clean_v2.pgm"))
    cmp_img = np.full((H, W, 3), 255, np.uint8)
    cmp_img[v2 == 127] = (215, 215, 215)
    cmp_img[grid == 205] = np.minimum(cmp_img[grid == 205], 200)
    cmp_img[v2 == 0] = (230, 60, 60)
    cmp_img[grid == 0] = (0, 0, 0)
    both = (v2 == 0) & (grid == 0)
    cmp_img[both] = (120, 0, 120)
    Image.fromarray(cmp_img).resize((W * 3, H * 3), Image.NEAREST).save(HERE / "v2_red_vs_v3_black.png")

    geometry = {
        "status": "manual reconstruction v3; not SLAM, not field validated",
        "grid_angle_deg": 21.75,
        "grid_angle_basis": "dominant occupied-cell direction of raw f1_raw_20260914.pgm (whole map 21.75, hall region 21.5-22.0)",
        "uv_to_raw": "x = u*cos(a) - v*sin(a), y = u*sin(a) + v*cos(a), raw pixel units, image y down",
        "outline_uv": OUTLINE_UV,
        "outline_raw": [[round(x, 3), round(y, 3)] for x, y in outline],
        "open_edge_indices": sorted(OPEN_EDGES),
        "inner_walls_uv": INNER_WALLS_UV,
        "wall_half_width_px": WALL_HALF_WIDTH_PX,
        "policies": {
            "snap": "edge midpoint kept, except the idle-side wall (u=298.8) and corner (v=40.0) kept at v2 position beside f1_idle",
            "elevator": "rectangle u 219..256, v 129..169 (1.85 m x 2.0 m); door gap u 234..256 from v2",
            "glass_door": "locked half u 357.3..383.6 on the neck line v=204.5; open half u 383.6..410.0",
            "values": "map_saver style 254/0/205 with free_thresh 0.19",
        },
    }
    (HERE / "geometry_v3.json").write_text(json.dumps(geometry, ensure_ascii=False, indent=2) + "\n", encoding="utf-8", newline="\n")
    manifest = {
        "outputs": {f: sha256(HERE / f) for f in (f"{NAME}.pgm", f"{NAME}.yaml", f"{NAME}.png", "geometry_v3.json", "v2_red_vs_v3_black.png")},
        "inputs": {"v2_pgm": sha256(V2 / "f1_manual_clean_v2.pgm"), "raw_pgm": sha256(V2 / "source/f1_raw_20260914.pgm")},
        "classes": classes,
        "connectivity_cells_only": connectivity,
        "changed_cells_vs_v2_occupied": int(((v2 == 0) != (grid == 0)).sum()),
    }
    (HERE / "manifest_v3.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8", newline="\n")
    print(json.dumps(manifest, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
