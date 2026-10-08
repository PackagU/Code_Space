#!/usr/bin/env python3
"""Offline comparison of global-costmap inflation / planner cost weights (2026-09-15).

Path = optimal 8-connected grid path under Humble SmacPlanner2D traversal cost
(1 + cost_travel_multiplier * cost/252, x sqrt2 diagonal; nav2_smac_planner/src/node_2d.cpp humble),
on static-map inflation cost 252*exp(-k*(d - inscribed)) with inscribed = 0.033 m.
No obstacle layer, no smoother, no AMCL error: a relative comparison, not a Nav2 replay.
"""

import heapq
import json
import math
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import analyze_waypoints as aw  # noqa: E402

REPO = HERE.parents[1]
F2_NOTCH = (-12.95, -5.25)  # user-confirmed wall-hug pocket (crop of f2_recess_candidates.png)
CANDIDATES = {
    "baseline": {"inflation_radius": 0.55, "cost_scaling_factor": 3.0, "cost_travel_multiplier": 2.0},
    "push_A": {"inflation_radius": 1.0, "cost_scaling_factor": 2.0, "cost_travel_multiplier": 3.0},
    "push_B": {"inflation_radius": 1.0, "cost_scaling_factor": 1.5, "cost_travel_multiplier": 4.0},
}
ROUTES = {
    "F2": [("f2_delivery_destination", (-39.925, -32.625), "f2_elevator_staging_v1", (-11.175, -3.075))],
    "F1_v3": [
        ("f1_idle_v2_seed", (-4.518, 2.179), "f1_locker", (-5.125, -8.625)),
        ("f1_locker", (-5.125, -8.625), "f1_elevator_entry", (-8.725, -0.175)),
        ("f1_elevator_exit", (-8.725, -0.175), "f1_idle_v2_seed", (-4.518, 2.179)),
    ],
}


def distance_field(grid, cells_mask, max_m):
    k = int(math.ceil(max_m / grid.res))
    dist = np.full(cells_mask.shape, max_m, np.float32)
    dist[cells_mask] = 0.0
    padded = np.pad(cells_mask, k, constant_values=False)
    offsets = sorted(((dr, dc) for dr in range(-k, k + 1) for dc in range(-k, k + 1)
                      if 0 < math.hypot(dr, dc) * grid.res <= max_m), key=lambda o: math.hypot(*o))
    for dr, dc in offsets:
        shifted = padded[k + dr:k + dr + grid.h, k + dc:k + dc + grid.w]
        dist = np.where(shifted, np.minimum(dist, math.hypot(dr, dc) * grid.res), dist)
    return dist


def plan(grid, d_occ, start, goal, p):
    ins = aw.INSCRIBED
    cost = np.where(d_occ <= ins, 253.0,
                    np.where(d_occ > p["inflation_radius"], 0.0, 252.0 * np.exp(-p["cost_scaling_factor"] * (d_occ - ins))))
    valid = (grid.cls == aw.GridMap.FREE) & (cost < 253)
    s, g = grid.cell(*start), grid.cell(*goal)
    assert valid[s] and valid[g], (start, goal)
    best, parent, heap = {s: 0.0}, {}, [(0.0, s)]
    m = p["cost_travel_multiplier"]
    while heap:
        acc, node = heapq.heappop(heap)
        if node == g:
            break
        if acc > best[node]:
            continue
        r, c = node
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == dc == 0:
                    continue
                nr, nc = r + dr, c + dc
                if 0 <= nr < grid.h and 0 <= nc < grid.w and valid[nr, nc]:
                    step = (math.sqrt(2) if dr and dc else 1.0) * (1.0 + m * cost[nr, nc] / 252.0)
                    if acc + step < best.get((nr, nc), math.inf):
                        best[(nr, nc)] = acc + step
                        parent[(nr, nc)] = node
                        heapq.heappush(heap, (acc + step, (nr, nc)))
    path = [g]
    while path[-1] != s:
        path.append(parent[path[-1]])
    return path[::-1]


def metrics(grid, d_nonfree, path, notch=None):
    pts = [grid.world(r, c) for r, c in path]
    clear = [float(d_nonfree[r, c]) for r, c in path]
    length = sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))
    out = {
        "length_m": round(length, 2),
        "min_center_clearance_m": round(min(clear), 3),
        "path_cells_below_0.45m": sum(v < 0.45 for v in clear),
        "path_cells_below_0.35m": sum(v < 0.35 for v in clear),
        "mean_center_clearance_m": round(sum(clear) / len(clear), 3),
    }
    if notch:
        i = min(range(len(pts)), key=lambda j: math.dist(pts[j], notch))
        out["closest_to_notch_m"] = round(math.dist(pts[i], notch), 3)
        out["clearance_at_that_point_m"] = round(clear[i], 3)
        # Local minimum clearance within 3 m of the notch (corridor-to-lobby corner).
        near = [clear[j] for j in range(len(pts)) if math.dist(pts[j], notch) <= 3.0]
        out["min_clearance_within_3m_of_notch_m"] = round(min(near), 3) if near else None
    return out


def main():
    maps = {
        "F2": aw.GridMap(aw.MAPS["F2"]["yaml"], aw.MAPS["F2"]["pgm_sha256"]),
        "F1_v3": aw.GridMap(REPO / "artifacts/f1_manual_clean_20260915_v3/f1_manual_clean_v3.yaml",
                            aw.sha256(REPO / "artifacts/f1_manual_clean_20260915_v3/f1_manual_clean_v3.pgm")),
    }
    report = {"method": __doc__.strip().splitlines()[0], "candidates": CANDIDATES, "results": {}}
    for floor, grid in maps.items():
        d_occ = distance_field(grid, grid.cls == aw.GridMap.OCCUPIED, 1.3)
        d_nonfree = distance_field(grid, grid.cls != aw.GridMap.FREE, 1.3)
        drawn = {}
        for a_name, a, b_name, b in ROUTES[floor]:
            key = f"{a_name}->{b_name}"
            for cand, params in CANDIDATES.items():
                path = plan(grid, d_occ, a, b, params)
                report["results"].setdefault(floor, {}).setdefault(key, {})[cand] = metrics(
                    grid, d_nonfree, path, F2_NOTCH if floor == "F2" else None)
                drawn.setdefault(key, {})[cand] = path
        # overlay of the first route for each floor
        key = next(iter(drawn))
        base = np.full((grid.h, grid.w, 3), 205, np.uint8)
        base[grid.cls == aw.GridMap.FREE] = 255
        base[grid.cls == aw.GridMap.OCCUPIED] = 0
        img = Image.fromarray(base)
        d = ImageDraw.Draw(img)
        for cand, col in (("baseline", (230, 40, 40)), ("push_A", (0, 150, 0)), ("push_B", (0, 90, 255))):
            d.line([(c, r) for r, c in drawn[key][cand]], fill=col, width=1)
        center = F2_NOTCH if floor == "F2" else (-4.8, -3.3)
        half = 4.5 if floor == "F2" else 7.0
        r0, c0 = grid.cell(center[0] - half, center[1] + half)
        r1, c1 = grid.cell(center[0] + half, center[1] - half)
        crop = img.crop((max(c0, 0), max(r0, 0), min(c1, grid.w), min(r1, grid.h)))
        scale = 8 if floor == "F2" else 5
        crop = crop.resize((crop.width * scale, crop.height * scale), Image.NEAREST)
        if floor == "F2":
            nr, nc = grid.cell(*F2_NOTCH)
            x, y = (nc - max(c0, 0) + 0.5) * scale, (nr - max(r0, 0) + 0.5) * scale
            ImageDraw.Draw(crop).ellipse([x - 40, y - 40, x + 40, y + 40], outline=(255, 140, 0), width=3)
        crop.save(HERE / f"wall_push_paths_{floor}.png")
    out = HERE / "wall_push_evaluation.json"
    out.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8", newline="\n")
    for floor, routes in report["results"].items():
        for key, cands in routes.items():
            print(floor, key)
            for cand, m in cands.items():
                print("   ", cand, m)


if __name__ == "__main__":
    main()
