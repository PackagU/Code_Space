#!/usr/bin/env python3
"""Offline waypoint/footprint/connectivity check for the 2026-09-15 field review.

No ROS, no robot. Reads PGM/YAML copies whose SHA256 matches the Jetson files and
approximates Nav2 costmap semantics from nav2_params.yaml (inflation 0.55 m,
cost_scaling_factor 3.0, footprint polygon). Planner path is a Dijkstra
approximation of SmacPlanner2D's cost weighting, not a Nav2 replay.
"""

import hashlib
import heapq
import json
import math
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
COLLECT = REPO / "logs/field_execution/20260915_autonomy_test_collection/extracted/ros2_ws/maps/field"

FOOTPRINT = [(0.033, 0.219), (0.033, -0.219), (-0.327, -0.219), (-0.327, 0.219)]
INFLATION_RADIUS = 0.55
COST_SCALING = 3.0
# Nav2 inscribed radius = min distance from base_footprint origin to a footprint edge.
INSCRIBED = min(abs(v) for xy in FOOTPRINT for v in xy)
CIRCUMSCRIBED = max(math.hypot(x, y) for x, y in FOOTPRINT)
HALF_WIDTH = 0.219
COST_TRAVEL_MULTIPLIER = 2.0

MAPS = {
    "F1_v2": {
        "yaml": COLLECT / "f1/f1_manual_clean_v2.yaml",
        "pgm_sha256": "3f9a40c42bbdb73933eac76cd187853b8502d2587e398b24b7537e4b8e3ab9d1",
    },
    "F2": {
        "yaml": COLLECT / "f2/f2_nav_unknown_v1.yaml",
        "pgm_sha256": "f89bfeb9f92811e8fa3349051c6e9e8ab3da57bb545089be7f6dd0dc396e4b31",
    },
}


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def parse_yaml(path):
    data = {}
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        if ":" in line:
            key, value = line.split(":", 1)
            data[key.strip()] = value.strip()
    origin = [float(v) for v in data["origin"].strip("[]").split(",")]
    return {
        "image": Path(path).parent / data["image"],
        "resolution": float(data["resolution"]),
        "origin": origin,
        "occupied_thresh": float(data["occupied_thresh"]),
        "free_thresh": float(data["free_thresh"]),
        "negate": int(data["negate"]),
    }


class GridMap:
    FREE, UNKNOWN, OCCUPIED = 0, 1, 2

    def __init__(self, yaml_path, expected_sha):
        meta = parse_yaml(yaml_path)
        actual = sha256(meta["image"])
        if actual != expected_sha:
            raise SystemExit(f"PGM SHA256 mismatch for {meta['image']}: {actual}")
        self.meta = meta
        self.pgm_sha256 = actual
        self.yaml_sha256 = sha256(yaml_path)
        pixels = np.asarray(Image.open(meta["image"]), dtype=np.float64)
        occ = (pixels if meta["negate"] else 255.0 - pixels) / 255.0
        cls = np.full(pixels.shape, self.UNKNOWN, dtype=np.uint8)
        cls[occ > meta["occupied_thresh"]] = self.OCCUPIED
        cls[occ < meta["free_thresh"]] = self.FREE
        self.cls = cls
        self.h, self.w = cls.shape
        self.res = meta["resolution"]
        self.ox, self.oy = meta["origin"][:2]

    def cell(self, x, y):
        col = int(math.floor((x - self.ox) / self.res))
        row = self.h - 1 - int(math.floor((y - self.oy) / self.res))
        return row, col

    def world(self, row, col):
        return self.ox + (col + 0.5) * self.res, self.oy + (self.h - 1 - row + 0.5) * self.res

    def window(self, x, y, radius):
        row, col = self.cell(x, y)
        k = int(math.ceil(radius / self.res)) + 1
        r0, r1 = max(0, row - k), min(self.h, row + k + 1)
        c0, c1 = max(0, col - k), min(self.w, col + k + 1)
        rows, cols = np.mgrid[r0:r1, c0:c1]
        wx = self.ox + (cols + 0.5) * self.res
        wy = self.oy + (self.h - 1 - rows + 0.5) * self.res
        return self.cls[r0:r1, c0:c1], wx, wy


def inflation_cost(distance):
    if distance <= INSCRIBED:
        return 253
    if distance > INFLATION_RADIUS:
        return 0
    return int(252 * math.exp(-COST_SCALING * (distance - INSCRIBED)))


def footprint_world(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + px * c - py * s, y + px * s + py * c) for px, py in FOOTPRINT]


def point_polygon_distance(px, py, polygon):
    """Vectorized distance from points to polygon boundary; 0 when inside."""
    inside = np.zeros(px.shape, dtype=bool)
    best = np.full(px.shape, np.inf)
    n = len(polygon)
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        dx, dy = x2 - x1, y2 - y1
        t = np.clip(((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy), 0.0, 1.0)
        best = np.minimum(best, np.hypot(px - (x1 + t * dx), py - (y1 + t * dy)))
        crosses = ((y1 > py) != (y2 > py)) & (px < (x2 - x1) * (py - y1) / (dy if dy else 1e-12) + x1)
        inside ^= crosses
    best[inside] = 0.0
    return best


def pose_report(grid, name, x, y, yaw):
    row, col = grid.cell(x, y)
    cls, wx, wy = grid.window(x, y, 2.5)
    d = np.hypot(wx - x, wy - y)
    occ = cls == GridMap.OCCUPIED
    nonfree = cls != GridMap.FREE
    d_occ = float(d[occ].min()) if occ.any() else math.inf
    d_nonfree = float(d[nonfree].min()) if nonfree.any() else math.inf
    polygon = footprint_world(x, y, yaw)
    poly_d = point_polygon_distance(wx, wy, polygon)
    fp_occ = float(poly_d[occ].min()) if occ.any() else math.inf
    fp_nonfree = float(poly_d[nonfree].min()) if nonfree.any() else math.inf
    inside = poly_d == 0.0
    names = {0: "free", 1: "unknown", 2: "occupied"}
    return {
        "name": name,
        "pose": [x, y, yaw],
        "cell_row_col": [row, col],
        "center_class": names[int(grid.cls[row, col])],
        "center_to_occupied_m": round(d_occ, 3),
        "center_to_nonfree_m": round(d_nonfree, 3),
        "center_inflation_cost_static": inflation_cost(d_occ),
        "center_start_lethal_static": bool(grid.cls[row, col] != GridMap.FREE or inflation_cost(d_occ) >= 253),
        "footprint_edge_to_occupied_m": round(fp_occ, 3),
        "footprint_edge_to_nonfree_m": round(fp_nonfree, 3),
        "footprint_cells_occupied": int((inside & occ).sum()),
        "footprint_cells_unknown": int((inside & (cls == GridMap.UNKNOWN)).sum()),
    }


def nonfree_distance_field(grid, max_m):
    """Distance (m) from each cell to nearest non-free cell, capped at max_m."""
    k = int(math.ceil(max_m / grid.res))
    blocked = grid.cls != GridMap.FREE
    dist = np.full(blocked.shape, max_m, dtype=np.float32)
    dist[blocked] = 0.0
    offsets = sorted(
        ((dr, dc) for dr in range(-k, k + 1) for dc in range(-k, k + 1) if 0 < math.hypot(dr, dc) * grid.res <= max_m),
        key=lambda o: math.hypot(*o),
    )
    padded = np.pad(blocked, k, constant_values=True)
    for dr, dc in offsets:
        shifted = padded[k + dr: k + dr + grid.h, k + dc: k + dc + grid.w]
        dist = np.where(shifted, np.minimum(dist, math.hypot(dr, dc) * grid.res), dist)
    return dist


def occupied_distance_field(grid, max_m):
    k = int(math.ceil(max_m / grid.res))
    occ = grid.cls == GridMap.OCCUPIED
    dist = np.full(occ.shape, max_m, dtype=np.float32)
    dist[occ] = 0.0
    padded = np.pad(occ, k, constant_values=False)
    for dr in range(-k, k + 1):
        for dc in range(-k, k + 1):
            r = math.hypot(dr, dc) * grid.res
            if 0 < r <= max_m:
                shifted = padded[k + dr: k + dr + grid.h, k + dc: k + dc + grid.w]
                dist = np.where(shifted, np.minimum(dist, r), dist)
    return dist


def cost_path(grid, d_occ, start, goal):
    """Dijkstra on free, non-inscribed cells with Smac2D-like cost weighting (approximation)."""
    cost = np.where(
        d_occ <= INSCRIBED, 253, np.where(d_occ > INFLATION_RADIUS, 0, 252 * np.exp(-COST_SCALING * (d_occ - INSCRIBED)))
    )
    valid = (grid.cls == GridMap.FREE) & (cost < 253)
    s, g = grid.cell(*start), grid.cell(*goal)
    if not valid[s] or not valid[g]:
        return None, {"start_valid": bool(valid[s]), "goal_valid": bool(valid[g])}
    best = {s: 0.0}
    parent = {}
    heap = [(0.0, s)]
    steps = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0), (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)]
    while heap:
        acc, node = heapq.heappop(heap)
        if node == g:
            break
        if acc > best.get(node, math.inf):
            continue
        r, c = node
        for dr, dc, length in steps:
            nr, nc = r + dr, c + dc
            if 0 <= nr < grid.h and 0 <= nc < grid.w and valid[nr, nc]:
                value = acc + length * (1.0 + COST_TRAVEL_MULTIPLIER * cost[nr, nc] / 252.0)
                if value < best.get((nr, nc), math.inf):
                    best[(nr, nc)] = value
                    parent[(nr, nc)] = node
                    heapq.heappush(heap, (value, (nr, nc)))
    if g not in parent and g != s:
        return None, {"start_valid": True, "goal_valid": True, "connected": False}
    path = [g]
    while path[-1] != s:
        path.append(parent[path[-1]])
    path.reverse()
    return path, {"start_valid": True, "goal_valid": True, "connected": True}


def reachable(grid, clearance_field, start, goal, clearance):
    mask = clearance_field >= clearance
    s, g = grid.cell(*start), grid.cell(*goal)
    if not mask[s] or not mask[g]:
        return {"clearance_m": clearance, "start_ok": bool(mask[s]), "goal_ok": bool(mask[g]), "connected": False}
    seen = np.zeros(mask.shape, dtype=bool)
    seen[s] = True
    frontier = seen.copy()
    while frontier.any() and not seen[g]:
        grown = np.zeros_like(frontier)
        grown[1:, :] |= frontier[:-1, :]
        grown[:-1, :] |= frontier[1:, :]
        grown[:, 1:] |= frontier[:, :-1]
        grown[:, :-1] |= frontier[:, 1:]
        frontier = grown & mask & ~seen
        seen |= frontier
    return {"clearance_m": clearance, "start_ok": True, "goal_ok": True, "connected": bool(seen[g])}


def path_profile(grid, d_nonfree, path, near, radius):
    rows = []
    for r, c in path:
        x, y = grid.world(r, c)
        if near is None or math.hypot(x - near[0], y - near[1]) <= radius:
            rows.append((x, y, float(d_nonfree[r, c])))
    if not rows:
        return {}
    worst = min(rows, key=lambda item: item[2])
    return {
        "cells": len(rows),
        "min_center_clearance_m": round(worst[2], 3),
        "min_at_xy": [round(worst[0], 3), round(worst[1], 3)],
        "cells_below_0.39m": sum(1 for row in rows if row[2] < CIRCUMSCRIBED),
        "cells_below_0.30m": sum(1 for row in rows if row[2] < 0.30),
    }


def overlay(grid, path, poses, crop_center, crop_m, out, scale=4, glass=None):
    base = np.full((grid.h, grid.w, 3), 205, dtype=np.uint8)
    base[grid.cls == GridMap.FREE] = (255, 255, 255)
    base[grid.cls == GridMap.OCCUPIED] = (0, 0, 0)
    image = Image.fromarray(base)
    draw = ImageDraw.Draw(image)
    if path:
        draw.line([(c, r) for r, c in path], fill=(0, 140, 255), width=1)
    for label, (x, y, yaw), color in poses:
        polygon = footprint_world(x, y, yaw)
        pts = []
        for px, py in polygon:
            r, c = grid.cell(px, py)
            pts.append((c, r))
        draw.polygon(pts, outline=color)
        r, c = grid.cell(x, y)
        draw.ellipse([c - 1, r - 1, c + 1, r + 1], fill=color)
        hr, hc = grid.cell(x + 0.4 * math.cos(yaw), y + 0.4 * math.sin(yaw))
        draw.line([(c, r), (hc, hr)], fill=color)
    r, c = grid.cell(*crop_center)
    k = int(crop_m / grid.res / 2)
    image = image.crop((max(0, c - k), max(0, r - k), min(grid.w, c + k), min(grid.h, r + k)))
    image = image.resize((image.width * scale, image.height * scale), Image.NEAREST)
    image.save(out)
    return {"file": out.name, "sha256": sha256(out)}


def main():
    report = {
        "status": "offline verification (map copy + Nav2 param approximation); no ROS, no robot",
        "footprint": FOOTPRINT,
        "nav2_inscribed_radius_m": INSCRIBED,
        "circumscribed_radius_m": round(CIRCUMSCRIBED, 4),
        "inflation": {"radius_m": INFLATION_RADIUS, "cost_scaling_factor": COST_SCALING},
    }

    f2 = GridMap(MAPS["F2"]["yaml"], MAPS["F2"]["pgm_sha256"])
    f2_poses = {
        "f2_elevator_entry": (-11.375, -2.525, 2.312744),
        "f2_elevator_staging_v1": (-11.175, -3.075, 2.312744),
        "f2_delivery_destination": (-39.925, -32.625, -2.33),
    }
    f2_rep = {"map_yaml_sha256": f2.yaml_sha256, "map_pgm_sha256": f2.pgm_sha256, "poses": {}}
    for name, pose in f2_poses.items():
        f2_rep["poses"][name] = pose_report(f2, name, *pose)
    d_nonfree = nonfree_distance_field(f2, 1.2)
    d_occ = occupied_distance_field(f2, 0.6)
    start = f2_poses["f2_delivery_destination"][:2]
    f2_rep["connectivity"] = {}
    for goal_name in ("f2_elevator_entry", "f2_elevator_staging_v1"):
        goal = f2_poses[goal_name][:2]
        f2_rep["connectivity"][goal_name] = [reachable(f2, d_nonfree, start, goal, c) for c in (HALF_WIDTH, CIRCUMSCRIBED)]
    paths = {}
    for goal_name in ("f2_elevator_entry", "f2_elevator_staging_v1"):
        path, status = cost_path(f2, d_occ, start, f2_poses[goal_name][:2])
        paths[goal_name] = path
        entry = {"status": status}
        if path:
            entry["whole_path"] = path_profile(f2, d_nonfree, path, None, 0)
            entry["last_3m_before_goal"] = path_profile(f2, d_nonfree, path, f2_poses[goal_name][:2], 3.0)
        f2_rep.setdefault("approx_planner_path", {})[goal_name] = entry
    f2_rep["overlay"] = overlay(
        f2,
        paths["f2_elevator_staging_v1"],
        [
            ("entry", f2_poses["f2_elevator_entry"], (220, 0, 0)),
            ("staging", f2_poses["f2_elevator_staging_v1"], (0, 160, 0)),
        ],
        (-11.4, -3.0),
        7.0,
        HERE / "f2_staging_overlay.png",
        scale=6,
    )
    report["F2"] = f2_rep

    f1 = GridMap(MAPS["F1_v2"]["yaml"], MAPS["F1_v2"]["pgm_sha256"])
    f1_poses = {
        "f1_idle": (-3.725, 2.075, -1.701),
        "f1_locker": (-5.125, -8.625, -1.69),
        "f1_elevator_exit": (-8.725, -0.175, 0.918927),
        "f1_elevator_entry": (-8.725, -0.175, -2.222665),
    }
    f1_rep = {"map_yaml_sha256": f1.yaml_sha256, "map_pgm_sha256": f1.pgm_sha256, "poses": {}}
    for name, pose in f1_poses.items():
        f1_rep["poses"][name] = pose_report(f1, name, *pose)
    # Distance from current idle toward the robot's right side (yaw - 90 deg) until the first occupied cell.
    x, y, yaw = f1_poses["f1_idle"]
    right = yaw - math.pi / 2
    ray = None
    for step in range(1, 200):
        px, py = x + step * 0.01 * math.cos(right), y + step * 0.01 * math.sin(right)
        r, c = f1.cell(px, py)
        if f1.cls[r, c] != GridMap.FREE:
            ray = {"first_nonfree_m": round(step * 0.01, 2), "at_xy": [round(px, 3), round(py, 3)], "class": int(f1.cls[r, c])}
            break
    f1_rep["idle_right_side_ray"] = ray
    # Field placement hint only (user: real idle is further right, near the wall). Not a waypoint.
    f1_rep["idle_v2_seed_hints"] = {}
    for shift in (0.8, 1.0):
        sx, sy = x + shift * math.cos(right), y + shift * math.sin(right)
        f1_rep["idle_v2_seed_hints"][f"right_{shift:.1f}m"] = pose_report(f1, f"idle_seed_right_{shift:.1f}m", round(sx, 3), round(sy, 3), yaw)
    f1_d_nonfree = nonfree_distance_field(f1, 1.2)
    f1_d_occ = occupied_distance_field(f1, 0.6)
    f1_rep["connectivity_idle_to_locker"] = [
        reachable(f1, f1_d_nonfree, f1_poses["f1_idle"][:2], f1_poses["f1_locker"][:2], c) for c in (HALF_WIDTH, CIRCUMSCRIBED)
    ]
    path, status = cost_path(f1, f1_d_occ, f1_poses["f1_idle"][:2], f1_poses["f1_locker"][:2])
    f1_rep["approx_planner_path_idle_to_locker"] = {"status": status}
    if path:
        f1_rep["approx_planner_path_idle_to_locker"]["whole_path"] = path_profile(f1, f1_d_nonfree, path, None, 0)
        # Narrowest free width across the glass-door neck along the approximate path.
        f1_rep["approx_planner_path_idle_to_locker"]["narrowest_point"] = path_profile(f1, f1_d_nonfree, path, None, 0)
    f1_rep["overlay"] = overlay(
        f1,
        path,
        [
            ("idle", f1_poses["f1_idle"], (220, 0, 0)),
            ("locker", f1_poses["f1_locker"], (0, 160, 0)),
        ],
        (-4.8, -3.3),
        14.0,
        HERE / "f1_v2_idle_locker_overlay.png",
        scale=4,
    )
    report["F1_v2"] = f1_rep

    out = HERE / "waypoint_offline_check.json"
    out.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8", newline="\n")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
