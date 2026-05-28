#!/usr/bin/env python3
"""Generate Gazebo .world files for KKU pre-simulation map.

Source of truth: src/common_pkg/config/kku_pre_simulation_map.yaml
Outputs:         src/common_pkg/worlds/kku_f{1,2,3}.world
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml

WALL_HEIGHT = 2.4
WALL_THICKNESS = 0.12
CORRIDOR_HALF = 1.0
ELEV_HALF = 0.8
ELEV_DOOR_HALF = 0.5
ROOM_HALF_ALONG = 1.25
ROOM_DEPTH = 3.0
ROOM_DOOR_HALF = 0.45
PARCEL_DOOR_HALF = 0.45


@dataclass(frozen=True)
class Wall:
    name: str
    cx: float
    cy: float
    sx: float
    sy: float


def along_x(name: str, x0: float, x1: float, y: float) -> Wall:
    return Wall(name, (x0 + x1) / 2, y, abs(x1 - x0), WALL_THICKNESS)


def along_y(name: str, y0: float, y1: float, x: float) -> Wall:
    return Wall(name, x, (y0 + y1) / 2, WALL_THICKNESS, abs(y1 - y0))


def elevator_with_shoulders(prefix: str) -> list[Wall]:
    walls: list[Wall] = []
    walls.append(along_x(f"{prefix}_elev_n", -ELEV_HALF, ELEV_HALF, ELEV_HALF))
    walls.append(along_x(f"{prefix}_elev_s", -ELEV_HALF, ELEV_HALF, -ELEV_HALF))
    walls.append(along_y(f"{prefix}_elev_w", -ELEV_HALF, ELEV_HALF, -ELEV_HALF))
    walls.append(along_y(f"{prefix}_elev_e_s", -ELEV_HALF, -ELEV_DOOR_HALF, ELEV_HALF))
    walls.append(along_y(f"{prefix}_elev_e_n", ELEV_DOOR_HALF, ELEV_HALF, ELEV_HALF))
    walls.append(along_y(f"{prefix}_shoulder_n", ELEV_HALF, CORRIDOR_HALF, ELEV_HALF))
    walls.append(along_y(f"{prefix}_shoulder_s", -CORRIDOR_HALF, -ELEV_HALF, ELEV_HALF))
    return walls


def right_corridor(prefix: str, x_end: float, parcel_door_x: float | None) -> list[Wall]:
    walls: list[Wall] = []
    walls.append(along_x(f"{prefix}_rc_n_w", ELEV_HALF, 1.0, CORRIDOR_HALF))
    walls.append(along_x(f"{prefix}_rc_n_e", 3.0, x_end, CORRIDOR_HALF))
    if parcel_door_x is None:
        walls.append(along_x(f"{prefix}_rc_s", ELEV_HALF, x_end, -CORRIDOR_HALF))
    else:
        d0 = parcel_door_x - PARCEL_DOOR_HALF
        d1 = parcel_door_x + PARCEL_DOOR_HALF
        walls.append(along_x(f"{prefix}_rc_s_w", ELEV_HALF, d0, -CORRIDOR_HALF))
        walls.append(along_x(f"{prefix}_rc_s_e", d1, x_end, -CORRIDOR_HALF))
    walls.append(along_y(f"{prefix}_rc_end", -CORRIDOR_HALF, CORRIDOR_HALF, x_end))
    return walls


def back_corridor_form_only(prefix: str, y_end: float) -> list[Wall]:
    return [
        along_y(f"{prefix}_bc_w", CORRIDOR_HALF, y_end, 1.0),
        along_y(f"{prefix}_bc_e", CORRIDOR_HALF, y_end, 3.0),
        along_x(f"{prefix}_bc_end", 1.0, 3.0, y_end),
    ]


def back_corridor_with_rooms(prefix: str, y_end: float, rooms: list[dict]) -> list[Wall]:
    walls: list[Wall] = []
    west_door_ys = sorted(r["door_center"]["y"] for r in rooms if r["side"] == "west_wall")
    east_door_ys = sorted(r["door_center"]["y"] for r in rooms if r["side"] == "east_wall")

    def segments(door_ys: list[float]) -> list[tuple[float, float]]:
        breaks = [CORRIDOR_HALF]
        for dy in door_ys:
            breaks.append(dy - ROOM_DOOR_HALF)
            breaks.append(dy + ROOM_DOOR_HALF)
        breaks.append(y_end)
        return [(breaks[i], breaks[i + 1]) for i in range(0, len(breaks), 2)]

    for i, (y0, y1) in enumerate(segments(west_door_ys)):
        walls.append(along_y(f"{prefix}_bc_w_{i}", y0, y1, 1.0))
    for i, (y0, y1) in enumerate(segments(east_door_ys)):
        walls.append(along_y(f"{prefix}_bc_e_{i}", y0, y1, 3.0))
    walls.append(along_x(f"{prefix}_bc_end", 1.0, 3.0, y_end))

    for room in rooms:
        rid = room["id"]
        door_y = room["door_center"]["y"]
        y0 = door_y - ROOM_HALF_ALONG
        y1 = door_y + ROOM_HALF_ALONG
        if room["side"] == "west_wall":
            x_far = 1.0 - ROOM_DEPTH
            walls.append(along_x(f"{prefix}_r{rid}_s", x_far, 1.0, y0))
            walls.append(along_x(f"{prefix}_r{rid}_n", x_far, 1.0, y1))
            walls.append(along_y(f"{prefix}_r{rid}_far", y0, y1, x_far))
        else:
            x_far = 3.0 + ROOM_DEPTH
            walls.append(along_x(f"{prefix}_r{rid}_s", 3.0, x_far, y0))
            walls.append(along_x(f"{prefix}_r{rid}_n", 3.0, x_far, y1))
            walls.append(along_y(f"{prefix}_r{rid}_far", y0, y1, x_far))
    return walls


def parcel_alcove(prefix: str, pz: dict) -> list[Wall]:
    cx = pz["area"]["center"]["x"]
    sx = pz["area"]["size"]["x"]
    sy = pz["area"]["size"]["y"]
    x0 = cx - sx / 2
    x1 = cx + sx / 2
    y_north = -CORRIDOR_HALF
    y_south = y_north - sy
    return [
        along_x(f"{prefix}_pz_s", x0, x1, y_south),
        along_y(f"{prefix}_pz_e", y_south, y_north, x1),
        along_y(f"{prefix}_pz_w", y_south, y_north, x0),
    ]


def floor_walls(floor: dict) -> list[Wall]:
    fid = floor["id"].lower()
    corridors = {c["id"]: c for c in floor["corridors"]}
    right = corridors[f"{fid}_right_corridor"]
    back = corridors[f"{fid}_back_corridor"]
    x_end = right["centerline"]["end"]["x"]
    y_end = back["centerline"]["end"]["y"]
    pz = floor.get("parcel_zone")
    rooms = floor.get("rooms", [])

    parcel_door_x = pz["enclosure"]["door"]["center"]["x"] if pz else None
    walls = elevator_with_shoulders(fid)
    walls += right_corridor(fid, x_end, parcel_door_x)
    walls += back_corridor_with_rooms(fid, y_end, rooms) if rooms else back_corridor_form_only(fid, y_end)
    if pz:
        walls += parcel_alcove(fid, pz)
    return walls


def render_world(world_name: str, walls: list[Wall]) -> str:
    z = WALL_HEIGHT / 2
    lines: list[str] = [
        '<?xml version="1.0" ?>',
        '<sdf version="1.6">',
        f'  <world name="{world_name}">',
        '    <include><uri>model://sun</uri></include>',
        '    <include><uri>model://ground_plane</uri></include>',
        '    <physics name="default_physics" default="0" type="ode">',
        '      <max_step_size>0.001</max_step_size>',
        '      <real_time_factor>1.0</real_time_factor>',
        '      <real_time_update_rate>1000</real_time_update_rate>',
        '    </physics>',
        '    <scene>',
        '      <ambient>0.4 0.4 0.4 1</ambient>',
        '      <background>0.7 0.7 0.7 1</background>',
        '      <shadows>true</shadows>',
        '    </scene>',
        f'    <model name="{world_name}_building">',
        '      <static>true</static>',
        '      <link name="walls">',
    ]
    for w in walls:
        pose = f"{w.cx:.4g} {w.cy:.4g} {z:.4g} 0 0 0"
        size = f"{w.sx:.4g} {w.sy:.4g} {WALL_HEIGHT:.4g}"
        lines += [
            f'        <collision name="{w.name}">',
            f'          <pose>{pose}</pose>',
            f'          <geometry><box><size>{size}</size></box></geometry>',
            f'        </collision>',
            f'        <visual name="{w.name}_v">',
            f'          <pose>{pose}</pose>',
            f'          <geometry><box><size>{size}</size></box></geometry>',
            f'          <material><ambient>0.7 0.7 0.7 1</ambient><diffuse>0.85 0.85 0.85 1</diffuse></material>',
            f'        </visual>',
        ]
    lines += [
        '      </link>',
        '    </model>',
        '  </world>',
        '</sdf>',
        '',
    ]
    return '\n'.join(lines)


def main() -> None:
    root = Path(__file__).resolve().parent.parent
    yaml_path = root / "src/common_pkg/config/kku_pre_simulation_map.yaml"
    out_dir = root / "src/common_pkg/worlds"
    out_dir.mkdir(parents=True, exist_ok=True)

    data = yaml.safe_load(yaml_path.read_text())
    floors = {f["id"]: f for f in data["floors"]}

    for fid in ("F1", "F2", "F3"):
        floor = floors[fid]
        world_name = f"kku_{fid.lower()}"
        walls = floor_walls(floor)
        out = out_dir / f"{world_name}.world"
        out.write_text(render_world(world_name, walls))
        print(f"wrote {out.relative_to(root)}  ({len(walls)} walls)")


if __name__ == "__main__":
    main()
