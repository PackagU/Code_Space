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
CORRIDOR_HALF = 2.5  # 메인(우측) 복도 반폭 (총 폭 5.0m). 사람+로봇이 여유있게 교행. 어깨/택배존/뒤복도 연동.
# 뒤복도(방 복도) 동서 벽 x. 서쪽은 엘리베이터(x=±ELEV_HALF)와 가까워 고정,
# 동쪽으로 넓혀 총 폭 3.0m (중심 x=2.5). 방·메인복도 개구부가 모두 여기에 연동.
BC_WEST = 1.0
BC_EAST = 4.0
ELEV_HALF = 0.8
ELEV_DOOR_HALF = 0.5
ROOM_HALF_ALONG = 1.25
ROOM_DEPTH = 3.0
ROOM_DOOR_HALF = 0.45
# 0.45(0.9m 문)에서 0.5(1.0m 문)로 확대 — 실측 전 가정치 조정 (2026-08-18).
# 근거: 반복 검증에서 로봇(폭 0.52m)+inflation 이 0.9m 문에서 산발 wedging
# (참값 스냅샷으로 확정, improvement_report §1.25). 1.0m 는 엘베 문으로 통과성 검증됨.
# 실측 후 hardware_spec §3 값으로 대체.
PARCEL_DOOR_HALF = 0.5


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
    walls.append(along_x(f"{prefix}_rc_n_w", ELEV_HALF, BC_WEST, CORRIDOR_HALF))
    walls.append(along_x(f"{prefix}_rc_n_e", BC_EAST, x_end, CORRIDOR_HALF))
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
        along_y(f"{prefix}_bc_w", CORRIDOR_HALF, y_end, BC_WEST),
        along_y(f"{prefix}_bc_e", CORRIDOR_HALF, y_end, BC_EAST),
        along_x(f"{prefix}_bc_end", BC_WEST, BC_EAST, y_end),
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
        walls.append(along_y(f"{prefix}_bc_w_{i}", y0, y1, BC_WEST))
    for i, (y0, y1) in enumerate(segments(east_door_ys)):
        walls.append(along_y(f"{prefix}_bc_e_{i}", y0, y1, BC_EAST))
    walls.append(along_x(f"{prefix}_bc_end", BC_WEST, BC_EAST, y_end))

    for room in rooms:
        rid = room["id"]
        door_y = room["door_center"]["y"]
        y0 = door_y - ROOM_HALF_ALONG
        y1 = door_y + ROOM_HALF_ALONG
        if room["side"] == "west_wall":
            x_far = BC_WEST - ROOM_DEPTH
            walls.append(along_x(f"{prefix}_r{rid}_s", x_far, BC_WEST, y0))
            walls.append(along_x(f"{prefix}_r{rid}_n", x_far, BC_WEST, y1))
            walls.append(along_y(f"{prefix}_r{rid}_far", y0, y1, x_far))
        else:
            x_far = BC_EAST + ROOM_DEPTH
            walls.append(along_x(f"{prefix}_r{rid}_s", BC_EAST, x_far, y0))
            walls.append(along_x(f"{prefix}_r{rid}_n", BC_EAST, x_far, y1))
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


# 복도 정적 장애물 (저장맵에는 없음 -> Nav2 가 LiDAR 로 실시간 회피).
# building 모델과 분리된 독립 모델이라 world swap(building 교체) 후에도 유지된다.
# 넓힌 복도(y: -1.5~1.5)에서 사람은 북측, 박스는 남측에 staggered 배치 -> 로봇이 중앙으로 위빙.
_PERSON = {"shape": "cylinder", "radius": 0.20, "height": 1.7, "z": 0.85,
           "color": "0.8 0.6 0.5 1", "diffuse": "0.9 0.7 0.6 1"}
# 박스(카트/적재물): LiDAR 빔이 laser z=0.75 m 평면이라 그보다 낮으면 감지 안 됨.
# 높이 1.0 m (z 0.0~1.0) 로 빔을 확실히 가로지르게 한다.
_BOX = {"shape": "box", "sx": 0.4, "sy": 0.4, "sz": 1.0, "z": 0.5,
        "color": "0.4 0.25 0.1 1", "diffuse": "0.55 0.35 0.15 1"}
OBSTACLES = {
    # 복도(y: -1.5~1.5)를 따라 ±0.8 로 좌우 staggered -> 로봇이 중앙을 위빙하며 통과.
    "kku_f1": [
        {**_PERSON, "name": "obstacle_person_1", "x": 3.5, "y": 0.8},
        {**_BOX, "name": "obstacle_box_1", "x": 6.0, "y": -0.8},
        {**_PERSON, "name": "obstacle_person_2", "x": 9.0, "y": 0.8},
        {**_BOX, "name": "obstacle_box_2", "x": 11.5, "y": -0.8, "sx": 0.5, "sy": 0.5, "sz": 1.1, "z": 0.55},
        # 택배 보관소(alcove) 안의 택배 상자 prop. 낮아서(높이 0.3) LiDAR 빔(0.75) 아래 ->
        # 로봇이 회피하지 않고 픽업 위치(5,-3.4) 앞까지 도달. 실제 적재는 안 하지만 목표 표식.
        {"shape": "box", "name": "parcel_box", "x": 5.0, "y": -3.9,
         "sx": 0.45, "sy": 0.6, "sz": 0.3, "z": 0.15,
         "color": "0.6 0.45 0.2 1", "diffuse": "0.72 0.55 0.28 1"},
    ],
}


# 움직이는 사람 (Gazebo actor). 복도를 가로질러 왕복한다.
# body collision 을 두어 LiDAR 가 감지하도록 시도 (검증 필요).
ACTORS = {
    # 비워둠: 이 Gazebo 빌드에는 libActorCollisionsPlugin 이 없어 actor 의 collision 이
    # LiDAR ray 센서에 감지되지 않음(실측 확인). 움직이는 사람을 "회피 가능한" 장애물로
    # 쓰려면 (a) ActorCollisionsPlugin 빌드, 또는 (b) collision 모델 + cmd_vel 노드 방식 필요.
}


def render_actor(a: dict) -> list[str]:
    half = a["period"] / 2
    return [
        f'    <actor name="{a["name"]}">',
        f'      <pose>{a["x"]:.4g} {a["y0"]:.4g} 0 0 0 1.5708</pose>',
        '      <skin><filename>walk.dae</filename></skin>',
        '      <animation name="walking"><filename>walk.dae</filename>'
        '<interpolate_x>true</interpolate_x></animation>',
        '      <link name="link">',
        f'        <collision name="c"><pose>0 0 0.9 0 0 0</pose>'
        f'<geometry><cylinder><radius>{a["radius"]:.4g}</radius>'
        f'<length>{a["height"]:.4g}</length></cylinder></geometry></collision>',
        '      </link>',
        '      <script>',
        '        <loop>true</loop><auto_start>true</auto_start>',
        '        <trajectory id="0" type="walking">',
        f'          <waypoint><time>0</time><pose>{a["x"]:.4g} {a["y0"]:.4g} 0 0 0 1.5708</pose></waypoint>',
        f'          <waypoint><time>{half:.4g}</time><pose>{a["x"]:.4g} {a["y1"]:.4g} 0 0 0 1.5708</pose></waypoint>',
        f'          <waypoint><time>{a["period"]:.4g}</time><pose>{a["x"]:.4g} {a["y0"]:.4g} 0 0 0 1.5708</pose></waypoint>',
        '        </trajectory>',
        '      </script>',
        '    </actor>',
    ]


def render_obstacle(o: dict) -> list[str]:
    if o["shape"] == "cylinder":
        geom = f'<cylinder><radius>{o["radius"]:.4g}</radius><length>{o["height"]:.4g}</length></cylinder>'
    else:
        geom = f'<box><size>{o["sx"]:.4g} {o["sy"]:.4g} {o["sz"]:.4g}</size></box>'
    pose = f'{o["x"]:.4g} {o["y"]:.4g} {o["z"]:.4g} 0 0 0'
    return [
        f'    <model name="{o["name"]}">',
        '      <static>true</static>',
        f'      <pose>{o["x"]:.4g} {o["y"]:.4g} 0 0 0 0</pose>',
        '      <link name="body">',
        f'        <collision name="c"><pose>0 0 {o["z"]:.4g} 0 0 0</pose>'
        f'<geometry>{geom}</geometry></collision>',
        f'        <visual name="v"><pose>0 0 {o["z"]:.4g} 0 0 0</pose>'
        f'<geometry>{geom}</geometry>'
        f'<material><ambient>{o["color"]}</ambient><diffuse>{o["diffuse"]}</diffuse></material></visual>',
        '      </link>',
        '    </model>',
    ]


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
        '    <!-- 동적 장애물(보행자)을 set_entity_state 로 위치제어하기 위한 플러그인 -->',
        '    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">',
        '      <ros><namespace>/gazebo</namespace></ros>',
        '      <update_rate>50.0</update_rate>',
        '    </plugin>',
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
    ]
    for o in OBSTACLES.get(world_name, []):
        lines += render_obstacle(o)
    for a in ACTORS.get(world_name, []):
        lines += render_actor(a)
    lines += [
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
