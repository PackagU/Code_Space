#!/usr/bin/env python3
"""Export the existing field coordinates and map pins for the camera delivery mission."""
import argparse
import json
import math
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]


def prepare(registry, pins, start_floor='F1', target_map='F2', display_floor='4',
            start='f1_initial_test', pickup='f1_locker', destination='f2_delivery_left_room4'):
    if registry.get('schema_version') != 1 or pins.get('schema_version') != 1:
        raise ValueError('field registry and map pins require schema_version 1')
    if start_floor == target_map:
        raise ValueError('an elevator mission needs different start/destination map floors')
    if not str(display_floor).isdigit() or not 1 <= int(display_floor) <= 4:
        raise ValueError('current button website supports destination buttons 1-4')
    floors = {}
    for floor in (start_floor, target_map):
        pin = pins['floors'][floor]
        points = {}
        for name, pose in registry['waypoints'].items():
            if pose['floor'] != floor:
                continue
            if pose.get('frame_id', 'map') != 'map':
                raise ValueError('field waypoint must use map frame: ' + name)
            values = [pose[key] for key in ('x', 'y', 'yaw_rad')]
            if any(type(v) not in (int, float) or not math.isfinite(v) for v in values):
                raise ValueError('invalid field waypoint: ' + name)
            points[name] = dict(type='field', x=pose['x'], y=pose['y'],
                                yaw_deg=math.degrees(pose['yaw_rad']))
        for role in ('elevator_entry', 'elevator_inside', 'elevator_exit'):
            points[role] = dict(points[floor.lower() + '_' + role])
        required = (start, pickup) if floor == start_floor else (destination,)
        for name in required:
            if name not in points:
                raise ValueError('missing field waypoint on %s: %s' % (floor, name))
        floors[floor] = dict(map_yaml=pin['yaml'], points=points,
                             initial_pose_id=start if floor == start_floor else 'elevator_inside')
    maps = {floor: dict(map_yaml=data['map_yaml'], points={name: data['points'][name]
            for name in ('elevator_inside', 'elevator_exit')}) for floor, data in floors.items()}
    mission = dict(start_floor=start_floor, target_floor=target_map, pickup_point=pickup,
                   elevator_entry_point='elevator_entry', elevator_inside_point='elevator_inside',
                   elevator_exit_point='elevator_exit', destination_point=destination,
                   mock_load_event='parcel_loaded', mock_delivery_event='delivered')
    return (dict(schema_version=1, frame_id='map', floors=floors),
            dict(schema_version=1, frame_id='map', default_spawn_point_id='elevator_inside', floors=maps),
            dict(schema_version=1, missions={'field_camera_delivery': mission}),
            dict(start_floor=start_floor, target_map=target_map, observed_target_floor='F' + str(int(display_floor))))


def main(argv=None):
    field = ROOT/'maps/field'
    if not (field/'waypoints.json').exists():
        field = ROOT/'src/slam_pkg/maps/field'
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--registry', type=Path, default=field/'waypoints.json')
    parser.add_argument('--pins', type=Path, default=field/'map_pins.json')
    parser.add_argument('--output', type=Path, default=ROOT/'logs/camera_delivery/config')
    parser.add_argument('--start-floor', default='F1')
    parser.add_argument('--target-map', default='F2')
    parser.add_argument('--display-floor', default='4', help='F2 map is the physical fourth floor')
    parser.add_argument('--start', default='f1_initial_test')
    parser.add_argument('--pickup', default='f1_locker')
    parser.add_argument('--destination', default='f2_delivery_left_room4')
    args = parser.parse_args(argv)
    try:
        configs = prepare(json.loads(args.registry.read_text(encoding='utf-8')),
                          json.loads(args.pins.read_text(encoding='utf-8')),
                          args.start_floor, args.target_map, args.display_floor,
                          args.start, args.pickup, args.destination)
        args.output.mkdir(parents=True, exist_ok=True)
        for name, data in zip(('points.yaml', 'maps.yaml', 'missions.yaml'), configs[:3]):
            (args.output/name).write_text(yaml.safe_dump(data, allow_unicode=True, sort_keys=False), encoding='utf-8')
        configs[3].update(pins=str(args.pins.resolve()), registry=str(args.registry.resolve()))
        (args.output/'launch.json').write_text(json.dumps(configs[3]), encoding='utf-8')
    except (OSError, ValueError, KeyError, TypeError) as exc:
        parser.error(str(exc))
    print('기존 좌표·지도 보존: %s → %s / 표시층 %s / %s' %
          (args.start_floor, args.target_map, args.display_floor, args.output))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
