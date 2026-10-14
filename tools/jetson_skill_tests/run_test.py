#!/usr/bin/env python3
"""Run one skill test on the Jetson (ROS 2 Humble container, after `source install/setup.bash`).

  python3 tools/jetson_skill_tests/run_test.py elevator_entry --load-state loaded          # dry run (no motion)
  python3 tools/jetson_skill_tests/run_test.py elevator_entry --load-state loaded --execute
  python3 tools/jetson_skill_tests/run_test.py elevator_ride --load-state loaded --execute [--phase press|exit]
  python3 tools/jetson_skill_tests/run_test.py parcel_pickup --execute
Options: --config <yaml> (default config/skill_tests.yaml), --floor F1|F2 (floor profile), --set tests.common.slow_v=0.04 (repeatable),
--log-dir (default logs/jetson_skill_tests, local only). Ctrl+C: stop the base, cancel the active arm request and
hold the lift, then record FAILED.
Prerequisites (README.md): base bring-up with odom/scan/TF and nav_safety_gate, robot_arm_pkg arm_sequence node,
tools/floor_reader server (camera frames + floor), Nav2/AMCL for map->odom, auto_floor_orchestrator for floors.
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))

from skilltests.elevator_entry import ElevatorEntry   # noqa: E402
from skilltests.elevator_ride import ElevatorRide     # noqa: E402
from skilltests.parcel_pickup import ParcelPickup     # noqa: E402

MACHINES = {'elevator_entry': ElevatorEntry, 'elevator_ride': ElevatorRide, 'parcel_pickup': ParcelPickup}
WAYPOINTS = ROOT/'src/slam_pkg/maps/field/waypoints.json'


def resolve(value, waypoints):
    """'waypoint:<name>' -> [x, y, yaw] from the field waypoints file (read only)."""
    if isinstance(value, str) and value.startswith('waypoint:'):
        wp = waypoints[value.split(':', 1)[1]]
        return [wp['x'], wp['y'], wp['yaw_rad']]
    return value


def apply_set(cfg, items):
    for item in items:
        key, _, raw = item.partition('=')
        node = cfg
        parts = key.split('.')
        for part in parts[:-1]:
            node = node[part]
        node[parts[-1]] = yaml.safe_load(raw)


def test_config(cfg, name, floor='F1'):
    """common + the test's own keys + its floors.<floor> profile (UP at the lobby, DOWN at the target floor, ...)."""
    waypoints = json.loads(WAYPOINTS.read_text())['waypoints'] if WAYPOINTS.exists() else {}
    own = {k: v for k, v in cfg['tests'][name].items() if k != 'floors'}
    merged = {**cfg['tests']['common'], **own, **cfg['tests'][name].get('floors', {}).get(floor, {})}
    return {k: resolve(v, waypoints) for k, v in merged.items()}


def validate(cfg, args):
    """Refuse inconsistent settings before anything can move."""
    errors = []
    lift = cfg['lift']
    if lift['mode'] not in ('ultrasonic', 'fixed'):
        errors.append('lift.mode must be ultrasonic or fixed')
    for name, view in cfg['arm']['views'].items():
        if view not in ('front_view', 'floor_view'):
            errors.append(f'arm.views.{name}={view}: arm_sequence accepts only front_view/floor_view')
        if view not in cfg['vision']['views']:
            errors.append(f'vision.views has no calibration for {view}')
    if args.test != 'parcel_pickup' and args.load_state is None:
        errors.append('--load-state loaded|empty is required for the elevator tests (front extent)')
    if args.test == 'parcel_pickup' and args.load_state == 'loaded':
        errors.append('parcel_pickup starts with an empty fork (--load-state empty or omit it)')
    return errors


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('test', choices=sorted(MACHINES))
    parser.add_argument('--config', default=str(HERE/'config/skill_tests.yaml'))
    parser.add_argument('--set', action='append', default=[], metavar='KEY=VALUE')
    parser.add_argument('--execute', action='store_true', help='allow base, arm and lift motion (default: dry run)')
    parser.add_argument('--phase', choices=['press', 'exit'], default=None, help='elevator_ride only')
    parser.add_argument('--load-state', choices=['loaded', 'empty'], default=None)
    parser.add_argument('--floor', choices=['F1', 'F2'], default='F1',
                        help='floor profile: entry F1 = lobby (UP), F2 = target floor (DOWN); ride F1->F2 or F2->F1')
    parser.add_argument('--log-dir', default=str(ROOT/'logs/jetson_skill_tests'))
    args = parser.parse_args()
    cfg = yaml.safe_load(Path(args.config).read_text())
    apply_set(cfg, args.set)
    errors = validate(cfg, args)
    if errors:
        raise SystemExit('config/arguments refused:\n  '+'\n  '.join(errors))
    tcfg = test_config(cfg, args.test, args.floor)
    run_dir = Path(args.log_dir)/f"{time.strftime('%Y%m%d_%H%M%S')}_{args.test}{'' if args.execute else '_dryrun'}"
    run_dir.mkdir(parents=True, exist_ok=True)
    (run_dir/'config_used.json').write_text(json.dumps({'cfg': cfg, 'test': tcfg, 'execute': args.execute,
                                                        'load_state': args.load_state}, indent=1, ensure_ascii=False))

    def log(entry):
        with open(run_dir/'adapter.jsonl', 'a', encoding='utf-8') as handle:
            handle.write(json.dumps({'wall': round(time.time(), 3), **entry}, default=str, ensure_ascii=False)+'\n')

    import rclpy
    from rclpy.signals import SignalHandlerOptions
    # Ctrl+C raises KeyboardInterrupt in Python while the ROS context stays valid, so cleanup can still publish.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('packagu_skill_test')
    io = machine = None
    code = 1
    try:
        from skilltests.ros_adapters import make_io
        io = make_io(node, cfg, args.execute, log)
        machine = MACHINES[args.test](io, tcfg, log_path=run_dir/'states.jsonl')
        if args.load_state is not None:
            io.base.set_loaded(args.load_state == 'loaded')
        elif args.test == 'parcel_pickup':
            io.base.set_loaded(False)                  # the pickup starts empty; it sets 'loaded' itself
        ok, details = io.base.ready(10.0)
        log({'readiness': details})
        if not ok:
            raise RuntimeError(f'not ready (fresh odom/scan/TF map->odom required): {details}')
        if args.execute:
            # arm_sequence requires a completed home (require_homed:=true) before views and presses
            homed = io.arm.command({'action': 'home'}, timeout=60)
            log({'arm_home': homed})
            if not homed.get('done'):
                raise RuntimeError(f'arm home failed: {homed}')
        if args.test == 'elevator_ride':
            target = tcfg['target_floor']
            if args.phase in (None, 'press'):
                if not io.floors.set_target(target):
                    raise RuntimeError('could not set the floor orchestrator target')

            def before_exit():
                # robot-side stand pose on the arrival floor: on the door axis at the cabin depth, facing the door
                d, n, cab = tcfg['exit_door_centre_map'], tcfg['exit_door_normal_map'], tcfg['exit_cabin_centre_map']
                if (cab[0]-d[0])*n[0]+(cab[1]-d[1])*n[1] < 0:
                    n = [-n[0], -n[1]]
                depth = (cab[0]-d[0])*n[0]+(cab[1]-d[1])*n[1]
                stand = (d[0]+n[0]*depth, d[1]+n[1]*depth, math.atan2(-n[1], -n[0]))
                if not io.floors.wait_ready(target, stand):
                    raise RuntimeError(f'arrival-floor map/localisation not ready for {target}')
            if args.phase == 'press':
                machine.run('press')
            elif args.phase == 'exit':
                machine.run('exit', before_exit=before_exit)
            else:
                machine.run('steps', before_exit=before_exit)
        else:
            machine.run()
        code = 0
    except KeyboardInterrupt:
        if machine is not None:
            machine.enter('FAILED', error='interrupted (Ctrl+C)')
    except Exception as exc:
        print(f'TEST FAILED: {exc}', flush=True)
        if machine is not None and machine.state != 'FAILED':
            machine.enter('FAILED', error=f'{type(exc).__name__}: {exc}')
    finally:
        for step in ('base.stop', 'arm.cancel_active', 'lift.halt'):
            try:
                if io is not None:
                    obj, meth = step.split('.')
                    getattr(getattr(io, obj), meth)()
            except Exception as exc:          # every cleanup step runs even if another fails
                log({'cleanup_error': step, 'error': str(exc)})
        if machine is not None:
            machine._flush()
            (run_dir/'result.json').write_text(json.dumps(machine.result(), indent=1, default=str,
                                                          ensure_ascii=False))
        node.destroy_node()
        rclpy.try_shutdown()
    print(f"RESULT {machine.state if machine else 'FAILED'} -> {run_dir}", flush=True)
    raise SystemExit(code)


if __name__ == '__main__':
    main()
