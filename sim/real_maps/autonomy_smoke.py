#!/usr/bin/env python3
"""Short autonomous check without the full round trip (simulation only, development aid).

Spawns at f1_locker in building_dressed.world and runs the shared skill-test machines in order (--steps):
  pickup  ParcelPickup (turn right, camera search, right-most parcel, dock, lift with ultrasonic stop)
  entry   Nav2 to f1_elevator_entry, ElevatorEntry (button area, camera, arm press, lamp check, door, enter)
  ride    ElevatorRide F1->F2 (turn, camera, approach, press, back, turn, display) + transfer checks + exit
  python3 sim/real_maps/autonomy_smoke.py --steps pickup,entry,ride --speed 3 [--gui]
Results: logs/real_map_sim/<name>/ (stages.csv, <stage>.json/.jsonl, camera frames on request).
"""
from host_exec import ensure_container
ensure_container(__file__, x11=True)

import argparse   # noqa: E402
import json       # noqa: E402
import time       # noqa: E402
from types import SimpleNamespace   # noqa: E402

import rclpy      # noqa: E402

from sim_stack import Stack, WORLD   # noqa: E402
from run_e2e import Mission, StageFailed   # noqa: E402


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--name', default=time.strftime('%Y%m%d_%H%M%S')+'_autonomy_smoke')
    parser.add_argument('--steps', default='pickup,entry,ride')
    parser.add_argument('--speed', type=float, default=1.0)
    parser.add_argument('--lift-stop', default=WORLD['lift']['carry']['mode'], choices=['ultrasonic', 'fixed'])
    parser.add_argument('--parcels', type=int, default=1)
    parser.add_argument('--gui', action='store_true')
    args = parser.parse_args()
    rclpy.init()
    stack = Stack(args.name, 'building_dressed', 'F1', 'P0', 'f1_locker', None, True, 'encoder', record=False,
                  arm=True, speed=args.speed, parcels=args.parcels, gui=args.gui, load='pallet',
                  lift_stop=args.lift_stop)
    (stack.dir/'EXPLORATORY.txt').write_text('autonomy_smoke.py 개발 점검(왕복 아님). 통계 제외.\n')
    ns = SimpleNamespace(arm_sim=True, load='pallet', cabin_mode='direct', goal_timeout=300.0, elevator_timeout=120.0,
                         lift_stop=args.lift_stop, **{f'reloc_{k}': None for k in WORLD['relocalization']})
    result, m = {'steps': args.steps, 'error': None}, None
    try:
        stack.start()
        m = Mission(stack, ns)
        from skilltests.elevator_entry import ElevatorEntry
        from skilltests.parcel_pickup import ParcelPickup
        for step in args.steps.split(','):
            if step == 'pickup':
                m.machine('F1_parcel_pickup', ParcelPickup(m.io, m.pickup_cfg()))
            elif step == 'entry':
                m.nav('locker_to_F1_elevator_entry', 'f1_elevator_entry')
                m.nav_button_area('F1')
                m.machine('F1_elevator_entry', ElevatorEntry(m.io, m.entry_cfg('F1')))
            elif step == 'ride':
                m.ride_machine('F1', 'F2')
    except (StageFailed, RuntimeError) as exc:
        result['error'] = str(exc)
        print('SMOKE_ERROR '+str(exc), flush=True)
    finally:
        result['stages'] = [{k: r.get(k) for k in ('stage', 'status', 'min_wall_m', 'note')} for r in
                            (m.rows if m is not None else [])]
        stack.finish()
        (stack.dir/'smoke.json').write_text(json.dumps(result, indent=1, default=str))
        rclpy.try_shutdown()
    print('SMOKE_DONE '+json.dumps({'error': result['error'], 'stages': [(s['stage'], s['status']) for s in
                                                                         result['stages']]}), flush=True)


if __name__ == '__main__':
    main()
