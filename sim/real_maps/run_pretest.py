#!/usr/bin/env python3
"""Saturday (10/10) field-test pre-tests in Gazebo (08 doc §3). Fresh stack per run. Simulation only.

  G1  F1 f1_initial_test -> f1_locker                         (9/16 field: FAILED 97 s)
  G2  F2 f2_delivery_left_room4 -> f2_elevator_staging_v1     (9/16 field: FAILED x2)
  G3  G1 with AMCL initial pose = old f1_idle (injected error, 9/15 S3 seed); AMCL recovery
  G5  F2 door-open world: f2_elevator_staging_v1 -> f2_elevator_entry (optional, U-X02 미정)

  python3 sim/real_maps/run_pretest.py --cases G1,G2 --params P0 --repeats 1 --prefix pre1
Results: logs/real_map_sim/<prefix>_<case>_<params>_r<n>/result.json and <prefix>_summary.csv
Criteria [제안값] (08 doc): SUCCEEDED, footprint-wall >= 0.15 m, lethal-start 0;
G3: last 10 AMCL samples all <= 0.25 m from ground truth.
"""
from host_exec import ensure_container
ensure_container(__file__)

import argparse   # noqa: E402
import csv        # noqa: E402
import json       # noqa: E402
import math       # noqa: E402
import time       # noqa: E402

import rclpy      # noqa: E402

from sim_stack import Stack, LOG, resolve_pose, add_common_args   # noqa: E402

CASES = {
    'G1': dict(world='f1', floor='F1', spawn='f1_initial_test', initial=None, goal='f1_locker'),
    'G2': dict(world='f2', floor='F2', spawn='f2_delivery_left_room4', initial=None, goal='f2_elevator_staging_v1'),
    'G3': dict(world='f1', floor='F1', spawn='f1_initial_test', initial='f1_idle', goal='f1_locker'),
    'G5': dict(world='f2_door_open', floor='F2', spawn='f2_elevator_staging_v1', initial=None,
               goal='f2_elevator_entry'),
}
MIN_WALL_M, RECOVERY_M = .15, .25
FIELDS = ['name', 'case', 'params', 'repeat', 'odom', 'lidar_noise', 'status', 'pass', 'error', 'sim_s', 'wall_s',
          'rtf', 'min_wall_m', 'corner_min_wall_m', 'collision_ahead', 'lethal_start', 'recoveries',
          'recovery_spin', 'recovery_backup', 'recovery_wait', 'costmap_clear', 'planner_fail', 'rpm_blocks',
          'ready_drops', 'arrival_pos_err_m', 'arrival_yaw_err_rad', 'in_place_rotations', 'in_place_rotation_deg',
          'angular_sign_changes', 'path_ratio', 'path_length_m', 'stop_restarts', 'initial_err_m',
          'amcl_last10_max_m', 'amcl_recovered', 'ready_rtf']


def run_one(case_id, params, repeat, prefix, timeout, lidar_noise, odom):
    case = CASES[case_id]
    name = f'{prefix}_{case_id}_{params}_r{repeat}'
    if (LOG/name/'result.json').exists():
        print(f'SKIP {name} (result exists)', flush=True)
        return json.loads((LOG/name/'result.json').read_text()).get('summary')
    goal = resolve_pose(case['goal'])
    stack = Stack(name, case['world'], case['floor'], params, case['spawn'], case['initial'],
                  lidar_noise, odom)
    row = {'name': name, 'case': case_id, 'params': params, 'repeat': repeat, 'odom': odom,
           'lidar_noise': lidar_noise, 'error': None}
    try:
        readiness = stack.start()
        row['ready_rtf'] = readiness['rtf']
        spawn = resolve_pose(case['spawn'])
        if case['initial']:
            init = resolve_pose(case['initial'])
            row['initial_err_m'] = math.hypot(init[0]-spawn[0], init[1]-spawn[1])
        seg = stack.begin(case_id)
        status = stack.navigate(goal, timeout)
        stack.stop()
        stack.node.sim_pause(2)
        seg_row = stack.end(seg, status, goal)
        row.update({k: v for k, v in seg_row.items() if k in FIELDS})
        row['status'] = status
        errors = stack.node.amcl_errors(since=seg['sim_start'], floor=case['floor'])
        if errors:
            last = [e['pos_err_m'] for e in errors[-10:]]
            row['amcl_last10_max_m'] = max(last)
            row['amcl_recovered'] = len(last) == 10 and max(last) <= RECOVERY_M
        stack.result['segment'] = seg_row
        stack.result['amcl_errors'] = errors
    except Exception as exc:
        row['error'] = str(exc)
        stack.result['error'] = str(exc)
        print(f'ERROR {name}: {exc}', flush=True)
    ok = (row.get('status') == 'SUCCEEDED' and not row['error'] and (row.get('min_wall_m') or 0) >= MIN_WALL_M
          and row.get('lethal_start', 1) == 0)
    if case_id == 'G3':
        ok = ok and bool(row.get('amcl_recovered'))
    row['pass'] = ok
    stack.result['summary'] = row
    stack.finish()
    print('RESULT '+json.dumps(row, default=str), flush=True)
    return row


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--cases', default='G1,G2')
    parser.add_argument('--repeats', type=int, default=1)
    parser.add_argument('--start-repeat', type=int, default=1)
    parser.add_argument('--prefix', default=time.strftime('pre_%Y%m%d_%H%M'))
    parser.add_argument('--goal-timeout', type=float, default=300.0, help='[제안값] sim seconds per goal')
    add_common_args(parser)
    args = parser.parse_args()
    rclpy.init()
    summary = LOG/f'{args.prefix}_summary.csv'
    rows = []
    try:
        for repeat in range(args.start_repeat, args.start_repeat+args.repeats):
            for case_id in args.cases.split(','):
                row = run_one(case_id, args.params, repeat, args.prefix, args.goal_timeout,
                              args.lidar_noise == 'on', args.odom)
                if row:
                    rows.append(row)
                    new = not summary.exists()
                    with open(summary, 'a', newline='') as handle:
                        writer = csv.DictWriter(handle, FIELDS, extrasaction='ignore')
                        if new:
                            writer.writeheader()
                        writer.writerow(row)
    finally:
        rclpy.try_shutdown()
    print(f'SUMMARY {summary}', flush=True)


if __name__ == '__main__':
    main()
