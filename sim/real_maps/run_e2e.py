#!/usr/bin/env python3
"""End-to-end round trip on the two-floor real-map world (building.world). Simulation only.

F1 f1_initial_test -> f1_locker (load marker) -> f1_elevator_entry -> call/wait open -> f1_elevator_inside
-> ride F1->F2 + map switch (auto_floor_orchestrator, floor_maps_real.yaml) + relocalization gate
-> f2_elevator_exit -> f2_delivery_left_room4 (unload marker) -> f2_elevator_staging_v1 -> f2_elevator_entry
-> call/wait open -> f2_elevator_inside -> ride F2->F1 + map switch + relocalization gate
-> f1_elevator_exit -> f1_initial_test

  python3 sim/real_maps/run_e2e.py [--name e2e_r1] [--params P0] [--repeats 1] [--cabin-mode nav2|direct]

Every stage has a time limit ([제안값] 300 s sim per goal) and on failure the robot is stopped, the stage is
recorded and the run is aborted. Nav2 goals advance only on final status SUCCEEDED. After each map switch
the run advances only when NEW AMCL samples (stamped after the switch) meet the relocalization criteria
(world_params.yaml relocalization, [제안값]); orchestrator READY / initialpose alone is not enough.
--cabin-mode direct replaces only cabin entry/exit with a simulated operator drive (ground truth,
/cmd_vel -> gate -> shim) — the 9/16 field procedure enters/exits the cabin manually. Arms are not
simulated: load/unload only publish /sim/payload_state. Mission code (이성덕) is not used.
"""
from host_exec import ensure_container
ensure_container(__file__)

import argparse   # noqa: E402
import csv        # noqa: E402
import json       # noqa: E402
import math       # noqa: E402
import time       # noqa: E402

import rclpy      # noqa: E402
from geometry_msgs.msg import Twist                    # noqa: E402
from rcl_interfaces.msg import Parameter as ParamMsg, ParameterType, ParameterValue   # noqa: E402
from rcl_interfaces.srv import SetParameters            # noqa: E402
from std_msgs.msg import Bool, String                   # noqa: E402
from std_srvs.srv import Trigger                        # noqa: E402

from sim_stack import Stack, LOG, WORLD, resolve_pose, add_common_args, wrap   # noqa: E402
from run_scenarios import pose_values                   # noqa: E402

FLOOR_OF_CABIN = {'F1': 'f1_elevator_inside', 'F2': 'f2_elevator_inside'}
STAGE_FIELDS = ['stage', 'kind', 'floor', 'status', 'sim_s', 'wall_s', 'rtf', 'min_wall_m', 'corner_min_wall_m',
                'collision_ahead', 'lethal_start', 'recoveries', 'costmap_clear', 'planner_fail', 'rpm_blocks',
                'ready_drops', 'arrival_pos_err_m', 'arrival_yaw_err_rad', 'in_place_rotations',
                'in_place_rotation_deg', 'angular_sign_changes', 'path_ratio', 'stop_restarts', 'door_reopen',
                'door_stall', 'wait_s', 'odom_jump_m', 'odom_jump_rad', 'cabin_rel_err_m', 'cabin_rel_err_rad',
                'reloc_new_samples', 'reloc_pos_err_m', 'reloc_yaw_err_rad', 'reloc_cov_xx', 'reloc_cov_yy',
                'reloc_cov_yaw', 'reloc_tf_jump_m', 'reloc_tf_jump_rad', 'reloc_converge_s',
                'initialpose_err_m', 'initialpose_err_rad', 'map_yaml', 'note']


class StageFailed(RuntimeError):
    pass


class Mission:
    def __init__(self, stack, args):
        self.stack, self.node, self.args = stack, stack.node, args
        self.rows = []
        self.payload = 'empty'
        self.reloc = dict(WORLD['relocalization'])
        for key in self.reloc:
            override = getattr(args, 'reloc_'+key, None) if key != 'source' else None
            if override is not None:
                self.reloc[key] = override
        self.transfer = WORLD['floor_transfer']
        self.armlift = None
        if args.arm_sim:
            from arm_lift import ArmLift
            self.armlift = ArmLift(stack, WORLD)

    # ------------------------------------------------------------ bookkeeping
    def wp(self, name):
        return resolve_pose(name)

    def record(self, row, seg=None, **extra):
        row = dict(row)
        row.update(extra)
        if seg is not None:
            for key, event in (('door_reopen', 'reopen'), ('door_stall', 'door_stall')):
                now = (self.node.elevator or {}).get('events', {}).get(event, 0)
                row[key] = now-seg.get('elevator_events', {}).get(event, 0)
        self.rows.append(row)
        self.node.stage_pub.publish(String(data=json.dumps(row, default=str)))
        print('STAGE '+json.dumps({k: row.get(k) for k in ('stage', 'status', 'floor', 'sim_s', 'min_wall_m',
                                                             'note')}, default=str), flush=True)
        with open(self.stack.dir/'stages.csv', 'w', newline='') as handle:
            writer = csv.DictWriter(handle, STAGE_FIELDS, extrasaction='ignore')
            writer.writeheader()
            writer.writerows(self.rows)
        return row

    def begin(self, label):
        seg = self.stack.begin(label)
        seg['elevator_events'] = dict((self.node.elevator or {}).get('events', {}))
        return seg

    def fail(self, row, why):
        self.stack.stop()
        row['status'] = row.get('status') or 'FAILED'
        row['note'] = why
        raise StageFailed(f"{row['stage']}: {why}")

    # ------------------------------------------------------------ stage kinds
    def ensure_stowed(self, label):
        """--arm-sim: never drive with the arm unfolded (fold = servo_protocol HOME/STOW pose)."""
        if self.armlift is None or self.armlift.stowed():
            return
        result = self.armlift.stow()
        if not result['reached']:
            raise StageFailed(f'{label}: arm not stowed before driving ({result})')

    def arm_press(self, label, cycle, on_press):
        seg = self.begin(label)
        out = self.armlift.press_cycle(cycle, on_press)
        ok = out['completed'] and out['pressed'] and out['stowed']
        press_tip = next((r['tip_map'] for r in out['steps'] if not r['send']), None)
        row = self.record(self.stack.end(seg, 'COMPLETED' if ok else 'FAILED'), seg, kind='arm',
                          note=(f"press cycle {cycle} ({out['label']}) steps {len(out['steps'])}, pressed={out['pressed']}, "
                                f"stowed={out['stowed']}, tip(map,[가정 기하]) at press {press_tip}; "
                                'PWM->angle·arm geometry [가정값], button contact not modelled'))
        (self.stack.dir/f'arm_{label}.json').write_text(json.dumps(out, indent=1))
        if not ok:
            self.fail(row, f'arm press cycle {cycle} failed: {[(r["pose"], r["reached"]) for r in out["steps"]]}')
        return row

    def nav(self, label, wp_name, kind='nav2'):
        self.ensure_stowed(label)
        goal = self.wp(wp_name)
        if goal[3] != self.stack.floor:
            raise StageFailed(f'{label}: {wp_name} is on {goal[3]}, robot on {self.stack.floor}')
        seg = self.begin(label)
        status = self.stack.navigate(goal, self.args.goal_timeout)
        self.stack.stop()
        row = self.record(self.stack.end(seg, status, goal), seg, kind=kind, note=f'goal {wp_name}')
        if status != 'SUCCEEDED':
            self.fail(row, f'Nav2 final status {status} (not SUCCEEDED)')
        return row

    def payload_marker(self, label, state):
        if self.armlift is not None:
            return self.load_unload(label, 'load' if state == 'loaded' else 'unload')
        seg = self.begin(label)
        self.payload = state
        self.node.payload_pub.publish(String(data=json.dumps({'state': state, 'arm_validated': False})))
        self.node.sim_pause(2)
        return self.record(self.stack.end(seg, 'MARKED'), seg, kind='payload',
                           note=f'payload={state} (marker only; arm/lift not simulated)')

    def load_unload(self, label, kind):
        seg = self.begin(label)
        out = self.armlift.load_unload(kind)
        self.payload = 'loaded' if kind == 'load' else 'unloaded'
        self.node.payload_pub.publish(String(data=json.dumps({'state': self.payload, 'arm_validated': False,
                                                              'lift_sim': True, 'parcel_mock': True})))
        row = self.record(self.stack.end(seg, 'COMPLETED' if out['completed'] else 'FAILED'), seg, kind='lift_parcel',
                          note=(f"lift {kind}: up={out.get('up')} parcel={out.get('parcel')} down={out.get('down')}; "
                                'lift_cycle.py joint-position check; parcel attach/detach mock (no contact/load)'))
        (self.stack.dir/f'lift_{label}.json').write_text(json.dumps(out, indent=1, default=str))
        if not out['completed']:
            self.fail(row, f'lift/parcel {kind} failed')
        return row

    def elevator(self, **cmd):
        self.node.elevator_cmd.publish(String(data=json.dumps(cmd)))

    def wait_elevator(self, predicate, timeout):
        end = self.node.now()+timeout
        return self.node.wait(lambda: self.node.elevator is not None and predicate(self.node.elevator)
                              or self.node.now() > end, timeout*20+30) and \
            self.node.elevator is not None and predicate(self.node.elevator)

    def call(self, label, floor):
        if self.armlift is not None:
            self.arm_press(f'{floor}_hall_button_arm', 1, lambda: self.elevator(cmd='call', floor=floor))
        else:
            self.elevator(cmd='call', floor=floor)
        seg = self.begin(label)
        start = self.node.now()
        ok = self.wait_elevator(lambda s: s['current_floor'] == floor and s['door_state'] == 'open',
                                self.args.elevator_timeout)
        row = self.stack.end(seg, 'OPEN' if ok else 'TIMEOUT')
        row = self.record(row, seg, kind='elevator', wait_s=self.node.now()-start,
                          note=f'call {floor}; boarding starts only after door_state=open (CR-01)')
        if not ok:
            self.fail(row, f'elevator did not open at {floor} within {self.args.elevator_timeout} s')
        return row

    def ensure_door_open(self, floor):
        state = self.node.elevator or {}
        if state.get('current_floor') == floor and state.get('door_state') == 'open':
            return False
        # Mock of pressing the door-open/call button (arm not simulated).
        self.elevator(cmd='call', floor=floor)
        if not self.wait_elevator(lambda s: s['current_floor'] == floor and s['door_state'] == 'open',
                                  self.args.elevator_timeout):
            raise StageFailed(f'door did not reopen at {floor}')
        return True

    def cabin_move(self, label, wp_name, floor):
        self.ensure_stowed(label)
        reopened = self.ensure_door_open(floor)
        if self.args.cabin_mode == 'nav2':
            row = self.nav(label, wp_name, kind='cabin_nav2')
        else:
            row = self.direct(label, wp_name)
        if reopened:
            row['note'] += '; door reopened by call (button mock)'
        return row

    def direct(self, label, wp_name, max_v=.08, max_w=.30, timeout=180.0):
        """Simulated operator drive (ground truth feedback) through /cmd_vel -> gate -> shim."""
        goal = self.wp(wp_name)
        node = self.node
        seg = self.begin(label)
        self.stack.resume()
        start = node.now()
        status, phase = 'SUCCEEDED', 'go'
        while True:
            if node.now()-start > timeout:
                status = 'TIMEOUT'
                break
            if node.samples and node.samples[-1][5] <= .015:
                status = 'CLEARANCE_STOP'
                break
            _, x, y, yaw = node.truth_map()
            dx, dy = goal[0]-x, goal[1]-y
            dist = math.hypot(dx, dy)
            cmd = Twist()
            if phase == 'go' and dist < .04:
                phase = 'turn'
            if phase == 'go':
                err = wrap(math.atan2(dy, dx)-yaw)
                if abs(err) > .12:
                    cmd.angular.z = float(max(-max_w, min(max_w, 1.2*err)))
                else:
                    cmd.linear.x = float(min(max_v, .5*dist+.02))
                    cmd.angular.z = float(max(-max_w, min(max_w, 1.5*err)))
            else:
                err = wrap(goal[2]-yaw)
                if abs(err) < .04:
                    break
                cmd.angular.z = float(max(-max_w, min(max_w, 1.2*err)))
            node.direct_pub.publish(cmd)
            node.pause(.1)
        for _ in range(5):
            node.direct_pub.publish(Twist())
            node.pause(.1)
        self.stack.stop()
        row = self.record(self.stack.end(seg, status, goal), seg, kind='cabin_direct',
                          note=f'simulated operator drive to {wp_name} (not Nav2)')
        if status != 'SUCCEEDED':
            self.fail(row, f'direct cabin drive {status}')
        return row

    def set_orchestrator_target(self, floor):
        client = self.node.create_client(SetParameters, '/floor_orchestrator_node/set_parameters')
        try:
            if not self.node.wait(client.service_is_ready, 15):
                raise StageFailed('orchestrator set_parameters unavailable')
            req = SetParameters.Request()
            for name, value in (('target_floor', floor), ('spawn_point_id', 'elevator_inside')):
                p = ParamMsg()
                p.name = name
                p.value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=value)
                req.parameters.append(p)
            future = client.call_async(req)
            if not self.node.wait(future.done, 10) or not all(r.successful for r in future.result().results):
                raise StageFailed('orchestrator parameter update failed')
        finally:
            self.node.destroy_client(client)
        client = self.node.create_client(Trigger, '/floor_orchestrator/request_switch')
        try:
            if not self.node.wait(client.service_is_ready, 15):
                raise StageFailed('orchestrator request_switch unavailable')
            future = client.call_async(Trigger.Request())
            if not self.node.wait(future.done, 10) or not future.result().success:
                raise StageFailed('orchestrator request_switch rejected: '+str(future.result()))
        finally:
            self.node.destroy_client(client)

    def cabin_relative(self, floor, pose):
        cx, cy, cyaw, _ = self.wp(FLOOR_OF_CABIN[floor])
        c, s = math.cos(-cyaw), math.sin(-cyaw)
        return (c*(pose[0]-cx)-s*(pose[1]-cy), s*(pose[0]-cx)+c*(pose[1]-cy), wrap(pose[2]-cyaw))

    def ride(self, src, dst):
        node, stack = self.node, self.stack
        self.stack.stop()
        # --- travel -------------------------------------------------------------------
        seg = self.begin(f'ride_{src}_to_{dst}')
        truth0 = node.truth_map()
        if truth0[0] != src:
            raise StageFailed(f'robot is on {truth0[0]}, expected {src}')
        odom0 = pose_values(node.odom.pose.pose)
        rel0 = self.cabin_relative(src, truth0[1:])
        node.initialpose_t = None
        self.set_orchestrator_target(dst)
        if self.armlift is not None:
            self.arm_press(f'{src}_car_button_arm', 2, lambda: self.elevator(cmd='go', floor=dst))
        else:
            self.elevator(cmd='go', floor=dst)
        self.elevator(cmd='close')
        moved = self.wait_elevator(lambda s: s['phase'] == 'moving', self.args.elevator_timeout)
        arrived = moved and self.wait_elevator(
            lambda s: s['current_floor'] == dst and s['door_state'] == 'open', self.args.elevator_timeout)
        node.sim_pause(.5)
        truth1 = node.truth_map()
        odom1 = pose_values(node.odom.pose.pose)
        rel1 = self.cabin_relative(dst, truth1[1:]) if truth1[0] == dst else None
        row = self.stack.end(seg, 'ARRIVED' if arrived and truth1[0] == dst else 'FAILED')
        row.update(odom_jump_m=math.hypot(odom1[0]-odom0[0], odom1[1]-odom0[1]),
                   odom_jump_rad=abs(wrap(odom1[2]-odom0[2])))
        if rel1 is not None:
            row.update(cabin_rel_err_m=math.hypot(rel1[0]-rel0[0], rel1[1]-rel0[1]),
                       cabin_rel_err_rad=abs(wrap(rel1[2]-rel0[2])))
        carry = (node.elevator or {}).get('last_carry')
        row['note'] = (f'robot floor {truth1[0]}; cabin-relative before {[round(v, 3) for v in rel0]}; '
                       f'carry {json.dumps(carry)}')
        row = self.record(row, seg, kind='elevator_ride')
        if row['status'] != 'ARRIVED':
            self.fail(row, f'elevator ride {src}->{dst} failed (moved={moved}, arrived={arrived}, '
                           f'robot floor {truth1[0]})')
        if row['odom_jump_m'] > self.transfer['odom_continuity_max_m'] or \
                row['odom_jump_rad'] > self.transfer['odom_continuity_max_rad']:
            self.fail(row, 'odom not continuous across floor transfer')
        if row.get('cabin_rel_err_m', 1) > self.transfer['cabin_relative_pose_max_m'] or \
                row.get('cabin_rel_err_rad', 1) > self.transfer['cabin_relative_pose_max_rad']:
            self.fail(row, 'cabin-relative pose (position/yaw) not preserved')
        stack.floor = dst
        # --- map switch ---------------------------------------------------------------
        seg = self.begin(f'map_switch_{dst}')
        ok = node.wait(lambda: node.orch is not None and node.orch.get('phase') == 'ready'
                       and node.orch.get('current_floor') == dst, 60)
        orch = dict(node.orch or {})
        # Only AMCL samples stamped after the orchestrator's /initialpose count as "new".
        since = node.initialpose_t-.05 if node.initialpose_t is not None else node.now()
        row = self.stack.end(seg, 'READY' if ok else 'FAILED')
        row.update(map_yaml=orch.get('map_yaml'), note=f"orchestrator {json.dumps(orch)}")
        row = self.record(row, seg, kind='map_switch')
        if not ok:
            self.fail(row, f'orchestrator did not reach READY for {dst}: {orch}')
        node.initialpose_t = None
        self.relocalize(dst, since)

    def relocalize(self, floor, since):
        """Advance only on NEW AMCL samples meeting the [제안값] criteria; else stop/record/abort."""
        node, crit = self.node, self.reloc
        seg = self.begin(f'relocalize_{floor}')
        start = node.now()
        first = None
        last_request = -1e9
        tf_window = []
        verdict, why = False, 'timeout'
        while node.now()-start < float(crit['timeout_s']):
            if node.now()-last_request >= 1.0:
                node.call_empty('/request_nomotion_update', 2.0)
                last_request = node.now()
            node.pause(.2)
            pose = node.tf_pose('map', 'base_footprint')
            if pose is not None:
                tf_window.append((node.now(), pose))
            tf_window = [(t, p) for t, p in tf_window if node.now()-t <= float(crit['tf_stable_window_s'])]
            errors = node.amcl_errors(since=since, floor=floor)
            if errors and first is None:
                first = errors[0]
            if len(errors) < int(crit['min_new_amcl_samples']):
                why = f'only {len(errors)} new AMCL samples'
                continue
            e = errors[-1]
            jump_m = max((math.hypot(p[0]-tf_window[0][1][0], p[1]-tf_window[0][1][1]) for _, p in tf_window),
                         default=None)
            jump_r = max((abs(wrap(p[2]-tf_window[0][1][2])) for _, p in tf_window), default=None)
            span = tf_window[-1][0]-tf_window[0][0] if tf_window else 0
            checks = {'pos': e['pos_err_m'] <= crit['position_error_max_m'],
                      'yaw': e['yaw_err_rad'] <= crit['yaw_error_max_rad'],
                      'cov_xy': max(e['cov_xx'], e['cov_yy']) <= crit['covariance_xy_max_m2'],
                      'cov_yaw': e['cov_yaw'] <= crit['covariance_yaw_max_rad2'],
                      'tf_window': span >= .8*float(crit['tf_stable_window_s']),
                      'tf_stable': jump_m is not None and jump_m <= crit['tf_stable_max_jump_m']
                      and jump_r <= crit['tf_stable_max_jump_rad']}
            why = 'pending: '+','.join(k for k, v in checks.items() if not v)
            if all(checks.values()):
                verdict = True
                break
        errors = node.amcl_errors(since=since, floor=floor)
        row = self.stack.end(seg, 'CONVERGED' if verdict else 'NOT_CONVERGED')
        if errors:
            e = errors[-1]
            row.update(reloc_new_samples=len(errors), reloc_pos_err_m=e['pos_err_m'],
                       reloc_yaw_err_rad=e['yaw_err_rad'], reloc_cov_xx=e['cov_xx'], reloc_cov_yy=e['cov_yy'],
                       reloc_cov_yaw=e['cov_yaw'])
        if first:
            row.update(initialpose_err_m=first['pos_err_m'], initialpose_err_rad=first['yaw_err_rad'])
        if tf_window:
            row['reloc_tf_jump_m'] = max(math.hypot(p[0]-tf_window[0][1][0], p[1]-tf_window[0][1][1])
                                         for _, p in tf_window)
            row['reloc_tf_jump_rad'] = max(abs(wrap(p[2]-tf_window[0][1][2])) for _, p in tf_window)
        row['reloc_converge_s'] = node.now()-start if verdict else None
        row['note'] = (f'criteria {json.dumps({k: v for k, v in crit.items() if k != "source"})} [제안값]; '
                       f'{why if not verdict else "all checks passed"}')
        row = self.record(row, seg, kind='relocalization')
        (self.stack.dir/f'relocalize_{floor}_{int(start)}.json').write_text(json.dumps(errors, indent=1))
        if not verdict:
            self.fail(row, f'relocalization not converged within {crit["timeout_s"]} s ({why})')
        return row

    # ------------------------------------------------------------ mission
    def run(self):
        self.nav('F1_start_to_locker', 'f1_locker')
        self.payload_marker('locker_load', 'loaded')
        self.nav('locker_to_F1_elevator_entry', 'f1_elevator_entry')
        self.call('F1_call_wait_open', 'F1')
        self.cabin_move('F1_board', 'f1_elevator_inside', 'F1')
        self.ride('F1', 'F2')
        self.cabin_move('F2_exit', 'f2_elevator_exit', 'F2')
        self.nav('F2_exit_to_delivery', 'f2_delivery_left_room4')
        self.payload_marker('delivery_unload', 'unloaded')
        self.nav('delivery_to_F2_staging', 'f2_elevator_staging_v1')
        self.nav('staging_to_F2_elevator_entry', 'f2_elevator_entry')
        self.call('F2_call_wait_open', 'F2')
        self.cabin_move('F2_board', 'f2_elevator_inside', 'F2')
        self.ride('F2', 'F1')
        self.cabin_move('F1_exit', 'f1_elevator_exit', 'F1')
        self.nav('F1_exit_to_start', 'f1_initial_test')


def run_once(args, name):
    stack = Stack(name, 'building', 'F1', args.params, 'f1_initial_test', None,
                  args.lidar_noise == 'on', args.odom, arm=args.arm_sim)
    mission = None
    result = {'name': name, 'validation_scope': 'simulation', 'provisional_dimensions': True,
              'cabin_mode': args.cabin_mode, 'params': args.params, 'odom': args.odom, 'arm_sim': args.arm_sim,
              'completed': False}
    wall0 = time.monotonic()
    try:
        stack.start()
        sim0 = stack.node.now()
        mission = Mission(stack, args)
        mission.run()
        result['completed'] = True
    except Exception as exc:
        result['error'] = str(exc)
        result['failed_stage'] = mission.rows[-1]['stage'] if mission and mission.rows else 'startup'
        print('E2E_ERROR '+str(exc), flush=True)
    finally:
        result['wall_s'] = time.monotonic()-wall0
        if mission is not None:
            result['sim_s'] = stack.node.now()-sim0
            result['stages'] = mission.rows
            result['min_wall_m'] = min((r['min_wall_m'] for r in mission.rows if r.get('min_wall_m') is not None),
                                       default=None)
        stack.result['e2e'] = result
        stack.finish()
    print('E2E_RESULT '+json.dumps({k: v for k, v in result.items() if k != 'stages'}, default=str), flush=True)
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--name', default=time.strftime('e2e_%Y%m%d_%H%M'))
    parser.add_argument('--repeats', type=int, default=1)
    parser.add_argument('--cabin-mode', default='nav2', choices=['nav2', 'direct'])
    parser.add_argument('--arm-sim', action='store_true',
                        help='sim-only lift (lift_cycle.py) + 4-DOF arm press/fold + parcel attach/detach mock')
    parser.add_argument('--goal-timeout', type=float, default=300.0, help='[제안값] sim seconds per goal')
    parser.add_argument('--elevator-timeout', type=float, default=120.0, help='[제안값] sim seconds')
    for key in ('position_error_max_m', 'yaw_error_max_rad', 'covariance_xy_max_m2', 'covariance_yaw_max_rad2',
                'timeout_s'):
        parser.add_argument('--reloc-'+key.replace('_', '-'), dest='reloc_'+key, type=float, default=None)
    add_common_args(parser)
    args = parser.parse_args()
    rclpy.init()
    results = []
    try:
        for repeat in range(1, args.repeats+1):
            name = args.name if args.repeats == 1 else f'{args.name}_r{repeat}'
            results.append(run_once(args, name))
    finally:
        rclpy.try_shutdown()
    done = sum(r['completed'] for r in results)
    print(f'E2E_SUMMARY completed {done}/{len(results)}', flush=True)
    raise SystemExit(0 if done == len(results) else 1)


if __name__ == '__main__':
    main()
