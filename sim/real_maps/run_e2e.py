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
import sys        # noqa: E402
import time       # noqa: E402
from pathlib import Path   # noqa: E402

import rclpy      # noqa: E402
from geometry_msgs.msg import Twist                    # noqa: E402
from rcl_interfaces.msg import Parameter as ParamMsg, ParameterType, ParameterValue   # noqa: E402
from rcl_interfaces.srv import SetParameters            # noqa: E402
from std_msgs.msg import Bool, String                   # noqa: E402
from std_srvs.srv import Trigger                        # noqa: E402

import arm_geometry as ag                               # noqa: E402
from sim_stack import Stack, LOG, WORLD, resolve_pose, add_common_args, wrap   # noqa: E402
from run_scenarios import pose_values                   # noqa: E402

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT/'tools/jetson_skill_tests'))   # shared skill-test state machines

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
        self.armlift = self.io = None
        if args.arm_sim:
            from arm_lift import ArmLift
            self.armlift = ArmLift(stack, WORLD)
            if args.load == 'pallet':
                # Autonomous mode: the shared Jetson skill-test state machines run on simulation adapters
                # (camera-found pallet docking, camera-located arm-pressed buttons, camera floor reading).
                from sim_adapters import make_io
                self.indicators = json.loads((HERE/'generated/elevator_indicators.json').read_text())
                self.doors = json.loads((HERE/'generated/doors.json').read_text())
                self.io = make_io(stack, self.armlift, WORLD, self.indicators)

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

    def machine(self, label, machine, phase='steps', **kw):
        """Run one phase of a shared skill-test state machine as one stage (events -> <label>.jsonl/.json)."""
        seg = self.begin(label)
        machine.log_path = self.stack.dir/f'{label}.jsonl'
        error = None
        try:
            machine.run(phase, **kw)
        except Exception as exc:
            error = f'{type(exc).__name__}: {exc}'
        row = self.stack.end(seg, 'FAILED' if error else 'COMPLETED')
        (self.stack.dir/f'{label}.json').write_text(json.dumps(machine.result(), indent=1, default=str))
        states = ' > '.join(e['state'] for e in machine.events[-40:])
        row = self.record(row, seg, kind=f'skill_{machine.name}',
                          note=(error or 'robot-side data only (camera, joints, odom, LiDAR, taught priors)')+
                          f' | states: {states}')
        if error:
            self.fail(row, error)
        return machine

    def skill_cfg(self, **extra):
        pp, arm = WORLD['perception'], WORLD['arm']
        cfg = {k: pp[k] for k in ('press_push_m', 'press_height_tol_m', 'front_stop_m', 'reaim_m', 'press_retries',
                                  'back_off_m', 'line_up_m', 'slow_v', 'cabin_v', 'door_timeout_s', 'floor_confirm_frames',
                                  'floor_period_s', 'floor_timeout_s', 'call_retries', 'door_open_margin_m',
                                  'door_align_m', 'door_through_m', 'press_envelope_m', 'floor_reconfirm_frames',
                                  'look_back_steps', 'look_back_m', 'confirm_window_s', 'recheck_max', 'door_width_m', 'door_lateral_tol_m', 'door_jamb_range_diff_m', 'door_match_m', 'door_match_rad', 'door_confirm_margin_m', 'depth_adjust_max_m', 'display_ahead_m',
                                  'door_centre_tol_m')}
        cfg['press_visual_required'] = False  # simulation: selected button lamp is the evidence
        cfg['press_view'] = {key: pp['press_view'][key] for key in ('call', 'destination')}
        # where the taught press cycles (servo_protocol.py, the Jetson's angles) put the arm tip: same geometry the
        # world used to place the demo button boards
        sp = ag.servo_protocol(ROOT)
        cfg['press_tip'] = {key: ag.taught_press(arm, sp, int(arm['press_cycle'][key])) for key in ('call', 'destination')}
        cfg['confirm_by'] = {key: list(pp['confirm_by'][key]) for key in ('call', 'destination')}
        cfg.update(extra)
        return cfg

    def floor_xyz(self, floor, model):
        off = WORLD['floors'][floor]['world_offset']
        return [model['on'][i]-off[i] for i in range(3)]

    def door_normal(self, floor):
        a = self.doors[floor]['angle']                  # door line direction; the normal side is fixed by door_axis()
        return [-math.sin(a), math.cos(a)]

    def entry_cfg(self, floor):
        ind = self.indicators['floors'][floor]
        hall = ind['models'][ind['hall_lit']]
        bx, by, bz = self.floor_xyz(floor, hall)
        n, view = hall['normal'], float(WORLD['perception']['button_view_m'])
        inside = self.wp(f'{floor.lower()}_elevator_inside')
        door = self.doors[floor]['center']
        return self.skill_cfg(
            floor=floor, call_label='UP' if floor == 'F1' else 'DOWN', call_button_prior_map=[bx, by, bz],
            button_view_pose_map=ind.get('hall_view') or [bx+math.cos(n)*view, by+math.sin(n)*view, wrap(n+math.pi)],
            cabin_centre_map=list(inside[:2]), door_centre_map=list(door), door_normal_map=self.door_normal(floor))

    def ride_cfg(self, src, dst):
        ind = self.indicators['floors'][src]
        car = ind['models'][ind['car_lit'][dst]]
        inside_dst = self.wp(f'{dst.lower()}_elevator_inside')
        return self.skill_cfg(
            floor=src, target_floor=dst, target_label=dst[1], target_button_prior_map=self.floor_xyz(src, car),
            door_centre_map=self.doors[src]['center'], cabin_centre_map=list(self.wp(f'{src.lower()}_elevator_inside')[:2]),
            display_prior_map=ind['display_map']['xyz'], exit_pose_map=list(self.wp(f'{dst.lower()}_elevator_exit')[:3]),
            exit_door_centre_map=self.doors[dst]['center'], exit_cabin_centre_map=list(inside_dst[:2]),
            exit_display_prior_map=self.indicators['floors'][dst]['display_map']['xyz'],
            door_normal_map=self.door_normal(src), exit_door_normal_map=self.door_normal(dst),
            exit_button_prior_map=self.floor_xyz(dst, self.indicators['floors'][dst]['models'][
                self.indicators['floors'][dst]['car_lit'][dst]]),
)

    def pickup_cfg(self):
        pp, pal = WORLD['perception'], WORLD['parcel']['pallet']
        from dressing import pallet_center_ahead
        return {**{k: pp[k] for k in ('scan_dist_m', 'scan_yaws_rad', 'scan_z_m', 'same_parcel_m', 'predock_m',
                                     'face_looks', 'face_z_m', 'pallet_face_width_m', 'dock_v', 'arm_clear_top_m',
                                     'approach_tol_m', 'undock_m')},
                'pallet_depth_m': pal['size_m'][0], 'fork_centre_ahead_m': pallet_center_ahead(pal),
                'approach_pose_map': list(self.wp('f1_locker')[:3])}

    def nav(self, label, wp_name, kind='nav2', goal=None):
        self.ensure_stowed(label)
        if self.io is not None:
            self.stack.update_footprint(self.io.base.loaded())
        goal = goal or self.wp(wp_name)
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
                          note=(f"lift {kind} ({self.armlift.load_mode}): up={out.get('up')} "
                                f"parcel={out.get('parcel')} down={out.get('down')}; "
                                'lift_cycle.py joint-position check; parcel attach/detach mock (no contact/load)'))
        (self.stack.dir/f'lift_{label}.json').write_text(json.dumps(out, indent=1, default=str))
        if not out['completed']:
            self.fail(row, f'lift/parcel {kind} failed')
        return row

    def elevator(self, **cmd):
        self.node.elevator_cmd.publish(String(data=json.dumps(cmd)))

    def wait_elevator(self, predicate, timeout):
        end = self.node.now()+timeout
        def ready():
            process = getattr(self.stack, 'elevator_process', None)
            state = self.node.elevator or {}
            if process is not None and process.poll() is not None:
                raise StageFailed('simulation elevator process stopped; see elevator.log')
            if state.get('phase') == 'fault':
                raise StageFailed('simulation elevator fault: '+str(state.get('transfer_error')))
            return bool(state and predicate(state)) or self.node.now() > end
        return self.node.wait(ready, timeout*20+30) and \
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
        # Mock of pressing the door-open/call button (door-open button is not modelled, also with --arm-sim).
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
            node.sim_pause(.1)   # control period in sim time (a wall-clock period coarsens control at --speed > 1)
        for _ in range(5):
            node.direct_pub.publish(Twist())
            node.sim_pause(.1)
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
        self.map_switch(dst)
        self.floor_check(dst)

    def map_switch(self, dst):
        node = self.node
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
        # The orchestrator seeds AMCL with the fixed cabin waypoint; after the button sequence the robot stands on
        # the door axis at the cabin depth facing the door (robot-side priors: door centre, door normal, cabin
        # depth), so seed that pose instead (no ground truth used).
        door, n = self.doors[dst]['center'], self.door_normal(dst)
        inside = self.wp(f'{dst.lower()}_elevator_inside')
        if (inside[0]-door[0])*n[0]+(inside[1]-door[1])*n[1] < 0:
            n = [-n[0], -n[1]]
        depth = (inside[0]-door[0])*n[0]+(inside[1]-door[1])*n[1]
        stand = (door[0]+n[0]*depth, door[1]+n[1]*depth, math.atan2(-n[1], -n[0]))
        self.stack.publish_initial(stand, xy_var=.04, yaw_sd=math.radians(8))
        row['note'] += f' | robot-side initialpose {[round(v, 3) for v in stand]}'
        since = node.now()-.05
        node.initialpose_t = None
        self.relocalize(dst, since)

    def ride_machine(self, src, dst):
        """Autonomous ride: floor-button phase of the shared ElevatorRide machine, the elevator transfer (checked
        at the carry instant, the robot may still move in the cabin), map switch + relocalisation, exit phase."""
        from skilltests.elevator_ride import ElevatorRide
        node = self.node
        node.initialpose_t = None
        self.set_orchestrator_target(dst)
        ride = ElevatorRide(self.io, self.ride_cfg(src, dst))
        self.machine(f'{src}_ride_button', ride, 'press')
        seg = self.begin(f'ride_{src}_to_{dst}')
        carried = self.wait_elevator(lambda s: s['current_floor'] == dst and (s.get('last_carry') or {}).get('to') == dst,
                                     self.args.elevator_timeout*2)
        carry = (node.elevator or {}).get('last_carry') or {}
        node.sim_pause(.6)
        row = self.stack.end(seg, 'ARRIVED' if carried and node.truth_map()[0] == dst else 'FAILED')
        t = carry.get('stamp')
        if t is not None:
            confirmed = carry.get('confirmed_at', t)
            # SetEntityState is asynchronous: the requested instant can still
            # contain a source-floor sample. Evaluate after the observed ACK
            # and stable destination pose, while retaining pre-transfer data.
            before, after = t-.3, confirmed+.1
            o_a, o_b = self.io.base.odom_at(before), self.io.base.odom_at(after)
            g_a, g_b = node.truth_at(before, src), node.truth_at(after, dst)
            if None not in (o_a, o_b, g_a, g_b):
                d_odom, d_cab = rel_se2(o_a, o_b), rel_se2(self.cabin_relative(src, g_a), self.cabin_relative(dst, g_b))
                row.update(odom_jump_m=math.hypot(d_odom[0]-d_cab[0], d_odom[1]-d_cab[1]),
                           odom_jump_rad=abs(wrap(d_odom[2]-d_cab[2])))
            g_c = node.truth_at(confirmed, dst)
            if g_c is not None:
                rel = self.cabin_relative(dst, g_c)
                row.update(cabin_rel_err_m=math.hypot(rel[0]-carry['cabin_relative'][0], rel[1]-carry['cabin_relative'][1]),
                           cabin_rel_err_rad=abs(wrap(rel[2]-carry['cabin_relative'][2])))
        row['note'] = (f'carry {json.dumps(carry)}; continuity = odom change vs cabin-relative truth change '
                       'from request -0.3 s to confirmation +0.1 s; cabin pose at confirmation')
        row = self.record(row, seg, kind='elevator_ride')
        if row['status'] != 'ARRIVED':
            self.fail(row, f'elevator ride {src}->{dst} failed')
        if row.get('odom_jump_m', 1) > self.transfer['odom_continuity_max_m'] or \
                row.get('odom_jump_rad', 1) > self.transfer['odom_continuity_max_rad']:
            self.fail(row, 'odom not continuous across floor transfer')
        if row.get('cabin_rel_err_m', 1) > self.transfer['cabin_relative_pose_max_m'] or \
                row.get('cabin_rel_err_rad', 1) > self.transfer['cabin_relative_pose_max_rad']:
            self.fail(row, 'cabin-relative pose (position/yaw) not preserved')
        self.stack.floor = dst
        self.machine(f'{dst}_ride_exit', ride, 'exit', before_exit=lambda: self.map_switch(dst))

    def floor_check(self, floor):
        """Before leaving the cabin: read the floor shown above the entrance. --arm-sim also turns the arm-tip camera
        to Kim's taught 'floor_view' and checks (geometry only) that the display is in its field of view."""
        seg = self.begin(f'{floor}_floor_check')
        view = None
        if self.armlift is not None:
            view = self.armlift.look('floor_view', floor)
        state = dict(self.node.elevator or {})
        shown = state.get('display_floor')
        ok = shown == floor and state.get('current_floor') == floor
        if self.armlift is not None:
            view['stow'] = self.armlift.stow()
        row = self.stack.end(seg, 'CONFIRMED' if ok else 'FAILED')
        row.update(note=(f'display shows {shown} (sim value from /elevator/state, not image recognition); '
                         f'camera view {json.dumps(view, default=str)}'))
        row = self.record(row, seg, kind='floor_check')
        if view is not None:
            (self.stack.dir/f'floor_check_{floor}.json').write_text(json.dumps(view, indent=1, default=str))
        if not ok:
            self.fail(row, f'floor display shows {shown}, expected {floor}')

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
        if self.io is not None:
            return self.run_autonomous()
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

    def run_autonomous(self):
        """--arm-sim --load pallet: every step between the Nav2 legs is done by the robot itself through the shared
        Jetson skill-test machines (no operator drive, no mock button presses or door commands)."""
        from skilltests.elevator_entry import ElevatorEntry
        from skilltests.parcel_pickup import ParcelPickup
        if getattr(self.args, 'start_stage', 'full') == 'return':
            self.ensure_stowed('return_start')
            self.ride_machine('F2', 'F1')
            self.nav('F1_exit_to_start', 'f1_initial_test')
            return
        if getattr(self.args, 'start_stage', 'full') == 'cabin':
            self.node.wait(lambda: (self.node.parcel or {}).get('state') == 'attached', 10)
            if not self.io.base.loaded():
                raise StageFailed('cabin scenario requires an initially carried pallet')
            self.ensure_stowed('cabin_start')
            carried = self.armlift.carry_up()
            if not carried.get('reached'):
                raise StageFailed('cabin scenario lift preparation failed')
            self.payload = 'loaded'
            print('SCENARIO_SETUP '+json.dumps({'start': 'F1 cabin', 'initial_payload': 'loaded',
                   'pickup_validated': False, 'lift': carried}), flush=True)
        else:
            self.nav('F1_start_to_locker', 'f1_locker')
            self.machine('F1_parcel_pickup', ParcelPickup(self.io, self.pickup_cfg()))
            self.payload = 'loaded'
            self.nav('locker_to_F1_elevator_entry', 'f1_elevator_entry')
            self.nav_button_area('F1')
            self.machine('F1_elevator_entry', ElevatorEntry(self.io, self.entry_cfg('F1')))
        self.ride_machine('F1', 'F2')
        self.nav('F2_exit_to_delivery', 'f2_delivery_left_room4')
        self.unload_autonomous('delivery_unload')
        self.nav('delivery_to_F2_staging', 'f2_elevator_staging_v1')
        self.nav('staging_to_F2_elevator_entry', 'f2_elevator_entry')
        self.nav_button_area('F2')
        self.machine('F2_elevator_entry', ElevatorEntry(self.io, self.entry_cfg('F2')))
        self.ride_machine('F2', 'F1')
        self.nav('F1_exit_to_start', 'f1_initial_test')

    def nav_button_area(self, floor):
        """Nav2 to the taught spot in front of the call board (plans around the scanned landing clutter); the entry
        machine's GO_TO_BUTTON_AREA is then only a short correction."""
        view = self.indicators['floors'][floor].get('hall_view')
        if view is not None:
            self.nav(f'{floor}_entry_to_button_area', f'{floor} call-board view', goal=(*view, floor))

    def unload_autonomous(self, label):
        """Lower the pallet to the floor, release it and reverse the fork out of the pocket."""
        seg = self.begin(label)
        io = self.io
        out = {'lower': io.lift.lower()}
        out['release'] = io.load.detach() if out['lower']['reached'] else None
        x, y, yaw = io.base.pose()
        back = float(WORLD['perception']['unload_back_out_m'])
        out['back_out'] = io.base.go_to((x-math.cos(yaw)*back, y-math.sin(yaw)*back, yaw), reverse=True, max_v=.05)
        ok = (out['lower']['reached'] and (out['release'] or {}).get('ok')
              and out['back_out']['status'] == 'REACHED')
        self.payload = 'unloaded' if ok else self.payload
        row = self.record(self.stack.end(seg, 'COMPLETED' if ok else 'FAILED'), seg, kind='lift_parcel',
                          note=f'lower+release+reverse out of the pocket: {json.dumps(out, default=str)}')
        (self.stack.dir/f'{label}.json').write_text(json.dumps(out, indent=1, default=str))
        if not ok:
            self.fail(row, 'unload failed')


def rel_se2(a, b):
    """Pose b expressed in the frame of pose a (x, y, yaw)."""
    c, s = math.cos(-a[2]), math.sin(-a[2])
    dx, dy = b[0]-a[0], b[1]-a[1]
    return (c*dx-s*dy, s*dx+c*dy, wrap(b[2]-a[2]))


MIN_WALL_M = .15   # [제안값] 08 doc criterion, applied to every driving stage
# Skill stages (button presses, pallet docking, cabin moves) come close to walls on purpose ("as close as possible"
# to the button); their body footprint is held to this smaller [제안값] margin instead (front stop is 0.02 m).
MIN_WALL_SKILL_M = .05


def criteria(result):
    """Safety/accuracy criteria, reported separately from route completion ([제안값])."""
    failed = []
    if not result.get('completed'):
        failed.append('not completed')
    for s in result.get('stages', []):
        limit = MIN_WALL_SKILL_M if str(s.get('kind', '')).startswith('skill_') else MIN_WALL_M
        if s.get('min_wall_m') is not None and s['min_wall_m'] < limit:
            failed.append(f"{s['stage']} min_wall {s['min_wall_m']:.3f} < {limit}")
        for key in ('lethal_start', 'rpm_blocks', 'ready_drops'):
            if s.get(key):
                failed.append(f"{s['stage']} {key}={s[key]}")
        if s.get('kind') == 'relocalization' and s.get('status') != 'CONVERGED':
            failed.append(f"{s['stage']} not converged")
    return {'pass': not failed, 'failed': failed, 'min_wall_threshold_m': MIN_WALL_M,
            'skill_min_wall_threshold_m': MIN_WALL_SKILL_M}


def run_once(args, name):
    cabin_start = args.start_stage in ('cabin', 'return')
    start_floor = 'F2' if args.start_stage == 'return' else 'F1'
    spawn = 'f1_initial_test'
    if cabin_start:
        inside = resolve_pose(f'{start_floor.lower()}_elevator_inside')
        door = WORLD['elevator'][start_floor]['door_center']
        angle = WORLD['elevator'][start_floor]['door_axis_rad']
        n = [-math.sin(angle), math.cos(angle)]
        if (inside[0]-door[0])*n[0]+(inside[1]-door[1])*n[1] < 0:
            n = [-n[0],-n[1]]
        depth = (inside[0]-door[0])*n[0]+(inside[1]-door[1])*n[1]
        spawn = (door[0]+n[0]*depth,door[1]+n[1]*depth,math.atan2(n[1],n[0]),start_floor)
    stack = Stack(name, 'building_dressed' if args.dressed else 'building', start_floor, args.params, spawn,
                  None, args.lidar_noise == 'on', args.odom, arm=args.arm_sim, speed=args.speed,
                  parcels=args.parcels, gui=args.gui, load=args.load, lift_stop=args.lift_stop,
                  initial_elevator_floor=start_floor if cabin_start else None, initial_load=args.start_stage == 'cabin')
    if args.speed != 1.0:
        # Same 1 ms physics, but wall-clock pacing differs from the judged 1x runs: keep it out of the 1x statistics.
        (stack.dir/'EXPLORATORY.txt').write_text(
            f'시연 배속 실행(--speed {args.speed:g}). 물리 step 1 ms는 같지만 1배속 판정 실행과 분리해 통계 제외.\n')
    mission = None
    result = {'name': name, 'validation_scope': 'simulation', 'provisional_dimensions': True,
              'cabin_mode': args.cabin_mode, 'params': args.params, 'odom': args.odom, 'arm_sim': args.arm_sim,
              'speed': 'max' if args.speed == 0 else args.speed, 'start_stage': args.start_stage, 'dressed': args.dressed, 'parcels': args.parcels if args.arm_sim else 0,
              'load': args.load if args.arm_sim else None,
              'lift_stop': args.lift_stop if args.arm_sim and args.load == 'pallet' else None,
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
        result['criteria'] = criteria(result)
        stack.result['e2e'] = result
        stack.finish(preserve_scene=args.gui and not result['completed'] and not args.close_on_failure)
        result['scene_preserved'] = stack.result.get('scene_preserved', False)
        if result['scene_preserved']:
            print('SCENE_PRESERVED '+str(stack.dir), flush=True)
        result['bag_status'] = stack.result.get('bag', {}).get('status')
        summary = {k: result.get(k) for k in ('name', 'cabin_mode', 'params', 'odom', 'arm_sim', 'speed', 'load',
                                              'parcels', 'completed', 'failed_stage', 'sim_s', 'wall_s', 'min_wall_m', 'bag_status', 'error')}
        summary.update(criteria_pass=result['criteria']['pass'], criteria_fail='; '.join(result['criteria']['failed']),
                       rtf=(result.get('sim_s') or 0)/result['wall_s'] if result.get('wall_s') else None)
        with open(stack.dir/'summary.csv', 'w', newline='') as handle:
            writer = csv.DictWriter(handle, list(summary))
            writer.writeheader()
            writer.writerow(summary)
        (stack.dir/'result.json').write_text(json.dumps(stack.result, indent=2, default=str)+'\n')
    print('E2E_RESULT '+json.dumps({k: v for k, v in result.items() if k != 'stages'}, default=str), flush=True)
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--name', default=None, help='default <YYYYmmdd_HHMMSS>_e2e_<cabin>_<params>[_arm]')
    parser.add_argument('--repeats', type=int, default=1)
    parser.add_argument('--start-stage', choices=['full','cabin','return'], default='full',
                        help='cabin: loaded F1 cabin; return: unloaded F2 cabin -> F1 -> origin')
    parser.add_argument('--cabin-mode', default='nav2', choices=['nav2', 'direct'])
    parser.add_argument('--arm-sim', action='store_true',
                        help='sim-only lift (lift_cycle.py) + 4-DOF arm press/fold + parcel attach/detach mock')
    parser.add_argument('--goal-timeout', type=float, default=300.0, help='[제안값] sim seconds per goal')
    parser.add_argument('--elevator-timeout', type=float, default=120.0, help='[제안값] sim seconds')
    parser.add_argument('--dressed', action='store_true',
                        help='building_dressed.world: visual-only cabin/landing/locker dressing (same collisions)')
    parser.add_argument('--parcels', type=int, default=1, choices=[1, 2, 3],
                        help='--arm-sim: pallets at pickup, one parcel each, first one carried (or stacked boxes with '
                             '--load box); attach/detach mock')
    parser.add_argument('--lift-stop', default=WORLD['lift']['carry']['mode'], choices=['ultrasonic', 'fixed'],
                        help='--load pallet carry height: stop when the mast-front ultrasonic sees the parcel top, '
                             'or raise only lift.carry.fixed_lift_m (keep the parcel below the arm and its camera)')
    parser.add_argument('--load', default='pallet', choices=['pallet', 'box'],
                        help='--arm-sim: floor pallet picked by the lift carrier (default) or the 10/8 shelf box')
    parser.add_argument('--gui', action='store_true', help='gzclient (camera follows the robot) + RViz')
    parser.add_argument('--close-on-failure', action='store_true', help='close failed GUI scene too (default: stop and retain it)')
    for key in ('position_error_max_m', 'yaw_error_max_rad', 'covariance_xy_max_m2', 'covariance_yaw_max_rad2',
                'timeout_s'):
        parser.add_argument('--reloc-'+key.replace('_', '-'), dest='reloc_'+key, type=float, default=None)
    add_common_args(parser)
    args = parser.parse_args()
    if args.start_stage in ('cabin','return') and not (args.arm_sim and args.load == 'pallet'):
        parser.error('--start-stage cabin/return requires --arm-sim --load pallet')
    if args.arm_sim and args.load == 'pallet':
        args.dressed = True     # buttons, floor display and pickup zone exist only in building_dressed.world
    rclpy.init()
    results = []
    try:
        for repeat in range(1, args.repeats+1):
            base = args.name or (time.strftime('%Y%m%d_%H%M%S')+f"_e2e_{args.cabin_mode}_{args.params}"
                                 + ('_arm' if args.arm_sim else '')
                                 + (('_box' if args.load == 'box' else '_pallet') if args.arm_sim else '')
                                 + (f'_p{args.parcels}' if args.arm_sim and args.parcels > 1 else '')
                                 + ('_dressed' if args.dressed else '')
                                 + (f'_x{args.speed:g}' if args.speed != 1.0 else ''))
            name = base if args.repeats == 1 and args.name else f'{base}_r{repeat}'
            results.append(run_once(args, name))
            if results[-1].get('scene_preserved'):
                break  # keep the failed scene; do not start another Gazebo over it
    finally:
        rclpy.try_shutdown()
    done = sum(r['completed'] for r in results)
    print(f'E2E_SUMMARY scope={args.start_stage} completed {done}/{len(results)}', flush=True)
    raise SystemExit(0 if done == len(results) else 1)


if __name__ == '__main__':
    main()
