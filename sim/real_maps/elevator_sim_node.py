#!/usr/bin/env python3
"""Simulation-only elevator controller for the two-floor real-map world (building.world).

State machine (09 L3): closed/idle -> (call) -> arrival wait (moving) -> doors opening -> open hold
-> doors closing -> (car call) moving -> destination doors opening. Timings and door geometry are
provisional values from world_params.yaml (아직 모름 U-E01/U-E02). Not a model of the real elevator.

Interfaces
  sub /sim/elevator/command  std_msgs/String JSON  {"cmd":"call"|"go","floor":"F1"} | {"cmd":"close"}
                                                    | {"cmd":"hold","seconds":20} | {"cmd":"reopen","value":true}
  pub /elevator/state        std_msgs/String JSON  current_floor, door_state(open|closed|opening|closing),
                                                    phase, moving, robot_inside, leaves (map frame), events
The /elevator/state keys current_floor/door_state are what auto_floor_orchestrator_node consumes.

When the car travels with the robot fully inside the cabin, the robot entity is moved to the other
floor's cabin keeping its cabin-relative position AND yaw (cabin frame = *_elevator_inside waypoint,
지도 기하). Ground truth is used only as the simulated door sensor / cabin occupancy, never for Nav2.
"""
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
import yaml

from arm_geometry import fk
from clearance import minimum_clearance, polygon

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
MAPS = ROOT/'maps/field'
ORDER = ['F1', 'F2']
DOOR_ZONE_DEPTH_M = .15   # [제안값] half-depth of the door-sensor zone across the door line


def yaw_of(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Elevator(Node):
    def __init__(self):
        super().__init__('sim_elevator')
        self.declare_parameter('robot_entity', 'packagu_sim')
        self.declare_parameter('indicators', False)   # building_dressed: floor digits + lit buttons (visual only)
        # --arm-sim: buttons register only when the arm tip (ground-truth pose + joint states + assumed arm FK)
        # touches the button face; commands on /sim/elevator/command still work for runs without the arm.
        self.declare_parameter('button_contact', False)
        self.declare_parameter('require_arm_pause', False)
        self.declare_parameter('initial_floor', '')
        params = yaml.safe_load((HERE/'world_params.yaml').read_text())
        self.e = params['elevator']
        self.offsets = {f: params['floors'][f]['world_offset'] for f in ORDER}
        self.doors = json.loads((HERE/'generated/doors.json').read_text())
        wps = json.loads((MAPS/'waypoints.json').read_text())['waypoints']
        self.cabin = {f: (wps[self.e[f]['inside_waypoint']]['x'], wps[self.e[f]['inside_waypoint']]['y'],
                          wps[self.e[f]['inside_waypoint']]['yaw_rad']) for f in ORDER}
        self.timing = self.e['timing']
        self.reopen = bool(self.e['reopen_on_obstruction'])
        self.current = self.get_parameter('initial_floor').value or self.e['initial_floor']
        if self.current not in ORDER:
            raise ValueError('initial_floor must be F1 or F2')
        self.fraction = 0.0
        self.phase = 'idle'
        self.requests = []
        self.request_ready_at = {}
        self.hold_until = self.travel_until = self.idle_since = None
        self.travel_to = None
        self.carry_robot = False
        self.events = {'calls': 0, 'reopen': 0, 'door_stall': 0, 'carry': 0, 'travel': 0}
        self.last_carry = None
        self.pending_carry = None
        self.transfer_prepare_since = None
        self.transfer_error = None
        self.arm_paused_since = None
        self.create_subscription(Bool, '/sim/arm_pose_writes_paused', self.on_arm_pause, 10)
        self.truth = None
        self.seq = 0
        self.last_sent = {}
        self.visual_requests = {}
        self.car_lit, self.hall_lit = set(), set()
        self.arm, self.contact = params['arm'], params['perception']
        body = params['robot']['footprint']
        self.rear_extent = -min(p[0] for p in body)
        self.half_width = max(abs(p[1]) for p in body)
        self.empty_front = max(max(p[0] for p in body), float(params['lift']['fork_tip_x_m']))
        pallet = params['parcel']['pallet']
        self.loaded_front = float(params['lift']['mast_x_m'])+.04+float(pallet['rear_gap_m'])+pallet['size_m'][0]
        self.loaded_half = max(self.half_width, pallet['size_m'][1]/2)
        self.loaded = False
        self.create_subscription(String, '/sim/parcel/state', self.on_payload, 10)
        self.joints, self.touching, self.presses = {}, set(), []
        self.create_subscription(JointState, '/joint_states', self.on_joints, 20)
        self.indicators = None
        path = HERE/'generated/elevator_indicators.json'
        if self.get_parameter('indicators').value and path.exists():
            self.indicators = json.loads(path.read_text())
        self.state_pub = self.create_publisher(String, '/elevator/state', 10)
        self.create_subscription(String, '/sim/elevator/command', self.on_command, 10)
        self.create_subscription(Odometry, '/sim/ground_truth', lambda m: setattr(self, 'truth', m), 10)
        self.client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.last_tick = None
        self.create_timer(.05, self.tick)
        self.get_logger().info(f'sim elevator ready: floor={self.current} reopen_on_obstruction={self.reopen} '
                               f'timing={self.timing} (provisional, simulation only)')

    # ---------------------------------------------------------------- geometry
    def now(self):
        return self.get_clock().now().nanoseconds/1e9

    def robot_map_pose(self, floor):
        if self.truth is None:
            return None
        p = self.truth.pose.pose
        off = self.offsets[floor]
        return p.position.x-off[0], p.position.y-off[1], yaw_of(p.orientation)

    def robot_floor(self):
        if self.truth is None:
            return None
        x = self.truth.pose.pose.position.x
        return min(ORDER, key=lambda f: abs(x-self.offsets[f][0]-self.cabin[f][0]))

    def door_box(self, floor):
        d = self.doors[floor]
        return [d['center'][0], d['center'][1], d['opening_m'], 2*DOOR_ZONE_DEPTH_M, d['angle']]

    def obstructing(self):
        if self.robot_floor() != self.current:
            return False
        x, y, yaw = self.robot_map_pose(self.current)
        return minimum_clearance(np.array([self.door_box(self.current)]), x, y, yaw, self.footprint()) <= .02

    def on_payload(self, msg):
        try:
            self.loaded = json.loads(msg.data).get('state') == 'attached'
        except (ValueError, AttributeError):
            pass

    def footprint(self):
        if not self.get_parameter('require_arm_pause').value:
            return None
        front = self.loaded_front if self.loaded else self.empty_front
        half = self.loaded_half if self.loaded else self.half_width
        return [[-self.rear_extent,-half],[front,-half],[front,half],[-self.rear_extent,half]]

    def inside(self, floor=None):
        floor = floor or self.current
        if self.robot_floor() != floor:
            return False
        x, y, yaw = self.robot_map_pose(floor)
        d = self.doors[floor]
        cx, cy = d['center']
        ix, iy, _ = self.cabin[floor]
        normal = np.array([-math.sin(d['angle']), math.cos(d['angle'])])
        if np.dot([ix-cx, iy-cy], normal) < 0:
            normal = -normal
        depth = (polygon(x, y, yaw, self.footprint())-[cx, cy]) @ normal
        return bool(depth.min() > .05 and depth.max() < 3.0)

    def leaf_poses(self, floor):
        out = {}
        for name, pos in self.doors[floor]['leaves'].items():
            f = self.fraction if floor == self.current else 0.0
            out[name] = [pos['closed'][i]+f*(pos['open'][i]-pos['closed'][i]) for i in range(2)]+[pos['closed'][2]]
        return out

    # ---------------------------------------------------------------- gazebo
    def set_entity(self, name, x, y, z, yaw):
        if not self.client.service_is_ready():
            return None
        req = SetEntityState.Request()
        req.state.name, req.state.reference_frame = name, 'world'
        req.state.pose.position.x, req.state.pose.position.y, req.state.pose.position.z = x, y, z
        req.state.pose.orientation.z, req.state.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        return self.client.call_async(req)

    def place_visual(self, name, pose, key):
        """Acknowledge scene writes before caching them; failed/unanswered requests are retried."""
        pending = self.visual_requests.get(name)
        if pending is not None:
            future, saved_key, sent_at = pending
            if future.done():
                self.visual_requests.pop(name, None)
                try:
                    response = future.result()
                    if response is not None and response.success:
                        self.last_sent[name] = saved_key
                        if saved_key == key:
                            return True
                except Exception:
                    pass
            elif self.now()-sent_at <= 3.0:
                return False
            else:
                self.visual_requests.pop(name, None)
        future = self.set_entity(name, *pose)
        if future is not None:
            self.visual_requests[name] = (future,key,self.now())
        return False

    def push_leaves(self, force=False):
        for floor in ORDER:
            off = self.offsets[floor]
            for name, (x, y, yaw) in self.leaf_poses(floor).items():
                key = (round(x, 3), round(y, 3))
                if force or self.last_sent.get(name) != key:
                    if self.place_visual(name, (x+off[0], y+off[1], off[2], yaw), key):
                        self.last_sent[name] = key

    def carry(self, src, dst):
        """Confirm service ACK and destination ground truth before announcing arrival.

        A single asynchronous SetEntityState call can race joint pose updates.
        Retry the same saved cabin-relative target, never derive it again from
        a pose that may already be at the destination. This is world simulation,
        not robot perception/control input.
        """
        if self.pending_carry is None:
            self.transfer_prepare_since = self.transfer_prepare_since if self.transfer_prepare_since is not None else self.now()
            velocity = self.truth.twist.twist
            stopped = math.hypot(velocity.linear.x, velocity.linear.y) <= .01 and abs(velocity.angular.z) <= .02
            arm_ready = (not self.get_parameter('require_arm_pause').value or
                         self.arm_paused_since is not None and self.now()-self.arm_paused_since >= 1.0)
            if not stopped or not arm_ready:
                if self.now()-self.transfer_prepare_since > 5.0:
                    self.transfer_error = 'robot motion/arm writes did not stop before cabin transfer'
                    self.phase = 'fault'
                return False
            x, y, yaw = self.robot_map_pose(src)
            sx, sy, syaw = self.cabin[src]
            dx, dy, dyaw = self.cabin[dst]
            c, s = math.cos(-syaw), math.sin(-syaw)
            rel = (c*(x-sx)-s*(y-sy), s*(x-sx)+c*(y-sy), wrap(yaw-syaw))
            c, s = math.cos(dyaw), math.sin(dyaw)
            nx, ny = dx+c*rel[0]-s*rel[1], dy+s*rel[0]+c*rel[1]
            nyaw = wrap(dyaw+rel[2])
            z = self.truth.pose.pose.position.z-self.offsets[src][2]+self.offsets[dst][2]
            off = self.offsets[dst]
            self.pending_carry = {'target': (nx+off[0],ny+off[1],z,nyaw), 'stable': 0,
                                  'future': None, 'attempts': 0, 'started': self.transfer_prepare_since, 'ack': False,
                                  'wall_started': time.monotonic(),
                                  'record': {'from': src,'to':dst,'stamp':self.now(),'src_map_pose':[x,y,yaw],
                                             'cabin_relative':list(rel),'dst_map_pose':[nx,ny,nyaw]}}
        pending = self.pending_carry
        if self.get_parameter('require_arm_pause').value:
            if self.arm_paused_since is None or self.now()-self.arm_paused_since < 1.0:
                if self.now()-pending['started'] > 5.0:
                    self.transfer_error = 'arm pose writes did not acknowledge a quiescent cabin transfer'
                    self.phase = 'fault'
                return False
        future = pending['future']
        if future is not None and future.done():
            try:
                response = future.result()
                pending['ack'] = bool(response is not None and response.success)
            except Exception as exc:
                self.get_logger().warning(f'transfer service error: {type(exc).__name__}')
                pending['ack'] = False
            pending['future'] = None
        at_destination = self.robot_floor() == dst
        pending['stable'] = pending['stable']+1 if at_destination and pending['ack'] else 0
        if pending['stable'] >= 4:
            self.events['carry'] += 1
            self.last_carry = {**pending['record'], 'service_ack': True,
                               'confirmed_at': self.now(), 'attempts': pending['attempts'],
                               'arm_pose_pause_ack': self.arm_paused_since is not None}
            self.get_logger().info('carried robot '+json.dumps(self.last_carry))
            self.pending_carry = None
            self.transfer_prepare_since = None
            return True
        # 5 s of simulation time is under 1 s of wall time at --speed max: a Gazebo service busy for a moment is
        # not a fault, so the deadline also needs 10 s of wall time
        if self.now()-pending['started'] > 5.0 and time.monotonic()-pending['wall_started'] > 10.0:
            self.transfer_error = 'SetEntityState did not produce a stable destination pose'
            self.phase = 'fault'
            self.get_logger().error(self.transfer_error+' '+json.dumps({'attempts': pending['attempts'],
                                    'ack': pending['ack'], 'robot_floor': self.robot_floor(),
                                    'target': pending['target'], 'arm_paused_since': self.arm_paused_since}))
            return False
        if not at_destination and pending['future'] is None:
            pending['future'] = self.set_entity(self.get_parameter('robot_entity').value,*pending['target'])
            pending['attempts'] += 1
        return False

    # ---------------------------------------------------------------- state machine
    def on_command(self, msg):
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning(f'ignoring malformed command {msg.data!r}')
            return
        kind = cmd.get('cmd')
        if kind in ('call', 'go'):
            self.request(kind, str(cmd.get('floor', '')).upper())
        elif kind == 'close' and self.phase == 'open':
            self.hold_until = self.now()
        elif kind == 'hold' and self.phase in ('open', 'opening'):
            self.hold_until = max(self.hold_until or 0, self.now()+float(cmd.get('seconds', 10)))
        elif kind == 'reopen':
            self.reopen = bool(cmd.get('value', True))
        self.get_logger().info(f'command {cmd} -> phase={self.phase} requests={self.requests}')

    def request(self, kind, floor):
        if floor not in ORDER:
            self.get_logger().warning(f'unknown floor {floor!r}')
            return
        self.events['calls'] += 1
        (self.hall_lit if kind == 'call' else self.car_lit).add(floor)   # button lamp on until the door opens there
        if floor not in self.requests:
            self.requests.append(floor)
        self.request_ready_at[floor] = self.now()+(float(self.timing['call_response_s']) if kind == 'call' else 0.0)
        if self.phase == 'open' and floor == self.current:
            self.hold_until = self.now()+float(self.timing['hold_open_s'])

    def on_joints(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.joints[name] = pos

    def on_arm_pause(self, msg):
        self.arm_paused_since = (self.arm_paused_since if self.arm_paused_since is not None else self.now()) if msg.data else None

    def check_buttons(self):
        """Arm-tip contact with a button face on the robot's floor -> hall call / car floor request."""
        names = [j['name'] for j in self.arm['joints']]
        if self.indicators is None or self.truth is None or any(n not in self.joints for n in names):
            return
        floor = self.robot_floor()
        entry = self.indicators['floors'].get(floor)
        if entry is None:
            return
        p = self.truth.pose.pose
        tip = fk(self.arm, self.joints, (p.position.x, p.position.y, p.position.z, yaw_of(p.orientation)))[0]
        buttons = {entry['hall_lit']: ('call', floor)} if entry.get('hall_lit') else {}
        buttons.update({name: ('go', target) for target, name in entry['car_lit'].items()})
        radius = float(self.contact['contact_radius_m'])
        lo, hi = self.contact['contact_depth_m']
        now_touching = set()
        for name, (kind, target) in buttons.items():
            model = entry['models'][name]
            face = np.array(model['on'][:3])
            n = np.array([math.cos(model['normal']), math.sin(model['normal']), 0.0])
            d = tip-face
            depth = -float(d@n)
            lateral = float(np.linalg.norm(d-(d@n)*n))
            if lateral <= radius and lo <= depth <= hi:
                now_touching.add(name)
                if name not in self.touching:
                    self.presses.append({'button': name, 'request': [kind, target], 'sim_time': self.now(),
                                         'lateral_m': round(lateral, 4), 'depth_m': round(depth, 4)})
                    self.get_logger().info(f'button pressed by arm tip: {self.presses[-1]}')
                    self.request(kind, target)
        self.touching = now_touching

    def tick(self):
        now = self.now()
        dt = 0.0 if self.last_tick is None else max(0.0, now-self.last_tick)
        self.last_tick = now
        t = self.timing
        if self.phase == 'idle':
            if self.requests:
                self.idle_since = self.idle_since or now
                if now >= self.request_ready_at.get(self.requests[0], now):
                    self.idle_since = None
                    target = self.requests[0]
                    if target == self.current:
                        self.requests.pop(0)
                        self.phase = 'opening'
                        self.arrived(self.current)
                    else:
                        self.phase = 'moving'
                        self.travel_to = target
                        self.carry_robot = self.inside(self.current)
                        hops = abs(ORDER.index(target)-ORDER.index(self.current))
                        self.travel_until = now+hops*float(t['travel_per_floor_s'])
                        self.events['travel'] += 1
        elif self.phase == 'moving':
            if now >= self.travel_until:
                carried = not self.carry_robot or self.carry(self.current, self.travel_to)
                if carried:
                    self.current = self.travel_to
                    self.requests = [f for f in self.requests if f != self.current]
                    self.travel_to = None
                    self.phase = 'opening'
                    self.arrived(self.current)
        elif self.phase == 'opening':
            self.fraction = min(1.0, self.fraction+dt/float(t['door_open_s']))
            if self.fraction >= 1.0:
                self.phase = 'open'
                self.hold_until = now+float(t['hold_open_s'])
        elif self.phase == 'open':
            if self.current in self.requests and now >= self.request_ready_at.get(self.current, now):
                self.requests.remove(self.current)
                self.arrived(self.current)
                self.hold_until = now+float(t['hold_open_s'])
            if now >= self.hold_until:
                self.phase = 'closing'
        elif self.phase == 'closing':
            if self.obstructing():
                if self.reopen:
                    self.events['reopen'] += 1
                    self.phase = 'opening'
                else:
                    self.events['door_stall'] += 1
            else:
                self.fraction = max(0.0, self.fraction-dt/float(t['door_close_s']))
                if self.fraction <= 0.0:
                    self.phase = 'idle'
        if self.get_parameter('button_contact').value:
            self.check_buttons()
        self.push_leaves()
        self.push_indicators()
        self.publish(now)

    def arrived(self, floor):
        self.car_lit.discard(floor)
        self.hall_lit.discard(floor)
        self.request_ready_at.pop(floor, None)

    def push_indicators(self):
        """Each cabin copy shows the elevator's current floor; lamps follow car_lit / hall_lit."""
        if self.indicators is None:
            return
        hidden = float(self.indicators['hidden_z'])
        for copy, entry in self.indicators['floors'].items():
            wanted = {name: shown == self.current for shown, name in entry['digits'].items()}
            wanted.update({name: target in self.car_lit for target, name in entry['car_lit'].items()})
            if entry.get('hall_lit'):
                wanted[entry['hall_lit']] = copy in self.hall_lit
            for name, on in wanted.items():
                if self.last_sent.get(name) == on:
                    continue
                x, y, z, yaw = entry['models'][name]['on']
                if self.place_visual(name, (x, y, z if on else z+hidden, yaw), on):
                    self.last_sent[name] = on

    def publish(self, now):
        door = {'idle': 'closed', 'moving': 'closed', 'opening': 'opening', 'open': 'open',
                'closing': 'closing', 'fault': 'closed'}[self.phase]
        self.seq += 1
        state = {'current_floor': self.current, 'door_state': door, 'door_fraction': round(self.fraction, 3),
                 'phase': self.phase, 'moving': self.phase == 'moving', 'target_floor': self.travel_to,
                 'transferring': self.phase == 'moving' and now >= self.travel_until-3.0,
                 'requests': self.requests, 'robot_floor': self.robot_floor(),
                 'robot_inside': self.inside() if self.truth is not None else None,
                 'robot_obstructing': self.obstructing() if self.truth is not None else None,
                 'reopen_on_obstruction': self.reopen, 'leaves': self.leaf_poses(self.current),
                 'transfer_error': self.transfer_error, 'events': self.events, 'last_carry': self.last_carry, 'seq': self.seq, 'sim_time': now,
                 'display_floor': self.current, 'car_lit': sorted(self.car_lit), 'hall_lit': sorted(self.hall_lit),
                 'button_presses': self.presses[-10:], 'touching': sorted(self.touching),
                 'validation_scope': 'simulation'}
        self.state_pub.publish(String(data=json.dumps(state)))


def spin_simulator(node, spin_once, okay):
    failures = 0
    while okay():
        try:
            spin_once(node, timeout_sec=.1)
        except RuntimeError as exc:
            if 'Unable to convert call argument to Python object' not in str(exc) or failures >= 3:
                raise
            failures += 1
            node.events['native_message_drop'] = failures
            node.get_logger().warning(f'dropped native ROS conversion error ({failures}/3); waiting for a fresh message')


def main():
    rclpy.init()
    node = Elevator()
    try:
        spin_simulator(node, rclpy.spin_once, rclpy.ok)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
