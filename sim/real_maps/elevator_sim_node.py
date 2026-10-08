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
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import yaml

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
        params = yaml.safe_load((HERE/'world_params.yaml').read_text())
        self.e = params['elevator']
        self.offsets = {f: params['floors'][f]['world_offset'] for f in ORDER}
        self.doors = json.loads((HERE/'generated/doors.json').read_text())
        wps = json.loads((MAPS/'waypoints.json').read_text())['waypoints']
        self.cabin = {f: (wps[self.e[f]['inside_waypoint']]['x'], wps[self.e[f]['inside_waypoint']]['y'],
                          wps[self.e[f]['inside_waypoint']]['yaw_rad']) for f in ORDER}
        self.timing = self.e['timing']
        self.reopen = bool(self.e['reopen_on_obstruction'])
        self.current = self.e['initial_floor']
        self.fraction = 0.0
        self.phase = 'idle'
        self.requests = []
        self.hold_until = self.travel_until = self.idle_since = None
        self.travel_to = None
        self.carry_robot = False
        self.events = {'calls': 0, 'reopen': 0, 'door_stall': 0, 'carry': 0, 'travel': 0}
        self.last_carry = None
        self.truth = None
        self.seq = 0
        self.last_sent = {}
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
        return minimum_clearance(np.array([self.door_box(self.current)]), x, y, yaw) <= .02

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
        depth = (polygon(x, y, yaw)-[cx, cy]) @ normal
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
            return False
        req = SetEntityState.Request()
        req.state.name, req.state.reference_frame = name, 'world'
        req.state.pose.position.x, req.state.pose.position.y, req.state.pose.position.z = x, y, z
        req.state.pose.orientation.z, req.state.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        self.client.call_async(req)
        return True

    def push_leaves(self, force=False):
        for floor in ORDER:
            off = self.offsets[floor]
            for name, (x, y, yaw) in self.leaf_poses(floor).items():
                key = (round(x, 3), round(y, 3))
                if force or self.last_sent.get(name) != key:
                    if self.set_entity(name, x+off[0], y+off[1], off[2], yaw):
                        self.last_sent[name] = key

    def carry(self, src, dst):
        """Move the robot entity to the destination cabin, preserving cabin-relative pose and yaw."""
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
        self.set_entity(self.get_parameter('robot_entity').value, nx+off[0], ny+off[1], z, nyaw)
        self.events['carry'] += 1
        self.last_carry = {'from': src, 'to': dst, 'stamp': self.now(), 'src_map_pose': [x, y, yaw],
                           'cabin_relative': list(rel), 'dst_map_pose': [nx, ny, nyaw]}
        self.get_logger().info('carried robot '+json.dumps(self.last_carry))

    # ---------------------------------------------------------------- state machine
    def on_command(self, msg):
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning(f'ignoring malformed command {msg.data!r}')
            return
        kind = cmd.get('cmd')
        if kind in ('call', 'go'):
            floor = str(cmd.get('floor', '')).upper()
            if floor not in ORDER:
                self.get_logger().warning(f'unknown floor {floor!r}')
                return
            self.events['calls'] += 1
            if floor not in self.requests:
                self.requests.append(floor)
            if self.phase == 'open' and floor == self.current:
                self.hold_until = self.now()+float(self.timing['hold_open_s'])
        elif kind == 'close' and self.phase == 'open':
            self.hold_until = self.now()
        elif kind == 'hold' and self.phase in ('open', 'opening'):
            self.hold_until = max(self.hold_until or 0, self.now()+float(cmd.get('seconds', 10)))
        elif kind == 'reopen':
            self.reopen = bool(cmd.get('value', True))
        self.get_logger().info(f'command {cmd} -> phase={self.phase} requests={self.requests}')

    def tick(self):
        now = self.now()
        dt = 0.0 if self.last_tick is None else max(0.0, now-self.last_tick)
        self.last_tick = now
        t = self.timing
        if self.phase == 'idle':
            if self.requests:
                self.idle_since = self.idle_since or now
                if now-self.idle_since >= float(t['call_response_s']):
                    self.idle_since = None
                    target = self.requests[0]
                    if target == self.current:
                        self.requests.pop(0)
                        self.phase = 'opening'
                    else:
                        self.phase = 'moving'
                        self.travel_to = target
                        self.carry_robot = self.inside(self.current)
                        hops = abs(ORDER.index(target)-ORDER.index(self.current))
                        self.travel_until = now+hops*float(t['travel_per_floor_s'])
                        self.events['travel'] += 1
        elif self.phase == 'moving':
            if now >= self.travel_until:
                if self.carry_robot:
                    self.carry(self.current, self.travel_to)
                self.current = self.travel_to
                self.requests = [f for f in self.requests if f != self.current]
                self.travel_to = None
                self.phase = 'opening'
        elif self.phase == 'opening':
            self.fraction = min(1.0, self.fraction+dt/float(t['door_open_s']))
            if self.fraction >= 1.0:
                self.phase = 'open'
                self.hold_until = now+float(t['hold_open_s'])
        elif self.phase == 'open':
            if self.current in self.requests:
                self.requests.remove(self.current)
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
        self.push_leaves()
        self.publish(now)

    def publish(self, now):
        door = {'idle': 'closed', 'moving': 'closed', 'opening': 'opening', 'open': 'open',
                'closing': 'closing'}[self.phase]
        self.seq += 1
        state = {'current_floor': self.current, 'door_state': door, 'door_fraction': round(self.fraction, 3),
                 'phase': self.phase, 'moving': self.phase == 'moving', 'target_floor': self.travel_to,
                 'requests': self.requests, 'robot_floor': self.robot_floor(),
                 'robot_inside': self.inside() if self.truth is not None else None,
                 'robot_obstructing': self.obstructing() if self.truth is not None else None,
                 'reopen_on_obstruction': self.reopen, 'leaves': self.leaf_poses(self.current),
                 'events': self.events, 'last_carry': self.last_carry, 'seq': self.seq, 'sim_time': now,
                 'validation_scope': 'simulation'}
        self.state_pub.publish(String(data=json.dumps(state)))


def main():
    rclpy.init()
    node = Elevator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
