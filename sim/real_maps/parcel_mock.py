#!/usr/bin/env python3
"""Simulation-only parcel mock: attach/detach visual loads to the lift carrier (--arm-sim).

carry_mode 'pallet' (default, --load pallet): one parcel per 2-way plastic pallet. The first pallet lies to the right
of the f1_locker entry direction with its pocket facing that stop pose; stack_count (--parcels) > 1 adds waiting
pallets beside it (scenery, never carried). Attach needs the fork within attach_max_gap_m of the pocket entry, so
the robot must have found and docked to it (camera + odometry, skills.py).
The lift carrier (fork) enters the centre pocket at lift 0; on attach the pallet follows the fork and rests on it
once the fork top reaches the deck underside; on detach it is left on the floor.
Virtual front ultrasonic (/sim/ultrasonic_front, sensor_msgs/Range): geometric distance from the mast-front sensor
([가정값] world_params lift.ultrasonic) to the pallet/parcel boxes only (walls and people are not seen).
carry_mode 'box' (--load box, 10/8 reproduction): one box on the assumed locker shelf slot, plus stack_count-1
boxes stacked on it (world_params parcel.stack_items).

Loads have NO collision and NO mass: they are moved kinematically. Contact, grasp, pocket alignment and payload
load are never validated; attach is only refused when the fork is farther than max_attach_gap_m ([가정값]).
  sub /sim/parcel/command  std_msgs/String JSON {"cmd":"attach"} | {"cmd":"detach"}
  pub /sim/parcel/state    std_msgs/String JSON {state, pose(world), attach_gap_m, count, mode, ...}
"""
import json
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState, SpawnEntity
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Range
from std_msgs.msg import String
import yaml

from dressing import LIFT_BASE_X, LIFT_ORIGIN_Z, MAST_HALF, pallet_model_xml, parcel_model_xml, pickup_pallet_pose

HERE = Path(__file__).resolve().parent
MAPS = HERE.parents[1]/'maps/field'


def yaw_of(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def ahead(pose, dx, dy=0.0):
    x, y, z, yaw = pose
    return [x+math.cos(yaw)*dx-math.sin(yaw)*dy, y+math.sin(yaw)*dx+math.cos(yaw)*dy, z, yaw]


class Parcel(Node):
    def __init__(self):
        super().__init__('sim_parcel_mock')
        self.declare_parameter('max_attach_gap_m', 0.5)
        self.declare_parameter('stack_count', 1)
        self.declare_parameter('carry_mode', 'pallet')
        self.declare_parameter('initial_attached', False)  # scenario setup, never a successful pickup claim
        params = yaml.safe_load((HERE/'world_params.yaml').read_text())
        self.p = params['parcel']
        self.mode = str(self.get_parameter('carry_mode').value)
        count = int(self.get_parameter('stack_count').value)
        f1_offset = params['floors']['F1']['world_offset']
        wp = json.loads((MAPS/'waypoints.json').read_text())['waypoints']['f1_locker']
        stop = [wp['x']+f1_offset[0], wp['y']+f1_offset[1], f1_offset[2], wp['yaw_rad']]
        # Each item: entity name, SDF, pose relative to the base entity (dx, dy, dz, dyaw).
        self.scenery = []   # (name, xml, world pose, box) never carried
        if self.mode == 'pallet':
            pallet = self.pallet = self.p['pallet']
            waiting = pallet['waiting_offsets_m']
            if not 1 <= count <= 1+len(waiting):
                raise SystemExit(f'stack_count must be 1..{1+len(waiting)} pallets (one parcel each)')
            parcel, deck = pallet['parcel'], pallet['size_m'][2]
            rel = [parcel['xy'][0], parcel['xy'][1], deck+parcel['size_m'][2]/2, float(parcel['yaw_rad'])]
            self.items = [{'name': 'pallet', 'xml': pallet_model_xml('pallet', pallet), 'rel': [0.0, 0.0, 0.0, 0.0],
                           'box': (pallet['size_m'], 'bottom')},
                          {'name': 'parcel', 'xml': parcel_model_xml('parcel', parcel['size_m'], parcel['color']),
                           'rel': rel, 'box': (parcel['size_m'], 'centre')}]
            px, py, pyaw = pickup_pallet_pose([wp['x'], wp['y'], wp['yaw_rad']], pallet)
            self.pose = [px+f1_offset[0], py+f1_offset[1], f1_offset[2], pyaw]   # pallet bottom centre, floor
            self.max_gap = float(pallet['attach_max_gap_m'])
            for k, (dx, dy) in enumerate(waiting[:count-1]):
                base = ahead(self.pose, dx, dy)
                top = ahead(base, rel[0], rel[1])
                self.scenery += [(f'pallet_{k+2}', pallet_model_xml(f'pallet_{k+2}', pallet), base),
                                 (f'parcel_{k+2}', parcel_model_xml(f'parcel_{k+2}', parcel['size_m'],
                                                                     pallet['waiting_colors'][k]),
                                  [top[0], top[1], base[2]+rel[2], base[3]+rel[3]])]
        elif self.mode == 'box':
            extra = self.p.get('stack_items', [])
            if not 1 <= count <= 1+len(extra):
                raise SystemExit(f'stack_count must be 1..{1+len(extra)}')
            self.items = [{'name': 'parcel', 'xml': parcel_model_xml('parcel', self.p['size_m'], self.p['color']),
                           'rel': [0.0, 0.0, 0.0, 0.0], 'box': (self.p['size_m'], 'centre')}]
            top = self.p['size_m'][2]/2
            for k, item in enumerate(extra[:count-1]):
                h = item['size_m'][2]
                self.items.append({'name': f'parcel_{k+2}', 'xml': parcel_model_xml(f'parcel_{k+2}', item['size_m'],
                                                                                    item['color']),
                                   'rel': [0.0, 0.0, top+h/2, float(item.get('yaw_rad', 0.0))],
                                   'box': (item['size_m'], 'centre')})
                top += h
            self.max_gap = float(self.get_parameter('max_attach_gap_m').value)
            dx, dy, dz = self.p['locker_slot_offset_m']
            self.pose = ahead(stop, dx, dy)
            self.pose[2] += dz                                             # box centre on the assumed shelf
        else:
            raise SystemExit(f'unknown carry_mode {self.mode}')
        self.count = count
        self.us = params['lift']['ultrasonic']
        self.us_range = math.inf
        self.us_pub = self.create_publisher(Range, '/sim/ultrasonic_front', 10)
        self.create_timer(1.0/float(self.us['rate_hz']), self.ultrasonic)
        self.state, self.gap, self.events = ('attached' if self.get_parameter('initial_attached').value else 'waiting'), None, []
        if self.state == 'attached':
            self.events.append({'scenario_setup': 'initial_attached', 'pickup_validated': False})
        self.truth = None
        self.lift = 0.0
        self.create_subscription(Odometry, '/sim/ground_truth', lambda m: setattr(self, 'truth', m), 20)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 20)
        self.create_subscription(String, '/sim/parcel/command', self.on_command, 10)
        self.pub = self.create_publisher(String, '/sim/parcel/state', 10)
        self.set_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.spawned = False
        self.create_timer(.05, self.tick)

    def on_joints(self, msg):
        if 'lift_joint' in msg.name:
            self.lift = msg.position[msg.name.index('lift_joint')]

    def fork(self):
        """Lift carrier origin (fork) in world: LIFT_BASE_X ahead of the robot, at lift height."""
        p = self.truth.pose.pose
        return ahead([p.position.x, p.position.y, p.position.z+LIFT_ORIGIN_Z+self.lift, yaw_of(p.orientation)],
                     LIFT_BASE_X)

    def floor_z(self):
        return self.truth.pose.pose.position.z

    def carried_pose(self):
        """Base entity pose while attached."""
        fork = self.fork()
        if self.mode == 'pallet':
            pal = self.pallet
            base = ahead(fork, MAST_HALF+float(pal['rear_gap_m'])+pal['size_m'][0]/2)
            # The pallet sits on the floor until the fork top reaches the deck underside (pocket height).
            base[2] = max(self.floor_z(), fork[2]+float(pal['fork_half_thickness_m'])-float(pal['pocket_height_m']))
            return base
        ox, oy, oz = self.p['carrier_offset_m']
        base = ahead(fork, ox, oy)
        base[2] += oz
        return base

    def attach_gap(self):
        if self.mode == 'pallet':
            # Where the fork should be for this pallet: pocket entry at the lift-0 fork height.
            entry = ahead(self.pose, -(MAST_HALF+float(self.pallet['rear_gap_m'])+self.pallet['size_m'][0]/2))
            entry[2] = self.pose[2]+LIFT_ORIGIN_Z
            return math.dist(self.fork()[:3], entry[:3])
        return math.dist(self.carried_pose()[:3], self.pose[:3])

    def on_command(self, msg):
        cmd = json.loads(msg.data).get('cmd')
        if self.truth is None:
            return
        if cmd == 'attach' and self.state != 'attached':
            self.gap = self.attach_gap()
            ok = self.gap <= self.max_gap
            self.events.append({'cmd': 'attach', 'gap_m': self.gap, 'accepted': ok, 'lift_m': self.lift})
            if ok:
                self.state = 'attached'
        elif cmd == 'detach' and self.state == 'attached':
            self.state = 'placed'
            self.pose = self.carried_pose()
            self.pose[2] = self.floor_z()+(0.0 if self.mode == 'pallet' else self.p['size_m'][2]/2)
            self.events.append({'cmd': 'detach', 'pose': self.pose, 'count': self.count, 'lift_m': self.lift})
            self.send_all()
        self.get_logger().info(f'{self.mode} {cmd} -> {self.state} x{self.count} (mock, no contact/load validation)')

    def item_boxes(self):
        """(centre xyz, size, yaw) of every load box (carried items first, then waiting scenery)."""
        out = []
        for item in self.items:
            pose = self.item_pose(item)
            size, origin = item['box']
            out.append(([pose[0], pose[1], pose[2]+(size[2]/2 if origin == 'bottom' else 0.0)], size, pose[3]))
        return out

    def ultrasonic(self):
        """Virtual front ultrasonic: nearest load box along 5 rays of the beam cone (centre + 4 edges)."""
        if self.truth is None:
            return
        p = self.truth.pose.pose
        yaw = yaw_of(p.orientation)
        mx, my, mz = self.us['mount_xyz']
        origin = ahead([p.position.x, p.position.y, p.position.z+mz, yaw], mx, my)
        half = float(self.us['field_of_view_rad'])/2
        best = math.inf
        for dyaw, dpitch in ((0, 0), (half, 0), (-half, 0), (0, half), (0, -half)):
            d = (math.cos(yaw+dyaw)*math.cos(dpitch), math.sin(yaw+dyaw)*math.cos(dpitch), math.sin(dpitch))
            for centre, size, byaw in self.item_boxes():
                hit = ray_box(origin[:3], d, centre, size, byaw)
                if hit is not None and float(self.us['range_min_m']) <= hit <= float(self.us['range_max_m']):
                    best = min(best, hit)
        self.us_range = best
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ultrasonic_front'
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = float(self.us['field_of_view_rad'])
        msg.min_range, msg.max_range = float(self.us['range_min_m']), float(self.us['range_max_m'])
        msg.range = best
        self.us_pub.publish(msg)

    def load_top(self):
        """Highest point of the carried items above the floor (m)."""
        if self.truth is None:
            return None
        return max(c[2]+s[2]/2 for c, s, _ in self.item_boxes()[:len(self.items)])-self.floor_z()

    def item_pose(self, item):
        dx, dy, dz, dyaw = item['rel']
        pose = ahead(self.pose, dx, dy)
        return [pose[0], pose[1], pose[2]+dz, pose[3]+dyaw]

    def send_all(self):
        for item in self.items:
            self.send(self.item_pose(item), item['name'])

    def send(self, pose, name):
        if not self.set_client.service_is_ready():
            return
        req = SetEntityState.Request()
        req.state.name, req.state.reference_frame = name, 'world'
        req.state.pose.position.x, req.state.pose.position.y, req.state.pose.position.z = pose[:3]
        req.state.pose.orientation.z, req.state.pose.orientation.w = math.sin(pose[3]/2), math.cos(pose[3]/2)
        self.set_client.call_async(req)

    def tick(self):
        if not self.spawned and self.spawn_client.service_is_ready():
            for name, xml, pose in [(i['name'], i['xml'], self.item_pose(i)) for i in self.items]+self.scenery:
                req = SpawnEntity.Request()
                req.name, req.xml = name, xml
                req.initial_pose.position.x, req.initial_pose.position.y, req.initial_pose.position.z = pose[:3]
                req.initial_pose.orientation.z = math.sin(pose[3]/2)
                req.initial_pose.orientation.w = math.cos(pose[3]/2)
                self.spawn_client.call_async(req)
            self.spawned = True
        if self.state == 'attached' and self.truth is not None:
            self.pose = self.carried_pose()
            self.send_all()
        self.pub.publish(String(data=json.dumps({'state': self.state, 'pose': self.pose, 'attach_gap_m': self.gap,
                                                 'count': self.count, 'mode': self.mode, 'events': self.events,
                                                 'load_top_m': self.load_top(),
                                                 'ultrasonic_range_m': None if math.isinf(self.us_range) else self.us_range,
                                                 'note': self.p['mode']})))


def ray_box(origin, direction, centre, size, yaw):
    """Distance along a unit ray to a yaw-rotated box (slab test), None if missed."""
    c, s = math.cos(-yaw), math.sin(-yaw)
    ox, oy, oz = (origin[i]-centre[i] for i in range(3))
    o = (c*ox-s*oy, s*ox+c*oy, oz)
    d = (c*direction[0]-s*direction[1], s*direction[0]+c*direction[1], direction[2])
    near, far = -math.inf, math.inf
    for axis in range(3):
        half = size[axis]/2
        if abs(d[axis]) < 1e-12:
            if abs(o[axis]) > half:
                return None
            continue
        t1, t2 = (-half-o[axis])/d[axis], (half-o[axis])/d[axis]
        near, far = max(near, min(t1, t2)), min(far, max(t1, t2))
    if near > far or far < 0:
        return None
    return max(near, 0.0)


def main():
    rclpy.init()
    node = Parcel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
