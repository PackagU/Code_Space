#!/usr/bin/env python3
"""Simulation-only parcel mock: attach/detach a visual box to the lift carrier (--arm-sim).

The box has NO collision and NO mass: it is moved kinematically to follow the lift carrier while
attached and placed on the floor when detached. It never validates contact, grasp or payload load.
  sub /sim/parcel/command  std_msgs/String JSON {"cmd":"attach"} | {"cmd":"detach"}
  pub /sim/parcel/state    std_msgs/String JSON {state, pose(world), attach_gap_m, ...}
Attach is refused when the carrier target point is farther than max_attach_gap_m from the box
([가정값] reach check; locker slot pose and box size are assumptions in world_params.yaml parcel).
"""
import json
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState, SpawnEntity
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import yaml

HERE = Path(__file__).resolve().parent
MAPS = HERE.parents[1]/'maps/field'
LIFT_BASE_X, LIFT_ORIGIN_Z = 0.165, 0.05   # URDF lead_screw_base_joint x, lift_joint origin z


def yaw_of(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


class Parcel(Node):
    def __init__(self):
        super().__init__('sim_parcel_mock')
        self.declare_parameter('max_attach_gap_m', 0.5)
        params = yaml.safe_load((HERE/'world_params.yaml').read_text())
        self.p = params['parcel']
        f1_offset = params['floors']['F1']['world_offset']
        wp = json.loads((MAPS/'waypoints.json').read_text())['waypoints']['f1_locker']
        dx, dy, dz = self.p['locker_slot_offset_m']
        c, s = math.cos(wp['yaw_rad']), math.sin(wp['yaw_rad'])
        self.pose = [wp['x']+c*dx-s*dy+f1_offset[0], wp['y']+s*dx+c*dy+f1_offset[1], dz+f1_offset[2],
                     wp['yaw_rad']]
        self.state, self.gap, self.events = 'on_shelf', None, []
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

    def carrier_target(self):
        p = self.truth.pose.pose
        yaw = yaw_of(p.orientation)
        ox, oy, oz = self.p['carrier_offset_m']
        x = LIFT_BASE_X+ox
        return [p.position.x+math.cos(yaw)*x-math.sin(yaw)*oy, p.position.y+math.sin(yaw)*x+math.cos(yaw)*oy,
                p.position.z+LIFT_ORIGIN_Z+self.lift+oz, yaw]

    def on_command(self, msg):
        cmd = json.loads(msg.data).get('cmd')
        if self.truth is None:
            return
        target = self.carrier_target()
        if cmd == 'attach' and self.state != 'attached':
            self.gap = math.dist(target[:3], self.pose[:3])
            ok = self.gap <= float(self.get_parameter('max_attach_gap_m').value)
            self.events.append({'cmd': 'attach', 'gap_m': self.gap, 'accepted': ok})
            if ok:
                self.state = 'attached'
        elif cmd == 'detach' and self.state == 'attached':
            self.state = 'placed'
            self.pose = [target[0], target[1], self.truth.pose.pose.position.z+self.p['size_m'][2]/2, target[3]]
            self.events.append({'cmd': 'detach', 'pose': self.pose})
            self.send(self.pose)
        self.get_logger().info(f'parcel {cmd} -> {self.state} (mock, no contact/load validation)')

    def send(self, pose):
        if not self.set_client.service_is_ready():
            return
        req = SetEntityState.Request()
        req.state.name, req.state.reference_frame = 'parcel', 'world'
        req.state.pose.position.x, req.state.pose.position.y, req.state.pose.position.z = pose[:3]
        req.state.pose.orientation.z, req.state.pose.orientation.w = math.sin(pose[3]/2), math.cos(pose[3]/2)
        self.set_client.call_async(req)

    def tick(self):
        if not self.spawned and self.spawn_client.service_is_ready():
            sx, sy, sz = self.p['size_m']
            req = SpawnEntity.Request()
            req.name = 'parcel'
            req.xml = (f'<sdf version="1.6"><model name="parcel"><static>true</static><link name="box"><visual name="v">'
                       f'<geometry><box><size>{sx} {sy} {sz}</size></box></geometry><material><ambient>.6 .4 .2 1</ambient>'
                       f'<diffuse>.6 .4 .2 1</diffuse></material></visual></link></model></sdf>')
            req.initial_pose.position.x, req.initial_pose.position.y, req.initial_pose.position.z = self.pose[:3]
            req.initial_pose.orientation.z = math.sin(self.pose[3]/2)
            req.initial_pose.orientation.w = math.cos(self.pose[3]/2)
            self.spawn_client.call_async(req)
            self.spawned = True
        if self.state == 'attached' and self.truth is not None:
            self.pose = self.carrier_target()
            self.send(self.pose)
        self.pub.publish(String(data=json.dumps({'state': self.state, 'pose': self.pose, 'attach_gap_m': self.gap,
                                                 'events': self.events, 'mode': self.p['mode']})))


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
