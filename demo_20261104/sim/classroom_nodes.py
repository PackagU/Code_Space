#!/usr/bin/env python3
"""Simulation-only scene nodes for the classroom demo (run inside the isolated container).

  python3 classroom_nodes.py parcels  --ros-args -p use_sim_time:=true
      sim/real_maps parcel_mock.Parcel (attach/detach mock, virtual ultrasonic) with the classroom layout: P1 and P2
      pallets at area A, each parcel carrying its shipping label (bold digits) on the face that looks at the robot.
  python3 classroom_nodes.py elevator --ros-args -p use_sim_time:=true
      The elevator stand-in at wall B: an arm-tip contact with a cap face (assumed arm FK + ground truth, the same
      criterion as sim/real_maps elevator_sim_node) registers a press; a floor button press moves the floor display
      one floor per floor_step_s toward that floor (the operator's role in the real demo). Publishes /classroom/state.
Ground truth is used here only as the world side (contact, parcel pose); the robot never reads these topics.
"""
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
REAL = HERE.parents[1]/'sim/real_maps'
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(REAL))
import rclpy                                             # noqa: E402
from rclpy.node import Node                              # noqa: E402
from gazebo_msgs.srv import SetEntityState               # noqa: E402
from nav_msgs.msg import Odometry                        # noqa: E402
from sensor_msgs.msg import JointState                   # noqa: E402
from std_msgs.msg import String                          # noqa: E402

import dressing as dr                                    # noqa: E402
import glyphs                                            # noqa: E402
from arm_geometry import fk                              # noqa: E402

GEN = HERE/'generated'
WORLD = yaml.safe_load((REAL/'world_params.yaml').read_text())
CFG = yaml.safe_load((HERE/'classroom.yaml').read_text())


def yaw_of(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def scene():
    return json.loads((GEN/'classroom_scene.json').read_text())


# ------------------------------------------------------------------ labelled parcel
def labeled_parcel_xml(name, size, color, text, label):
    """Cardboard box (origin at its centre) with tape on top and the shipping label on its -x face (the face the
    robot sees while docking: the pallet's fork entry side). Digits read left to right for a viewer facing +x."""
    sdf = ET.Element('sdf', version='1.6')
    model = ET.SubElement(sdf, 'model', name=name)
    ET.SubElement(model, 'static').text = 'true'
    link = ET.SubElement(model, 'link', name='box')
    sx, sy, sz = size
    dr.add_visual(link, 'body', (0, 0, 0), (0, 0, 0), 'box', (sx, sy, sz), dr.rgba(color))
    dr.add_visual(link, 'tape_top', (0, 0, sz/2+.0008), (0, 0, 0), 'box', (sx+.002, .048, .0016), dr.TAPE)
    lw, lh = label['size_m']
    face = -sx/2
    dr.add_visual(link, 'label', (face-.0008, 0, 0), (0, 0, 0), 'box', (.0016, lw, lh), dr.WHITE)
    gw, gh = label['glyph_m']
    right = (0.0, -1.0)                                  # viewer facing +x: right hand is -y
    for k, ch in enumerate(str(text)):
        u0 = (k-(len(text)-1)/2)*(gw+float(label['gap_m']))
        for i, (u, v, du, dv) in enumerate(glyphs.label_strokes(ch, gw, gh, float(label['stroke_m']))):
            uu = u0+u
            dr.add_visual(link, f'glyph_{k}_{i}', (face-.0018, right[1]*uu, v), (0, 0, 0), 'box', (.0012, du, dv),
                          dr.BLACK)
    return ET.tostring(sdf, encoding='unicode')


def run_parcels():
    from parcel_mock import Parcel, ahead
    from dressing import pallet_model_xml

    class ClassroomParcels(Parcel):
        def __init__(self):
            super().__init__()                            # pallet mode, mock, ultrasonic (f1 layout replaced below)
            sc = scene()
            yaw = float(sc['parcels']['pallet_yaw'])
            items, label = sc['parcels']['items'], sc['parcels']['label']
            pal = self.pallet
            parcel = pal['parcel']
            deck = pal['size_m'][2]
            rel = [parcel['xy'][0], parcel['xy'][1], deck+parcel['size_m'][2]/2, float(parcel['yaw_rad'])]
            self.items[1]['xml'] = labeled_parcel_xml('parcel', parcel['size_m'], parcel['color'], items[0]['label'], label)
            self.items[1]['rel'] = rel
            self.pose = [items[0]['xy'][0], items[0]['xy'][1], 0.0, yaw]
            self.scenery = []
            for k, item in enumerate(items[1:], start=2):
                base = [item['xy'][0], item['xy'][1], 0.0, yaw]
                top = ahead(base, rel[0], rel[1])
                self.scenery += [(f'pallet_{k}', pallet_model_xml(f'pallet_{k}', pal), base),
                                 (f'parcel_{k}', labeled_parcel_xml(f'parcel_{k}', parcel['size_m'],
                                                                    pal['waiting_colors'][0], item['label'], label),
                                  [top[0], top[1], rel[2], yaw+rel[3]])]
            self.get_logger().info(f'classroom parcels: P1 {items[0]} carried, others {items[1:]} (mock, no contact)')

    rclpy.init()
    node = ClassroomParcels()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


# ------------------------------------------------------------------ buttons + floor display
class ClassroomElevator(Node):
    def __init__(self):
        super().__init__('classroom_elevator')
        sc = scene()
        self.caps = sc['caps']
        self.digits, self.hidden = sc['display']['digits'], float(sc['display']['hidden_z'])
        self.arm, self.contact = WORLD['arm'], WORLD['perception']
        self.step_s = float(CFG['elevator']['floor_step_s'])
        self.current, self.target, self.next_at = int(CFG['elevator']['initial_floor']), None, None
        self.joints, self.truth, self.touching, self.presses, self.moves = {}, None, set(), [], []
        self.shown, self.pending = {}, {}
        self.create_subscription(JointState, '/joint_states', self.on_joints, 20)
        self.create_subscription(Odometry, '/sim/ground_truth', lambda m: setattr(self, 'truth', m), 10)
        self.pub = self.create_publisher(String, '/classroom/state', 10)
        self.client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.create_timer(.05, self.tick)
        self.last_pub = None
        self.get_logger().info(f'classroom elevator stand-in: caps {sorted(self.caps)} display {self.current}')

    def now(self):
        return self.get_clock().now().nanoseconds/1e9

    def on_joints(self, msg):
        self.joints.update(dict(zip(msg.name, msg.position)))

    def check_buttons(self):
        names = [j['name'] for j in self.arm['joints']]
        if self.truth is None or any(n not in self.joints for n in names):
            return
        p = self.truth.pose.pose
        tip = fk(self.arm, self.joints, (p.position.x, p.position.y, p.position.z, yaw_of(p.orientation)))[0]
        radius = float(self.contact['contact_radius_m'])
        lo, hi = self.contact['contact_depth_m']
        now_touching = set()
        for label, cap in self.caps.items():
            face = np.array(cap['xyz'])
            n = np.array([math.cos(cap['normal']), math.sin(cap['normal']), 0.0])
            d = tip-face
            depth, lateral = -float(d@n), float(np.linalg.norm(d-(d@n)*n))
            if lateral <= radius and lo <= depth <= hi:
                now_touching.add(label)
                if label not in self.touching:
                    self.presses.append({'label': label, 'sim_time': round(self.now(), 2), 'lateral_m': round(lateral, 4),
                                         'depth_m': round(depth, 4)})
                    self.get_logger().info(f'button pressed by arm tip: {self.presses[-1]}')
                    if label in '1234' and int(label) != self.current:
                        self.target, self.next_at = int(label), self.now()+self.step_s
        self.touching = now_touching

    def presses_changed(self):
        return len(self.presses) != getattr(self, 'published_presses', 0)

    def show(self):
        for d, entry in self.digits.items():
            on = int(d) == self.current
            if self.shown.get(d) == on:
                continue
            fut = self.pending.get(d)
            if fut is not None and not fut.done():
                continue
            if fut is not None and fut.done():
                self.pending.pop(d)
                res = fut.result()
                if res is not None and res.success and getattr(fut, 'want', None) == on:
                    self.shown[d] = on
                    continue
            if not self.client.service_is_ready():
                return
            x, y, z, yaw = entry['on']
            req = SetEntityState.Request()
            req.state.name, req.state.reference_frame = entry['name'], 'world'
            req.state.pose.position.x, req.state.pose.position.y = x, y
            req.state.pose.position.z = z if on else z+self.hidden
            req.state.pose.orientation.z, req.state.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
            fut = self.client.call_async(req)
            fut.want = on
            self.pending[d] = fut

    def tick(self):
        self.check_buttons()
        if self.target is not None and self.now() >= self.next_at:
            self.current += 1 if self.target > self.current else -1
            self.moves.append({'floor': self.current, 'sim_time': round(self.now(), 2)})
            self.next_at = self.now()+self.step_s
            if self.current == self.target:
                self.target = None
        self.show()
        if self.last_pub is not None and self.now()-self.last_pub < .2 and not self.presses_changed():
            return                                           # 5 Hz state (contact is still checked at 20 Hz)
        self.last_pub = self.now()
        self.published_presses = len(self.presses)
        shown = [int(d) for d, on in self.shown.items() if on]
        self.pub.publish(String(data=json.dumps({'display_floor': self.current, 'display_shown': shown,
                                                 'moving_to': self.target, 'presses': self.presses,
                                                 'moves': self.moves[-20:], 'touching': sorted(self.touching),
                                                 'validation_scope': 'simulation'})))


def run_elevator():
    rclpy.init()
    node = ClassroomElevator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    which = sys.argv[1] if len(sys.argv) > 1 else ''
    sys.argv = [sys.argv[0]]+sys.argv[2:]
    {'parcels': run_parcels, 'elevator': run_elevator}[which]()
