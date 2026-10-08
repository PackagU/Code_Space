#!/usr/bin/env python3
"""Drive-model diagnostic: wheel-odometry (encoder) vs ground truth on a flat floor.

Applies caster contact variants to generated/robot.urdf, then commands /sim/cmd_vel_drive directly
(diff drive input) and compares /odom (encoder) with /sim/ground_truth for: idle creep, straight
0.08 m/s, in-place rotation 0.25 rad/s. Model diagnostic only — not a Nav2 scenario, not hardware.
  python3 sim/real_maps/probe_drive_slip.py --name slip_probe_YYYYmmdd [--variants V0,V1]
"""
from host_exec import ensure_container
ensure_container(__file__)

import argparse                       # noqa: E402
import json                           # noqa: E402
import math                           # noqa: E402
import time                           # noqa: E402
import xml.etree.ElementTree as ET    # noqa: E402

import rclpy                          # noqa: E402
from rclpy.node import Node           # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from geometry_msgs.msg import Twist   # noqa: E402
from nav_msgs.msg import Odometry     # noqa: E402

from run_scenarios import Processes, pose_values   # noqa: E402
from sim_stack import HERE, LOG, assert_isolated    # noqa: E402

# name: (front aux caster lift m, rear caster mu, front caster mu)
VARIANTS = {'V0': (0.0, .5, .5), 'V1': (.002, .5, .5), 'V2': (.002, .3, .3), 'V3': (0.0, .3, .3),
            'V4': (.002, .1, .1)}
STEPS = [('idle', 0.0, 0.0, 10.0), ('straight', .08, 0.0, 6.0), ('stop1', 0.0, 0.0, 2.0),
         ('rotate', 0.0, .25, 8.0), ('stop2', 0.0, 0.0, 2.0), ('arc', .08, .2, 6.0), ('stop3', 0.0, 0.0, 10.0)]


def variant_urdf(name, directory):
    lift, rear_mu, front_mu = VARIANTS[name]
    # Original 9/15 contacts (robot_world.urdf) with encoder odometry, so variants are absolute.
    tree = ET.parse(HERE/'generated/robot_world.urdf').getroot()
    tree.find(".//plugin[@name='diff_drive']/odometry_source").text = '0'
    joint = tree.find("joint[@name='caster_front_joint']/origin")
    xyz = [float(v) for v in joint.get('xyz').split()]
    xyz[2] += lift
    joint.set('xyz', ' '.join(str(v) for v in xyz))
    for ref, mu in (('caster_wheel', rear_mu), ('caster_wheel_front', front_mu)):
        g = tree.find(f"gazebo[@reference='{ref}']")
        g.find('mu1').text = g.find('mu2').text = str(mu)
    path = directory/f'robot_{name}.urdf'
    ET.ElementTree(tree).write(path, encoding='utf-8', xml_declaration=False)
    return path


def flat_world(directory):
    text = (HERE/'generated/f1.world').read_text()
    root = ET.fromstring(text.split('?>', 1)[1] if text.startswith('<?xml') else text)
    world = root.find('world')
    world.remove(world.find("model[@name='f1_real_building']"))
    path = directory/'flat.world'
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)
    return path


class Probe(Node):
    def __init__(self):
        super().__init__('drive_slip_probe')
        self.set_parameters([Parameter('use_sim_time', value=True)])
        self.odom = self.truth = None
        self.create_subscription(Odometry, '/odom', lambda m: setattr(self, 'odom', m), 20)
        self.create_subscription(Odometry, '/sim/ground_truth', lambda m: setattr(self, 'truth', m), 20)
        self.pub = self.create_publisher(Twist, '/sim/cmd_vel_drive', 10)

    def now(self):
        return self.get_clock().now().nanoseconds/1e9

    def run(self, v, w, seconds):
        end = self.now()+seconds
        wall_end = time.monotonic()+seconds*10+10
        while self.now() < end and time.monotonic() < wall_end:
            msg = Twist()
            msg.linear.x, msg.angular.z = float(v), float(w)
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=.02)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--name', required=True)
    parser.add_argument('--variants', default=','.join(VARIANTS))
    args = parser.parse_args()
    assert_isolated()
    base = LOG/args.name
    base.mkdir(parents=True, exist_ok=True)
    world = flat_world(base)
    rclpy.init()
    summary = []
    for name in args.variants.split(','):
        directory = base/name
        directory.mkdir(exist_ok=True)
        urdf = variant_urdf(name, directory)
        procs = Processes(directory)
        node = Probe()
        try:
            procs.start('gazebo', ['gzserver', '--verbose', str(world), '-s', 'libgazebo_ros_init.so',
                                   '-s', 'libgazebo_ros_factory.so'])
            procs.start('spawn', ['ros2', 'run', 'gazebo_ros', 'spawn_entity.py', '-entity', 'probe', '-file',
                                  str(urdf), '-x', '0', '-y', '0', '-z', '.005', '-timeout', '60'])
            end = time.monotonic()+90
            while (node.odom is None or node.truth is None) and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.1)
            if node.odom is None or node.truth is None:
                raise RuntimeError('odom/truth unavailable')
            node.run(0, 0, 3)
            row = {'variant': name, 'front_lift_m': VARIANTS[name][0], 'rear_mu': VARIANTS[name][1],
                   'front_mu': VARIANTS[name][2]}
            for label, v, w, seconds in STEPS:
                o0, t0 = pose_values(node.odom.pose.pose), pose_values(node.truth.pose.pose)
                node.run(v, w, seconds)
                o1, t1 = pose_values(node.odom.pose.pose), pose_values(node.truth.pose.pose)
                row[label] = {'odom_dist': math.dist(o0[:2], o1[:2]), 'truth_dist': math.dist(t0[:2], t1[:2]),
                              'odom_dyaw': math.atan2(math.sin(o1[2]-o0[2]), math.cos(o1[2]-o0[2])),
                              'truth_dyaw': math.atan2(math.sin(t1[2]-t0[2]), math.cos(t1[2]-t0[2]))}
            s, r = row['straight'], row['rotate']
            row['straight_slip_ratio'] = s['odom_dist']/s['truth_dist'] if s['truth_dist'] > 1e-6 else None
            row['rotate_slip_ratio'] = r['odom_dyaw']/r['truth_dyaw'] if abs(r['truth_dyaw']) > 1e-6 else None
            row['idle_creep_m_per_s'] = row['idle']['truth_dist']/10.0
            row['final_stop_creep_m_per_s'] = row['stop3']['truth_dist']/10.0
            print(json.dumps(row), flush=True)
            summary.append(row)
        except Exception as exc:
            summary.append({'variant': name, 'error': str(exc)})
            print('ERROR', name, exc, flush=True)
        finally:
            procs.cleanup()
            node.destroy_node()
            time.sleep(2)
    (base/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
