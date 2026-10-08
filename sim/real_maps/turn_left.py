#!/usr/bin/env python3
"""User-requested in-place turn through the actual WASD teleop; hold stop after."""
import argparse
import json
import math
import os
import pty
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from std_msgs.msg import Bool, String
from run_scenarios import ROOT, HERE, LOG, Observer, pose_values


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--degrees', type=float, default=90)
    args = parser.parse_args()
    if not 0 < args.degrees <= 180:
        raise SystemExit('degrees must be 0..180')
    rclpy.init()
    node = Observer()
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
    if not node.wait(lambda: node.truth is not None, 10):
        raise SystemExit('ground truth unavailable')
    # Fail closed if any Nav2 controller command source is still running.
    for path in Path('/proc').glob('[0-9]*/cmdline'):
        try:
            executable = path.read_bytes().split(b'\0')[0]
        except OSError:
            continue
        if executable.endswith(b'/nav2_controller/controller_server'):
            raise SystemExit('stop Nav2 before the user manual turn')
    initial = pose_values(node.truth.pose.pose)
    target = initial[2]+math.radians(args.degrees)
    marker = node.create_publisher(String, '/sim/manual_stage', 10)
    keys = node.create_publisher(String, '/sim/manual_key', 10)
    master, slave = pty.openpty()
    directory = LOG/'user_manual_control'
    directory.mkdir(exist_ok=True)
    log = open(directory/f'turn_{time.time_ns()}.log', 'w')
    proc = subprocess.Popen([sys.executable, str(ROOT/'src/drive_pkg/drive_pkg/keyboard_teleop.py'),
        '--ros-args', '-p', 'max_linear_speed:=0.05', '-p', 'max_angular_speed:=0.20'],
        stdin=slave, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    os.close(slave)
    result = {'action': 'user_left_turn', 'degrees': args.degrees, 'initial_pose': initial,
              'controller': 'actual WASD a/k on PTY', 'scope': 'simulation'}
    try:
        node.pause(1)
        marker.publish(String(data=json.dumps(result)))
        node.stop.publish(Bool(data=False))
        node.pause(.3)
        deadline = time.monotonic()+45
        while True:
            yaw = pose_values(node.truth.pose.pose)[2]
            error = math.atan2(math.sin(target-yaw), math.cos(target-yaw))
            if abs(error) < .035:
                break
            if time.monotonic() > deadline or proc.poll() is not None:
                raise RuntimeError('manual turn did not finish within deadline')
            key = b'a' if error > 0 else b'd'
            os.write(master, key)
            keys.publish(String(data=json.dumps({'stage': 'user_left_turn', 'key': key.decode()})))
            node.pause(.12)
        os.write(master, b'k')
        node.pause(.7)
        result['completed'] = True
    except Exception as exc:
        result['completed'], result['error'] = False, str(exc)
    finally:
        os.write(master, b'k\x03')
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, 2)
            proc.wait(timeout=3)
        os.close(master)
        log.close()
        for _ in range(3):
            node.stop.publish(Bool(data=True))
            node.pause(.2)
        result['final_pose'] = pose_values(node.truth.pose.pose)
        result['yaw_error_deg'] = math.degrees(math.atan2(math.sin(target-result['final_pose'][2]),
                                                        math.cos(target-result['final_pose'][2])))
        result['translation_m'] = math.dist(initial[:2], result['final_pose'][:2])
        result['software_stop'] = True
        (directory/f'turn_result_{time.time_ns()}.json').write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps(result), flush=True)
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if result['completed'] else 1)


if __name__ == '__main__':
    main()
