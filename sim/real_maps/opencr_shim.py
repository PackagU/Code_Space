#!/usr/bin/env python3
"""Reuse the actual bridge command tick with a simulated serial transport.

No hardware access. Gazebo remains the sole odom/TF source. Feedback is derived
from Gazebo odom; this validates the host bridge contract, not real firmware.
"""
import json
import math
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src/drive_pkg'))
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from drive_pkg.opencr_bridge_node import OpencrBridgeNode
from drive_pkg.opencr_protocol import twist_to_wheel_rpm


class SimSerial:
    def __init__(self):
        self.line = b''
        self.publisher = None

    def readline(self):
        line, self.line = self.line, b''
        return line

    def write(self, data):
        _, left, right = data.decode().split()
        left, right = float(left), float(right)
        msg = Twist()
        k = 2*math.pi*.033/60
        msg.linear.x = (left+right)*k/2
        msg.angular.z = (right-left)*k/.4323
        if self.publisher is not None:
            self.publisher.publish(msg)


class Shim(OpencrBridgeNode):
    def __init__(self):
        self.rpm_blocks = self.ready_drops = 0
        self.previous_ready = False
        transport = SimSerial()
        super().__init__(transport=transport)
        self.destroy_publisher(self.odom_pub)
        self.odom_pub = self.create_publisher(Odometry, '/sim/bridge_odom_unused', 10)
        transport.publisher = self.create_publisher(Twist, '/sim/cmd_vel_drive', 10)
        self.stats_pub = self.create_publisher(String, '/sim/shim_stats', 10)
        self.create_subscription(Odometry, '/odom', self.feedback, 10)

    def feedback(self, msg):
        left, right = twist_to_wheel_rpm(msg.twist.twist.linear.x, msg.twist.twist.angular.z,
                                        self.wheel_radius, self.wheel_separation)
        self.transport.line = f'F {left:.6f} {right:.6f}\n'.encode()

    def _publish_odom(self):
        # The production feedback state machine runs, but cannot publish duplicate odom/TF.
        pass

    def send_command_tick(self):
        super().send_command_tick()
        self.rpm_blocks += int(self._ready_reason == 'wheel RPM exceeds configured limit')
        self.ready_drops += int(self.previous_ready and not self.motion_ready)
        self.previous_ready = self.motion_ready
        if hasattr(self, 'stats_pub'):
            msg = String()
            msg.data = json.dumps({'rpm_blocks':self.rpm_blocks, 'ready_drops':self.ready_drops,
                                  'ready':self.motion_ready, 'reason':self._ready_reason})
            self.stats_pub.publish(msg)


def main():
    rclpy.init()
    node = Shim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send_zero_command_safe()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
