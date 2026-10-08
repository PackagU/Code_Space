#!/usr/bin/env python3
"""Print NavigateToPose goal states seen on the hidden action status topic (simulation helper)."""
import json
import time

import rclpy
from action_msgs.msg import GoalStatusArray
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

NAMES = {0: 'UNKNOWN', 1: 'ACCEPTED', 2: 'EXECUTING', 3: 'CANCELING', 4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}


def main():
    rclpy.init()
    node = rclpy.create_node('sim_action_status_check')
    seen = {}
    qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status',
                             lambda m: seen.update({bytes(s.goal_info.goal_id.uuid).hex(): NAMES.get(s.status, s.status)
                                                    for s in m.status_list}), qos)
    end = time.monotonic()+3
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.1)
    active = [g for g, s in seen.items() if s in ('ACCEPTED', 'EXECUTING', 'CANCELING')]
    print(json.dumps({'goals': seen, 'active_goals': len(active)}))
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
