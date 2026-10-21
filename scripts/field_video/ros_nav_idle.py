#!/usr/bin/env python3
"""촬영 리허설용 구동 전 확인 (ROS 컨테이너 안에서 실행).

키보드 조종·직진 같은 직접 /cmd_vel 구동을 열기 전에 다음 두 가지를 보장한다.
1. NavigateToPose 에 살아 있는 goal(ACCEPTED·EXECUTING·CANCELING)이 없다.
   있으면 전체 취소를 보내고, 상태 토픽에서 모두 끝난 것을 볼 때까지 기다린다.
2. /cmd_vel 발행자가 Nav2(대기 중엔 조용함) 말고는 없다. 웹 UI·다른 텔레옵이 있으면 거부한다.

결과는 마지막 줄 `NAV_IDLE_RESULT {json}` 으로 출력한다. 하나라도 못 지키면 종료 코드 1.
"""

import argparse
import json
import sys
import time

ACTIVE = (1, 2, 3)  # action_msgs/GoalStatus: ACCEPTED, EXECUTING, CANCELING
ALLOWED_CMD_VEL_PUBLISHERS = {"velocity_smoother", "behavior_server", "controller_server"}


def active_goals(status_array):
    if status_array is None:
        return []
    return [bytes(s.goal_info.goal_id.uuid).hex() for s in status_array.status_list if s.status in ACTIVE]


def foreign_publishers(names):
    return sorted(n for n in names if n not in ALLOWED_CMD_VEL_PUBLISHERS)


def emit(**fields):
    print("NAV_IDLE_RESULT " + json.dumps(fields, ensure_ascii=False, sort_keys=True), flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=8.0)
    args = parser.parse_args(argv)

    import rclpy
    from action_msgs.msg import GoalStatusArray
    from action_msgs.srv import CancelGoal
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    rclpy.init()
    node = rclpy.create_node("packagu_field_video_idle_check")
    latest = {"msg": None}
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(GoalStatusArray, "/navigate_to_pose/_action/status",
                             lambda m: latest.__setitem__("msg", m), qos)
    result = {"ok": False}
    try:
        cancel = node.create_client(CancelGoal, "/navigate_to_pose/_action/cancel_goal")
        if not cancel.wait_for_service(timeout_sec=3.0):
            result["reason"] = "NavigateToPose action server unavailable (is Nav2 running?)"
            return 1
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        before = active_goals(latest["msg"])
        result["active_before"] = before
        if before:
            future = cancel.call_async(CancelGoal.Request())  # zero id + zero stamp = cancel all
            rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
            deadline = time.monotonic() + args.timeout
            while time.monotonic() < deadline and active_goals(latest["msg"]):
                rclpy.spin_once(node, timeout_sec=0.05)
        after = active_goals(latest["msg"])
        result["active_after"] = after
        if after:
            result["reason"] = f"{len(after)} Nav2 goal(s) still active after cancel"
            return 1
        names = {info.node_name for info in node.get_publishers_info_by_topic("/cmd_vel")}
        result["cmd_vel_publishers"] = sorted(names)
        foreign = foreign_publishers(names)
        if foreign:
            result["reason"] = "other /cmd_vel publishers present: " + ", ".join(foreign)
            return 1
        result["ok"] = True
        return 0
    finally:
        emit(**result)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
