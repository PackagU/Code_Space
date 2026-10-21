#!/usr/bin/env python3
"""촬영 리허설용 구동 전 확인 (ROS 컨테이너 안에서 실행).

키보드 조종·직진 같은 직접 /cmd_vel 구동을 열기 전에 다음을 보장한다.
1. Nav2 구동 action(NavigateToPose, NavigateThroughPoses)에 살아 있는 goal 이 없다.
   - 먼저 상태 토픽(TRANSIENT_LOCAL, 서버가 goal 을 한 번이라도 받았으면 마지막 상태가 남아 있음)을
     최대 3 초 받는다. 여기서 ACCEPTED·EXECUTING·CANCELING 인 goal 은 모두 기다릴 대상이다.
     (이미 CANCELING 인 goal 은 취소 응답에 다시 나오지 않으므로 상태로 잡아야 한다.)
   - 항상 '전체 취소'를 보낸다. 응답의 goals_canceling 도 기다릴 대상에 더한다.
     취소 응답을 못 받으면 실패(서버와 연결 미확인 = 구동 금지).
   - 기다릴 대상이 모두 끝난 상태(SUCCEEDED·CANCELED·ABORTED)로 보일 때까지 기다린다. 못 보면 실패.
   - 상태 메시지가 한 번도 없고 취소 응답도 비었으면, 그 서버는 goal 을 받은 적이 없다고 본다.
2. /cmd_vel 발행자가 Nav2(대기 중엔 조용함) 말고는 없다. 웹 UI·다른 텔레옵이 있으면 거부한다.

결과는 마지막 줄 `NAV_IDLE_RESULT {json}` 으로 출력한다. 하나라도 못 지키면 종료 코드 1.
"""

import argparse
import json
import sys
import time

TERMINAL = (4, 5, 6)  # action_msgs/GoalStatus: SUCCEEDED, CANCELED, ABORTED
ACTIVE = (1, 2, 3)  # ACCEPTED, EXECUTING, CANCELING
ACTIONS = (("/navigate_to_pose", True), ("/navigate_through_poses", False))  # (이름, 필수 여부)
ALLOWED_CMD_VEL_PUBLISHERS = {"velocity_smoother", "behavior_server", "controller_server"}


def goal_hex(goal_info):
    return bytes(goal_info.goal_id.uuid).hex()


def unfinished(goal_ids, status_array):
    """status_array 에서 끝난 것으로 확인되지 않은 goal id. 상태를 못 받았으면 전부 미확인."""
    if status_array is None:
        return sorted(goal_ids)
    done = {goal_hex(s.goal_info) for s in status_array.status_list if s.status in TERMINAL}
    return sorted(set(goal_ids) - done)


def active_in(status_array):
    if status_array is None:
        return []
    return sorted(goal_hex(s.goal_info) for s in status_array.status_list if s.status in ACTIVE)


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
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    result = {"ok": False, "actions": {}}
    try:
        for action, required in ACTIONS:
            latest = {"msg": None}
            node.create_subscription(GoalStatusArray, action + "/_action/status",
                                     lambda m, box=latest: box.__setitem__("msg", m), qos)
            client = node.create_client(CancelGoal, action + "/_action/cancel_goal")
            if not client.wait_for_service(timeout_sec=3.0):
                if required:
                    result["reason"] = f"{action} action server unavailable (is Nav2 running?)"
                    return 1
                result["actions"][action] = "absent"
                continue
            deadline = time.monotonic() + 3.0
            while latest["msg"] is None and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            seen_active = active_in(latest["msg"])
            future = client.call_async(CancelGoal.Request())  # zero id + zero stamp = cancel all
            rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
            response = future.result() if future.done() else None
            if response is None:
                result["reason"] = f"{action}: no cancel response — goal state unknown"
                return 1
            canceling = [goal_hex(g) for g in response.goals_canceling]
            wait_for = sorted(set(canceling) | set(seen_active))
            deadline = time.monotonic() + args.timeout
            left = unfinished(wait_for, latest["msg"]) if wait_for else []
            while left and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                left = unfinished(wait_for, latest["msg"])
            result["actions"][action] = {"active_seen": seen_active, "canceled": canceling, "unfinished": left,
                                         "status_seen": latest["msg"] is not None,
                                         "cancel_return_code": int(response.return_code)}
            if left:
                result["reason"] = f"{action}: {len(left)} goal(s) not confirmed finished after cancel"
                return 1
        deadline = time.monotonic() + 1.0  # 발행자 정보 발견 시간
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
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
