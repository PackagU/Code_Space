#!/usr/bin/env python3
"""촬영 리허설용 직진·후진 (ROS 컨테이너 안에서 실행).

/odom 이동 거리를 보며 /cmd_vel 로 곧게 움직인다. 명령은 기존 경로
/cmd_vel → nav_safety_gate → /cmd_vel_safe → OpenCR 를 그대로 지난다.
Nav2 goal 이 없을 때만 실행한다(호출하는 쪽이 보장).

--stdin-stop: distance 는 최대 거리 상한이 되고, 표준입력으로 `STOP` 줄이 오면 멈춘다
(호스트 진행기가 초음파 거리를 보고 보낸다). 표준입력이 끊겨도(EOF) 즉시 멈추고 실패로 끝낸다.
이 모드에서 STOP 없이 최대 거리에 닿으면 실패다(센서가 대상을 못 봤다는 뜻).

결과는 마지막 줄 `STRAIGHT_RESULT {json}` 으로 출력한다.
"""

import argparse
import json
import math
import sys
import threading
import time

MAX_DISTANCE_M = 1.0
MAX_SPEED_MPS = 0.10  # nav_safety.yaml max_linear_speed 0.12 아래
MIN_SPEED_MPS = 0.02
ACCEL_MPS2 = 0.05
ODOM_STALE_SEC = 0.5
DONE_TOLERANCE_M = 0.005


def plan_speed(elapsed, remaining, cruise):
    """가속·감속 프로파일. 남은 거리가 짧으면 천천히, 최소 속도는 유지한다."""
    accel_limited = MIN_SPEED_MPS + ACCEL_MPS2 * elapsed
    decel_limited = max(MIN_SPEED_MPS, remaining * 1.0)
    return max(MIN_SPEED_MPS, min(cruise, accel_limited, decel_limited))


def validate(distance, speed):
    if not math.isfinite(distance) or not 0.01 <= abs(distance) <= MAX_DISTANCE_M:
        raise ValueError(f"distance must be 0.01..{MAX_DISTANCE_M} m in magnitude")
    if not math.isfinite(speed) or not MIN_SPEED_MPS <= speed <= MAX_SPEED_MPS:
        raise ValueError(f"speed must be {MIN_SPEED_MPS}..{MAX_SPEED_MPS} m/s")


def emit(**fields):
    print("STRAIGHT_RESULT " + json.dumps(fields, ensure_ascii=False, sort_keys=True), flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--distance", type=float, required=True, help="+ 전진 / - 후진 (m)")
    parser.add_argument("--speed", type=float, default=0.05)
    parser.add_argument("--timeout", type=float, default=0.0, help="0 이면 거리/속도 x2 + 5 s")
    parser.add_argument("--stdin-stop", action="store_true", help="표준입력 STOP 줄로 멈춤(초음파 접근)")
    args = parser.parse_args(argv)
    external = {"stop": False, "eof": False}
    if args.stdin_stop:
        def watch_stdin():
            for line in sys.stdin:
                if line.strip().upper() == "STOP":
                    external["stop"] = True
                    return
            external["eof"] = True
        threading.Thread(target=watch_stdin, daemon=True).start()
    try:
        validate(args.distance, args.speed)
    except ValueError as exc:
        emit(ok=False, reason=str(exc))
        return 2

    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry

    rclpy.init()
    node = rclpy.create_node("packagu_field_video_straight")
    publisher = node.create_publisher(Twist, "/cmd_vel", 10)
    latest = {"msg": None, "at": 0.0}

    def on_odom(msg):
        latest["msg"], latest["at"] = msg, time.monotonic()

    node.create_subscription(Odometry, "/odom", on_odom, 10)
    sign = 1.0 if args.distance > 0 else -1.0
    target = abs(args.distance)
    timeout = args.timeout or target / args.speed * 2.0 + 5.0
    traveled = 0.0
    result = {"ok": False, "target_m": round(args.distance, 3)}

    def publish(linear):
        twist = Twist()
        twist.linear.x = float(linear)
        publisher.publish(twist)

    try:
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and (latest["msg"] is None or publisher.get_subscription_count() < 1):
            rclpy.spin_once(node, timeout_sec=0.05)
        if latest["msg"] is None:
            result["reason"] = "/odom not received within 3 s"
            return 1
        if publisher.get_subscription_count() < 1:
            result["reason"] = "/cmd_vel has no subscriber (nav_safety_gate missing)"
            return 1
        start = latest["msg"].pose.pose.position
        x0, y0 = start.x, start.y
        began = time.monotonic()
        while True:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if now - latest["at"] > ODOM_STALE_SEC:
                result["reason"] = "/odom stale during move"
                return 1
            p = latest["msg"].pose.pose.position
            traveled = math.hypot(p.x - x0, p.y - y0)
            remaining = target - traveled
            if external["stop"]:
                result.update(ok=True, reason="stopped by sensor")
                return 0
            if external["eof"]:
                result["reason"] = "controller disconnected (stdin closed)"
                return 1
            if remaining <= DONE_TOLERANCE_M:
                if args.stdin_stop:
                    result["reason"] = "max distance reached before sensor stop"
                    return 1
                result.update(ok=True, reason="distance reached")
                return 0
            if now - began > timeout:
                result["reason"] = f"timeout {timeout:.1f} s (gate stopped or wheels blocked?)"
                return 1
            publish(sign * plan_speed(now - began, remaining, args.speed))
    except KeyboardInterrupt:
        result["reason"] = "interrupted by operator"
        return 130
    finally:
        for _ in range(10):
            publish(0.0)
            time.sleep(0.05)
        result["traveled_m"] = round(traveled, 3)
        emit(**result)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
