#!/usr/bin/env python3
"""리프트(Z축) 상승→하강 사이클 실행 + /joint_states 검증 — 원샷 노드.

최종 시나리오의 '택배함 픽업(리프트)'/'F2 하차(리프트)' mock 동작.
/set_lift_trajectory (URDF lift_trajectory 플러그인)로 lift_joint 를 움직이고,
/joint_states 에서 실제 위치가 목표에 도달했는지 확인한다 — 발행만 하고 끝나는
검증 없는 mock 이 되지 않게(리프트가 안 움직이면 FAIL).

사용: python3 lift_cycle.py --tag pickup [--up 0.30] [--tol 0.03] [--timeout 30]
종료 코드: 0 = 상승·하강 모두 검증 완료, 1 = 실패.
"""
import argparse
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

LIFT_JOINT = "lift_joint"


class LiftCycle(Node):
    def __init__(self):
        super().__init__("lift_cycle")
        self.pub = self.create_publisher(JointTrajectory, "/set_lift_trajectory", 10)
        self.pos = None
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)

    def _on_js(self, msg):
        # 콜백은 위치 갱신만 한다(초기화 금지 — pedestrians a6a5115 회귀 참조).
        try:
            i = msg.name.index(LIFT_JOINT)
        except ValueError:
            return
        self.pos = msg.position[i]

    def _command(self, target, move_time_s=2.0):
        traj = JointTrajectory()
        # gazebo_ros_joint_pose_trajectory 는 frame_id 가 비면 "Plugin needs a reference link []
        # as frame_id, aborting" 으로 메시지를 버린다(2026-08-21 G004 run1 실측). "world"/"map" 이면
        # 참조 링크 없이 조인트 위치만 설정한다(모델 포즈 불변) — 리프트 mock 에 맞는 모드.
        traj.header.frame_id = "world"
        traj.joint_names = [LIFT_JOINT]
        pt = JointTrajectoryPoint()
        pt.positions = [float(target)]
        pt.time_from_start = Duration(sec=int(move_time_s))
        traj.points = [pt]
        self.pub.publish(traj)

    def move_and_verify(self, target, tol, timeout_s):
        # 재발행 루프: 플러그인/구독 늦은 기동으로 첫 발행이 유실될 수 있다.
        deadline = self.get_clock().now().nanoseconds / 1e9 + timeout_s
        next_pub = 0.0
        while self.get_clock().now().nanoseconds / 1e9 < deadline:
            now = self.get_clock().now().nanoseconds / 1e9
            if now >= next_pub:
                self._command(target)
                next_pub = now + 3.0
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.pos is not None and abs(self.pos - target) <= tol:
                self.get_logger().info(f"lift at {self.pos:.3f} (target {target}) — OK")
                return True
        self.get_logger().error(
            f"lift did not reach {target} in {timeout_s}s (last pos: {self.pos})"
        )
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", default="lift")
    ap.add_argument("--up", type=float, default=0.30)
    ap.add_argument("--down", type=float, default=0.0)
    ap.add_argument("--tol", type=float, default=0.03)
    ap.add_argument("--timeout", type=float, default=30.0)
    args = ap.parse_args()

    rclpy.init()
    node = LiftCycle()
    ok = False
    try:
        node.get_logger().info(f"[{args.tag}] lift up -> {args.up}")
        if node.move_and_verify(args.up, args.tol, args.timeout):
            node.get_logger().info(f"[{args.tag}] lift down -> {args.down}")
            ok = node.move_and_verify(args.down, args.tol, args.timeout)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(f"LIFT {'PASS' if ok else 'FAIL'}: {args.tag}")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
