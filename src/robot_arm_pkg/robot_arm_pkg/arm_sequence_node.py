#!/usr/bin/env python3
"""엘베 층 전환 완료 시 버튼 누르기 사이클(Kim 실기 프로토콜)을 실행하는 노드.

- 구독: /floor_orchestrator/status (std_msgs/String, JSON)
- 발행: /packagu_arm/joint_cmd (sensor_msgs/JointState, position=PWM 근사 보간)
  — 시뮬 부하/관측용. 실서보의 실제 보간은 컨트롤러가 T(ms) 명령으로 수행.
- 시리얼: serial_port 파라미터 지정 시 포즈 스텝마다 {#000P....T....!} 묶음 명령 전송.
  비어 있으면 토픽 발행만(시뮬/드라이런). 스텝 진행은 타이머 기반 — 콜백에서 sleep 없음.
- 사이클 선택: press_cycle 파라미터(1~3, 기본 1) → servo_protocol.PRESS_CYCLES(스텝) + POSE_TABLES(포즈).
  사이클별로 포즈 PWM 을 독립 튜닝한다(POSES_N). home 은 공용(HOME).
- 기동 homing: 시리얼이 열리면 즉시 home 명령 1회 전송 (전원 인가 직후 전 모터 1500 →
  대기 자세 home). home_on_start:=false 로 끌 수 있다. 대기 중에는 명령을 보내지 않는다.

벤치 단독 테스트(주변 확인 후!): ros2 run robot_arm_pkg arm_sequence --ros-args \
  -p self_test:=true -p serial_port:=/dev/arm_servo
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from robot_arm_pkg import servo_protocol as sp
from robot_arm_pkg.arm_sequence import FloorReadyTrigger


class SerialPoseDriver:
    """실서보 시리얼 백엔드 — Kim 벤치 스크립트와 동일 명령 문자열을 쓴다."""

    def __init__(self, port, baud, logger, poses=sp.POSES):
        import serial  # python3-serial — 이미지에 포함

        self._logger = logger
        self._poses = poses  # 선택된 사이클의 포즈 표 (press_cycle N → POSES_N)
        self._conn = serial.Serial(port, baud, timeout=0.1)
        self._logger.info(f"arm serial open: {port} @ {baud}")

    def send_pose(self, pose_name, duration_ms):
        self._conn.write(sp.pose_command(pose_name, duration_ms, self._poses).encode("ascii"))

    def stop_all(self):
        for servo_id in sp.SERVO_IDS:
            self._conn.write(sp.stop_command(servo_id).encode("ascii"))

    def close(self):
        self._conn.close()


class ArmSequenceNode(Node):
    def __init__(self):
        super().__init__("packagu_arm_sequence")
        self.declare_parameter("status_topic", "/floor_orchestrator/status")
        self.declare_parameter("cmd_topic", "/packagu_arm/joint_cmd")
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("initial_floor", "F1")
        self.declare_parameter("serial_port", "")  # 예: /dev/arm_servo (launch 인자로만 지정)
        self.declare_parameter("serial_baud", 115200)
        self.declare_parameter("self_test", False)
        self.declare_parameter("home_on_start", True)  # 기동 시 home 자세로 이동 (실서보일 때만 의미)
        self.declare_parameter("press_cycle", 1)  # 1~3 — servo_protocol.PRESS_CYCLES 선택

        self._rate_hz = float(self.get_parameter("rate_hz").value)
        self._cycle_id = int(self.get_parameter("press_cycle").value)
        self._cycle = sp.get_cycle(self._cycle_id)  # 잘못된 번호는 여기서 ValueError 로 즉시 실패
        self._poses = sp.get_poses(self._cycle_id)  # 같은 번호의 포즈 표 (사이클별 독립 튜닝)
        self._trigger = FloorReadyTrigger(str(self.get_parameter("initial_floor").value))
        self._elapsed_ms = None  # None = 대기, float = 사이클 진행 중 (사이클 기준 경과)
        self._step_index = 0
        self._tick_count = 0

        self._driver = None
        port = str(self.get_parameter("serial_port").value).strip()
        if port:
            try:
                self._driver = SerialPoseDriver(
                    port, int(self.get_parameter("serial_baud").value), self.get_logger(), self._poses
                )
            except Exception as exc:  # noqa: BLE001 — 시리얼 실패는 부하테스트를 막지 않는다
                self.get_logger().error(f"arm serial open failed ({exc}) — topic-only로 계속")

        # 전원 인가 직후 서보는 전부 1500 — 대기 자세(home)로 맞춘다. 컨트롤러가 자체 보간하므로
        # 여기서는 명령 1회만 보내고 기다리지 않는다(self_test 는 homing 끝난 뒤 시작).
        homing_sec = 0.0
        if self._driver is not None and bool(self.get_parameter("home_on_start").value):
            homing_sec = self._go_home()

        self._pub = self.create_publisher(JointState, str(self.get_parameter("cmd_topic").value), 10)
        self._sub = self.create_subscription(
            String, str(self.get_parameter("status_topic").value), self._on_status, 10
        )
        self._timer = self.create_timer(1.0 / self._rate_hz, self._on_tick)

        if bool(self.get_parameter("self_test").value):
            delay_sec = 2.0 + homing_sec
            self.get_logger().info(f"self_test: {delay_sec:.1f}초 후 사이클 1회 실행 — 팔 주변 공간 확보!")
            self._self_test_timer = self.create_timer(delay_sec, self._start_self_test)

        self.get_logger().info(
            f"packagu_arm_sequence ready — trigger={self._trigger.last_floor} 이후 층 전환, "
            f"cycle#{self._cycle_id}={sp.cycle_duration_ms(self._cycle) / 1000.0:.1f}s "
            f"({len(self._cycle)} steps) @ {self._rate_hz:.0f}Hz, "
            f"driver={'serial' if self._driver else 'topic-only'}"
        )

    def _start_self_test(self):
        self._self_test_timer.cancel()
        self._start_sequence("self_test")

    def _go_home(self):
        """기동 homing 명령 1회 전송. 반환: 이동에 걸리는 초(실패 시 0.0, driver 비활성화)."""
        try:
            self._driver.send_pose(sp.HOME_POSE, sp.HOMING_DURATION_MS)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"arm homing failed ({exc}) — driver 비활성화")
            try:
                self._driver.stop_all()
            except Exception:  # noqa: BLE001
                pass
            self._driver = None
            return 0.0
        self.get_logger().info(f"기동 homing: 중립(1500) → {sp.HOME_POSE} {sp.HOMING_DURATION_MS / 1000.0:.1f}s")
        return sp.HOMING_DURATION_MS / 1000.0

    def _on_status(self, msg):
        floor = self._trigger.observe(msg.data)
        if floor is None:
            return
        self._start_sequence(floor)

    def _start_sequence(self, reason):
        if self._elapsed_ms is not None:
            self.get_logger().warn(f"시퀀스 진행 중 — {reason} 트리거 무시")
            return
        self._elapsed_ms = 0.0
        self._step_index = 0
        self._tick_count = 0
        self._send_step_command()
        # 로그 포맷 주의: smoke 의 verify_arm_sequence 가 "버튼 시퀀스 시작 (F2)" 문자열을 grep 한다.
        self.get_logger().info(
            f"버튼 시퀀스 시작 ({reason}) — cycle#{self._cycle_id} {sp.cycle_duration_ms(self._cycle) / 1000.0:.1f}s"
        )

    def _send_step_command(self):
        pose_name, duration_ms, send = self._cycle[self._step_index]
        if not (send and self._driver):
            return
        try:
            self._driver.send_pose(pose_name, duration_ms)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"arm serial send failed ({exc}) — driver 비활성화")
            try:
                self._driver.stop_all()
            except Exception:  # noqa: BLE001
                pass
            self._driver = None

    def _on_tick(self):
        if self._elapsed_ms is None:
            return
        pwm = sp.interpolate_pwm(self._elapsed_ms, self._cycle, poses=self._poses)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [f"servo_{sid}" for sid in sp.SERVO_IDS]
        msg.position = [float(pwm[sid]) for sid in sp.SERVO_IDS]
        self._pub.publish(msg)
        self._tick_count += 1
        self._elapsed_ms += 1000.0 / self._rate_hz

        # 현재 스텝 경계를 넘었으면 다음 스텝 시작 명령 전송
        boundary = sum(d for _, d, _ in self._cycle[: self._step_index + 1])
        while self._elapsed_ms >= boundary and self._step_index + 1 < len(self._cycle):
            self._step_index += 1
            self._send_step_command()
            boundary = sum(d for _, d, _ in self._cycle[: self._step_index + 1])

        if self._elapsed_ms >= sp.cycle_duration_ms(self._cycle):
            self.get_logger().info(f"버튼 시퀀스 완료 — {self._tick_count} ticks")
            self._elapsed_ms = None


def main(args=None):
    rclpy.init(args=args)
    node = ArmSequenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._driver is not None:
            node._driver.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
