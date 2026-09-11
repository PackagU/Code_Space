#!/usr/bin/env python3
"""P05 arm lifecycle/feedback contract tests; no ROS and no serial device."""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "robot_arm_pkg"))

from robot_arm_pkg import servo_protocol as sp  # noqa: E402
from robot_arm_pkg.arm_execution import ArmExecutionContract  # noqa: E402
from robot_arm_pkg.arm_sequence import parse_arm_command  # noqa: E402


class FakeDriver:
    def __init__(self):
        self.sent = []
        self.stop_calls = 0
        self.positions = dict(sp.HOME)
        self.fail_send = False
        self.fail_read = False
        self.fail_stop = False

    def send_pose(self, pose_name, duration_ms, poses):
        if self.fail_send:
            raise IOError("injected send failure")
        self.sent.append((pose_name, duration_ms, dict(poses[pose_name])))

    def read_positions(self):
        if self.fail_read:
            raise TimeoutError("injected read failure")
        return dict(self.positions)

    def stop_all(self):
        self.stop_calls += 1
        if self.fail_stop:
            raise IOError("injected stop failure")


class FakeSerialConnection:
    def __init__(self):
        self.writes = []
        self.closed = False
        self._last_servo = None

    def write(self, payload):
        self.writes.append(payload)
        text = payload.decode("ascii")
        if text.endswith("PRAD!"):
            self._last_servo = text[1:4]
        return len(payload)

    def flush(self):
        pass

    def read_until(self, _separator):
        servo_id = self._last_servo
        self._last_servo = None
        return f"#{servo_id}P{sp.HOME[servo_id]}!".encode("ascii")

    def close(self):
        self.closed = True


class FakeSerialModule:
    def __init__(self, connection):
        self.connection = connection

    def Serial(self, _port, _baud, timeout):
        assert timeout == 0.1
        return self.connection


def press(request_id="press-1", cycle=1):
    return {
        "request_id": request_id,
        "action": "press",
        "target": "destination",
        "button": "F2",
        "press_cycle": cycle,
    }


def complete_home(contract, driver, request_id="home-1", start=0.0):
    status = contract.submit({"request_id": request_id, "action": "home"}, start)
    assert status["event"] == "started" and status["state"] == "busy"
    driver.positions = dict(sp.HOME)
    status = contract.tick(start + sp.HOMING_DURATION_MS)
    assert status["event"] == "completed"
    assert status["state"] == "completed"
    assert status["completion_basis"] == "measured_position"
    assert status["homed"] is True and status["homing_basis"] == "measured_position"
    return start + sp.HOMING_DURATION_MS


def main():
    assert parse_arm_command('{"request_id":"h1","action":"home"}') == {
        "request_id": "h1",
        "action": "home",
    }
    assert parse_arm_command(
        '{"request_id":"p1","action":"press","target":"call",'
        '"button":"UP","press_cycle":2}'
    )["target"] == "call"
    assert parse_arm_command(
        '{"request_id":"c1","action":"cancel","target_request_id":"p1"}'
    )["target_request_id"] == "p1"
    for invalid in (
        "not-json",
        '{"request_id":"","action":"home"}',
        '{"request_id":"p","action":"press","target":"x","button":"UP","press_cycle":1}',
        '{"request_id":"p","action":"press","target":"call","button":"UP","press_cycle":9}',
    ):
        try:
            parse_arm_command(invalid)
        except ValueError:
            pass
        else:
            raise AssertionError(f"invalid command accepted: {invalid}")

    # Construction and default no-port path emit zero physical commands.
    driver = FakeDriver()
    contract = ArmExecutionContract(driver)
    assert driver.sent == [] and contract.state == "idle" and not contract.homed
    unavailable = ArmExecutionContract().submit({"request_id": "h0", "action": "home"}, 0)
    assert unavailable["state"] == "failed" and unavailable["error"] == "hardware_unavailable"
    assert unavailable["completion_basis"] == "none"

    # A press cannot start until home has been measured.
    status = contract.submit(press("premature"), 0)
    assert status["state"] == "failed" and status["error"] == "home_not_measured"
    assert driver.sent == [] and driver.stop_calls == 1

    # Restart after failure, then complete all steps from measured feedback.
    now = complete_home(contract, driver, "restart-home", 10)
    status = contract.submit(press("press-ok", 2), now + 10)
    assert status["event"] == "started" and status["pose"] == "press_ready"
    duplicate = contract.submit(press("press-ok", 2), now + 20)
    assert duplicate["event"] == "rejected" and duplicate["error"] == "duplicate_request_id"
    busy = contract.submit(press("other", 2), now + 30)
    assert busy["event"] == "rejected" and busy["error"] == "busy"
    cycle = sp.get_cycle(2)
    poses = sp.get_poses(2)
    tick_time = now + 10
    for index, (pose_name, duration_ms, _send) in enumerate(cycle):
        tick_time += duration_ms
        driver.positions = dict(poses[pose_name])
        status = contract.tick(tick_time)
        assert status is not None, f"step {index} did not advance"
    assert status["event"] == "completed" and status["state"] == "completed"
    assert status["completion_basis"] == "measured_position" and status["homed"]

    # Send failure is terminal, never topic-only success.
    send_driver = FakeDriver()
    send_driver.fail_send = True
    send_contract = ArmExecutionContract(send_driver)
    status = send_contract.submit({"request_id": "send-fail", "action": "home"}, 0)
    assert status["state"] == "failed" and status["error"].startswith("serial_send_failed")
    assert send_driver.stop_calls == 1

    # Wrong feedback waits to a bounded deadline, then fails and sends stop.
    timeout_driver = FakeDriver()
    timeout_contract = ArmExecutionContract(timeout_driver, feedback_timeout_ms=500)
    timeout_contract.submit({"request_id": "timeout", "action": "home"}, 0)
    timeout_driver.positions = {sid: 1500 for sid in sp.SERVO_IDS}
    assert timeout_contract.tick(sp.HOMING_DURATION_MS) is None
    status = timeout_contract.tick(sp.HOMING_DURATION_MS + 500)
    assert status["state"] == "failed" and status["error"] == "feedback_timeout"
    assert timeout_driver.stop_calls == 1 and not status["physical_stop_verified"]

    # Read failure, cancellation, target mismatch, and restart are distinct.
    read_driver = FakeDriver()
    read_contract = ArmExecutionContract(read_driver)
    read_contract.submit({"request_id": "read-fail", "action": "home"}, 0)
    read_driver.fail_read = True
    status = read_contract.tick(sp.HOMING_DURATION_MS)
    assert status["state"] == "failed" and status["error"].startswith("feedback_read_failed")

    cancel_driver = FakeDriver()
    cancel_contract = ArmExecutionContract(cancel_driver, require_homed=False)
    cancel_contract.submit(press("active"), 0)
    mismatch = cancel_contract.submit(
        {"request_id": "cancel-wrong", "action": "cancel", "target_request_id": "other"}, 1
    )
    assert mismatch["event"] == "rejected" and cancel_contract.state == "busy"
    cancelled = cancel_contract.submit(
        {"request_id": "cancel-ok", "action": "cancel", "target_request_id": "active"}, 2
    )
    assert cancelled["state"] == "cancelled"
    assert cancelled["error"] == "mechanical_stop_not_independently_verified"
    assert cancel_driver.stop_calls == 1 and not cancelled["homed"]
    complete_home(cancel_contract, cancel_driver, "home-after-cancel", 10)

    # Explicit simulation has a distinct terminal state and basis.
    simulated = ArmExecutionContract(simulation_mode=True)
    status = simulated.submit({"request_id": "sim-home", "action": "home"}, 0)
    assert status["state"] == "busy"
    status = simulated.tick(sp.HOMING_DURATION_MS)
    assert status["state"] == "simulated_complete"
    assert status["completion_basis"] == "simulation_timing"
    assert status["homing_basis"] == "simulation_timing"

    # The actual serial backend is also exercised through an injected connection.
    connection = FakeSerialConnection()
    serial_driver = sp.SerialPoseDriver(
        "/dev/mock", 115200, serial_module=FakeSerialModule(connection)
    )
    serial_driver.send_pose(sp.HOME_POSE, 100, {sp.HOME_POSE: sp.HOME})
    assert serial_driver.read_positions() == sp.HOME
    serial_driver.stop_all()
    serial_driver.close()
    assert connection.closed
    assert sum(payload.endswith(b"PRAD!") for payload in connection.writes) == 4
    assert sum(payload.endswith(b"PDPT!") for payload in connection.writes) == 4

    launch_text = (ROOT / "src/robot_arm_pkg/launch/arm_sequence.launch.py").read_text(
        encoding="utf-8"
    )
    node_text = (ROOT / "src/robot_arm_pkg/robot_arm_pkg/arm_sequence_node.py").read_text(
        encoding="utf-8"
    )
    bench_text = (ROOT / "scripts/run_arm_press.py").read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument("home_on_start", default_value="false")' in launch_text
    assert 'DeclareLaunchArgument("enable_floor_trigger", default_value="false")' in launch_text
    assert 'DeclareLaunchArgument("simulation_mode", default_value="false")' in launch_text
    assert 'self.declare_parameter("publish_command_estimate", False)' in node_text
    assert "topic-only" not in node_text
    assert "glob.glob" not in bench_text and 'args.port != DEFAULT_PORT' in bench_text
    assert 'parser.add_argument("--execute"' in bench_text

    print("PASS P05 arm execution contract: startup/home/busy/measured/fail/cancel/restart/sim")


if __name__ == "__main__":
    main()
