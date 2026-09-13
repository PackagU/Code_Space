"""Pure safety policy for commands sent to a physical differential drive."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class GateResult:
    linear_x: float
    angular_z: float
    system_ready: bool
    command_forwarded: bool
    reason: str


class SafetyGate:
    """Fail-closed freshness, range, and subsystem-interlock checks."""

    def __init__(
        self,
        sensor_timeout_sec=0.5,
        drive_ready_timeout_sec=0.5,
        command_timeout_sec=0.3,
        max_linear_speed=0.10,
        max_angular_speed=0.35,
    ):
        values = {
            "sensor_timeout_sec": sensor_timeout_sec,
            "drive_ready_timeout_sec": drive_ready_timeout_sec,
            "command_timeout_sec": command_timeout_sec,
            "max_linear_speed": max_linear_speed,
            "max_angular_speed": max_angular_speed,
        }
        invalid = [name for name, value in values.items() if not math.isfinite(value) or value <= 0.0]
        if invalid:
            raise ValueError(f"parameters must be finite and positive: {invalid}")

        self.sensor_timeout_sec = sensor_timeout_sec
        self.drive_ready_timeout_sec = drive_ready_timeout_sec
        self.command_timeout_sec = command_timeout_sec
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.scan_time = None
        self.odom_time = None
        self.drive_ready_time = None
        self.drive_ready = False
        self.software_stop = False
        self.operation_inhibit_reason = ""

    @staticmethod
    def _fresh(sample_time, now, timeout):
        if sample_time is None or not math.isfinite(sample_time) or not math.isfinite(now):
            return False
        age = now - sample_time
        return 0.0 <= age <= timeout

    def observe_scan(self, now):
        self.scan_time = now if math.isfinite(now) else None

    def observe_odom(self, now):
        self.odom_time = now if math.isfinite(now) else None

    def observe_drive_ready(self, ready, now):
        self.drive_ready = bool(ready)
        self.drive_ready_time = now if math.isfinite(now) else None

    def set_software_stop(self, stopped):
        self.software_stop = bool(stopped)

    def set_operation_inhibit(self, reason):
        """Block all wheel motion while an arm/lift/transfer owner is active.

        The ROS contract uses an empty string to clear the interlock and a
        non-empty, request-scoped reason to assert it.
        """
        self.operation_inhibit_reason = str(reason).strip()

    def readiness(self, now):
        if self.software_stop:
            return False, "software stop asserted"
        if self.operation_inhibit_reason:
            return False, f"operation inhibit: {self.operation_inhibit_reason}"
        if not self._fresh(self.scan_time, now, self.sensor_timeout_sec):
            return False, "scan stale"
        if not self._fresh(self.odom_time, now, self.sensor_timeout_sec):
            return False, "odom stale"
        if not self._fresh(self.drive_ready_time, now, self.drive_ready_timeout_sec):
            return False, "drive ready stale"
        if not self.drive_ready:
            return False, "drive not ready"
        return True, "scan, odom, and drive ready"

    def filter_command(self, linear_x, angular_z, command_time, now):
        ready, reason = self.readiness(now)
        if not ready:
            return GateResult(0.0, 0.0, False, False, reason)
        if not self._fresh(command_time, now, self.command_timeout_sec):
            return GateResult(0.0, 0.0, True, False, "command stale")
        if not all(math.isfinite(value) for value in (linear_x, angular_z)):
            return GateResult(0.0, 0.0, True, False, "non-finite command")
        if abs(linear_x) > self.max_linear_speed or abs(angular_z) > self.max_angular_speed:
            return GateResult(0.0, 0.0, True, False, "command exceeds field limit")
        return GateResult(linear_x, angular_z, True, True, "command forwarded")
