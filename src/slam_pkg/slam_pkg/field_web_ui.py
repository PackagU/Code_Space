#!/usr/bin/env python3
"""Local-only browser console for physical mapping, teleop, and saved-map Nav2."""

from __future__ import annotations

import base64
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import mimetypes
import os
from pathlib import Path
import re
import secrets
import signal
import subprocess
import threading
import time
from urllib.parse import urlparse

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

from slam_pkg.map_contract import MapContractError, validate_map_yaml


MAP_ROOT = Path("/ros2_ws/maps/field")
LOG_ROOT = Path("/ros2_ws/logs/web_ui")
SAFE_NAME = re.compile(r"^[A-Za-z0-9_-]{1,40}$")
MAP_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
MODE_LABELS = {"idle": "대기", "mapping": "매핑 중", "navigation": "네비게이션"}


class OperatorError(RuntimeError):
    pass


class FieldWebNode(Node):
    def __init__(self):
        super().__init__("packagu_field_web_ui")
        self.declare_parameter("bind_host", "127.0.0.1")
        self.declare_parameter("port", 8080)
        self.declare_parameter("linear_speed", 0.10)
        self.declare_parameter("angular_speed", 0.35)
        self.declare_parameter("command_deadman_sec", 0.30)
        self.declare_parameter("sensor_timeout_sec", 0.50)
        self.declare_parameter("map_root", str(MAP_ROOT))

        self.bind_host = str(self.get_parameter("bind_host").value)
        self.port = int(self.get_parameter("port").value)
        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        self.deadman_sec = float(self.get_parameter("command_deadman_sec").value)
        self.sensor_timeout = float(self.get_parameter("sensor_timeout_sec").value)
        self.map_root = Path(str(self.get_parameter("map_root").value)).resolve()
        if self.bind_host not in ("127.0.0.1", "::1"):
            raise ValueError("field web UI must bind to loopback; use an SSH tunnel")
        if not 1024 <= self.port <= 65535:
            raise ValueError("port must be 1024..65535")
        for name, value in (
            ("linear_speed", self.linear_speed),
            ("angular_speed", self.angular_speed),
            ("command_deadman_sec", self.deadman_sec),
            ("sensor_timeout_sec", self.sensor_timeout),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")

        self._lock = threading.RLock()
        self._operation_lock = threading.Lock()
        self._latest_map = None
        self._scan_time = None
        self._odom_time = None
        self._drive_time = None
        self._drive_ready = False
        self._software_stop = False
        self._command = (0.0, 0.0)
        self._command_expiry = 0.0
        self._last_message = "베이스 상태를 확인하고 있습니다."
        self._last_error = False
        self._processes = {"mapping": None, "navigation": None}
        self._process_logs = {"mapping": None, "navigation": None}
        self._navigation_status = "준비 중"
        self._goal_handle = None
        self._goal_future = None
        self._pending_goal = None

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.stop_pub = self.create_publisher(Bool, "/nav_safety/stop", 10)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", MAP_QOS
        )
        self.create_subscription(OccupancyGrid, "/map", self._on_map, MAP_QOS)
        self.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(Bool, "/drive/ready", self._on_drive_ready, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.create_timer(0.05, self._command_tick)
        self.create_timer(0.20, self._goal_tick)

        self.web_root = Path(get_package_share_directory("slam_pkg")) / "web"
        self.token = secrets.token_urlsafe(32)
        handler = make_handler(self)
        self.httpd = ThreadingHTTPServer((self.bind_host, self.port), handler)
        self.http_thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.http_thread.start()
        self.get_logger().info(
            f"field UI ready at http://{self.bind_host}:{self.port}; access through SSH tunnel"
        )

    def _now(self):
        return time.monotonic()

    def _on_map(self, msg):
        with self._lock:
            self._latest_map = msg

    def _on_scan(self, _msg):
        self._scan_time = self._now()

    def _on_odom(self, _msg):
        self._odom_time = self._now()

    def _on_drive_ready(self, msg):
        self._drive_time = self._now()
        self._drive_ready = bool(msg.data)

    def _age(self, sample_time):
        return None if sample_time is None else max(0.0, self._now() - sample_time)

    def _process_running(self, kind):
        process = self._processes[kind]
        if process is None:
            return False
        if process.poll() is None:
            return True
        log_handle = self._process_logs[kind]
        if log_handle is not None:
            log_handle.close()
        self._processes[kind] = None
        self._process_logs[kind] = None
        return False

    def base_ready(self):
        scan_age = self._age(self._scan_time)
        odom_age = self._age(self._odom_time)
        drive_age = self._age(self._drive_time)
        return (
            scan_age is not None and scan_age <= self.sensor_timeout
            and odom_age is not None and odom_age <= self.sensor_timeout
            and drive_age is not None and drive_age <= self.sensor_timeout
            and self._drive_ready and not self._software_stop
        )

    def mode(self):
        if self._process_running("mapping"):
            return "mapping"
        if self._process_running("navigation"):
            return "navigation"
        return "idle"

    def status(self):
        mode = self.mode()
        return {
            "mode": mode,
            "mode_label": MODE_LABELS[mode],
            "mapping_running": mode == "mapping",
            "navigation_running": mode == "navigation",
            "navigation_status": self._navigation_status,
            "scan_age_sec": self._age(self._scan_time),
            "odom_age_sec": self._age(self._odom_time),
            "drive_age_sec": self._age(self._drive_time),
            "drive_ready": self._drive_ready,
            "base_ready": self.base_ready(),
            "software_stop": self._software_stop,
            "message": self._last_message,
            "error": self._last_error,
        }

    def map_payload(self):
        with self._lock:
            msg = self._latest_map
        if msg is None or msg.info.width <= 0 or msg.info.height <= 0:
            return {"available": False}
        pose = None
        try:
            transform = self.tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
            q = transform.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            pose = {
                "x": transform.transform.translation.x,
                "y": transform.transform.translation.y,
                "yaw": yaw,
            }
        except Exception:
            pose = None
        raw = bytes((int(value) + 256) % 256 for value in msg.data)
        return {
            "available": True,
            "frame_id": msg.header.frame_id or "map",
            "width": int(msg.info.width),
            "height": int(msg.info.height),
            "resolution": float(msg.info.resolution),
            "origin_x": float(msg.info.origin.position.x),
            "origin_y": float(msg.info.origin.position.y),
            "data_b64": base64.b64encode(raw).decode("ascii"),
            "robot_pose": pose,
        }

    def _set_message(self, message, error=False):
        self._last_message = str(message)
        self._last_error = bool(error)

    def set_command(self, direction):
        commands = {
            "forward": (self.linear_speed, 0.0),
            "backward": (-self.linear_speed, 0.0),
            "left": (0.0, self.angular_speed),
            "right": (0.0, -self.angular_speed),
            "stop": (0.0, 0.0),
        }
        if direction not in commands:
            raise OperatorError("알 수 없는 주행 명령입니다.")
        if direction != "stop" and not self.base_ready():
            self.publish_zero()
            raise OperatorError("LiDAR, odom, 구동 준비 상태를 먼저 확인하세요.")
        if direction != "stop" and self._process_running("navigation"):
            self.cancel_goal()
            self._set_message("수동 조작으로 네비게이션 목표를 취소했습니다.")
        with self._lock:
            self._command = commands[direction]
            self._command_expiry = self._now() + self.deadman_sec if direction != "stop" else 0.0
        if direction == "stop":
            self.publish_zero()

    def _command_tick(self):
        with self._lock:
            command = self._command if self._now() <= self._command_expiry else (0.0, 0.0)
            if command == (0.0, 0.0):
                self._command = command
        msg = Twist()
        msg.linear.x, msg.angular.z = command
        self.cmd_pub.publish(msg)

    def publish_zero(self):
        with self._lock:
            self._command = (0.0, 0.0)
            self._command_expiry = 0.0
        self.cmd_pub.publish(Twist())

    def assert_stop(self):
        self.publish_zero()
        self.cancel_goal()
        self._software_stop = True
        msg = Bool(); msg.data = True
        self.stop_pub.publish(msg)
        self._set_message("소프트 정지가 걸렸습니다. 물리 E-Stop을 대신하지 않습니다.")

    def clear_stop(self):
        self.publish_zero()
        self._software_stop = False
        msg = Bool(); msg.data = False
        self.stop_pub.publish(msg)
        self._set_message("소프트 정지를 해제했습니다. 센서와 구동 준비를 다시 확인하세요.")

    def _start_process(self, kind, command):
        LOG_ROOT.mkdir(parents=True, exist_ok=True)
        log_path = LOG_ROOT / f"{time.strftime('%Y%m%d_%H%M%S')}_{kind}.log"
        log_handle = log_path.open("ab", buffering=0)
        try:
            process = subprocess.Popen(
                command,
                stdout=log_handle,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        except Exception:
            log_handle.close()
            raise
        self._processes[kind] = process
        self._process_logs[kind] = log_handle

    def _stop_process(self, kind):
        process = self._processes[kind]
        if process is None:
            return
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=4)
        log_handle = self._process_logs[kind]
        if log_handle is not None:
            log_handle.close()
        self._processes[kind] = None
        self._process_logs[kind] = None

    def start_mapping(self, floor):
        if floor not in ("F1", "F2", "F3"):
            raise OperatorError("층은 F1, F2, F3 중 하나여야 합니다.")
        with self._operation_lock:
            if self.mode() != "idle":
                raise OperatorError("실행 중인 매핑 또는 네비게이션을 먼저 종료하세요.")
            if not self.base_ready():
                raise OperatorError("LiDAR, odom, 구동 준비 상태가 모두 최신이어야 합니다.")
            self.publish_zero()
            self._start_process(
                "mapping",
                ["ros2", "launch", "slam_pkg", "field_mapping_only.launch.py", "rviz:=false"],
            )
            self._set_message(f"{floor} 매핑을 시작했습니다. 천천히 왕복해 폐루프를 만드세요.")
            return {"message": self._last_message}

    def _safe_map_path(self, raw_path):
        path = Path(str(raw_path)).resolve()
        try:
            path.relative_to(self.map_root)
        except ValueError as exc:
            raise OperatorError("허용된 지도 폴더 밖의 경로입니다.") from exc
        if path.suffix.lower() != ".yaml":
            raise OperatorError("지도 YAML 파일을 선택하세요.")
        try:
            validate_map_yaml(path)
        except MapContractError as exc:
            raise OperatorError(f"지도 검증 실패: {exc}") from exc
        return path

    def save_and_stop_mapping(self, floor, requested_name):
        if floor not in ("F1", "F2", "F3"):
            raise OperatorError("층은 F1, F2, F3 중 하나여야 합니다.")
        with self._operation_lock:
            if not self._process_running("mapping"):
                raise OperatorError("실행 중인 매핑이 없습니다.")
            self.publish_zero()
            name = requested_name or f"kku_{floor.lower()}_{time.strftime('%Y%m%d_%H%M%S')}"
            if not SAFE_NAME.fullmatch(name):
                raise OperatorError("지도 이름은 영문, 숫자, 밑줄, 대시만 사용할 수 있습니다.")
            output_dir = self.map_root / floor.lower()
            output_dir.mkdir(parents=True, exist_ok=True)
            prefix = output_dir / name
            if any(Path(str(prefix) + suffix).exists() for suffix in (".yaml", ".pgm", ".posegraph", ".data", ".sha256")):
                raise OperatorError("같은 이름의 지도가 이미 있습니다.")

            save = subprocess.run(
                ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", str(prefix)],
                capture_output=True,
                text=True,
                timeout=30,
            )
            if save.returncode != 0:
                raise OperatorError("지도 이미지 저장에 실패했습니다. 매핑은 계속 실행 중입니다.")
            serialize = subprocess.run(
                [
                    "ros2", "service", "call", "/slam_toolbox/serialize_map",
                    "slam_toolbox/srv/SerializePoseGraph", f"{{filename: '{prefix}'}}",
                ],
                capture_output=True,
                text=True,
                timeout=30,
            )
            output = serialize.stdout + serialize.stderr
            if serialize.returncode != 0 or not re.search(r"result[=:]\s*0|result=0", output):
                raise OperatorError("posegraph 저장에 실패했습니다. 매핑은 계속 실행 중입니다.")
            try:
                validate_map_yaml(str(prefix) + ".yaml")
            except MapContractError as exc:
                raise OperatorError(f"저장 지도 검증 실패: {exc}") from exc

            manifest_lines = []
            for suffix in (".yaml", ".pgm", ".posegraph", ".data"):
                file_path = Path(str(prefix) + suffix)
                digest = hashlib.sha256(file_path.read_bytes()).hexdigest()
                manifest_lines.append(f"{digest}  {file_path.name}")
            Path(str(prefix) + ".sha256").write_text("\n".join(manifest_lines) + "\n", encoding="utf-8")
            (output_dir / "latest_map.txt").write_text(str(prefix) + ".yaml\n", encoding="utf-8")
            self._stop_process("mapping")
            self._set_message(f"지도 {name}을 저장하고 매핑을 종료했습니다.")
            return {"message": self._last_message, "map": str(prefix) + ".yaml"}

    def list_maps(self):
        maps = []
        if not self.map_root.is_dir():
            return {"maps": maps}
        for path in sorted(self.map_root.glob("*/*.yaml"), key=lambda item: item.stat().st_mtime, reverse=True):
            try:
                self._safe_map_path(path)
            except OperatorError:
                continue
            maps.append({"path": str(path), "floor": path.parent.name.upper(), "name": path.stem})
        return {"maps": maps}

    def start_navigation(self, raw_map):
        with self._operation_lock:
            if self.mode() != "idle":
                raise OperatorError("실행 중인 매핑 또는 네비게이션을 먼저 종료하세요.")
            if not self.base_ready():
                raise OperatorError("LiDAR, odom, 구동 준비 상태가 모두 최신이어야 합니다.")
            map_path = self._safe_map_path(raw_map)
            self.publish_zero()
            self._navigation_status = "기동 중"
            self._start_process(
                "navigation",
                [
                    "ros2", "launch", "slam_pkg", "kku_navigation.launch.py",
                    f"map:={map_path}", "use_sim_time:=false", "rviz:=false",
                ],
            )
            self._set_message("저장 지도를 열었습니다. 초기 위치를 먼저 지정하세요.")
            return {"message": self._last_message}

    def stop_navigation(self):
        with self._operation_lock:
            self.publish_zero()
            self.cancel_goal()
            self._stop_process("navigation")
            self._navigation_status = "정지"
            self._set_message("네비게이션을 종료했습니다.")
            return {"message": self._last_message}

    @staticmethod
    def _validated_pose(data):
        try:
            x, y, yaw = (float(data[key]) for key in ("x", "y", "yaw"))
        except (KeyError, TypeError, ValueError) as exc:
            raise OperatorError("위치 값이 올바르지 않습니다.") from exc
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            raise OperatorError("위치 값은 유한한 숫자여야 합니다.")
        if abs(x) > 10000 or abs(y) > 10000 or abs(yaw) > math.tau * 2:
            raise OperatorError("위치 값이 허용 범위를 벗어났습니다.")
        return x, y, yaw

    def publish_initial_pose(self, data):
        if not self._process_running("navigation"):
            raise OperatorError("저장 지도를 먼저 여세요.")
        x, y, yaw = self._validated_pose(data)
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685
        self.initial_pose_pub.publish(msg)
        self._set_message("초기 위치를 보냈습니다. 스캔이 벽과 겹치는지 확인하세요.")
        return {"message": self._last_message}

    def queue_goal(self, data):
        if not self._process_running("navigation"):
            raise OperatorError("저장 지도를 먼저 여세요.")
        if not self.base_ready():
            raise OperatorError("센서 또는 구동 준비 상태가 끊겼습니다.")
        self._pending_goal = self._validated_pose(data)
        self._navigation_status = "목표 대기"
        self._set_message("목표를 확인 중입니다.")
        return {"message": self._last_message}

    def _goal_tick(self):
        if self._pending_goal is None or self._goal_future is not None:
            return
        if not self.nav_client.server_is_ready():
            self._navigation_status = "Nav2 준비 중"
            return
        x, y, yaw = self._pending_goal
        self._pending_goal = None
        goal = NavigateToPose.Goal()
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self._goal_future = self.nav_client.send_goal_async(goal)
        self._goal_future.add_done_callback(self._goal_response)
        self._navigation_status = "목표 전송 중"

    def _goal_response(self, future):
        self._goal_future = None
        try:
            handle = future.result()
        except Exception as exc:
            self._navigation_status = "목표 오류"
            self._set_message(f"목표 전송 실패: {exc}", True)
            return
        if not handle.accepted:
            self._navigation_status = "목표 거절"
            self._set_message("Nav2가 목표를 거절했습니다.", True)
            return
        self._goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._goal_result)
        self._navigation_status = "주행 중"
        self._set_message("목표 주행을 시작했습니다.")

    def _goal_result(self, future):
        try:
            status = int(future.result().status)
        except Exception as exc:
            self._navigation_status = "결과 오류"
            self._set_message(f"주행 결과 확인 실패: {exc}", True)
        else:
            self._navigation_status = "목표 종료"
            self._set_message(f"Nav2 목표가 종료됐습니다. 상태 코드: {status}")
        self._goal_handle = None

    def cancel_goal(self):
        self._pending_goal = None
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
        self._navigation_status = "목표 취소"

    def shutdown(self):
        self.assert_stop()
        for _ in range(2):
            self.publish_zero()
            time.sleep(0.03)
        for kind in ("mapping", "navigation"):
            try:
                self._stop_process(kind)
            except Exception as exc:
                self.get_logger().error(f"failed to stop {kind}: {exc}")
        self.httpd.shutdown()
        self.httpd.server_close()
        self.http_thread.join(timeout=2)


def make_handler(node):
    class Handler(BaseHTTPRequestHandler):
        server_version = "PackaguFieldUI/1.0"

        def log_message(self, fmt, *args):
            node.get_logger().debug(fmt % args)

        def _headers(self, status, content_type="application/json; charset=utf-8", length=None):
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("X-Frame-Options", "DENY")
            self.send_header("Referrer-Policy", "no-referrer")
            self.send_header(
                "Content-Security-Policy",
                "default-src 'self'; script-src 'self'; style-src 'self'; img-src 'self' data:; connect-src 'self'; frame-ancestors 'none'",
            )
            if length is not None:
                self.send_header("Content-Length", str(length))
            self.end_headers()

        def _json(self, status, payload):
            body = (json.dumps(payload, ensure_ascii=False, allow_nan=False) + "\n").encode("utf-8")
            self._headers(status, length=len(body))
            self.wfile.write(body)

        def _authorized(self):
            return secrets.compare_digest(self.headers.get("X-Packagu-Token", ""), node.token)

        def _body(self):
            try:
                length = int(self.headers.get("Content-Length", "0"))
            except ValueError as exc:
                raise OperatorError("잘못된 요청 길이입니다.") from exc
            if length < 0 or length > 16384:
                raise OperatorError("요청 본문이 너무 큽니다.")
            try:
                return json.loads(self.rfile.read(length).decode("utf-8")) if length else {}
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                raise OperatorError("JSON 요청이 올바르지 않습니다.") from exc

        def do_GET(self):
            path = urlparse(self.path).path
            if path.startswith("/api/"):
                if not self._authorized():
                    self._json(403, {"error": "인증 토큰이 없습니다."})
                    return
                try:
                    payload = {
                        "/api/status": node.status,
                        "/api/map": node.map_payload,
                        "/api/maps": node.list_maps,
                    }[path]()
                except KeyError:
                    self._json(404, {"error": "없는 API입니다."})
                except Exception as exc:
                    self._json(500, {"error": str(exc)})
                else:
                    self._json(200, payload)
                return
            asset_name = "index.html" if path == "/" else path.lstrip("/")
            if asset_name not in ("index.html", "styles.css", "app.js"):
                self._json(404, {"error": "파일을 찾을 수 없습니다."})
                return
            asset = node.web_root / asset_name
            if not asset.is_file():
                self._json(404, {"error": "화면 파일을 찾을 수 없습니다."})
                return
            body = asset.read_bytes()
            if asset_name == "index.html":
                body = body.replace(b"__PACKAGU_TOKEN__", node.token.encode("ascii"))
            self._headers(200, mimetypes.guess_type(asset_name)[0] or "application/octet-stream", len(body))
            self.wfile.write(body)

        def do_POST(self):
            if not self._authorized():
                self._json(403, {"error": "인증 토큰이 없습니다."})
                return
            path = urlparse(self.path).path
            try:
                body = self._body()
                routes = {
                    "/api/cmd": lambda: (node.set_command(body.get("direction")), {"message": "주행 명령을 갱신했습니다."})[1],
                    "/api/stop": lambda: (node.assert_stop(), {"message": node._last_message})[1],
                    "/api/stop/reset": lambda: (node.clear_stop(), {"message": node._last_message})[1],
                    "/api/mapping/start": lambda: node.start_mapping(body.get("floor", "F1")),
                    "/api/mapping/stop": lambda: node.save_and_stop_mapping(body.get("floor", "F1"), body.get("name", "")),
                    "/api/navigation/start": lambda: node.start_navigation(body.get("map", "")),
                    "/api/navigation/stop": node.stop_navigation,
                    "/api/navigation/initial-pose": lambda: node.publish_initial_pose(body),
                    "/api/navigation/goal": lambda: node.queue_goal(body),
                }
                result = routes[path]()
            except KeyError:
                self._json(404, {"error": "없는 API입니다."})
            except OperatorError as exc:
                node._set_message(str(exc), True)
                self._json(409, {"error": str(exc)})
            except Exception as exc:
                node._set_message(str(exc), True)
                self._json(500, {"error": str(exc)})
            else:
                self._json(200, result)

    return Handler


def main():
    rclpy.init()
    node = FieldWebNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
