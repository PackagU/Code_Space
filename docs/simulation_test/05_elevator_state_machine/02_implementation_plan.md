# Elevator Mission MVP Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build an isolated ROS2 PoC that drives a delivery mission through elevator entry, manual floor switch ack, relocalization, and Nav2 point navigation without modifying the existing `src/` packages.

**Architecture:** Keep Claude's 4-Layer design, but implement the floor transition as a manual 2-Phase MVP. The mission layer owns the py_trees sequence, skill behaviors wrap Nav2/elevator/floor operations, infra nodes provide mock elevator state and floor switch ack, and config files live inside the test workspace.

**Tech Stack:** ROS2 Humble, Python 3, `rclpy`, `py_trees`, `py_trees_ros` where available, Nav2 `NavigateToPose`, `std_msgs/String`, `std_srvs/Trigger`, `geometry_msgs/PoseWithCovarianceStamped`, YAML.

---

## Scope And Non-Negotiables

- Implement only under @test_workspace/elevator_mission/.
- Do not modify @src/common_pkg/, @src/slam_pkg/, @src/drive_pkg/, @src/robot_arm_pkg/.
- Do not create `elevator_msgs` in MVP.
- Do not implement Gazebo automatic world reload in MVP.
- Do not delete or overwrite existing files.
- Read @docs/simulation_test/05_elevator_state_machine/01_claude_handoff.md before starting.

## Target File Map

| Path | Action | Responsibility |
|------|--------|----------------|
| `test_workspace/elevator_mission/config/kku_nav_points.yaml` | Create | Isolated point registry |
| `test_workspace/elevator_mission/config/delivery_missions.yaml` | Create | Sample mission definitions |
| `test_workspace/elevator_mission/scripts/point_registry.py` | Create | YAML parser and pose lookup |
| `test_workspace/elevator_mission/scripts/test_point_registry.py` | Create | Offline registry tests |
| `test_workspace/elevator_mission/scripts/capture_nav_point.py` | Create | Print YAML snippet from current AMCL pose |
| `test_workspace/elevator_mission/docs/point_capture_guide.md` | Create | Procedure for collecting room and elevator points |
| `test_workspace/elevator_mission/src/elevator_sim_pkg/` | Create | Mock elevator FSM package |
| `test_workspace/elevator_mission/src/floor_orchestrator_pkg/` | Create | Manual floor switch package |
| `test_workspace/elevator_mission/src/elevator_mission_pkg/` | Create | Mission tree and behavior package |
| `test_workspace/elevator_mission/scripts/test_behaviors_dryrun.py` | Create | Offline mission dry-run |
| `test_workspace/elevator_mission/scripts/run_demo.sh` | Create | Demo command guide |
| `test_workspace/elevator_mission/docs/completion_report.md` | Create | Final implementation report |

## Task 1: Point Registry Config And Offline Test

**Files:**
- Create: `test_workspace/elevator_mission/config/kku_nav_points.yaml`
- Create: `test_workspace/elevator_mission/config/delivery_missions.yaml`
- Create: `test_workspace/elevator_mission/scripts/point_registry.py`
- Create: `test_workspace/elevator_mission/scripts/test_point_registry.py`

- [ ] **Step 1: Write `kku_nav_points.yaml`**

Use coordinates from @src/common_pkg/config/kku_pre_simulation_map.yaml. Keep frame as `map` because Nav2 runs one active floor at a time.

```yaml
schema_version: 1
frame_id: map
floors:
  F1:
    map_yaml: src/slam_pkg/maps/kku_virtual/f1/kku_f1.yaml
    initial_pose_id: elevator_exit
    points:
      elevator_entry:
        type: elevator
        x: 1.6
        y: 0.0
        yaw_deg: 180.0
      elevator_inside:
        type: elevator
        x: 0.0
        y: 0.0
        yaw_deg: 0.0
      elevator_exit:
        type: elevator
        x: 1.6
        y: 0.0
        yaw_deg: 0.0
      parcel_access:
        type: parcel
        x: 5.0
        y: -0.6
        yaw_deg: -90.0
      parcel_pickup:
        type: parcel
        x: 5.0
        y: -1.4
        yaw_deg: -90.0
  F2:
    map_yaml: src/slam_pkg/maps/kku_virtual/f2/kku_f2.yaml
    initial_pose_id: elevator_exit
    points:
      elevator_entry:
        type: elevator
        x: 1.6
        y: 0.0
        yaw_deg: 180.0
      elevator_inside:
        type: elevator
        x: 0.0
        y: 0.0
        yaw_deg: 0.0
      elevator_exit:
        type: elevator
        x: 1.6
        y: 0.0
        yaw_deg: 0.0
      "201": {type: room, x: 1.65, y: 3.0, yaw_deg: 180.0}
      "202": {type: room, x: 2.35, y: 3.0, yaw_deg: 0.0}
      "203": {type: room, x: 1.65, y: 6.0, yaw_deg: 180.0}
      "204": {type: room, x: 2.35, y: 6.0, yaw_deg: 0.0}
      "205": {type: room, x: 1.65, y: 9.0, yaw_deg: 180.0}
      "206": {type: room, x: 2.35, y: 9.0, yaw_deg: 0.0}
      "207": {type: room, x: 1.65, y: 12.0, yaw_deg: 180.0}
      "208": {type: room, x: 2.35, y: 12.0, yaw_deg: 0.0}
  F3:
    map_yaml: src/slam_pkg/maps/kku_virtual/f3/kku_f3.yaml
    initial_pose_id: elevator_exit
    points:
      elevator_entry:
        type: elevator
        x: 1.6
        y: 0.0
        yaw_deg: 180.0
      elevator_inside:
        type: elevator
        x: 0.0
        y: 0.0
        yaw_deg: 0.0
      elevator_exit:
        type: elevator
        x: 1.6
        y: 0.0
        yaw_deg: 0.0
      "301": {type: room, x: 1.65, y: 3.0, yaw_deg: 180.0}
      "302": {type: room, x: 2.35, y: 3.0, yaw_deg: 0.0}
      "303": {type: room, x: 1.65, y: 6.0, yaw_deg: 180.0}
      "304": {type: room, x: 2.35, y: 6.0, yaw_deg: 0.0}
      "305": {type: room, x: 1.65, y: 9.0, yaw_deg: 180.0}
      "306": {type: room, x: 2.35, y: 9.0, yaw_deg: 0.0}
      "307": {type: room, x: 1.65, y: 12.0, yaw_deg: 180.0}
      "308": {type: room, x: 2.35, y: 12.0, yaw_deg: 0.0}
```

- [ ] **Step 2: Write `delivery_missions.yaml`**

```yaml
schema_version: 1
missions:
  parcel_to_208:
    start_floor: F1
    target_floor: F2
    pickup_point: parcel_pickup
    elevator_entry_point: elevator_entry
    elevator_inside_point: elevator_inside
    elevator_exit_point: elevator_exit
    destination_point: "208"
    mock_load_event: parcel_loaded
    mock_delivery_event: delivered
  parcel_to_307:
    start_floor: F1
    target_floor: F3
    pickup_point: parcel_pickup
    elevator_entry_point: elevator_entry
    elevator_inside_point: elevator_inside
    elevator_exit_point: elevator_exit
    destination_point: "307"
    mock_load_event: parcel_loaded
    mock_delivery_event: delivered
```

- [ ] **Step 3: Write the failing registry test**

Create `test_point_registry.py` with assertions for valid lookup, floor-qualified lookup, and missing point failure.

```python
#!/usr/bin/env python3
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from point_registry import PointRegistry


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def main():
    registry = PointRegistry.from_file(ROOT / "config" / "kku_nav_points.yaml")

    p = registry.get("F2", "208")
    require(p.floor == "F2", "208 should be on F2")
    require(abs(p.x - 2.35) < 1e-6, "208 x mismatch")
    require(abs(p.y - 12.0) < 1e-6, "208 y mismatch")
    require(p.yaw_deg == 0.0, "208 yaw mismatch")

    inside = registry.get_qualified("elevator_inside@F1")
    require(inside.floor == "F1", "qualified floor mismatch")
    require(inside.point_id == "elevator_inside", "qualified point mismatch")

    try:
        registry.get("F2", "999")
    except KeyError as exc:
        require("999" in str(exc), "missing point error should include id")
    else:
        raise AssertionError("missing point should raise KeyError")

    print("PASS point registry")


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: Run test to verify it fails**

```bash
python3 test_workspace/elevator_mission/scripts/test_point_registry.py
```

Expected: FAIL with `ModuleNotFoundError: No module named 'point_registry'`.

- [ ] **Step 5: Implement `point_registry.py`**

```python
from dataclasses import dataclass
from pathlib import Path
import math
import yaml


@dataclass(frozen=True)
class NavPoint:
    floor: str
    point_id: str
    point_type: str
    x: float
    y: float
    yaw_deg: float

    @property
    def yaw_rad(self):
        return math.radians(self.yaw_deg)


class PointRegistry:
    def __init__(self, frame_id, floors):
        self.frame_id = frame_id
        self.floors = floors

    @classmethod
    def from_file(cls, path):
        data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        floors = {}
        for floor_id, floor_data in data["floors"].items():
            points = {}
            for point_id, point_data in floor_data["points"].items():
                points[str(point_id)] = NavPoint(
                    floor=floor_id,
                    point_id=str(point_id),
                    point_type=point_data["type"],
                    x=float(point_data["x"]),
                    y=float(point_data["y"]),
                    yaw_deg=float(point_data["yaw_deg"]),
                )
            floors[floor_id] = {
                "map_yaml": floor_data["map_yaml"],
                "initial_pose_id": floor_data["initial_pose_id"],
                "points": points,
            }
        return cls(frame_id=data.get("frame_id", "map"), floors=floors)

    def get(self, floor, point_id):
        floor_key = str(floor).upper()
        point_key = str(point_id)
        if floor_key not in self.floors:
            raise KeyError(f"unknown floor: {floor_key}")
        points = self.floors[floor_key]["points"]
        if point_key not in points:
            raise KeyError(f"unknown point on {floor_key}: {point_key}")
        return points[point_key]

    def get_qualified(self, qualified_id):
        if "@" not in qualified_id:
            raise ValueError(f"expected point@floor, got: {qualified_id}")
        point_id, floor = qualified_id.split("@", 1)
        return self.get(floor, point_id)
```

- [ ] **Step 6: Run test to verify it passes**

```bash
python3 test_workspace/elevator_mission/scripts/test_point_registry.py
```

Expected: `PASS point registry`.

- [ ] **Step 7: Commit**

```bash
git add test_workspace/elevator_mission/config/kku_nav_points.yaml \
  test_workspace/elevator_mission/config/delivery_missions.yaml \
  test_workspace/elevator_mission/scripts/point_registry.py \
  test_workspace/elevator_mission/scripts/test_point_registry.py
git commit -m "test: add elevator mission point registry"
```

## Task 1.5: Point Acquisition Workflow

**Files:**
- Create: `test_workspace/elevator_mission/scripts/capture_nav_point.py`
- Create: `test_workspace/elevator_mission/docs/point_capture_guide.md`
- Modify: `test_workspace/elevator_mission/config/kku_nav_points.yaml`

This task makes point collection explicit. The initial values in `kku_nav_points.yaml` are only seed values from @src/common_pkg/config/kku_pre_simulation_map.yaml. Before a real mission demo, each important point must be captured or corrected from the active Nav2 map using AMCL pose.

- [ ] **Step 1: Create `capture_nav_point.py`**

The script subscribes to `/amcl_pose`, converts the current robot orientation to `yaw_deg`, and prints a YAML snippet. It does not edit `kku_nav_points.yaml` automatically, so it cannot overwrite existing point data.

```python
#!/usr/bin/env python3
import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class PoseCapture:
    def __init__(self, node):
        self.node = node
        self.message = None
        self.subscription = node.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self._on_pose,
            10,
        )

    def _on_pose(self, message):
        self.message = message


def main():
    parser = argparse.ArgumentParser(
        description="Print a kku_nav_points.yaml entry from the current /amcl_pose."
    )
    parser.add_argument("--floor", required=True, help="Floor id such as F1, F2, or F3")
    parser.add_argument("--point", required=True, help="Point id such as 303 or elevator_exit")
    parser.add_argument("--type", required=True, help="Point type such as room, elevator, or parcel")
    parser.add_argument("--timeout-sec", type=float, default=5.0)
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("capture_nav_point")
    capture = PoseCapture(node)

    deadline = node.get_clock().now().nanoseconds + int(args.timeout_sec * 1e9)
    while rclpy.ok() and capture.message is None:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.get_clock().now().nanoseconds > deadline:
            node.destroy_node()
            rclpy.shutdown()
            print("ERROR: timed out waiting for /amcl_pose", file=sys.stderr)
            return 1

    pose = capture.message.pose.pose
    yaw_deg = math.degrees(yaw_from_quaternion(pose.orientation))

    print(f"# Paste under floors.{args.floor}.points in kku_nav_points.yaml")
    print(f'"{args.point}": {{type: {args.type}, x: {pose.position.x:.3f}, y: {pose.position.y:.3f}, yaw_deg: {yaw_deg:.1f}}}')

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
```

- [ ] **Step 2: Create `point_capture_guide.md`**

````markdown
# Elevator Mission Point Capture Guide

작성일: 2026-05-28

## 목적

`kku_nav_points.yaml`의 초기 좌표는 설계 좌표다. 실제 Nav2 mission 전에 각 층의 `elevator_entry`, `elevator_inside`, `elevator_exit`, 배송지 방 번호, `parcel_pickup` 좌표를 AMCL 기준 pose로 보정한다.

## 준비

해당 층의 Gazebo, Nav2, RViz를 실행한다.

```bash
ros2 launch common_pkg gazebo.launch.py floor:=F3 use_sim_time:=true
ros2 launch slam_pkg kku_navigation.launch.py floor:=F3
```

RViz에서 `2D Pose Estimate`로 현재 위치를 맞춘 뒤, Nav2 goal 또는 teleop으로 로봇을 기록할 위치에 세운다.

## 좌표 캡처

로봇이 목표 지점에서 문 또는 엘리베이터 출구를 바라보게 한 뒤 아래 명령을 실행한다.

```bash
python3 test_workspace/elevator_mission/scripts/capture_nav_point.py --floor F3 --point 303 --type room
```

출력 예시는 다음과 같다.

```yaml
# Paste under floors.F3.points in kku_nav_points.yaml
"303": {type: room, x: 1.650, y: 6.000, yaw_deg: 180.0}
```

출력된 한 줄을 `test_workspace/elevator_mission/config/kku_nav_points.yaml`의 해당 floor 아래에 반영한다. 기존 값을 바꿀 때는 이 테스트 워크스페이스 파일만 수정한다.

## 필수 캡처 목록

| Floor | Points |
|------|--------|
| F1 | `parcel_pickup`, `elevator_entry`, `elevator_inside` |
| F2 | `elevator_inside`, `elevator_exit`, `201`부터 `208` 중 테스트 대상 |
| F3 | `elevator_inside`, `elevator_exit`, `301`부터 `308` 중 테스트 대상 |

## 검증

좌표 반영 후 point registry 테스트를 먼저 실행한다.

```bash
python3 test_workspace/elevator_mission/scripts/test_point_registry.py
```

그 다음 단층 Nav2에서 해당 point로 이동시켜 문 앞 정지 위치와 yaw를 확인한다. 로봇이 문에 너무 붙거나 복도 중앙을 벗어나면 `x`, `y`, `yaw_deg`를 다시 캡처하거나 수동 보정한다.
````

- [ ] **Step 3: Capture elevator points first**

Run the guide for the elevator points before room points. These points are used by both the manual B MVP and the later A3-lite teleport upgrade.

```bash
python3 test_workspace/elevator_mission/scripts/capture_nav_point.py --floor F1 --point elevator_entry --type elevator
python3 test_workspace/elevator_mission/scripts/capture_nav_point.py --floor F1 --point elevator_inside --type elevator
python3 test_workspace/elevator_mission/scripts/capture_nav_point.py --floor F2 --point elevator_inside --type elevator
python3 test_workspace/elevator_mission/scripts/capture_nav_point.py --floor F2 --point elevator_exit --type elevator
python3 test_workspace/elevator_mission/scripts/capture_nav_point.py --floor F3 --point elevator_inside --type elevator
python3 test_workspace/elevator_mission/scripts/capture_nav_point.py --floor F3 --point elevator_exit --type elevator
```

Expected: each command prints one YAML line with the requested point id.

- [ ] **Step 4: Capture one delivery target for the MVP demo**

For the first F1 to F3 test, capture `303` or `307` only. Add more room points after the mission flow works.

```bash
python3 test_workspace/elevator_mission/scripts/capture_nav_point.py --floor F3 --point 303 --type room
```

Expected: the output can replace the seed value for `floors.F3.points."303"` in `kku_nav_points.yaml`.

- [ ] **Step 5: Verify the updated registry**

```bash
python3 test_workspace/elevator_mission/scripts/test_point_registry.py
```

Expected: `PASS point registry`.

- [ ] **Step 6: Commit**

```bash
git add test_workspace/elevator_mission/scripts/capture_nav_point.py \
  test_workspace/elevator_mission/docs/point_capture_guide.md \
  test_workspace/elevator_mission/config/kku_nav_points.yaml
git commit -m "test: add elevator mission point capture workflow"
```

## Task 2: Elevator Sim Package

**Files:**
- Create: `test_workspace/elevator_mission/src/elevator_sim_pkg/package.xml`
- Create: `test_workspace/elevator_mission/src/elevator_sim_pkg/setup.py`
- Create: `test_workspace/elevator_mission/src/elevator_sim_pkg/setup.cfg`
- Create: `test_workspace/elevator_mission/src/elevator_sim_pkg/resource/elevator_sim_pkg`
- Create: `test_workspace/elevator_mission/src/elevator_sim_pkg/elevator_sim_pkg/__init__.py`
- Create: `test_workspace/elevator_mission/src/elevator_sim_pkg/elevator_sim_pkg/elevator_sim_node.py`
- Create: `test_workspace/elevator_mission/src/elevator_sim_pkg/launch/elevator_sim_only.launch.py`

- [ ] **Step 1: Create package metadata**

Use `ament_python`. Dependencies are `rclpy` and `std_msgs`.

```xml
<?xml version="1.0"?>
<package format="3">
  <name>elevator_sim_pkg</name>
  <version>0.1.0</version>
  <description>Mock elevator FSM for the isolated mission PoC</description>
  <maintainer email="inonewater@todo.com">PackagU</maintainer>
  <license>Apache-2.0</license>
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <buildtool_depend>ament_python</buildtool_depend>
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] **Step 2: Create resource marker and `setup.cfg`**

The resource marker file is intentionally empty. `setup.cfg` installs console scripts into the ROS2 package lib directory.

```text
```

```ini
[develop]
script_dir=$base/lib/elevator_sim_pkg
[install]
install_scripts=$base/lib/elevator_sim_pkg
```

- [ ] **Step 3: Write `setup.py`**

```python
from glob import glob
from setuptools import setup

package_name = "elevator_sim_pkg"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="PackagU",
    maintainer_email="inonewater@todo.com",
    description="Mock elevator FSM for the isolated mission PoC",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "elevator_sim_node = elevator_sim_pkg.elevator_sim_node:main",
        ],
    },
)
```

- [ ] **Step 4: Implement elevator FSM**

`/elevator/call` receives `F1`, `F2`, `F3`, `open`, or `close`. `/elevator/state` publishes JSON at 5Hz.

```python
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ElevatorSimNode(Node):
    def __init__(self):
        super().__init__("elevator_sim_node")
        self.declare_parameter("initial_floor", "F1")
        self.declare_parameter("floor_travel_time_sec", 5.0)
        self.declare_parameter("door_open_time_sec", 3.0)

        self.current_floor = self.get_parameter("initial_floor").value
        self.target_floor = self.current_floor
        self.state = "IDLE"
        self.door_state = "open"
        self.state_started = time.monotonic()

        self.state_pub = self.create_publisher(String, "/elevator/state", 10)
        self.call_sub = self.create_subscription(String, "/elevator/call", self.on_call, 10)
        self.create_timer(0.2, self.tick)

    def on_call(self, msg):
        command = msg.data.strip()
        if command in ("F1", "F2", "F3"):
            self.target_floor = command
            if self.target_floor == self.current_floor:
                self.state = "ARRIVED_OPEN"
                self.door_state = "open"
            else:
                self.state = "CALLED"
                self.door_state = "closing"
            self.state_started = time.monotonic()
        elif command == "close":
            self.state = "CLOSING"
            self.door_state = "closing"
            self.state_started = time.monotonic()
        elif command == "open":
            self.state = "ARRIVED_OPEN"
            self.door_state = "open"
            self.state_started = time.monotonic()
        else:
            self.get_logger().warning(f"ignored elevator command: {command}")

    def tick(self):
        elapsed = time.monotonic() - self.state_started
        travel_time = float(self.get_parameter("floor_travel_time_sec").value)
        door_time = float(self.get_parameter("door_open_time_sec").value)

        if self.state == "CALLED" and elapsed >= 0.5:
            self.state = "MOVING"
            self.door_state = "closed"
            self.state_started = time.monotonic()
        elif self.state == "CLOSING" and elapsed >= 0.5:
            self.state = "MOVING"
            self.door_state = "closed"
            self.state_started = time.monotonic()
        elif self.state == "MOVING" and elapsed >= travel_time:
            self.current_floor = self.target_floor
            self.state = "ARRIVED_OPEN"
            self.door_state = "open"
            self.state_started = time.monotonic()
        elif self.state == "ARRIVED_OPEN" and elapsed >= door_time:
            self.state = "IDLE"

        payload = {
            "current_floor": self.current_floor,
            "target_floor": self.target_floor,
            "door_state": self.door_state,
            "state": self.state,
        }
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.state_pub.publish(msg)


def main():
    rclpy.init()
    node = ElevatorSimNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

- [ ] **Step 5: Add launch file**

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="elevator_sim_pkg",
            executable="elevator_sim_node",
            name="elevator_sim_node",
            output="screen",
            parameters=[{
                "initial_floor": "F1",
                "floor_travel_time_sec": 5.0,
                "door_open_time_sec": 3.0,
            }],
        )
    ])
```

- [ ] **Step 6: Build and verify**

```bash
cd test_workspace/elevator_mission
colcon build --symlink-install --packages-select elevator_sim_pkg
source install/setup.bash
ros2 launch elevator_sim_pkg elevator_sim_only.launch.py
```

In another terminal:

```bash
cd test_workspace/elevator_mission
source install/setup.bash
ros2 topic echo /elevator/state --once
ros2 topic pub --once /elevator/call std_msgs/msg/String "{data: F2}"
```

Expected: `/elevator/state` JSON changes to target/current `F2` after travel time.

- [ ] **Step 7: Commit**

```bash
git add test_workspace/elevator_mission/src/elevator_sim_pkg
git commit -m "feat: add mock elevator FSM"
```

## Task 3: Floor Orchestrator Manual Mode

**Files:**
- Create: `test_workspace/elevator_mission/src/floor_orchestrator_pkg/package.xml`
- Create: `test_workspace/elevator_mission/src/floor_orchestrator_pkg/setup.py`
- Create: `test_workspace/elevator_mission/src/floor_orchestrator_pkg/setup.cfg`
- Create: `test_workspace/elevator_mission/src/floor_orchestrator_pkg/resource/floor_orchestrator_pkg`
- Create: `test_workspace/elevator_mission/src/floor_orchestrator_pkg/floor_orchestrator_pkg/__init__.py`
- Create: `test_workspace/elevator_mission/src/floor_orchestrator_pkg/floor_orchestrator_pkg/floor_orchestrator_node.py`
- Create: `test_workspace/elevator_mission/src/floor_orchestrator_pkg/launch/floor_orchestrator.launch.py`

- [ ] **Step 1: Create package metadata**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>floor_orchestrator_pkg</name>
  <version>0.1.0</version>
  <description>Manual 2-Phase floor switch orchestrator for elevator mission PoC</description>
  <maintainer email="inonewater@todo.com">PackagU</maintainer>
  <license>Apache-2.0</license>
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>std_srvs</depend>
  <buildtool_depend>ament_python</buildtool_depend>
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] **Step 2: Create resource marker and `setup.cfg`**

The resource marker file is intentionally empty.

```text
```

```ini
[develop]
script_dir=$base/lib/floor_orchestrator_pkg
[install]
install_scripts=$base/lib/floor_orchestrator_pkg
```

- [ ] **Step 3: Write `setup.py`**

```python
from glob import glob
from setuptools import setup

package_name = "floor_orchestrator_pkg"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="PackagU",
    maintainer_email="inonewater@todo.com",
    description="Manual 2-Phase floor switch orchestrator for elevator mission PoC",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "floor_orchestrator_node = floor_orchestrator_pkg.floor_orchestrator_node:main",
        ],
    },
)
```

- [ ] **Step 4: Implement manual request and ack contract**

The node exposes two Trigger services. `request_switch` accepts the current `target_floor` parameter and publishes pending status. `/floor_orchestrator/ack` is called by the user after manually relaunching Gazebo/Nav2 for the target floor.

```python
import json

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from std_srvs.srv import Trigger


class FloorOrchestratorNode(Node):
    def __init__(self):
        super().__init__("floor_orchestrator_node")
        self.declare_parameter("current_floor", "F1")
        self.declare_parameter("target_floor", "F2")
        self.declare_parameter("spawn_point_id", "elevator_inside")
        self.pending = False
        self.status_pub = self.create_publisher(String, "/floor_orchestrator/status", 10)
        self.create_service(Trigger, "/floor_orchestrator/request_switch", self.on_request_switch)
        self.create_service(Trigger, "/floor_orchestrator/ack", self.on_ack)
        self.create_timer(0.5, self.publish_status)

    def on_request_switch(self, request, response):
        del request
        self.pending = True
        target = self.get_parameter("target_floor").value
        spawn = self.get_parameter("spawn_point_id").value
        response.success = True
        response.message = (
            f"manual floor switch requested: target={target}, spawn={spawn}. "
            "Relaunch Gazebo/Nav2 for target floor, then call /floor_orchestrator/ack."
        )
        self.get_logger().warning(response.message)
        self.publish_status()
        return response

    def on_ack(self, request, response):
        del request
        if not self.pending:
            response.success = False
            response.message = "no pending floor switch"
            return response
        target = self.get_parameter("target_floor").value
        self.set_parameters([Parameter("current_floor", value=target)])
        self.pending = False
        response.success = True
        response.message = f"floor switch acknowledged: current_floor={target}"
        self.get_logger().info(response.message)
        self.publish_status()
        return response

    def publish_status(self):
        payload = {
            "current_floor": self.get_parameter("current_floor").value,
            "target_floor": self.get_parameter("target_floor").value,
            "spawn_point_id": self.get_parameter("spawn_point_id").value,
            "pending": self.pending,
            "mode": "manual_2phase",
        }
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = FloorOrchestratorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

- [ ] **Step 5: Add launch file**

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="floor_orchestrator_pkg",
            executable="floor_orchestrator_node",
            name="floor_orchestrator_node",
            output="screen",
            parameters=[{
                "current_floor": "F1",
                "target_floor": "F2",
                "spawn_point_id": "elevator_inside",
            }],
        )
    ])
```

- [ ] **Step 6: Build and service-test**

```bash
cd test_workspace/elevator_mission
colcon build --symlink-install --packages-select floor_orchestrator_pkg
source install/setup.bash
ros2 launch floor_orchestrator_pkg floor_orchestrator.launch.py
```

In another terminal:

```bash
cd test_workspace/elevator_mission
source install/setup.bash
ros2 param set /floor_orchestrator_node target_floor F2
ros2 service call /floor_orchestrator/request_switch std_srvs/srv/Trigger
ros2 topic echo /floor_orchestrator/status --once
ros2 service call /floor_orchestrator/ack std_srvs/srv/Trigger
```

Expected: request returns `success=True`, status shows `pending=true`, ack returns `success=True`, status shows `current_floor=F2`.

- [ ] **Step 7: Commit**

```bash
git add test_workspace/elevator_mission/src/floor_orchestrator_pkg
git commit -m "feat: add manual floor orchestrator"
```

## Task 4: Mission Package Dry-Run First

**Files:**
- Create: `test_workspace/elevator_mission/src/elevator_mission_pkg/package.xml`
- Create: `test_workspace/elevator_mission/src/elevator_mission_pkg/setup.py`
- Create: `test_workspace/elevator_mission/src/elevator_mission_pkg/setup.cfg`
- Create: `test_workspace/elevator_mission/src/elevator_mission_pkg/resource/elevator_mission_pkg`
- Create: `test_workspace/elevator_mission/src/elevator_mission_pkg/elevator_mission_pkg/__init__.py`
- Create: `test_workspace/elevator_mission/src/elevator_mission_pkg/elevator_mission_pkg/mission_model.py`
- Create: `test_workspace/elevator_mission/src/elevator_mission_pkg/elevator_mission_pkg/mission_tree.py`
- Create: `test_workspace/elevator_mission/scripts/test_behaviors_dryrun.py`

- [ ] **Step 1: Create package metadata**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>elevator_mission_pkg</name>
  <version>0.1.0</version>
  <description>Delivery mission behavior tree for isolated elevator PoC</description>
  <maintainer email="inonewater@todo.com">PackagU</maintainer>
  <license>Apache-2.0</license>
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>std_srvs</depend>
  <depend>geometry_msgs</depend>
  <depend>nav2_msgs</depend>
  <exec_depend>py_trees</exec_depend>
  <exec_depend>py_trees_ros</exec_depend>
  <buildtool_depend>ament_python</buildtool_depend>
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] **Step 2: Create resource marker and `setup.cfg`**

The resource marker file is intentionally empty.

```text
```

```ini
[develop]
script_dir=$base/lib/elevator_mission_pkg
[install]
install_scripts=$base/lib/elevator_mission_pkg
```

- [ ] **Step 3: Write `setup.py`**

```python
from glob import glob
from setuptools import setup

package_name = "elevator_mission_pkg"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="PackagU",
    maintainer_email="inonewater@todo.com",
    description="Delivery mission behavior tree for isolated elevator PoC",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "delivery_mission_node = elevator_mission_pkg.delivery_mission_node:main",
        ],
    },
)
```

- [ ] **Step 4: Write dry-run test**

The first mission test must not require Gazebo, Nav2, or ROS action servers. It checks that the mission sequence is built in the required order from `delivery_missions.yaml`.

```python
#!/usr/bin/env python3
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / "src" / "elevator_mission_pkg"
sys.path.insert(0, str(PKG))
sys.path.insert(0, str(ROOT / "scripts"))

from elevator_mission_pkg.mission_model import load_mission
from elevator_mission_pkg.mission_tree import build_dryrun_steps


def main():
    mission = load_mission(ROOT / "config" / "delivery_missions.yaml", "parcel_to_208")
    steps = build_dryrun_steps(mission)
    expected = [
        "NavigateToPoint:F1:parcel_pickup",
        "WaitForAck:parcel_loaded",
        "NavigateToPoint:F1:elevator_entry",
        "CallElevator:F2",
        "WaitElevatorArrived:F1:open",
        "NavigateToPoint:F1:elevator_inside",
        "SwitchFloor:F2:elevator_inside",
        "WaitElevatorArrived:F2:open",
        "Relocalize:F2:elevator_exit",
        "NavigateToPoint:F2:elevator_exit",
        "NavigateToPoint:F2:208",
        "WaitForAck:delivered",
    ]
    if steps != expected:
        raise AssertionError(f"unexpected steps: {steps}")
    print("PASS behavior dry-run")


if __name__ == "__main__":
    main()
```

- [ ] **Step 5: Run dry-run test to verify it fails**

```bash
python3 test_workspace/elevator_mission/scripts/test_behaviors_dryrun.py
```

Expected: FAIL with missing `elevator_mission_pkg.mission_model`.

- [ ] **Step 6: Implement mission model and dry-run builder**

```python
from dataclasses import dataclass
from pathlib import Path
import yaml


@dataclass(frozen=True)
class DeliveryMission:
    mission_id: str
    start_floor: str
    target_floor: str
    pickup_point: str
    elevator_entry_point: str
    elevator_inside_point: str
    elevator_exit_point: str
    destination_point: str
    mock_load_event: str
    mock_delivery_event: str


def load_mission(path, mission_id):
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    mission_data = data["missions"][mission_id]
    return DeliveryMission(mission_id=mission_id, **mission_data)
```

```python
def build_dryrun_steps(mission):
    return [
        f"NavigateToPoint:{mission.start_floor}:{mission.pickup_point}",
        f"WaitForAck:{mission.mock_load_event}",
        f"NavigateToPoint:{mission.start_floor}:{mission.elevator_entry_point}",
        f"CallElevator:{mission.target_floor}",
        f"WaitElevatorArrived:{mission.start_floor}:open",
        f"NavigateToPoint:{mission.start_floor}:{mission.elevator_inside_point}",
        f"SwitchFloor:{mission.target_floor}:{mission.elevator_inside_point}",
        f"WaitElevatorArrived:{mission.target_floor}:open",
        f"Relocalize:{mission.target_floor}:{mission.elevator_exit_point}",
        f"NavigateToPoint:{mission.target_floor}:{mission.elevator_exit_point}",
        f"NavigateToPoint:{mission.target_floor}:{mission.destination_point}",
        f"WaitForAck:{mission.mock_delivery_event}",
    ]
```

- [ ] **Step 7: Verify dry-run passes**

```bash
python3 test_workspace/elevator_mission/scripts/test_behaviors_dryrun.py
```

Expected: `PASS behavior dry-run`.

- [ ] **Step 8: Commit**

```bash
git add test_workspace/elevator_mission/src/elevator_mission_pkg \
  test_workspace/elevator_mission/scripts/test_behaviors_dryrun.py
git commit -m "test: add elevator mission dry run"
```

## Task 5: ROS Mission Behaviors

**Files:**
- Create: `test_workspace/elevator_mission/src/elevator_mission_pkg/elevator_mission_pkg/behaviors.py`
- Create: `test_workspace/elevator_mission/src/elevator_mission_pkg/elevator_mission_pkg/delivery_mission_node.py`
- Create: `test_workspace/elevator_mission/src/elevator_mission_pkg/launch/elevator_mission_demo.launch.py`

- [ ] **Step 1: Implement behavior boundaries**

Create one class per behavior. Keep each class small. The first implementation may support `dry_run:=true` so mission sequencing can be tested without Nav2.

| Behavior | ROS dependency | Success condition |
|----------|----------------|-------------------|
| `NavigateToPoint` | Nav2 `NavigateToPose` action | action result success, or dry-run immediate success |
| `CallElevator` | `/elevator/call` publisher | publish command once |
| `WaitElevatorArrived` | `/elevator/state` subscriber | JSON has target floor and `door_state=open` |
| `SwitchFloor` | param client + `/floor_orchestrator/request_switch` + `/floor_orchestrator/status` | status current floor equals target and pending false |
| `Relocalize` | `/initialpose` publisher | publish pose once |
| `WaitForAck` | parameter or CLI mock | dry-run immediate success in MVP |

- [ ] **Step 2: Add launch file**

The launch file starts mission, elevator sim, and floor orchestrator. Gazebo/Nav2 still run in separate terminals for MVP.

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="elevator_sim_pkg",
            executable="elevator_sim_node",
            name="elevator_sim_node",
            output="screen",
        ),
        Node(
            package="floor_orchestrator_pkg",
            executable="floor_orchestrator_node",
            name="floor_orchestrator_node",
            output="screen",
        ),
        Node(
            package="elevator_mission_pkg",
            executable="delivery_mission_node",
            name="delivery_mission_node",
            output="screen",
            parameters=[{
                "mission_id": "parcel_to_208",
                "dry_run_nav2": False,
            }],
        ),
    ])
```

- [ ] **Step 3: Build all test packages**

```bash
cd test_workspace/elevator_mission
colcon build --symlink-install
source install/setup.bash
```

Expected: build finishes with `elevator_mission_pkg`, `elevator_sim_pkg`, and `floor_orchestrator_pkg`.

- [ ] **Step 4: Commit**

```bash
git add test_workspace/elevator_mission/src/elevator_mission_pkg
git commit -m "feat: add elevator delivery mission node"
```

## Task 6: Manual 2-Phase Demo Script

**Files:**
- Create: `test_workspace/elevator_mission/scripts/run_demo.sh`

- [ ] **Step 1: Write command guide script**

The script prints the terminals to open and exact commands to run. It should not kill processes.

```bash
#!/usr/bin/env bash
set -euo pipefail

cat <<'EOF'
Elevator mission MVP manual demo

Terminal A: F1 Gazebo
  cd /ros2_ws
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  ros2 launch common_pkg gazebo.launch.py floor:=F1 use_sim_time:=true

Terminal B: F1 Nav2
  cd /ros2_ws
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  ros2 launch slam_pkg kku_navigation.launch.py floor:=F1

Terminal C: Mission workspace
  cd /ros2_ws/test_workspace/elevator_mission
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  ros2 launch elevator_mission_pkg elevator_mission_demo.launch.py

When mission logs WAITING_FOR_USER_ACK:
  1. Stop F1 Gazebo/Nav2 terminals.
  2. Relaunch with F2:
     ros2 launch common_pkg gazebo.launch.py floor:=F2 use_sim_time:=true
     ros2 launch slam_pkg kku_navigation.launch.py floor:=F2
  3. Ack:
     ros2 service call /floor_orchestrator/ack std_srvs/srv/Trigger
EOF
```

- [ ] **Step 2: Verify script syntax**

```bash
bash -n test_workspace/elevator_mission/scripts/run_demo.sh
```

Expected: no output and exit code 0.

- [ ] **Step 3: Commit**

```bash
git add test_workspace/elevator_mission/scripts/run_demo.sh
git commit -m "docs: add elevator mission demo commands"
```

## Task 7: Completion Report

**Files:**
- Create: `test_workspace/elevator_mission/docs/completion_report.md`

- [ ] **Step 1: Write report after implementation**

Use this exact structure and fill it with observed commands and outcomes from the implementation session.

````markdown
# Elevator Mission MVP Completion Report

작성일: 2026-05-28

## 1. 구현 범위

- test workspace: @test_workspace/elevator_mission/
- 기존 src 패키지 수정 여부: 없음
- floor switch 방식: manual 2-Phase
- custom msg/srv 여부: 없음

## 2. 생성 파일

| 파일 | 역할 |
|------|------|
| `config/kku_nav_points.yaml` | point registry |
| `config/delivery_missions.yaml` | delivery mission config |
| `src/elevator_sim_pkg/` | mock elevator FSM |
| `src/floor_orchestrator_pkg/` | manual floor switch ack |
| `src/elevator_mission_pkg/` | mission tree and behaviors |

## 3. 검증 결과

```bash
python3 test_workspace/elevator_mission/scripts/test_point_registry.py
python3 test_workspace/elevator_mission/scripts/test_behaviors_dryrun.py
cd test_workspace/elevator_mission && colcon build --symlink-install
```

## 4. 수동 데모 결과

F1 `parcel_pickup` 에서 F2 `208`까지 수동 floor switch ack를 포함해 성공했는지 기록한다.

## 5. 승격 판단

성공하면 다음 후보로 승격한다.

- `elevator_mission_pkg` → 기존 `src/drive_pkg` 또는 새 production package
- `elevator_sim_pkg` → 실제 엘리베이터 bridge로 교체 가능한 mock package
- `floor_orchestrator_pkg` → A3-lite teleport 구현으로 내부 교체
````

- [ ] **Step 2: Commit**

```bash
git add test_workspace/elevator_mission/docs/completion_report.md
git commit -m "docs: add elevator mission completion report"
```

## Final Verification

Run these commands before claiming completion.

```bash
python3 test_workspace/elevator_mission/scripts/test_point_registry.py
python3 test_workspace/elevator_mission/scripts/test_behaviors_dryrun.py
bash -n test_workspace/elevator_mission/scripts/run_demo.sh
cd test_workspace/elevator_mission && colcon build --symlink-install
```

Expected:

- point registry test prints `PASS point registry`
- dry-run test prints `PASS behavior dry-run`
- shell syntax check exits with code 0
- colcon build completes for all test packages

## Self-Review Checklist

- Every implementation file is under @test_workspace/elevator_mission/.
- No existing `src/` package is modified.
- `elevator_msgs` is not created in MVP.
- All ROS interfaces use `std_msgs/String`, `std_srvs/Trigger`, or existing Nav2/geometry messages.
- Manual floor switch instructions are visible in logs and @test_workspace/elevator_mission/scripts/run_demo.sh.
- @test_workspace/elevator_mission/docs/completion_report.md is written after verification.
