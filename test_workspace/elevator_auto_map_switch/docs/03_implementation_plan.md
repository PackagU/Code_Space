# Auto Map Switch Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build an isolated ROS2 PoC that switches Nav2 to the target floor map automatically when the elevator reports arrival at that floor.

**Architecture:** Keep `test_workspace/elevator_mission/` as legacy manual 2-Phase. Create a new auto map switch workspace with a focused orchestrator package, offline tests, map registry config, and a minimal mission harness. The orchestrator owns elevator arrival detection, map loading, costmap clearing, and ready status publication.

**Tech Stack:** ROS2 Humble, Python 3, `rclpy`, `std_msgs/String`, `std_srvs/Trigger`, `nav2_msgs/srv/LoadMap`, `geometry_msgs/PoseWithCovarianceStamped`, YAML.

---

## File Structure

| Path | Action | Responsibility |
|------|--------|----------------|
| `test_workspace/elevator_auto_map_switch/config/floor_maps.yaml` | Create | 층별 map yaml과 initial pose id |
| `test_workspace/elevator_auto_map_switch/scripts/floor_map_registry.py` | Create | YAML parser, map path resolver |
| `test_workspace/elevator_auto_map_switch/scripts/test_floor_map_registry.py` | Create | offline config test |
| `test_workspace/elevator_auto_map_switch/scripts/test_auto_switch_state_machine.py` | Create | automatic switch dry-run state test |
| `test_workspace/elevator_auto_map_switch/src/auto_floor_orchestrator_pkg/` | Create | ROS2 orchestrator package |
| `test_workspace/elevator_auto_map_switch/src/auto_floor_orchestrator_pkg/auto_floor_orchestrator_pkg/auto_floor_orchestrator_node.py` | Create | elevator arrival, map load, status publish |
| `test_workspace/elevator_auto_map_switch/src/auto_floor_orchestrator_pkg/launch/auto_floor_orchestrator.launch.py` | Create | orchestrator launch |
| `test_workspace/elevator_auto_map_switch/docs/04_verification_plan.md` | Modify | 구현 후 검증 결과 기록 |

## Task 1: Floor Map Registry

**Files:**

- Create: `test_workspace/elevator_auto_map_switch/config/floor_maps.yaml`
- Create: `test_workspace/elevator_auto_map_switch/scripts/floor_map_registry.py`
- Create: `test_workspace/elevator_auto_map_switch/scripts/test_floor_map_registry.py`

- [ ] **Step 1: Write failing registry test**

```python
#!/usr/bin/env python3
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from floor_map_registry import FloorMapRegistry


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def main():
    registry = FloorMapRegistry.from_file(ROOT / "config" / "floor_maps.yaml")

    f2 = registry.get("F2")
    require(f2.floor == "F2", "floor id mismatch")
    require(f2.map_yaml.endswith("kku_f2.yaml"), "F2 map yaml mismatch")
    require(f2.initial_pose_id == "elevator_exit", "F2 initial pose mismatch")

    try:
        registry.get("B1")
    except KeyError as exc:
        require("B1" in str(exc), "missing floor error should include floor id")
    else:
        raise AssertionError("unknown floor should raise KeyError")

    print("PASS floor map registry")


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Run test and confirm failure**

```bash
python3 test_workspace/elevator_auto_map_switch/scripts/test_floor_map_registry.py
```

Expected:

```text
ModuleNotFoundError: No module named 'floor_map_registry'
```

- [ ] **Step 3: Write `floor_maps.yaml`**

```yaml
schema_version: 1
frame_id: map
floors:
  F1:
    map_yaml: src/slam_pkg/maps/kku_virtual/f1/kku_f1.yaml
    initial_pose_id: elevator_exit
  F2:
    map_yaml: src/slam_pkg/maps/kku_virtual/f2/kku_f2.yaml
    initial_pose_id: elevator_exit
  F3:
    map_yaml: src/slam_pkg/maps/kku_virtual/f3/kku_f3.yaml
    initial_pose_id: elevator_exit
```

- [ ] **Step 4: Implement registry**

```python
from dataclasses import dataclass
from pathlib import Path
import yaml


@dataclass(frozen=True)
class FloorMap:
    floor: str
    map_yaml: str
    initial_pose_id: str


class FloorMapRegistry:
    def __init__(self, frame_id, floors):
        self.frame_id = frame_id
        self.floors = floors

    @classmethod
    def from_file(cls, path):
        data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        floors = {}
        for floor_id, floor_data in data["floors"].items():
            floor_key = str(floor_id).upper()
            floors[floor_key] = FloorMap(
                floor=floor_key,
                map_yaml=str(floor_data["map_yaml"]),
                initial_pose_id=str(floor_data["initial_pose_id"]),
            )
        return cls(frame_id=data.get("frame_id", "map"), floors=floors)

    def get(self, floor):
        floor_key = str(floor).upper()
        if floor_key not in self.floors:
            raise KeyError(f"unknown floor: {floor_key}")
        return self.floors[floor_key]
```

- [ ] **Step 5: Run registry test**

```bash
python3 test_workspace/elevator_auto_map_switch/scripts/test_floor_map_registry.py
```

Expected:

```text
PASS floor map registry
```

## Task 2: Auto Switch State Machine Dry-Run

**Files:**

- Create: `test_workspace/elevator_auto_map_switch/scripts/auto_switch_core.py`
- Create: `test_workspace/elevator_auto_map_switch/scripts/test_auto_switch_state_machine.py`

- [ ] **Step 1: Write failing state machine test**

```python
#!/usr/bin/env python3
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from auto_switch_core import AutoSwitchCore


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def main():
    core = AutoSwitchCore(current_floor="F1")

    status = core.request_switch(target_floor="F2", spawn_point_id="elevator_inside")
    require(status["pending"] is True, "switch should become pending")
    require(status["phase"] == "waiting_elevator", "phase should wait for elevator")

    status = core.on_elevator_state({
        "current_floor": "F1",
        "target_floor": "F2",
        "door_state": "closed",
        "state": "MOVING",
    })
    require(status["pending"] is True, "wrong floor should keep pending")

    status = core.on_elevator_state({
        "current_floor": "F2",
        "target_floor": "F2",
        "door_state": "open",
        "state": "ARRIVED_OPEN",
    })
    require(status["phase"] == "ready", "arrival should finish dry-run switch")
    require(status["pending"] is False, "arrival should clear pending")
    require(status["map_loaded"] is True, "dry-run map load should be true")

    print("PASS auto switch state machine")


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Run test and confirm failure**

```bash
python3 test_workspace/elevator_auto_map_switch/scripts/test_auto_switch_state_machine.py
```

Expected:

```text
ModuleNotFoundError: No module named 'auto_switch_core'
```

- [ ] **Step 3: Implement dry-run core**

```python
class AutoSwitchCore:
    def __init__(self, current_floor="F1"):
        self.current_floor = current_floor
        self.target_floor = current_floor
        self.spawn_point_id = "elevator_inside"
        self.pending = False
        self.phase = "idle"
        self.map_loaded = False
        self.error = ""

    def request_switch(self, target_floor, spawn_point_id):
        self.target_floor = str(target_floor).upper()
        self.spawn_point_id = str(spawn_point_id)
        self.pending = True
        self.phase = "waiting_elevator"
        self.map_loaded = False
        self.error = ""
        return self.status()

    def on_elevator_state(self, state):
        if not self.pending:
            return self.status()
        arrived = (
            state.get("current_floor") == self.target_floor
            and state.get("door_state") == "open"
        )
        if not arrived:
            return self.status()
        self.current_floor = self.target_floor
        self.phase = "ready"
        self.pending = False
        self.map_loaded = True
        return self.status()

    def status(self):
        return {
            "current_floor": self.current_floor,
            "target_floor": self.target_floor,
            "spawn_point_id": self.spawn_point_id,
            "pending": self.pending,
            "phase": self.phase,
            "map_loaded": self.map_loaded,
            "error": self.error,
            "mode": "auto_map_switch_dry_run",
        }
```

- [ ] **Step 4: Run dry-run test**

```bash
python3 test_workspace/elevator_auto_map_switch/scripts/test_auto_switch_state_machine.py
```

Expected:

```text
PASS auto switch state machine
```

## Task 3: ROS2 Auto Orchestrator Package

**Files:**

- Create: `test_workspace/elevator_auto_map_switch/src/auto_floor_orchestrator_pkg/package.xml`
- Create: `test_workspace/elevator_auto_map_switch/src/auto_floor_orchestrator_pkg/setup.py`
- Create: `test_workspace/elevator_auto_map_switch/src/auto_floor_orchestrator_pkg/setup.cfg`
- Create: `test_workspace/elevator_auto_map_switch/src/auto_floor_orchestrator_pkg/resource/auto_floor_orchestrator_pkg`
- Create: `test_workspace/elevator_auto_map_switch/src/auto_floor_orchestrator_pkg/auto_floor_orchestrator_pkg/__init__.py`
- Create: `test_workspace/elevator_auto_map_switch/src/auto_floor_orchestrator_pkg/auto_floor_orchestrator_pkg/auto_floor_orchestrator_node.py`

- [ ] **Step 1: Create package metadata**

Use a standard `ament_python` ROS2 package. Required dependencies:

```xml
<depend>rclpy</depend>
<depend>std_msgs</depend>
<depend>std_srvs</depend>
<depend>geometry_msgs</depend>
<depend>nav2_msgs</depend>
```

- [ ] **Step 2: Implement node parameters**

Node parameters:

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `current_floor` | `F1` | starting logical floor |
| `target_floor` | `F2` | requested target floor |
| `spawn_point_id` | `elevator_inside` | requested spawn point |
| `dry_run_map_load` | `true` | skip Nav2 load service when true |
| `load_map_service` | `/map_server/load_map` | Nav2 map load service |
| `status_period_sec` | `0.5` | status publish period |

- [ ] **Step 3: Implement subscriptions and services**

ROS interfaces:

| Interface | Type | Role |
|-----------|------|------|
| `/elevator/state` | `std_msgs/String` | detect target floor arrival |
| `/floor_orchestrator/request_switch` | `std_srvs/Trigger` | start switch |
| `/floor_orchestrator/status` | `std_msgs/String` | publish status JSON |

- [ ] **Step 4: Add Nav2 map load client**

When `dry_run_map_load=false`, call:

```text
/map_server/load_map
```

with type:

```text
nav2_msgs/srv/LoadMap
```

Expected behavior:

- service unavailable: keep `phase=loading_map`, retry until timeout is added later
- response success: set `map_loaded=true`, `pending=false`, `phase=ready`
- response failure: set `phase=failed`, `error=<message>`, `pending=false`

## Task 4: Mission Compatibility Behavior

**Files:**

- Create: `test_workspace/elevator_auto_map_switch/scripts/test_switch_floor_compatibility.py`
- Create or modify in new package only: `auto_switch_floor_behavior.py`

- [ ] **Step 1: Write compatibility test**

The test should prove that a behavior waiting on `/floor_orchestrator/status` succeeds when status contains:

```json
{
  "current_floor": "F2",
  "target_floor": "F2",
  "pending": false,
  "map_loaded": true,
  "phase": "ready"
}
```

and keeps running when `map_loaded=false`.

- [ ] **Step 2: Implement minimal compatibility behavior**

Do not edit legacy `test_workspace/elevator_mission/src/elevator_mission_pkg/elevator_mission_pkg/behaviors.py`. Implement a new behavior or adapter in this workspace first.

## Task 5: Verification And Documentation

**Files:**

- Modify: `test_workspace/elevator_auto_map_switch/docs/04_verification_plan.md`
- Create after implementation: `test_workspace/elevator_auto_map_switch/docs/completion_report.md`

- [ ] **Step 1: Run offline tests**

```bash
python3 test_workspace/elevator_auto_map_switch/scripts/test_floor_map_registry.py
python3 test_workspace/elevator_auto_map_switch/scripts/test_auto_switch_state_machine.py
python3 test_workspace/elevator_auto_map_switch/scripts/test_switch_floor_compatibility.py
```

Expected:

```text
PASS floor map registry
PASS auto switch state machine
PASS switch floor compatibility
```

- [ ] **Step 2: Run ROS2 build in container**

```bash
cd /ros2_ws/test_workspace/elevator_auto_map_switch
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 pkg list | grep auto_floor_orchestrator_pkg
```

Expected:

```text
auto_floor_orchestrator_pkg
```

- [ ] **Step 3: Run dry-run ROS smoke**

```bash
ros2 run auto_floor_orchestrator_pkg auto_floor_orchestrator_node \
  --ros-args \
  -p dry_run_map_load:=true \
  -p current_floor:=F1 \
  -p target_floor:=F2
```

In another terminal:

```bash
ros2 service call /floor_orchestrator/request_switch std_srvs/srv/Trigger
# NOTE: --once may be dropped by the DDS discovery race. Publish several times.
ros2 topic pub --times 10 --rate 2 /elevator/state std_msgs/msg/String \
  "{data: '{\"current_floor\":\"F2\",\"target_floor\":\"F2\",\"door_state\":\"open\",\"state\":\"ARRIVED_OPEN\"}'}"
ros2 topic echo /floor_orchestrator/status --once
```

Expected status:

```json
{"current_floor":"F2","map_loaded":true,"pending":false,"phase":"ready"}
```

## Self-Review Notes

- Scope is isolated to `test_workspace/elevator_auto_map_switch/`.
- Legacy manual PoC is not modified.
- Plan separates dry-run state validation from live Nav2 map service validation.
- The open question about Gazebo world or robot respawn is intentionally kept outside this first PoC; this plan automates Nav2 map switching after elevator arrival.
