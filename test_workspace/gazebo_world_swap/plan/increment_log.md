# Gazebo World Swap Increment Log

## 2026-06-29

### Increment 5: F1 Pickup Route Before World Swap

Goal: make the one-command smoke visually match the intended delivery flow:
start at the F1 elevator-front charge station, drive to the F1 parcel storage
area, return to the elevator, enter the elevator, then perform the F1 to F2 map
and Gazebo world swap.

Verification method:

```bash
python3 test_workspace/gazebo_world_swap/scripts/test_gazebo_launch_spawn_point.py
python3 test_workspace/gazebo_world_swap/scripts/test_smoke_scripts_contract.py
bash -n test_workspace/gazebo_world_swap/scripts/run_l3_world_swap_smoke.sh
docker exec ros2_humble bash -lc 'cd /ros2_ws && bash test_workspace/gazebo_world_swap/scripts/run_l3_world_swap_smoke.sh'
docker exec ros2_humble bash -lc 'cd /ros2_ws && source /opt/ros/humble/setup.bash && python3 test_workspace/gazebo_world_swap/scripts/test_world_model.py && python3 test_workspace/gazebo_world_swap/scripts/test_swap_trigger.py && python3 test_workspace/gazebo_world_swap/scripts/test_world_swap_node_contract.py && python3 test_workspace/gazebo_world_swap/scripts/test_gazebo_launch_spawn_point.py && python3 test_workspace/gazebo_world_swap/scripts/test_package_metadata.py && python3 test_workspace/gazebo_world_swap/scripts/test_smoke_scripts_contract.py'
```

Result:

```text
RED: charge_station spawn point was not accepted by gazebo.launch.py.
RED: smoke contract did not contain the F1 pickup route or RViz option.
GREEN: PASS gazebo launch spawn_point resolver.
GREEN: PASS smoke script contracts.
Full smoke PASS:
- f1_parcel_corridor SUCCEEDED
- f1_parcel_storage SUCCEEDED
- f1_parcel_pickup SUCCEEDED
- f1_elevator_entry SUCCEEDED
- f1_elevator_inside SUCCEEDED
- world swap verifier PASS
- f2_corridor SUCCEEDED
Offline 6-test bundle PASS.
```

## 2026-06-25

### Increment 0: Orientation And Plan

Goal: read the mandated context and create a project-local implementation plan.

Verification method:

```bash
sed -n '1,260p' test_workspace/elevator_auto_map_switch/docs/07_world_swap_design.md
sed -n '1,300p' test_workspace/elevator_auto_map_switch/docs/04_verification_plan.md
sed -n '1,340p' test_workspace/elevator_auto_map_switch/docs/completion_report.md
```

Result:

```text
Context read. Plan written in plan/implementation_plan.md.
```

### Increment 1: Gazebo Launch Spawn Point

Goal: add a backward-compatible `spawn_point` argument to
`common_pkg gazebo.launch.py`.

Verification method:

```bash
docker exec ros2_humble bash -lc 'cd /ros2_ws && source /opt/ros/humble/setup.bash && python3 test_workspace/gazebo_world_swap/scripts/test_gazebo_launch_spawn_point.py'
docker exec ros2_humble bash -lc 'cd /ros2_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select common_pkg'
ros2 launch common_pkg gazebo.launch.py floor:=F2 spawn_point:=elevator_inside use_sim_time:=true
ros2 topic list | grep -E '^/clock$|^/scan$|^/odom$'
```

Result:

```text
RED: AttributeError for missing resolve_spawn_pose.
GREEN: PASS gazebo launch spawn_point resolver.
Build: common_pkg finished.
Launch smoke: elevator_robot_f2 spawned; /clock, /odom, and /scan were present.
Cleanup: stale Gazebo smoke PIDs were killed explicitly.
```

### Increment 2: Pure Runtime Swap Helpers

Goal: add pure helpers for extracting floor building models and deduplicating
ready status messages.

Verification method:

```bash
python3 test_workspace/gazebo_world_swap/scripts/test_world_model.py
python3 test_workspace/gazebo_world_swap/scripts/test_swap_trigger.py
```

Result:

```text
RED: both tests failed with ModuleNotFoundError for gazebo_world_swap_pkg.
GREEN: PASS world model extraction.
GREEN: PASS world swap trigger.
```

### Increment 3: Simulation-Only World Swap Node

Goal: add `gazebo_world_swap_pkg` with a status-watching ROS node, model-swap
mode, and restart fallback helpers.

Verification method:

```bash
python3 test_workspace/gazebo_world_swap/scripts/test_world_swap_node_contract.py
python3 test_workspace/gazebo_world_swap/scripts/test_package_metadata.py
docker exec ros2_humble bash -lc 'cd /ros2_ws/test_workspace/gazebo_world_swap && source /opt/ros/humble/setup.bash && colcon build --symlink-install'
docker exec ros2_humble bash -lc 'source /opt/ros/humble/setup.bash && source /ros2_ws/test_workspace/gazebo_world_swap/install/setup.bash && ros2 pkg list | grep gazebo_world_swap_pkg'
docker exec ros2_humble bash -lc 'source /opt/ros/humble/setup.bash && source /ros2_ws/test_workspace/gazebo_world_swap/install/setup.bash && ros2 pkg executables gazebo_world_swap_pkg'
docker exec ros2_humble bash -lc 'cd /ros2_ws/test_workspace/gazebo_world_swap && source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && source install/setup.bash && timeout 5s ros2 run gazebo_world_swap_pkg world_swap_node --ros-args -p use_sim_time:=false'
```

Result:

```text
RED: contract test failed with missing world_swap_node; metadata test failed with missing package.xml.
GREEN: PASS world swap node contract.
GREEN: PASS gazebo world swap package metadata.
Build: gazebo_world_swap_pkg finished.
Discovery: ros2 pkg list found gazebo_world_swap_pkg; executable world_swap_node listed.
Debug 1: node smoke failed because use_sim_time was declared twice. Root cause: rclpy already owns use_sim_time when an override is present. Fixed by removing the explicit declaration.
Debug 2: node smoke failed when only the isolated install was sourced. Root cause: common_pkg is in /ros2_ws/install, so the root workspace must be sourced first. Reran with project-required sourcing.
Debug 3: timeout shutdown produced ExternalShutdownException. Fixed by catching rclpy.executors.ExternalShutdownException in main().
Final smoke: node logged world swap ready with common_pkg world_dir and exited under timeout without traceback.
```

### Increment 4: One-Command L3 World Swap Smoke

Goal: add a one-command headless smoke runner that builds the required
workspaces, launches Gazebo/Nav2/orchestrator/world-swap, triggers F1 to F2,
and verifies the map and Gazebo world match.

Verification method:

```bash
python3 test_workspace/gazebo_world_swap/scripts/test_smoke_scripts_contract.py
docker exec ros2_humble bash -lc 'cd /ros2_ws && bash test_workspace/gazebo_world_swap/scripts/run_l3_world_swap_smoke.sh'
docker exec ros2_humble bash -lc 'pgrep -af "gzserver|gzclient|component_container_isolated|auto_floor_orchestrator_node|world_swap_node|robot_state_publisher|ros2 launch" || true'
```

Result:

```text
RED: script contract failed because run_l3_world_swap_smoke.sh was missing.
GREEN: PASS smoke script contracts.
Debug 1: runner failed immediately under set -u while sourcing ROS setup. Root cause: ROS setup reads unset AMENT_TRACE_SETUP_FILES. Fixed with source_setup() that disables nounset while sourcing.
Debug 2: /get_entity_state was unavailable. Actual Gazebo services were /spawn_entity, /delete_entity, and /get_model_list (gazebo_msgs/srv/GetModelList). Fixed verifier and runner to use /get_model_list.
Debug 3: /spawn_entity rejected a bare <model> XML and tried deprecated parsing. Fixed by wrapping extracted building model XML as <sdf version="1.6">...</sdf>.
Debug 4: verifier missed late /gazebo_world_swap/status and /map. Fixed with transient-local QoS for the swap status publisher and verifier subscriptions.
Debug 5: failed runs left child processes alive. Fixed cleanup to kill the actual long-lived node/process names, not only launch PIDs.
Smoke PASS:
- status phase=ready pending=false current_floor=F2 map_loaded=true
- map width=477 height=299 origin=-1.99,-1.0 (당시 값 — 복도 5m 확장 후 498x348 / -2.46,-2.96, verifier는 이제 맵 yaml에서 자동 로드)
- entity kku_f2_building present
- entity kku_f1_building absent
- /scan has finite ranges
Nav2 F2 corridor goal:
- `(2.0, 12.0)` near room 208 finished with status SUCCEEDED.
Known waypoint issue:
- Direct goal to named point `208` `(2.35, 12.0)` aborted because the planner could not make a valid path. Recorded in `docs/improvement_report.md`.
Cleanup check: no Gazebo/Nav2/orchestrator/world-swap processes left.
```
