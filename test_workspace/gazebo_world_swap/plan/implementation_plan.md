# Gazebo World Swap Implementation Plan

> For agentic workers: execute one small increment at a time. Each increment has
> a goal, a failing test or observable check, the minimum implementation, a
> headless verification step, and a commit.

## Goal

Make the existing F1 to F2 auto map switch also switch Gazebo from F1 geometry
to F2 geometry, then leave a reproducible command/script so a human can rerun
the simulation and verify the result by topics and services.

## Constraints

- Do not modify `auto_floor_orchestrator_pkg`; the new world swapper observes
  `/floor_orchestrator/status`.
- Do not use gzclient/RViz GUI for success claims.
- Do not use `/elevator/state --once`; use repeated publication or the elevator
  sim.
- Do not sync or upload to Notion.
- Do not read `.env`, tokens, credentials, SSH keys, auth/session files, or
  untracked `.env.example`.
- Stage `test_workspace/` and `docs/` files with `git add --sparse`.

## Approach

### Option A: Gazebo Restart

Add `spawn_point:=elevator_exit|elevator_inside` to
`src/common_pkg/launch/gazebo.launch.py`. The default remains
`elevator_exit` for backward compatibility. The swapper can restart Gazebo with
`floor:=F2 spawn_point:=elevator_inside`.

This is the first runnable PoC path because it reuses existing launch files. It
may reset sim time and can require process cleanup, so it is mainly a fallback
and a compatibility path.

### Option B: Runtime Building Model Swap

Keep gzserver alive. Extract the `<model name="kku_f2_building">...</model>`
block from `src/common_pkg/worlds/kku_f2.world`, call `/delete_entity` for the
old building, then call `/spawn_entity` for the new building. The robot remains
at the elevator-inside origin.

This is the target path because `/clock`, Nav2, and AMCL stay alive. The world
files already contain one static building model per floor, so model extraction
is small and testable.

## File Map

| File | Action | Responsibility |
|------|--------|----------------|
| `src/common_pkg/launch/gazebo.launch.py` | Modify | Add backward-compatible `spawn_point` launch argument and pure pose resolver |
| `test_workspace/gazebo_world_swap/src/gazebo_world_swap_pkg/package.xml` | Create | ROS package metadata |
| `test_workspace/gazebo_world_swap/src/gazebo_world_swap_pkg/setup.py` | Create | Install launch files and console scripts |
| `test_workspace/gazebo_world_swap/src/gazebo_world_swap_pkg/setup.cfg` | Create | ament Python script paths |
| `test_workspace/gazebo_world_swap/src/gazebo_world_swap_pkg/gazebo_world_swap_pkg/world_model.py` | Create | Pure world path and building SDF extraction helpers |
| `test_workspace/gazebo_world_swap/src/gazebo_world_swap_pkg/gazebo_world_swap_pkg/swap_trigger.py` | Create | Pure status dedupe/trigger logic |
| `test_workspace/gazebo_world_swap/src/gazebo_world_swap_pkg/gazebo_world_swap_pkg/world_swap_node.py` | Create | ROS node that watches status and swaps Gazebo world |
| `test_workspace/gazebo_world_swap/src/gazebo_world_swap_pkg/launch/world_swap.launch.py` | Create | Start the simulation-only swap node |
| `test_workspace/gazebo_world_swap/scripts/test_gazebo_launch_spawn_point.py` | Create | Failing-then-passing test for launch spawn pose resolver |
| `test_workspace/gazebo_world_swap/scripts/test_world_model.py` | Create | Failing-then-passing test for model extraction |
| `test_workspace/gazebo_world_swap/scripts/test_swap_trigger.py` | Create | Failing-then-passing test for status trigger dedupe |
| `test_workspace/gazebo_world_swap/scripts/run_l3_world_swap_smoke.sh` | Create | One-command headless L3 plus world swap smoke runner |
| `test_workspace/gazebo_world_swap/scripts/verify_world_swap_state.sh` | Create | Topic/service checks for map, status, model, scan |
| `test_workspace/gazebo_world_swap/verification/` | Update | Store command outputs from build and smoke runs |
| `test_workspace/gazebo_world_swap/debug/` | Update | Store repeated failure analysis if needed |

## Increment 1: Gazebo Launch Spawn Point

Goal: keep existing starts unchanged while allowing swap starts at
`elevator_inside`.

Validation:

```bash
python3 test_workspace/gazebo_world_swap/scripts/test_gazebo_launch_spawn_point.py
```

Expected RED before implementation:

```text
AttributeError: module 'gazebo_launch' has no attribute 'resolve_spawn_pose'
```

Implementation:

- Add `SPAWN_POINTS` with `elevator_exit` and `elevator_inside`.
- Add `resolve_spawn_pose(floor, spawn_point)`.
- Add `DeclareLaunchArgument("spawn_point", default_value="elevator_exit")`.
- Use the resolver in `_launch_setup`.

Expected GREEN:

```text
PASS gazebo launch spawn_point resolver
```

Then run in container:

```bash
cd /ros2_ws && colcon build --symlink-install --packages-select common_pkg
source /ros2_ws/install/setup.bash
ros2 launch common_pkg gazebo.launch.py floor:=F2 spawn_point:=elevator_inside use_sim_time:=true
```

Headless observation:

```bash
ros2 topic list | grep -E '^/clock$|^/scan$|^/odom$'
```

Commit:

```bash
git add src/common_pkg/launch/gazebo.launch.py test_workspace/gazebo_world_swap
git commit -m "feat: add gazebo spawn point selection"
```

## Increment 2: Pure Runtime Swap Helpers

Goal: extract a target floor building model from the existing `.world` file and
choose exactly one swap trigger per successful status transition.

Validation:

```bash
python3 test_workspace/gazebo_world_swap/scripts/test_world_model.py
python3 test_workspace/gazebo_world_swap/scripts/test_swap_trigger.py
```

Expected RED before implementation:

```text
ModuleNotFoundError: No module named 'gazebo_world_swap_pkg'
```

Implementation:

- Create `world_model.py` using `xml.etree.ElementTree`.
- Return a model XML string whose root is `<model name="kku_f2_building">`.
- Create `swap_trigger.py` with `WorldSwapTrigger`.
- Trigger only when status has `phase=ready`, `pending=false`,
  `map_loaded=true`, and `current_floor` differs from the last handled floor.
- Ignore malformed JSON, failed/pending statuses, and duplicate ready statuses.

Expected GREEN:

```text
PASS world model extraction
PASS world swap trigger
```

Commit:

```bash
git add --sparse test_workspace/gazebo_world_swap
git commit -m "feat: add world swap model helpers"
```

## Increment 3: Simulation-Only World Swap Node

Goal: run a separate node that observes the orchestrator and performs method B
by Gazebo services, with method A restart mode available as a fallback.

Validation:

```bash
cd /ros2_ws/test_workspace/gazebo_world_swap
colcon build --symlink-install
source install/setup.bash
ros2 pkg list | grep gazebo_world_swap_pkg
```

Implementation:

- Create `world_swap_node.py`.
- Parameters:
  - `method`: `model_swap` or `restart`, default `model_swap`
  - `initial_floor`: default `F1`
  - `world_dir`: default empty, resolved from `common_pkg`
  - `status_topic`: default `/floor_orchestrator/status`
  - `delete_entity_service`: default `/delete_entity`
  - `spawn_entity_service`: default `/spawn_entity`
  - `get_entity_state_service`: default `/get_entity_state`
  - `restart_command_template`: default
    `ros2 launch common_pkg gazebo.launch.py floor:={floor} spawn_point:=elevator_inside use_sim_time:=true`
- For `model_swap`, call `gazebo_msgs/srv/DeleteEntity` for the current building,
  then `gazebo_msgs/srv/SpawnEntity` with the target building XML and zero pose.
- Publish `/gazebo_world_swap/status` as JSON for verification.
- For `restart`, terminate Gazebo processes and start the formatted restart
  command in a new process group. Use only as fallback because method B is the
  target.

Expected GREEN:

```text
gazebo_world_swap_pkg
```

Commit:

```bash
git add --sparse test_workspace/gazebo_world_swap
git commit -m "feat: add gazebo world swap node"
```

## Increment 4: One-Command Headless Smoke

Goal: automate the L3 plus world swap smoke so a human can rerun it from the
container with one script.

Validation command:

```bash
cd /ros2_ws
bash test_workspace/gazebo_world_swap/scripts/run_l3_world_swap_smoke.sh
```

Script flow:

1. Clean stale Gazebo/Nav2/orchestrator/swap processes.
2. Build root workspace, `elevator_auto_map_switch`, `elevator_mission`, and
   `gazebo_world_swap`.
3. Launch Gazebo F1 headless.
4. Launch Nav2 F1 with `rviz:=false`.
5. Launch auto orchestrator with `dry_run_map_load:=false target_floor:=F2`.
6. Launch world swap node with `method:=model_swap initial_floor:=F1`.
7. Arm switch with `/floor_orchestrator/request_switch`.
8. Publish F2 `ARRIVED_OPEN` elevator state 10 times at 2 Hz.
9. Run `verify_world_swap_state.sh`.
10. Write raw outputs under `verification/latest/`.

Expected observations:

```text
status phase=ready pending=false current_floor=F2 map_loaded=true
map width=498 height=348 origin=-2.46,-2.96
entity kku_f2_building present
entity kku_f1_building absent
/scan has finite ranges
```

Commit:

```bash
git add --sparse test_workspace/gazebo_world_swap
git commit -m "test: add world swap smoke runner"
```

## Increment 5: L5 Mission Run

Goal: run the actual mission path far enough to prove F2 path generation and
arrival, or record the exact blocker with logs.

Validation command:

```bash
cd /ros2_ws
source install/setup.bash
source test_workspace/elevator_auto_map_switch/install/setup.bash
source test_workspace/elevator_mission/install/setup.bash
source test_workspace/gazebo_world_swap/install/setup.bash
ros2 run elevator_sim_pkg elevator_sim_node &
ros2 launch auto_floor_orchestrator_pkg auto_floor_orchestrator.launch.py dry_run_map_load:=false use_sim_time:=true &
ros2 launch gazebo_world_swap_pkg world_swap.launch.py method:=model_swap initial_floor:=F1 use_sim_time:=true &
ros2 run elevator_mission_pkg delivery_mission_node --ros-args -p mission_id:=parcel_to_208
```

Expected observations:

```text
SwitchFloor:F2:elevator_inside acknowledged
Relocalize:F2:elevator_exit initialpose published
NavigateRoute:F2:elevator_exit->208
MISSION COMPLETE
```

If this fails twice at the same point, write a debug note with symptoms, at
least three hypotheses, evidence, root cause, and next action.

Commit:

```bash
git add --sparse test_workspace/gazebo_world_swap
git commit -m "test: record world swap e2e verification"
```

## Increment 6: Final Report

Goal: leave a concise session wiki report with evidence and limits.

Before editing `docs/`, read:

```bash
sed -n '1,220p' docs/_style/notion_markdown_style.md
```

Create `docs/session_wiki/2026-06-25_gazebo_world_swap/` with:

- `README.md`
- `verification_report.md`

Content:

- Objective
- Method A and B summary
- Files changed
- Verification table
- Raw log references
- Known limitations
- Next steps

Do not run Notion sync.

Commit:

```bash
git add --sparse docs/session_wiki/2026-06-25_gazebo_world_swap test_workspace/gazebo_world_swap TODO.md
git commit -m "docs: record gazebo world swap verification"
```

## Done Definition

- `/floor_orchestrator/status` says F2 ready with `pending=false`.
- `/map` is F2 dimensions and origin.
- Gazebo has `kku_f2_building` and no `kku_f1_building`.
- `/scan` publishes finite ranges after swap.
- F2 Nav2 path/arrival is demonstrated, or an honest blocker report exists.
- Quickstart and one-command runner exist under `test_workspace/gazebo_world_swap/`.
- No Notion sync or external upload occurred.
