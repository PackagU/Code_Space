# Gazebo World Swap Workspace

This workspace contains the simulation-only Gazebo world swap work for the
elevator auto map switch PoC.

## Purpose

When the auto floor orchestrator changes the Nav2 map from F1 to F2, Gazebo
must also expose F2 geometry so `/scan`, AMCL, and Nav2 all agree.

## Layout

| Path | Role |
|------|------|
| `plan/` | Implementation plan, increment log, design decisions |
| `src/` | Simulation-only ROS2 package for world swap |
| `scripts/` | Offline tests and one-command runners |
| `verification/` | Captured command outputs and verification summaries |
| `debug/` | Debug notes when a loop fails twice or more |

## Current Status

Implemented and verified on 2026-06-25.
Extended and re-verified on 2026-06-29 so the one-command smoke starts at the
F1 elevator-front charge station, visits the F1 parcel pickup area, returns to
the elevator, then performs the F1 to F2 world swap before the F2 delivery
goal.

## Quickstart

Run this from the host while the `ros2_humble` container is running:

```bash
docker exec -it ros2_humble bash
cd /ros2_ws
bash test_workspace/gazebo_world_swap/scripts/run_l3_world_swap_smoke.sh
```

For a human-observable run with RViz and processes left alive at the end:

```bash
WITH_RVIZ=true KEEP_RUNNING=1 bash test_workspace/gazebo_world_swap/scripts/run_l3_world_swap_smoke.sh
```

Expected final lines:

```text
Goal finished with status: SUCCEEDED
Goal finished with status: SUCCEEDED
PASS world swap smoke
status phase=ready pending=false current_floor=F2 map_loaded=true
map width=498 height=348 origin=-2.46,-2.96
entity kku_f2_building present
entity kku_f1_building absent
/scan has finite ranges
Goal finished with status: SUCCEEDED
```

Logs are written to `test_workspace/gazebo_world_swap/verification/latest/`.
Set `KEEP_RUNNING=1` before the script if you want to leave the launched
Gazebo/Nav2 stack alive after verification.
Set `WITH_RVIZ=true` if you want Nav2 RViz to open during the run.
