# KKU Simulation Helper Scripts Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add one-command helper scripts and a quickstart document for running KKU Gazebo + SLAM simulation from Linux desktop/VSCode.

**Architecture:** Keep the helpers as small Bash scripts under `scripts/`. `run_kku_sim.sh` handles container startup, ROS build, and launch; `teleop.sh` and `save_kku_map.sh` run focused commands in the running container. A shell test script verifies syntax and expected command wiring without requiring Gazebo GUI.

**Tech Stack:** Bash, Docker Compose, ROS2 Humble, colcon, Gazebo, SLAM Toolbox, nav2_map_server.

---

### Task 1: Add Script Contract Test

**Files:**
- Create: `scripts/test_kku_sim_scripts.sh`

- [x] **Step 1: Write failing test**

Create a Bash test that expects the helper scripts to exist, be executable, pass `bash -n`, and contain the core commands: `docker compose`, `colcon build`, `kku_simulation.launch.py`, `teleop_twist_keyboard`, and `map_saver_cli`.

- [x] **Step 2: Run test to verify it fails**

Run: `bash scripts/test_kku_sim_scripts.sh`

Expected: fail because the helper scripts do not exist yet.

### Task 2: Add Helper Scripts

**Files:**
- Create: `scripts/run_kku_sim.sh`
- Create: `scripts/teleop.sh`
- Create: `scripts/save_kku_map.sh`

- [x] **Step 1: Implement `run_kku_sim.sh`**

Validate floor input, open X11 access when `xhost` exists, start `ros2_humble` through `docker_env/compose/docker-compose.linux.yml`, build `common_pkg` and `slam_pkg`, then launch `slam_pkg kku_simulation.launch.py floor:=F1|F2|F3`.

- [x] **Step 2: Implement `teleop.sh`**

Validate the container is running and execute `ros2 run teleop_twist_keyboard teleop_twist_keyboard` inside `/ros2_ws`.

- [x] **Step 3: Implement `save_kku_map.sh`**

Validate floor input, create `/ros2_ws/maps/kku_virtual/f{1,2,3}`, and run `nav2_map_server map_saver_cli` with the standard filename.

- [x] **Step 4: Make scripts executable**

Run: `chmod +x scripts/run_kku_sim.sh scripts/teleop.sh scripts/save_kku_map.sh scripts/test_kku_sim_scripts.sh`

### Task 3: Add Quickstart Document

**Files:**
- Create: `docs/simulation_test/02_gazebo_slam_mapping/03_kku_simulation_quickstart.md`

- [x] **Step 1: Document daily commands**

Show the normal daily flow:

```bash
./scripts/run_kku_sim.sh F1
./scripts/teleop.sh
./scripts/save_kku_map.sh F1
```

- [x] **Step 2: Document first-time-only setup**

List VSCode Dev Containers install, Docker permission, X11 session, and when to use the longer A-to-Z document.

### Task 4: Verify

**Files:**
- Test: `scripts/test_kku_sim_scripts.sh`
- Test: `bash -n` on all helper scripts

- [x] **Step 1: Run contract test**

Run: `bash scripts/test_kku_sim_scripts.sh`

Expected: PASS.

- [x] **Step 2: Run syntax checks**

Run: `bash -n scripts/run_kku_sim.sh scripts/teleop.sh scripts/save_kku_map.sh scripts/test_kku_sim_scripts.sh`

Expected: no output and exit code 0.
