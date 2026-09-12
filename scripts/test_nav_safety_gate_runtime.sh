#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "${ROOT_DIR}/install/setup.bash"
set -u
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-228}" python3 "${ROOT_DIR}/scripts/nav_safety_gate_probe.py"
