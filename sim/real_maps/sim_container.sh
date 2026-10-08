#!/usr/bin/env bash
# Isolated simulation container (shared by start_sim.sh, run_e2e.py, run_pretest.py, run_suite.sh).
#
# Isolation is mandatory because the field Jetson is on the same LAN: simulated /cmd_vel must never
# reach the real robot. The container is refused unless ALL of these hold:
#   - Docker network is --internal (no route to the LAN or internet)
#   - ROS_LOCALHOST_ONLY=1 and ROS_DOMAIN_ID is a non-zero number (default 77)
#   - not host network, not privileged, no devices passed through
#   - no FastDDS LAN peer profile (scripts/fastdds_lan_peers.xml) in use
# Usage: sim_container.sh up [--gui] | verify | down | exec <cmd...> | name
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
IMAGE="${PACKAGU_SIM_IMAGE:-ghcr.io/packagu/ros2-humble-slam:humble}"
NAME="${PACKAGU_SIM_CONTAINER:-packagu_sim_e2e}"
NET="${PACKAGU_SIM_NETWORK:-packagu_sim_isolated}"
DOMAIN="${PACKAGU_SIM_DOMAIN:-77}"

die() { echo "sim_container: $*" >&2; exit 1; }
# Never touch the field/dev container (ros2_humble) or anything not created here.
[[ "$NAME" =~ ^packagu_sim[A-Za-z0-9_.-]*$ ]] || die "container name must start with packagu_sim: $NAME"
[[ "$NET" =~ ^packagu_sim[A-Za-z0-9_.-]*$ ]] || die "network name must start with packagu_sim: $NET"
[[ "$DOMAIN" =~ ^[0-9]+$ ]] && (( DOMAIN >= 1 && DOMAIN <= 232 )) || die "ROS_DOMAIN_ID must be 1..232 (not 0): $DOMAIN"

verify() {
  local mode internal env devices privileged
  docker inspect "$NAME" >/dev/null 2>&1 || die "container $NAME not running"
  mode="$(docker inspect -f '{{.HostConfig.NetworkMode}}' "$NAME")"
  [[ "$mode" == "$NET" ]] || die "REFUSED: network mode is '$mode', expected isolated '$NET'"
  internal="$(docker network inspect -f '{{.Internal}}' "$NET")"
  [[ "$internal" == true ]] || die "REFUSED: network $NET is not --internal (LAN reachable)"
  env="$(docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' "$NAME" | grep -E '^(ROS_|RMW_|FASTRTPS_|CYCLONEDDS_)' || true)"
  grep -Fxq 'ROS_LOCALHOST_ONLY=1' <<<"$env" || die "REFUSED: ROS_LOCALHOST_ONLY=1 missing"
  grep -Eq '^ROS_DOMAIN_ID=([1-9][0-9]*)$' <<<"$env" || die "REFUSED: non-zero ROS_DOMAIN_ID missing"
  ! grep -q 'fastdds_lan_peers' <<<"$env" || die "REFUSED: LAN peer DDS profile configured"
  devices="$(docker inspect -f '{{len .HostConfig.Devices}}' "$NAME")"
  [[ "$devices" == 0 ]] || die "REFUSED: $devices host devices passed through"
  privileged="$(docker inspect -f '{{.HostConfig.Privileged}}' "$NAME")"
  [[ "$privileged" == false ]] || die "REFUSED: privileged container"
  echo "ISOLATION OK: container=$NAME network=$NET(internal) $(grep -E '^ROS_(DOMAIN_ID|LOCALHOST_ONLY)=' <<<"$env" | tr '\n' ' ')"
}

up() {
  local gui=0
  [[ "${1:-}" == --gui ]] && gui=1
  if ! docker network inspect "$NET" >/dev/null 2>&1; then
    docker network create --internal "$NET" >/dev/null
  fi
  if docker inspect "$NAME" >/dev/null 2>&1; then
    [[ "$(docker inspect -f '{{.State.Running}}' "$NAME")" == true ]] || docker rm -f "$NAME" >/dev/null
  fi
  if ! docker inspect "$NAME" >/dev/null 2>&1; then
    docker image inspect "$IMAGE" >/dev/null 2>&1 || die "image $IMAGE not present; pull is a manual step (not done here)"
    local extra=()
    if (( gui )); then
      # Same-uid X11 socket only; no xhost or auth-file changes.
      extra+=(-e "DISPLAY=${DISPLAY:-:1}" -v /tmp/.X11-unix:/tmp/.X11-unix:ro -e LIBGL_ALWAYS_SOFTWARE=1)
    fi
    mkdir -p "$ROOT_DIR/logs/real_map_sim"
    docker run -d --init --name "$NAME" --network "$NET" --shm-size 1g \
      --user "$(id -u):$(id -g)" -e HOME=/tmp/simhome -e USER=sim \
      -e ROS_DOMAIN_ID="$DOMAIN" -e ROS_LOCALHOST_ONLY=1 -e PACKAGU_SIM_ISOLATED=1 \
      -e GAZEBO_MODEL_DATABASE_URI= -e PYTHONDONTWRITEBYTECODE=1 \
      -e ROS_LOG_DIR="/ros2_ws/logs/real_map_sim/ros_logs/$NAME" \
      -e GAZEBO_LOG_PATH="/ros2_ws/logs/real_map_sim/gazebo_logs/$NAME" \
      "${extra[@]}" \
      -v "$ROOT_DIR:/ros2_ws" \
      -v "$ROOT_DIR/src/slam_pkg/maps:/ros2_ws/maps:ro" \
      -v "$ROOT_DIR/logs:/ros2_ws/logs" \
      -w /ros2_ws --entrypoint sleep "$IMAGE" infinity >/dev/null
    docker exec "$NAME" mkdir -p /tmp/simhome
  fi
  verify
}

case "${1:-}" in
  up) shift; up "$@" ;;
  verify) verify ;;
  down)
    if docker inspect "$NAME" >/dev/null 2>&1; then docker rm -f "$NAME" >/dev/null; fi
    echo "removed $NAME (all simulation processes stopped)" ;;
  exec)
    shift; verify >/dev/null
    cmd_env=()
    [[ -n "${PACKAGU_SIM_CMD:-}" ]] && cmd_env=(-e PACKAGU_SIM_CMD)
    exec docker exec -i "${cmd_env[@]}" -w /ros2_ws "$NAME" bash -c \
      'source /opt/ros/humble/setup.bash && { [[ -f install/setup.bash ]] && source install/setup.bash; true; } && exec "$@"' bash "$@" ;;
  name) echo "$NAME" ;;
  *) echo "usage: $0 up [--gui] | verify | down | exec <cmd...> | name" >&2; exit 2 ;;
esac
