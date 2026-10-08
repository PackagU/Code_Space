#!/usr/bin/env bash
# Native GUI observes the dedicated container; no xhost/auth-file changes.
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SIM_CONTAINER="${1:-packagu_real_map_sim}"
SIM_DOMAIN="${2:-215}"
[[ "$SIM_CONTAINER" =~ ^packagu_real_map[A-Za-z0-9_.-]*$ ]] || exit 2
[[ "$SIM_DOMAIN" =~ ^[0-9]+$ ]] || exit 2
SIM_ADDRESS=""
for ((attempt=0; attempt<60; attempt++)); do
  SIM_ADDRESS="$(docker inspect --format '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' "$SIM_CONTAINER" 2>/dev/null || true)"
  [[ -n "$SIM_ADDRESS" ]] && break
  sleep .5
done
[[ "$SIM_ADDRESS" =~ ^[0-9.]+$ ]] || { echo 'simulation container unavailable'; exit 1; }
mkdir -p "$ROOT_DIR/logs/real_map_sim/gui"
SIM_DDS_FILE="$ROOT_DIR/logs/real_map_sim/gui/native_dds.xml"
python3 - "$SIM_ADDRESS" "$SIM_DDS_FILE" <<'PY'
import sys
from pathlib import Path
address, output = sys.argv[1:]
Path(output).write_text('''<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
<transport_descriptors><transport_descriptor><transport_id>gui_udp</transport_id><type>UDPv4</type></transport_descriptor></transport_descriptors>
<participant profile_name="gui" is_default_profile="true"><rtps>
<userTransports><transport_id>gui_udp</transport_id></userTransports><useBuiltinTransports>false</useBuiltinTransports>
<initialPeersList><locator><udpv4><address>'''+address+'''</address></udpv4></locator></initialPeersList>
</rtps></participant></profiles>''')
PY
set +u
source /opt/ros/humble/setup.bash
set -u
export LIBGL_ALWAYS_SOFTWARE=1
export GAZEBO_MASTER_URI="http://$SIM_ADDRESS:11345"
export GAZEBO_MODEL_DATABASE_URI=""
export ROS_DOMAIN_ID="$SIM_DOMAIN"
export FASTRTPS_DEFAULT_PROFILES_FILE="$SIM_DDS_FILE"
export ROS_LOG_DIR="$ROOT_DIR/logs/real_map_sim/gui/ros"
export GAZEBO_LOG_PATH="$ROOT_DIR/logs/real_map_sim/gui/gazebo"
if [[ -f "$ROOT_DIR/logs/real_map_sim/gui/gzclient.pid" ]]; then
  python3 - "$ROOT_DIR/logs/real_map_sim/gui/gzclient.pid" <<'PY2'
import os,signal,sys
from pathlib import Path
try:
 pid = int(Path(sys.argv[1]).read_text())
 words = Path('/proc')/str(pid)/'cmdline'
 if words.read_bytes().split(b'\0')[0].endswith(b'gzclient'):
  os.kill(pid,signal.SIGINT)
except (OSError,ValueError): pass
PY2
fi
# Only replace this workspace's previous RViz, if it exists.
python3 - "$ROOT_DIR/sim/real_maps/nav2_view.rviz" <<'PY'
import os,sys,signal
from pathlib import Path
for path in Path('/proc').glob('[0-9]*/cmdline'):
 try: words=path.read_bytes().split(b'\0')
 except OSError: continue
 if words and words[0].endswith(b'rviz2') and sys.argv[1].encode() in words:
  os.kill(int(path.parent.name), signal.SIGINT)
PY
nohup gzclient --verbose > "$ROOT_DIR/logs/real_map_sim/gui/gzclient.log" 2>&1 < /dev/null &
printf '%s\n' "$!" > "$ROOT_DIR/logs/real_map_sim/gui/gzclient.pid"
nohup rviz2 -d "$ROOT_DIR/sim/real_maps/nav2_view.rviz" --ros-args -p use_sim_time:=true \
  > "$ROOT_DIR/logs/real_map_sim/gui/rviz.log" 2>&1 < /dev/null &
printf '%s\n' "$!" > "$ROOT_DIR/logs/real_map_sim/gui/rviz.pid"
echo "Gazebo + RViz launched: $SIM_CONTAINER domain $SIM_DOMAIN"
# Keep the launcher alive in terminal/API sessions that reap detached children.
wait
