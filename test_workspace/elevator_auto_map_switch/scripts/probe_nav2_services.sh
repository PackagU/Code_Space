#!/usr/bin/env bash
# Probe the actual Nav2 service names before flipping dry_run_map_load:=false.
# Run inside the container while kku_navigation.launch.py is up.
set -u

echo "=== map load service candidates ==="
ros2 service list 2>/dev/null | grep -i "load_map" || echo "(none found — is Nav2 running?)"

echo
echo "=== costmap clear service candidates ==="
ros2 service list 2>/dev/null | grep -i "clear" || echo "(none found)"

echo
echo "=== expected types ==="
for srv in /map_server/load_map \
           /global_costmap/clear_entirely_global_costmap \
           /local_costmap/clear_entirely_local_costmap; do
  printf "%-50s " "$srv"
  ros2 service type "$srv" 2>/dev/null || echo "NOT FOUND"
done

echo
echo "If names differ, pass them as parameters:"
echo "  -p load_map_service:=/your/load_map"
echo "  -p costmap_clear_services:=\"['/your/global_clear','/your/local_clear']\""
