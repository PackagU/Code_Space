#!/usr/bin/env bash
# 2026-10-08: the 9/15 host-side GUI attached to the container over the bridge network. The simulation
# container is now LAN-isolated with ROS_LOCALHOST_ONLY=1, so a host RViz/gzclient cannot join it.
# GUI now runs inside the container over the same-uid X11 socket:
#   bash sim/real_maps/start_sim.sh --floor F1 --params P0 --spawn f1_initial_test --gui
echo "show_gui.sh is retired; use: bash sim/real_maps/start_sim.sh ... --gui" >&2
exit 2
