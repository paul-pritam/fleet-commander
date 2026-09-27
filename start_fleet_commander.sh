#!/usr/bin/env bash
set -e

# Source ROS 2 and fleet_commander workspace
source /opt/ros/jazzy/setup.bash
source /home/pritam/bcr_ws/install/setup.bash
source /home/pritam/fleet_commander/install/setup.bash

# Ensure X11/Wayland display and direct AMD GPU hardware acceleration
export DISPLAY=${DISPLAY:-:0}
export WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-wayland-1}
unset __GLX_VENDOR_LIBRARY_NAME
unset LIBVA_DRIVER_NAME

echo "Starting Fleet Commander GUI..."
exec /home/pritam/fleet_commander/install/fleet_commander/lib/fleet_commander/fleet_commander "$@"
