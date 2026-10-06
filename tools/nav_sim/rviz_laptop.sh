#!/bin/bash
# rviz2 for the Nav2 simulation, on the laptop (native ROS 2 Jazzy).   usage: PEER=<AGX IP> rviz_laptop.sh
here=$(dirname "$(readlink -f "$0")")
source "$here/../nav_bench/live/laptop_env.sh"
export QT_QPA_PLATFORM=xcb   # Wayland session: rviz2 (Ogre) needs XWayland
exec rviz2 -d "$here/nav_sim.rviz" --ros-args -p use_sim_time:=true
