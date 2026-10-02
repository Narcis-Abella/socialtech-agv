#!/bin/bash
# rviz2 for the live view, on the laptop (native ROS 2 Jazzy: see install_ros_jazzy.sh).   usage: PEER=<board IP> rviz_laptop.sh
here=$(dirname "$(readlink -f "$0")")
source "$here/laptop_env.sh"
export QT_QPA_PLATFORM=xcb   # Wayland session: rviz2 (Ogre) needs XWayland
exec rviz2 -d "$here/bench_a.rviz" --ros-args -p use_sim_time:=true
