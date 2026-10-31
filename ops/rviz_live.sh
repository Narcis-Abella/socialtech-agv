#!/bin/bash
# rviz2 on the laptop (native ROS 2 Jazzy) for ops/nav_live.sh running on a board.   usage: PEER=<board IP> ops/rviz_live.sh
# Reuses the bench's laptop_env.sh (DDS pinned to the interface that reaches PEER, domain 53) and bench_a.rviz, but on wall time.
# NB_LIVE: where tools/nav_bench/live is on this machine (PR #12 checkout).
NB_LIVE=${NB_LIVE:-$HOME/Documents/GitHub/socialtech-agv-nav/tools/nav_bench/live}
source "$NB_LIVE/laptop_env.sh"
export QT_QPA_PLATFORM=xcb   # Wayland session: rviz2 (Ogre) needs XWayland
exec rviz2 -d "${RVIZ_CFG:-$NB_LIVE/bench_a.rviz}" --ros-args -p use_sim_time:=false   # RVIZ_CFG=.../tools/nav_sim/nav_sim.rviz for the Nav2 view (goals, costmaps, plans)
