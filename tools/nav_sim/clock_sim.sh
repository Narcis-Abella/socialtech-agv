#!/bin/bash
# What does the simulated clock cost? Gazebo publishes /clock at every physics step; every use_sim_time node pays for it. Two idle rclpy nodes (idle_node.py sim / wall) in the running simulator:
# their CPU difference is the overhead each sim-time node of the stack carries on the robot-less side. usage: clock_sim.sh <out dir> [seconds=60]
# mounts: repo tools/ at /tools, out at /out, as lio_sim.sh (SIM_SRC, /pat, /navsrc); no stack runs, only the simulator
# env: ROS_DOMAIN_ID (55), SPAWN, SEED, SIM_WAIT
set -eo pipefail
out=${1:?out dir}; secs=${2:-60}
source "${ROS_ENV:-/ros_env.sh}"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-55}
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces></General><Discovery><Peers><Peer address="localhost"/></Peers><MaxAutoParticipantIndex>30</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
set_cyclone_uri   # this export replaced the URI of ros_env.sh: ask again for the 10 MB receive buffer (docker/cyclone_uri.sh)
source /tools/nav_sim/sim_up.sh
sleep 20
timeout 15 ros2 topic hz /clock --window 100 2>&1 | tail -3 | tee "$out/clock_hz.txt" || true
python3 /tools/nav_sim/idle_node.py sim & ps=$!
python3 /tools/nav_sim/idle_node.py wall & pw=$!
sleep 5
ticks() { awk '{print $14 + $15}' /proc/$1/stat; }
s0=$(ticks $ps); w0=$(ticks $pw); sleep "$secs"; s1=$(ticks $ps); w1=$(ticks $pw)
kill $ps $pw 2> /dev/null || true
python3 -c "import os; t=os.sysconf('SC_CLK_TCK'); print('idle node CPU over $secs s (%% of one core): use_sim_time=true %.2f   use_sim_time=false %.2f' % (($s1-$s0)/t/$secs*100, ($w1-$w0)/t/$secs*100))" | tee "$out/clock_cost.txt"
