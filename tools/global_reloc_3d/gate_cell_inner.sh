#!/usr/bin/env bash
# Inside socialtech:gate-can; one of the three roles of gate_cell_test.sh.
set -eo pipefail
source /ros_env.sh
set -u
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-143} ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
T=/tools/global_reloc_3d; OUT=/out
ev() { echo "$1 $(date +%s.%N)" >> $OUT/events.txt; }
case $1 in
cell)
  ip link set lo up || true
  ip link add dev vcan0 type vcan && ip link set up vcan0
  ip -br link > $OUT/cell_links.txt
  if ip -br link | grep -q '^can0'; then echo "physical can0 visible in the cell: aborting" | tee $OUT/ABORTED.txt; exit 3; fi
  candump -t a vcan0 > $OUT/candump_all.log & 
  ros2 run tracer_base tracer_base_node --ros-args -p port_name:=vcan0 > $OUT/tracer_base.log 2>&1 & base=$!
  python3 -B $T/gate_outlet.py /sock/gate.sock $OUT/outlet_stats.json > $OUT/outlet.log 2>&1 & outlet=$!
  python3 -B $T/can_deadman.py vcan0 $OUT/deadman.json > $OUT/deadman.log 2>&1 & dead=$!
  sleep 4; { ros2 node list; ros2 topic info -v /cmd_vel; } > $OUT/cell_graph.txt 2>&1 || true
  while [ ! -e $OUT/ctl/stop ]; do
    if [ -e $OUT/ctl/kill_outlet ]; then rm $OUT/ctl/kill_outlet; kill -KILL $outlet; ev outlet_killed; sleep 4
      python3 -B $T/gate_outlet.py /sock/gate.sock $OUT/outlet_stats2.json > $OUT/outlet2.log 2>&1 & outlet=$!; ev outlet_restarted; fi
    if [ -e $OUT/ctl/kill_driver ]; then rm $OUT/ctl/kill_driver; kill -KILL $(pgrep -x tracer_base_nod | head -1) $base 2>/dev/null || true; ev driver_killed; fi
    sleep .05
  done
  kill -TERM $outlet $dead 2>/dev/null || true; sleep 1;;
gate)
  ip link set lo up 2>/dev/null || true
  python3 -B $T/nav_gate.py --ros-args -p outlet_socket:=/sock/gate.sock > $OUT/nav_gate.log 2>&1 & gate=$!
  while [ ! -e $OUT/ctl/stop ]; do
    if [ -e $OUT/ctl/kill_gate ]; then rm $OUT/ctl/kill_gate; kill -KILL $gate; ev gate_killed; fi
    sleep .05
  done;;
graph)
  ls -la /sock > $OUT/graph_sees_sock.txt 2>&1 || true
  python3 -B $T/gate_probe.py $OUT/gate_probe.json > $OUT/gate_probe.log 2>&1 & probe=$!
  sleep 4; { ros2 node list; ros2 topic list; } > $OUT/main_graph.txt 2>&1 || true
  # 1. closed, open, revoked, open, silent, open, permit publisher exits
  python3 -B $T/permit_stub.py $OUT/permit_phases.json false:3 true:5 false:3 true:5 silent:3 true:5 > $OUT/permit_stub.log 2>&1
  sleep 3
  # 2. bypass attempts with the gate closed: the old gated topic, the driver's own topic name, and the socket itself
  ev bypass_start
  ros2 topic pub -t 20 -r 10 /cmd_vel_gated geometry_msgs/msg/Twist '{linear: {x: 0.2}}' > $OUT/bypass_gated.log 2>&1 || true
  kill -STOP $probe      # the probe owns /cmd_vel in this graph; while it is stopped a rogue publishes there with the gate closed
  ros2 topic pub -t 20 -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.3}}' > $OUT/bypass_cmd_vel.log 2>&1 || true
  kill -CONT $probe
  python3 -c "
import socket,struct,time
s=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM)
try:
    s.sendto(struct.pack('<QdddI',10**9,time.monotonic(),0.4,0.,0x47415445),'/sock/gate.sock'); print('SENT')
except OSError as e: print('REFUSED',e)" > $OUT/bypass_socket.log 2>&1
  ev bypass_end
  # 3. faults while OPEN (permit true throughout)
  python3 -B $T/permit_stub.py $OUT/permit_phases2.json true:40 > $OUT/permit_stub2.log 2>&1 & stub2=$!
  sleep 5; ev source_stopped; kill -STOP $probe; sleep 3; kill -CONT $probe; ev source_resumed
  sleep 4; touch $OUT/ctl/kill_outlet; sleep 8
  sleep 2; touch $OUT/ctl/kill_${FINAL:-gate}; sleep 5      # last fault: the gate (default) or the driver dies while open
  ev end_final
  kill -TERM $probe; wait $probe 2>/dev/null || true; kill $stub2 2>/dev/null || true;;
esac
