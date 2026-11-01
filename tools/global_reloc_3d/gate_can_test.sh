#!/usr/bin/env bash
# Gate against the REAL base driver, no robot: tracer_base (unmodified, from the image) talks to a virtual CAN bus that exists only
# inside this container's own network namespace. Shows what the driver would put on the wire with the gate closed, open, revoked
# and with the permit publisher dead. Host needs the vcan kernel module (sudo modprobe vcan); nothing else on the host is touched.
#   docker run --rm --cap-add NET_ADMIN -v <tools>:/tools:ro -v <out>:/out socialtech:gate-can bash /tools/global_reloc_3d/gate_can_test.sh
set -eo pipefail
source /ros_env.sh
set -u
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-142} ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
T=/tools/global_reloc_3d; OUT=/out
# The bus is called vcan0 and lives in this container's private network namespace; the host's physical can0 is not visible here.
ip link add dev vcan0 type vcan && ip link set up vcan0
ip -br link > $OUT/container_links.txt
if ip -br link | grep -q '^can0'; then echo "physical can0 visible in the test container: aborting" | tee $OUT/ABORTED.txt; exit 3; fi
candump -t a vcan0 > $OUT/candump_all.log & dumpall=$!
candump -t a vcan0,111:7FF > $OUT/candump.log & dump=$!
ros2 run tracer_base tracer_base_node --ros-args -p port_name:=vcan0 -r /cmd_vel:=/cmd_vel_gated > $OUT/tracer_base.log 2>&1 & base=$!
python3 -B $T/nav_gate.py > $OUT/nav_gate.log 2>&1 & gate=$!
python3 -B $T/gate_probe.py $OUT/gate_probe.json > $OUT/gate_probe.log 2>&1 & probe=$!
sleep 4
{ ros2 topic info -v /cmd_vel_gated; ros2 topic info -v /cmd_vel; ros2 node info /tracer; } > $OUT/topic_info.txt 2>&1 || true
# closed 3 s, open 5 s, revoked 3 s, open 5 s, alive but silent 3 s, open 5 s, then the publisher dies while true
python3 -B $T/permit_stub.py $OUT/permit_phases.json false:3 true:5 false:3 true:5 silent:3 true:5 > $OUT/permit_stub.log 2>&1
sleep 3
# the unsafe path, shown on purpose: a command published straight to the base topic never passes the gate
date +%s.%N > $OUT/bypass_epoch.txt
ros2 topic pub -t 5 -r 10 /cmd_vel_gated geometry_msgs/msg/Twist '{linear: {x: 0.2}}' > $OUT/bypass.log 2>&1 || true
sleep 2
# the gate itself dies while open: nothing zeroes the command any more. What stays on the wire is the driver's and the base's business.
python3 -B $T/permit_stub.py $OUT/permit_phases2.json true:8 > $OUT/permit_stub2.log 2>&1 & stub2=$!
sleep 5; date +%s.%N > $OUT/gate_kill_epoch.txt; kill -KILL $gate
sleep 5; date +%s.%N > $OUT/end_epoch.txt
kill -TERM $probe; wait $probe $stub2 2>/dev/null || true
kill -TERM $base $dump $dumpall 2>/dev/null || true; sleep 1; kill -KILL $base 2>/dev/null || true
python3 -B $T/gate_can_check.py $OUT | tee $OUT/result.json
