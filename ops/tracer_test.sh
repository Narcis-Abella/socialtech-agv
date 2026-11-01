#!/bin/bash
# Wheels-in-the-air test of the Tracer base over CAN, WITHOUT Nav2: does it obey /cmd_vel, and does it stop by itself when the commands stop?
#   ops/tracer_test.sh                      LIFT_SECS=5 (time to lift the robot)  SPEED=0.1 (m/s)  RUN_SECS=3 (how long the speed is commanded)  WATCH_SECS=10
# Timeline: t=0 the base node starts; LIFT_SECS of countdown (LIFT THE ROBOT: the wheels must be free); SPEED for RUN_SECS; then NOTHING is published for WATCH_SECS
# (a base with its own command timeout stops by itself; if it does not, the zero at the end does); always a final zero, also on Ctrl-C.
# The speed the base itself reports on /odom is printed with the time, to read the exact start and stop. Linear motion only, no rotation.
set -euo pipefail
IMAGE=${IMAGE:-socialtech:robot}
REPO=$(cd "$(dirname "$0")/.." && pwd)
die() { echo "FAIL: $*" >&2; exit 1; }

[[ $(cat /sys/class/net/can_tracer/operstate 2>/dev/null) == up ]] || die "can_tracer is not UP (ip -br link; host/setup_host.sh --check)"
docker image inspect "$IMAGE" >/dev/null 2>&1 || die "image $IMAGE not found"
[[ -z $(docker ps -q) ]] || die "a container is running (docker ps): stop the navigation stack first, nothing else may publish /cmd_vel"
python3 "$REPO/ops/base_state.py" || die "the base is not ready (see above)"

CAN_LOG=$HOME/tracer_test_can.log   # every frame on the bus, as candump -L: what we send (loopback) and what the base answers
candump -L can_tracer >"$CAN_LOG" &
candump_pid=$!
trap 'kill $candump_pid 2>/dev/null' EXIT
echo "TRACER TEST: ${LIFT_SECS:-5}s to lift it, then ${SPEED:-0.1} m/s for ${RUN_SECS:-3}s. Wheels FREE and hands clear. Ctrl-C = stop now."
docker run --rm --init -t --network host --user "$(id -u):$(id -g)" -e HOME=/tmp -e ROS_LOG_DIR=/tmp/roslog \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST -e DDS_IFACE=lo \
  -v "$REPO:$REPO:ro" -e REPO="$REPO" -e LIFT_SECS="${LIFT_SECS:-5}" -e SPEED="${SPEED:-0.1}" -e RUN_SECS="${RUN_SECS:-3}" -e WATCH_SECS="${WATCH_SECS:-10}" "$IMAGE" bash -c '
    set -m
    zero() { timeout 1.5 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1 || true; }
    trap exit INT TERM   # Ctrl-C leaves through the EXIT trap, so the final zero is sent once and always
    trap zero EXIT
    ros2 launch tracer_base tracer_base.launch.py port_name:=can_tracer odom_frame:=wheel_odom base_frame:=wheel_base_link >/tmp/base.log 2>&1 &
    sleep 3
    python3 "$REPO/ops/base_state.py" --require-can || { echo "aborted before any command: no countdown, nothing was sent"; exit 1; }   # the node has asked for CAN mode by now
    ros2 topic echo /odom --field twist.twist.linear.x </dev/null | while read -r v; do [ "$v" = "---" ] || echo "$(date +%T.%3N) base reports vx=$v"; done &
    for i in $(seq "$LIFT_SECS" -1 1); do echo "$(date +%T.%3N) LIFT NOW: $i"; sleep 1; done
    echo "$(date +%T.%3N) COMMAND vx=$SPEED for ${RUN_SECS}s"
    timeout "$RUN_SECS" ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: $SPEED}}" >/dev/null 2>&1 || true
    echo "$(date +%T.%3N) commands STOPPED (no zero sent): watching ${WATCH_SECS}s"
    sleep "$WATCH_SECS"
    echo "$(date +%T.%3N) sending the final zero"
    echo "--- base node log (tail):"; tail -n 12 /tmp/base.log
  '
kill "$candump_pid" 2>/dev/null || true
echo "--- CAN frames per id (hex):"
awk '{split($3, a, "#"); n[a[1]]++} END {for (i in n) print "  " i, n[i]}' "$CAN_LOG" | sort
echo "--- frames we sent (111 = motion command, 421 = control mode), first 6:"
grep -E " can_tracer (111|421)#" "$CAN_LOG" | head -6 || true
echo "--- latency from the bus timestamps (full capture: $CAN_LOG):"
python3 "$REPO/ops/can_latency.py" "$CAN_LOG"
