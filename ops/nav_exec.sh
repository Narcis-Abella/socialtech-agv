#!/bin/bash
# Run a ROS 2 command inside the running nav container with the stack's own DDS config (a bare docker exec fails when net.core.rmem_max is low).
#   [PEER=<laptop IP>] ops/nav_exec.sh ros2 param get /controller_server FollowPath.desired_linear_vel
set -eo pipefail
c=$(docker ps -q | head -1)
[[ -n $c ]] || { echo "FAIL: no container is running" >&2; exit 1; }
PEER=${PEER:-}
if [[ -n $PEER ]]; then
  iface=${DDS_IFACE:-$(ip -o route get "$PEER" 2>/dev/null | sed -n 's/.* dev \([^ ]*\).*/\1/p' || true)}
  [[ -n $iface ]] || { echo "FAIL: no route to PEER='$PEER' (it must be the laptop's IP address, not a host name; or set DDS_IFACE)" >&2; exit 1; }
else
  iface=${DDS_IFACE:-lo}
fi
# the same URI as run_live (the stack only listens on lo + iface, unicast): stack_cyclone_uri of docker/cyclone_uri.sh, then set_cyclone_uri adds the receive buffer
exec docker exec -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-53}" -e DDS_IFACE="$iface" -e PEER="$PEER" "$c" bash -c 'source /ros_env.sh; export CYCLONEDDS_URI=$(stack_cyclone_uri "$DDS_IFACE" "$PEER"); set_cyclone_uri; exec "$@"' _ "$@"
