#!/bin/bash
# Run a ROS 2 command inside the running nav container with the stack's own DDS config (a bare docker exec fails when net.core.rmem_max is low).
#   PEER=<laptop IP> ops/nav_exec.sh ros2 param get /controller_server FollowPath.desired_linear_vel
set -eo pipefail
: "${PEER:?set PEER to the laptop IP, as for nav_live.sh}"
c=$(docker ps -q | head -1)
[[ -n $c ]] || { echo "FAIL: no container is running" >&2; exit 1; }
iface=${DDS_IFACE:-$(ip -o route get "$PEER" | sed -n 's/.* dev \([^ ]*\).*/\1/p')}
ifs="<NetworkInterface name=\"lo\"/>"; [[ $iface == lo ]] || ifs+="<NetworkInterface name=\"$iface\"/>"   # same URI as nav_live.sh: the stack only listens on lo + iface, unicast
uri="<CycloneDDS><Domain><General><Interfaces>$ifs</Interfaces></General><Discovery><Peers><Peer address=\"$PEER\"/><Peer address=\"localhost\"/></Peers><MaxAutoParticipantIndex>30</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>"
exec docker exec -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-53}" -e CYCLONEDDS_URI="$uri" "$c" bash -c 'source /ros_env.sh; exec "$@"' _ "$@"
