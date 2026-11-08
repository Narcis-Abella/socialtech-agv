# Sourced by ops/rviz_live.sh. PEER = the board IP (required); IFACE (optional) = this machine interface, by default the one the kernel uses to reach PEER.
: "${PEER:?set PEER to the board IP (its IP on the LAN shared with the laptop)}"
export IFACE=${IFACE:-$(ip -o route get "$PEER" 2>/dev/null | sed -n 's/.* dev \([^ ]*\).*/\1/p' || true)}
[[ -n $IFACE ]] || { echo "FAIL: no route to PEER='$PEER' (it must be the board's IP address, not a host name; or set IFACE)" >&2; return 1 2>/dev/null || exit 1; }   # the interface the kernel uses to reach the board
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-53}
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$IFACE\"/></Interfaces></General><Discovery><Peers><Peer address=\"$PEER\"/></Peers><MaxAutoParticipantIndex>30</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>"
set +u; source /opt/ros/jazzy/setup.bash
