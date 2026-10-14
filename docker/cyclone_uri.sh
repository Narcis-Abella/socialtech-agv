# CYCLONEDDS_URI for the SocialTech images. Sourced by ros_env.sh; no ROS needed (see test_cyclone_uri.sh).
#
# A Livox scan is ~400 KB. DDS splits it into UDP datagrams, and when the socket's receive buffer is full the
# kernel drops one; the whole scan is lost and nothing warns about it. Measured on an Orin Nano running
# Voxel-SLAM at 1x: 24 % of the scans lost with the kernel's 208 KB default, 0 % with 64 MB.
# min=DDS_RCVBUF_MIN makes Cyclone ask for that buffer and refuse to start (every ROS node, "failed to create domain")
# when the host has not raised net.core.rmem_max (host/setup_host.sh does) instead of losing scans silently.
# The kernel grants 2x min(request, rmem_max), so rmem_max only has to reach half the request.

DDS_RCVBUF_MIN=10485760   # bytes

_rcvbuf_xml() { printf '<Internal><SocketReceiveBufferSize min="%sB"/></Internal>' "$DDS_RCVBUF_MIN"; }

# Prints the URI. DDS_IFACE (optional) pins DDS to that network interface (per robot, e.g. wlP1p1s0 on the Orin NX).
cyclone_uri() {
  local iface=""
  if [[ -n ${DDS_IFACE:-} ]]; then
    iface="<General><Interfaces><NetworkInterface name=\"${DDS_IFACE}\"/></Interfaces></General>"
  fi
  printf '<CycloneDDS><Domain>%s%s</Domain></CycloneDDS>' "$iface" "$(_rcvbuf_xml)"
}

# Exports the URI. A caller's CYCLONEDDS_URI replaces the whole configuration (interface included), but the buffer
# request is the fix for silent scan loss, so it is prepended: Cyclone merges comma-separated fragments and the last
# one wins on a conflict, so a caller that sets its own SocketReceiveBufferSize keeps it (and is left untouched).
set_cyclone_uri() {
  if [[ -z ${CYCLONEDDS_URI:-} ]]; then
    CYCLONEDDS_URI=$(cyclone_uri)
  elif [[ $CYCLONEDDS_URI != *SocketReceiveBufferSize* ]]; then
    CYCLONEDDS_URI="<CycloneDDS><Domain>$(_rcvbuf_xml)</Domain></CycloneDDS>,$CYCLONEDDS_URI"
  fi
  export CYCLONEDDS_URI
}

# Prints a warning (and fails) when the host's rmem_max cannot grant the buffer: Cyclone would then abort every node
# with an error that does not mention rmem_max. RMEM_MAX_FILE is overridable for the test.
rmem_max_warning() {
  local file=${RMEM_MAX_FILE:-/proc/sys/net/core/rmem_max} have
  [[ -r $file ]] || return 0
  have=$(<"$file")
  [[ $have =~ ^[0-9]+$ ]] || return 0
  if (( have * 2 < DDS_RCVBUF_MIN )); then
    echo "warning: host net.core.rmem_max is $have, below the $(( (DDS_RCVBUF_MIN + 1) / 2 )) bytes CycloneDDS needs for its ${DDS_RCVBUF_MIN}-byte receive buffer: every ROS node will fail to start. Run host/setup_host.sh on the host." >&2
    return 1
  fi
}
