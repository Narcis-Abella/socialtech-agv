# CYCLONEDDS_URI for the SocialTech images. Sourced by ros_env.sh; no ROS needed (see test_cyclone_uri.sh).
#
# A Livox scan is ~400 KB. DDS splits it into UDP datagrams, and when the socket's receive buffer is full the
# kernel drops one; the whole scan is lost and nothing warns about it. Measured on an Orin Nano running
# Voxel-SLAM at 1x: 24 % of the scans lost with the kernel's 208 KB default, 0 % with 64 MB.
# min="10MB" makes Cyclone ask for that buffer and refuse to start, with a clear error, when the host has not
# raised net.core.rmem_max (host/setup_host.sh does) instead of losing scans silently.

# Prints the URI. DDS_IFACE (optional) pins DDS to that network interface (per robot, e.g. wlP1p1s0 on the Orin NX).
cyclone_uri() {
  local iface=""
  if [[ -n ${DDS_IFACE:-} ]]; then
    iface="<General><Interfaces><NetworkInterface name=\"${DDS_IFACE}\"/></Interfaces></General>"
  fi
  printf '<CycloneDDS><Domain>%s<Internal><SocketReceiveBufferSize min="10MB"/></Internal></Domain></CycloneDDS>' "$iface"
}

# Exports the URI unless the caller already set one: an explicit CYCLONEDDS_URI always wins.
set_cyclone_uri() {
  if [[ -z ${CYCLONEDDS_URI:-} ]]; then
    CYCLONEDDS_URI=$(cyclone_uri)
    export CYCLONEDDS_URI
  fi
}
