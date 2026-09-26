# ROS environment for the SocialTech images. Sourced by the entrypoint and by interactive shells.
# shellcheck source=/dev/null  # only exists inside the image
source "/opt/ros/${ROS_DISTRO}/setup.bash" --
# shellcheck source=/dev/null
if [[ -f /opt/socialtech/setup.bash ]]; then source /opt/socialtech/setup.bash --; fi

# Pin DDS to the robot's network interface (per robot, e.g. DDS_IFACE=wlP1p1s0 on the Orin NX).
# Unset, CycloneDDS picks one interface "arbitrarily" among wlan/usb/tailscale/docker, which may not
# be the one the laptop is on. An explicit CYCLONEDDS_URI always wins.
if [[ -n ${DDS_IFACE:-} && -z ${CYCLONEDDS_URI:-} ]]; then
  export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${DDS_IFACE}\"/></Interfaces></General></Domain></CycloneDDS>"
fi
