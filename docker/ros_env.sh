# ROS environment for the SocialTech images. Sourced by the entrypoint and by interactive shells.
# shellcheck source=/dev/null  # only exists inside the image
source "/opt/ros/${ROS_DISTRO}/setup.bash" --
# shellcheck source=/dev/null
if [[ -f /opt/socialtech/setup.bash ]]; then source /opt/socialtech/setup.bash --; fi

# DDS configuration (interface pin and receive buffer): see cyclone_uri.sh. An explicit CYCLONEDDS_URI always wins.
# Unset DDS_IFACE, CycloneDDS picks an interface "arbitrarily" among wlan/usb/tailscale/docker, which may not
# be the one the laptop is on.
# shellcheck source=cyclone_uri.sh
source /cyclone_uri.sh
set_cyclone_uri
