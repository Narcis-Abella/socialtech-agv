#!/bin/bash
# Source ROS 2 and the SocialTech overlay, then run the command.
set -e
source "/opt/ros/${ROS_DISTRO}/setup.bash" --
if [[ -f /opt/socialtech/setup.bash ]]; then source /opt/socialtech/setup.bash --; fi
exec "$@"
