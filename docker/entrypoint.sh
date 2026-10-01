#!/bin/bash
# Load the ROS environment, then run the command.
set -e
user_uri=${CYCLONEDDS_URI:-}   # ros_env.sh always exports one; the warning is about what the caller chose
# shellcheck source-path=SCRIPTDIR source=ros_env.sh
source /ros_env.sh
if [[ -z ${DDS_IFACE:-} && -z $user_uri ]]; then
  echo "warning: DDS_IFACE not set; CycloneDDS will pick a network interface arbitrarily" >&2
fi
exec "$@"
