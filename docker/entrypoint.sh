#!/bin/bash
# Load the ROS environment, then run the command.
set -e
# shellcheck source-path=SCRIPTDIR source=ros_env.sh
source /ros_env.sh
if [[ -z ${DDS_IFACE:-} && -z ${CYCLONEDDS_URI:-} ]]; then
  echo "warning: DDS_IFACE not set; CycloneDDS will pick a network interface arbitrarily" >&2
fi
exec "$@"
