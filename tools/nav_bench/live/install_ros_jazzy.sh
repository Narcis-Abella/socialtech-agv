#!/bin/bash
# Installs ROS 2 Jazzy (desktop: rviz2, rqt) + CycloneDDS + the MCAP bag plugin on Ubuntu 24.04, from the official apt repository. Run as root. Idempotent.
# Commands from the official instructions (docs.ros.org, Jazzy, "Ubuntu (deb packages)"; read from ros2/ros2_documentation, branch jazzy).
set -e
[ "$(id -u)" = 0 ] || { echo "run with sudo"; exit 1; }
[ "$(. /etc/os-release && echo "$VERSION_CODENAME")" = noble ] || { echo "this is for Ubuntu 24.04 (noble)"; exit 1; }
grep -q -E "noble-updates" /etc/apt/sources.list.d/ubuntu.sources /etc/apt/sources.list 2>/dev/null || echo "WARNING: noble-updates not found in the apt sources: ros-dev-tools may conflict (see the ROS install notes)"
apt-get install -y software-properties-common curl
add-apt-repository -y universe
if ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
  ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
  curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo "${UBUNTU_CODENAME:-${VERSION_CODENAME}}")_all.deb"
  dpkg -i /tmp/ros2-apt-source.deb
fi
# an unrelated broken mirror (e.g. in nala-sources.list) makes apt-get update exit non-zero although the ROS index is fetched: tolerate it, but require the ROS packages to be installable
apt-get update || echo "WARNING: apt-get update reported errors (see above); continuing only if the ROS packages have an install candidate"
for p in ros-jazzy-desktop ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-rosbag2-storage-mcap; do
  apt-cache policy "$p" | grep -q "Candidate: [0-9]" || { echo "ERROR: $p has no install candidate"; exit 1; }
done
DEBIAN_FRONTEND=noninteractive apt-get install -y ros-jazzy-desktop ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-rosbag2-storage-mcap
echo "installed: $(dpkg -s ros-jazzy-desktop | grep Version)"; . /opt/ros/jazzy/setup.sh && ros2 --help | head -1
