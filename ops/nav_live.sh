#!/bin/bash
# Live localization on the robot over a map made by ops/make_map.sh: Livox driver + FAST-LIO2 + AMCL. Move the robot by hand; watch it in rviz2 on the laptop (ops/rviz_live.sh).
#   PEER=<laptop IP> ops/nav_live.sh MAP_NAME      MAP_NAME = folder in ~/maps; needs a tty (ssh -t): Ctrl-C stops
# Keep the robot still while FAST-LIO2 initialises (~5 s); then click "2D Pose Estimate" in rviz2 at the robot's real pose and drag to its heading.
# The odometry is levelled from the map's own Voxel-SLAM start pose (same IMU mounting, robot on the same floor); the yaw is whatever you click.
# env: DDS_IFACE (interface towards PEER)  ROS_DOMAIN_ID (53, as the laptop scripts)  LIVOX_HOST_IP / LIVOX_LIDAR_IP  IMAGE (socialtech:robot)  MAP_DIR (~/maps)
set -eo pipefail
REPO=$(cd "$(dirname "$0")/.." && pwd)
NB=$REPO/tools/nav_bench

if [[ ${1:-} == --inside ]]; then   # runs in the container
  source /ros_env.sh
  export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-53}
  export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$DDS_IFACE\"/></Interfaces></General><Discovery><Peers><Peer address=\"$PEER\"/></Peers><MaxAutoParticipantIndex>30</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>"
  h=$(python3 -c "import json; print(json.load(open('/map/report.json'))['sensor_height_m'])")
  read -r qx qy qz qw < <(python3 "$NB/pose_eval.py" level --ref /map/vs/run/alidarState.txt --map /map/map.yaml)
  "$REPO/ops/livox_config.sh" >/tmp/MID360_config.json
  sed -e "s|pcd_save_en:.*|pcd_save_en: false|" -e "s|map_en:.*|map_en: false|" -e "s|path_en:.*|path_en: false|" \
      -e "s|^\( *\)gyr_cov:.*|\1gyr_cov: 1.0|" -e "s|^\( *\)acc_cov:.*|\1acc_cov: 1.0|" \
      "$(ros2 pkg prefix fast_lio)/share/fast_lio/config/mid360.yaml" >/tmp/fastlio.yaml   # gyr_cov/acc_cov 1.0: as the bench (upstream 0.1 degrades in featureless corridors)
  echo "sensor height $h m; level quaternion $qx $qy $qz $qw; DDS $DDS_IFACE -> $PEER (domain $ROS_DOMAIN_ID)"
  exec ros2 launch "$REPO/ops/nav_live.launch.py" livox_config:=/tmp/MID360_config.json fastlio_params:=/tmp/fastlio.yaml map:=/map/map.yaml \
    overrides:="$REPO/ops/live_overrides.yaml" sensor_height:="$h" level_qx:="$qx" level_qy:="$qy" level_qz:="$qz" level_qw:="$qw"
fi

MAP_NAME=${1:?usage: PEER=<laptop IP> nav_live.sh MAP_NAME}
MAP=${MAP_DIR:-$HOME/maps}/$MAP_NAME
IMAGE=${IMAGE:-socialtech:robot}
HOST_IP=${LIVOX_HOST_IP:-192.168.1.50}
LIDAR_IP=${LIVOX_LIDAR_IP:-192.168.1.126}
RMEM_WANT=67108864
die() { echo "FAIL: $*" >&2; exit 1; }

: "${PEER:?set PEER to the laptop IP on the shared network (laptop: ip -br addr)}"
DDS_IFACE=${DDS_IFACE:-$(ip -o route get "$PEER" | sed -n 's/.* dev \([^ ]*\).*/\1/p')}
[[ -n $DDS_IFACE ]] || die "no route to $PEER"
for f in map.yaml report.json vs/run/alidarState.txt; do [[ -f $MAP/$f ]] || die "$MAP/$f missing: is $MAP_NAME a ops/make_map.sh output?"; done
docker image inspect "$IMAGE" >/dev/null 2>&1 || die "image $IMAGE not found"
docker run --rm "$IMAGE" ros2 pkg prefix planar_odom >/dev/null 2>&1 || die "$IMAGE has no planar_odom/nav2: rebuild it from this branch (ops/build_image.sh robot)"
[[ -z $(docker ps -q) ]] || die "a container is running (docker ps): one heavy job at a time"
ip -4 -o addr show | grep -q " $HOST_IP/" || die "no interface has $HOST_IP (the Livox link): check the ethernet cable"
ping -c1 -W2 "$LIDAR_IP" >/dev/null 2>&1 || die "Livox $LIDAR_IP does not answer: power or ethernet cable"
ping -c1 -W2 "$PEER" >/dev/null 2>&1 || echo "WARN: $PEER does not answer ping (rviz2 may not see the robot)" >&2
(($(sysctl -n net.core.rmem_max) >= RMEM_WANT)) || echo "WARN: net.core.rmem_max below $RMEM_WANT: scans may be lost (sudo sysctl -w net.core.rmem_max=$RMEM_WANT)" >&2

echo "localizing on $MAP (Ctrl-C to stop). Keep the robot still ~5 s, then 2D Pose Estimate in rviz2."
exec docker run --rm --init -t --network host --user "$(id -u):$(id -g)" -e HOME=/tmp -e ROS_LOG_DIR=/tmp/roslog \
  -e DDS_IFACE="$DDS_IFACE" -e PEER="$PEER" -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-53}" -e LIVOX_HOST_IP="$HOST_IP" -e LIVOX_LIDAR_IP="$LIDAR_IP" \
  -v "$MAP:/map:ro" -v "$REPO:$REPO:ro" -w /tmp "$IMAGE" "$REPO/ops/nav_live.sh" --inside
