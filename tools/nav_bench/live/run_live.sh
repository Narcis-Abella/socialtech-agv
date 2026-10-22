#!/bin/bash
# Live view of FAST-LIO2 + AMCL on a bag, watched from rviz2 on another machine (rviz_laptop.sh). Replays the bag PAUSED and gives AMCL NO initial pose:
# click "2D Pose Estimate" in rviz2 (the realistic path, default covariance), then resume with control.sh resume. Run inside the robot-nav image on the board.
# usage: run_live.sh <bag_dir> <map.yaml> <alidarState.txt> <sensor_height_m>
#   bag over ANOTHER bag's map (stage B): give the bag's own alidarState.txt (only used to level the odometry) and set NO_REF=1 (no reference path drawn)
# DDS: interface pinned (the 'arbitrary interface' trap), multicast left on for same-host discovery, PEER as unicast hint, 30 participants (default 10: a dozen ROS processes run here)
# env:   SCAN_PARAMS (pointcloud_to_laserscan YAML, default config/scan.yaml; config/scan_outdoor.yaml opens the upper height bound)  OVERLAY (colcon ws with planar_odom / a FAST-LIO build)  OUT (where fastlio.yaml, logs and the recorded amcl.tum/odom.tum go; default /tmp/live)  DDS_IFACE (the board's LAN interface, required)  PEER (the laptop's IP on that LAN, required)  ROS_DOMAIN_ID (53)  PLAY_RATE (1.0)  LID_TOPIC / IMU_TOPIC (/livox/lidar, /livox/imu)  FASTLIO_SED (sed script over the generated fastlio.yaml)
set -eo pipefail
bag=$1; map=$2; ref=$3; h=$4
tools=$(dirname "$(dirname "$(readlink -f "$0")")")
: "${DDS_IFACE:?set DDS_IFACE to the board LAN interface}" "${PEER:?set PEER to the laptop IP}"
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$DDS_IFACE\"/></Interfaces></General><Discovery><Peers><Peer address=\"$PEER\"/></Peers><MaxAutoParticipantIndex>30</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>"
source /ros_env.sh
[ -n "${OVERLAY:-}" ] && source "$OVERLAY/install/setup.bash"   # optional colcon overlay (planar_odom / FAST-LIO variants not in the image)
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-53}
out=${OUT:-/tmp/live}; mkdir -p "$out"
python3 "$tools/overlay.py" set_initial_pose=false > "$out/overrides.yaml"
sed -e "s|lid_topic:.*|lid_topic: \"${LID_TOPIC:-/livox/lidar}\"|" -e "s|imu_topic:.*|imu_topic: \"${IMU_TOPIC:-/livox/imu}\"|" -e "s|pcd_save_en:.*|pcd_save_en: false|" -e "s|map_en:.*|map_en: false|" -e "s|path_en:.*|path_en: false|" -e "s|^\( *\)gyr_cov:.*|\1gyr_cov: 1.0|" -e "s|^\( *\)acc_cov:.*|\1acc_cov: 1.0|" \
    "$(ros2 pkg prefix fast_lio)/share/fast_lio/config/mid360.yaml" > "$out/fastlio.yaml"   # gyr_cov/acc_cov 1.0 instead of upstream 0.1: stops FAST-LIO2 degrading in featureless corridors (M3DGR Corridor01)
[ -n "${FASTLIO_SED:-}" ] && sed -i -f "$FASTLIO_SED" "$out/fastlio.yaml"   # optional FAST-LIO2 parameter overrides (sed script, one expression per line)
read -r x y yaw < <(python3 "$tools/pose_eval.py" init --ref "$ref" --map "$map" --dist 0 --dyaw 0)   # only fills the launch arguments: AMCL ignores it (set_initial_pose=false)
read -r qx qy qz qw < <(python3 "$tools/pose_eval.py" level --ref "$ref" --map "$map")
setsid ros2 launch "$tools/launch/bench_a.launch.py" fastlio_params:="$out/fastlio.yaml" map:="$map" init_x:="$x" init_y:="$y" init_yaw:="$yaw" overrides:="$out/overrides.yaml" \
    ${SCAN_PARAMS:+scan_params:=$SCAN_PARAMS} sensor_height:="$h" level_qx:="$qx" level_qy:="$qy" level_qz:="$qz" level_qw:="$qw" > "$out/launch.log" 2>&1 &
launch=$!
path_ref=$ref; [ -z "${NO_REF:-}" ] || path_ref=-
python3 "$tools/live/live_paths.py" "$path_ref" "$map" --ros-args -p use_sim_time:=true > "$out/paths.log" 2>&1 &
paths=$!
python3 "$tools/record_poses.py" /amcl_pose amcl "$out/amcl.tum" --idle 0 --max-wait 86400 > "$out/rec_amcl.log" 2>&1 &
python3 "$tools/record_poses.py" /Odometry odom "$out/odom.tum" --idle 0 --max-wait 86400 > "$out/rec_odom.log" 2>&1 &
trap 'kill -TERM -- "-$launch" 2>/dev/null; kill -TERM $(jobs -p) 2>/dev/null' EXIT
cat "$bag"/*.mcap "$bag"/*.db3 > /dev/null 2>&1 || true   # warm the page cache: a cold read of the bag right after a reboot starved the player for 12 s in the middle of a turn
sleep 15
echo "ready: in rviz2 click 2D Pose Estimate near the robot start, then on the laptop: control.sh resume   (also: pause | toggle | rate 0.5)"
ros2 bag play "$bag" --clock --start-paused --rate "${PLAY_RATE:-1.0}" --read-ahead-queue-size 10000
