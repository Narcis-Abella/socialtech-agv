#!/bin/bash
# Live localization on the robot over a map made by ops/make_map.sh: Livox driver + FAST-LIO2 + AMCL. Move the robot by hand; watch it in rviz2 on the laptop (ops/rviz_live.sh).
#   PEER=<laptop IP> ops/nav_live.sh MAP_NAME      MAP_NAME = folder in ~/maps; needs a tty (ssh -t): Ctrl-C stops
# Keep the robot still while FAST-LIO2 initialises (~5 s); then click "2D Pose Estimate" in rviz2 at the robot's real pose and drag to its heading.
# The odometry is levelled from the map's own Voxel-SLAM start pose (same IMU mounting, robot on the same floor); the yaw is whatever you click.
# NAV=1 adds Nav2 (nav_robot.launch.py) with the base OFF: goals plan but nothing moves. NAV=1 BASE=1 also starts the Tracer base: then a goal MOVES THE ROBOT.
# SUPERVISOR=1 (default off) adds gn_supervisor (branch feat/gn-reloc-supervisor, staged in GRELOC=~/greloc): when gn_loc is lost, starved of scans, silent, or
# drifts while parked, it cancels every Nav2 goal (never publishes a velocity), relocalizes with the robot's own FAST-LIO2 and the 3D prior
# GRELOC/prior/MAP_NAME/prior/map.xyz, and publishes /initialpose only if gn_loc disagrees. It never re-sends a goal. State: /gn_supervisor/state.
# env: DDS_IFACE (interface towards PEER)  ROS_DOMAIN_ID (53, as the laptop scripts)  LIVOX_HOST_IP / LIVOX_LIDAR_IP  IMAGE (socialtech:robot)  MAP_DIR (~/maps)
set -eo pipefail
REPO=$(cd "$(dirname "$0")/.." && pwd)
NB=$REPO/tools/nav_bench
source "$REPO/ops/flags.sh"
normalize_flags NAV BASE SUPERVISOR   # NAV=0 / BASE=0 / SUPERVISOR=0 mean off (a non-empty test used to arm Nav2, the Tracer base and the supervisor)

if [[ ${1:-} == --inside ]]; then   # runs in the container
  source /ros_env.sh
  source "$REPO/ops/fastlio_check.sh"
  # ~/nav_overlay: optional, a colcon overlay built in this image that shadows its packages (an image built from this tree already has the patched RPP, scan_median and gn_loc). It is sourced after the image, so an old overlay wins over the image: delete it once the image is rebuilt
  if [[ -f /ov/install/setup.bash ]]; then
    source /ov/install/setup.bash
    echo "WARN: ~/nav_overlay is sourced after the image: its packages ($(ls /ov/install/share 2>/dev/null | grep -v -E '^(colcon-core|ament_index)$' | tr '\n' ' ')) shadow the ones built in the image"
  fi
  export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-53}
  # lo first: the stack's own traffic (driver -> FAST-LIO, recorder) must not ride on the link to the laptop. Pinned to DDS_IFACE alone,
  # it went as multicast on that interface, and when the USB/Wi-Fi link dropped every local send failed (IMU lost 7-8 s, 29 Oct).
  # lo disables multicast, so local discovery needs the localhost peer. Same interface twice is a Cyclone error: skip DDS_IFACE=lo.
  ifs="<NetworkInterface name=\"lo\"/>"; [[ $DDS_IFACE == lo ]] || ifs+="<NetworkInterface name=\"$DDS_IFACE\"/>"
  export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces>$ifs</Interfaces></General><Discovery><Peers><Peer address=\"$PEER\"/><Peer address=\"localhost\"/></Peers><MaxAutoParticipantIndex>30</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>"
  h=$(python3 -c "import json; print(json.load(open('/map/report.json'))['sensor_height_m'])")
  read -r qx qy qz qw < <(python3 "$NB/pose_eval.py" level --ref /map/vs/run/alidarState.txt --map /map/map.yaml)
  "$REPO/ops/livox_config.sh" >/tmp/MID360_config.json
  FSM=${FILTER_SIZE_MAP:-0.25}; PFN=${POINT_FILTER_NUM:-2}   # env, like GYR_COV: FILTER_SIZE_MAP=0.5 POINT_FILTER_NUM=3 are the upstream values
  [[ $FSM =~ ^[0-9]+(\.[0-9]+)?$ && $FSM != 0 && $PFN =~ ^[1-9][0-9]*$ ]] || { echo "FAIL: FILTER_SIZE_MAP='$FSM' must be a positive number and POINT_FILTER_NUM='$PFN' a positive integer" >&2; exit 1; }
  sed -e "s|pcd_save_en:.*|pcd_save_en: false|" -e "s|map_en:.*|map_en: false|" -e "s|path_en:.*|path_en: false|" \
      -e "s|^\( *\)gyr_cov:.*|\1gyr_cov: ${GYR_COV:-0.01}|" -e "s|^\( *\)acc_cov:.*|\1acc_cov: 1.0|" -e "s|^\( *\)blind:.*|\1blind: 0.1|" \
      -e "s|^\( *\)filter_size_map:.*|\1filter_size_map: $FSM|" -e "s|^\( *\)point_filter_num:.*|\1point_filter_num: $PFN|" \
      "$(ros2 pkg prefix fast_lio)/share/fast_lio/config/mid360.yaml" >/tmp/fastlio.yaml   # gyr_cov (env GYR_COV, default 0.01: parked yaw drift -25 %, live 2026-10-31; 1.0 = the bench value) / acc_cov 1.0 (upstream 0.1 degrades in featureless corridors)
  # filter_size_map 0.25 (upstream 0.5) + point_filter_num 2 (upstream 3), env FILTER_SIZE_MAP / POINT_FILTER_NUM (0.5 / 3 go back). Evidence: replays of 3 live passes of
  # one house (238 s each, deterministic, one replay per condition), not a run on the robot. Hypothesis: the ICP was weakly constrained in pitch/roll. In them the pitch
  # wobble while driving fell 1.5-2.1 -> 0.2-0.3 deg, isolated /scan flashes -44..-79 %, FAST-LIO CPU 17.5 -> 20.3 s per 238 s replay ON THE AGX ONLY: not measured on
  # the NX or the Nano, where a denser map and 1 of 2 points may not keep up with the 10 Hz of the lidar (SocialTech notes/ruido_lidar_2026-10-31.md).
  fastlio_check /tmp/fastlio.yaml filter_size_map="$FSM" point_filter_num="$PFN" gyr_cov="${GYR_COV:-0.01}" acc_cov=1.0 blind=0.1 || { echo "FAIL: the FAST-LIO config edit did not apply" >&2; exit 1; }
  echo "sensor height $h m; level quaternion $qx $qy $qz $qw; DDS $DDS_IFACE -> $PEER (domain $ROS_DOMAIN_ID)"
  launch=nav_live; extra=()
  if [[ -n ${NAV:-} ]]; then   # navigation: Nav2 params of the sim + the real robot's differences
    python3 "$REPO/ops/merge_params.py" "$(ros2 pkg prefix socialtech_nav)/share/socialtech_nav/config/nav2_params.yaml" "$REPO/ops/nav2_robot_overrides.yaml" >/tmp/nav2_params.yaml
    sup_topics=(); [[ -z ${SUPERVISOR:-} ]] || sup_topics=(/gn_loc/status /gn_loc/need_reloc /gn_supervisor/state /initialpose /navigate_through_poses/_action/status)   # only where they exist
    set -m   # job control: a background job of a non-interactive shell ignores SIGINT, and the recorder needs one to write its metadata.yaml
    python3 "$REPO/ops/nav_start_when_localized.py" &   # lifetime = the container's
    # Everything needed to judge a run offline (and to replay FAST-LIO on it: /Odometry, /livox/lidar, /livox/imu): the three speed stages, what the base reports, AMCL, BT, plans, scan, local costmap, TF, goals. Not the global costmap (static, big).
    ros2 bag record -s mcap -o /runs/bag /cmd_vel_nav /cmd_vel_smoothed /cmd_vel /odom /tracer_status /amcl_pose /behavior_tree_log /collision_monitor_state /diagnostics \
      /goal_pose /plan /local_plan /received_global_plan /speed_limit /scan /local_costmap/costmap /tf /tf_static /navigate_to_pose/_action/status /navigate_to_pose/_action/feedback \
      /Odometry /livox/lidar /livox/imu "${sup_topics[@]}" \
      </dev/null >/runs/bag.log 2>&1 &
    rec=$!
    [[ -z ${BASE:-} ]] || { sleep 12; python3 "$REPO/ops/base_state.py" --require-can || echo "!!! the base IGNORES Nav2's commands until this is fixed" >&2; } &   # tracer_base has asked for CAN mode by then
    launch=nav_robot; extra=(nav2_params:=/tmp/nav2_params.yaml base:=$([[ -n ${BASE:-} ]] && echo true || echo false))
  fi
  args=("$REPO/ops/$launch.launch.py" livox_config:=/tmp/MID360_config.json fastlio_params:=/tmp/fastlio.yaml map:=/map/map.yaml
    overrides:="$REPO/ops/live_overrides.yaml" sensor_height:="$h" level_qx:="$qx" level_qy:="$qy" level_qz:="$qz" level_qw:="$qw" "${extra[@]}")
  [[ -n ${NAV:-} ]] || exec ros2 launch "${args[@]}"
  # With the recorder running the launch is a child, not exec'd: when it ends (Ctrl-C reaches this script through tini and is forwarded) the recorder gets
  # its SIGINT and finishes the bag. Exec'd, nobody stopped it and the bag stayed without metadata.yaml (29-31 Oct runs).
  ros2 launch "${args[@]}" &
  lp=$!
  rc=0
  trap 'kill -INT "$lp" 2>/dev/null || true' INT TERM
  while kill -0 "$lp" 2>/dev/null; do wait "$lp" || rc=$?; done
  kill -INT "$rec" 2>/dev/null || true
  wait "$rec" || true
  exit "$rc"
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

SUP=()
if [[ -n ${SUPERVISOR:-} ]]; then   # validated first: an error must not come after the "a goal MOVES THE ROBOT" banner
  [[ -n ${NAV:-} ]] || die "SUPERVISOR=1 holds Nav2 goals: it needs NAV=1 (alone it would only relocalize beside a manual 2D Pose Estimate)"
  [[ ${LOCALIZER:-} != amcl ]] || die "SUPERVISOR=1 supervises gn_loc: unset LOCALIZER=amcl"
  G=${GRELOC:-$HOME/greloc}
  for f in tools/global_reloc_3d/gn_supervisor_node.py build/register_v2 build/libsmall_gicp.so prior/$MAP_NAME/prior/map.xyz; do
    [[ -f $G/$f ]] || die "$G/$f missing: stage the supervisor tools, register_v2 and the 3D prior of the map into GRELOC (stage_supervisor.sh, in the team's SocialTech workspace repo)"
  done
  docker run --rm "$IMAGE" python3 -c "import numpy, scipy, rclpy, action_msgs.msg, diagnostic_msgs.msg" >/dev/null 2>&1 || die "$IMAGE lacks a python module the supervisor imports (numpy, scipy, rclpy, action_msgs, diagnostic_msgs)"
  SUP=(-e SUPERVISOR=1 -e "SUP_PRIOR=/greloc/prior/$MAP_NAME/prior/map.xyz" -v "$G:/greloc:ro")
  echo "SUPERVISOR=1: gn_supervisor holds Nav2 when gn_loc cannot be trusted; it starts holding until gn_loc confirms the 2D Pose Estimate (state: /gn_supervisor/state)"
fi
if [[ -n ${NAV:-} ]]; then
  docker run --rm "$IMAGE" ros2 pkg prefix socialtech_nav >/dev/null 2>&1 || die "$IMAGE has no Nav2 (socialtech_nav): rebuild it (ops/build_image.sh robot)"
  if [[ -n ${BASE:-} ]]; then
    [[ $(cat /sys/class/net/can_tracer/operstate 2>/dev/null) == up ]] || die "can_tracer is not UP: the base will not answer (host/setup_host.sh --check)"
    python3 "$REPO/ops/base_state.py" || die "the base is not ready (see above)"
    echo "NAV=1 BASE=1: Nav2 AND THE TRACER BASE ARE ON: a goal MOVES THE ROBOT. Keep a hand on the power button."
  else
    echo "NAV=1: Nav2 on, base OFF: goals plan and drive nothing. Push the robot by hand."
  fi
fi
RUN=${NAV_RUNS:-$HOME/nav_runs}/$(date +%Y%m%d_%H%M%S)   # persists after the container is gone: ROS node logs (log/) and, with NAV, the bag (bag/)
mkdir -p "$RUN/log"
echo "localizing on $MAP (Ctrl-C to stop). Logs${NAV:+ and bag} in $RUN. Keep the robot still ~5 s, then 2D Pose Estimate in rviz2."
exec docker run --rm --init -t --network host --user "$(id -u):$(id -g)" -e HOME=/tmp -e ROS_LOG_DIR=/runs/log -e NAV="${NAV:-}" -e BASE="${BASE:-}" -e LOCALIZER="${LOCALIZER:-}" -e GYR_COV="${GYR_COV:-0.01}" -e SCAN_CONVERTER="${SCAN_CONVERTER:-}" \
  -e DDS_IFACE="$DDS_IFACE" -e PEER="$PEER" -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-53}" -e LIVOX_HOST_IP="$HOST_IP" -e LIVOX_LIDAR_IP="$LIDAR_IP" \
  -v "$MAP:/map:ro" -v "$REPO:$REPO:ro" -v "$RUN:/runs" -v "$HOME/nav_overlay:/ov:ro" "${SUP[@]}" -w /tmp "$IMAGE" "$REPO/ops/nav_live.sh" --inside
