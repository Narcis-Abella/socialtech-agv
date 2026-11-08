#!/bin/bash
# The robot's live stack over a map made by ops/make_map.sh: Livox driver + FAST-LIO2 + gn_loc (+ Nav2 with NAV=1). Move the robot by hand; watch it in rviz2 on the laptop (ops/rviz_live.sh).
#   [PEER=<laptop IP>] ops/nav_live.sh MAP_NAME      MAP_NAME = folder in ~/maps; Ctrl-C stops (with a terminal; ssh -t). PEER only for rviz2 on a laptop: without it DDS stays on loopback.
# This is the HOST half: it checks the board, the map, the image and the flags, then starts the stack in the container (src/bringup/socialtech_bringup/scripts/run_live, in the image).
# Keep the robot still while FAST-LIO2 initialises (~5 s); then click "2D Pose Estimate" in rviz2 at the robot's real pose and drag to its heading.
# The odometry is levelled from the map's own Voxel-SLAM start pose (same IMU mounting, robot on the same floor); the yaw is whatever you click.
# NAV=1 adds Nav2 (nav_robot.launch.py) with the base OFF: goals plan but nothing moves. NAV=1 BASE=1 also starts the Tracer base: then a goal MOVES THE ROBOT.
# PROD=1 (default off = DEV): the robot as deployed. No bag recorder (DEV records the speeds, the odometry and the raw /livox/lidar + /livox/imu of every NAV run) and no live_paths.py (a
# rviz aid); the ROS node logs still persist in $RUN/log.
# HEAVY_DDS=1 (default off): driver and FAST-LIO2 talk on loopback with 64 kB DDS datagrams (see heavy_env in nav_live.launch.py: not measured on the Nano yet).
# LEVEL_CORRECTOR (default ON; LEVEL_CORRECTOR=0 turns it off): level_corrector registers FAST-LIO2's cloud against the map's 3D points every 2 s and planar_odom levels odom with it (src/localization/level_corrector/README.md). Off = the static level of the mapping run.
# SUPERVISOR (default ON; SUPERVISOR=0 turns it off) runs gn_supervisor (src/localization/gn_supervisor): when gn_loc is lost, starved of scans, silent, or
# drifts while parked, it cancels every Nav2 goal (never publishes a velocity), relocalizes with the robot's own FAST-LIO2 and the 3D prior MAP/prior/map.xyz
# (made by ops/make_map.sh; built here, in seconds, for a map that has none), and publishes /initialpose only if gn_loc disagrees. It never re-sends a goal. State: /gn_supervisor/state.
# The image must be the one this checkout builds (ops/image_revision.sh image_freshness: the label of the image against src/, deps/, docker/ and this file): a stale one stops the run,
# STALE_IMAGE_OK=1 runs it anyway. OVERLAY=1 builds socialtech_bringup and socialtech_nav from this checkout's src/ at start-up (launch files, scripts, config, the Nav2 parameters and behavior trees: no image rebuild for those; C++ still needs one).
# env: DDS_IFACE (interface towards PEER; default: the route to PEER, or lo without PEER)  ROS_DOMAIN_ID (53, as the laptop scripts)  LIVOX_HOST_IP / LIVOX_LIDAR_IP  IMAGE (socialtech:robot)  MAP_DIR (~/maps)
set -eo pipefail
REPO=$(cd "$(dirname "$0")/.." && pwd)
BRINGUP=$REPO/src/bringup/socialtech_bringup/scripts   # the same files run in the image (run_live): the freshness check below keeps them one version
source "$BRINGUP/flags.sh"
source "$REPO/ops/image_revision.sh"
LEVEL_CORRECTOR=${LEVEL_CORRECTOR-1}   # on by default: only an explicit LEVEL_CORRECTOR=0 (or off/false/no) turns it off
SUPERVISOR=${SUPERVISOR-1}             # on by default, the same way
normalize_flags NAV BASE SUPERVISOR PROD LEVEL_CORRECTOR HEAVY_DDS OVERLAY STALE_IMAGE_OK   # NAV=0 / BASE=0 / SUPERVISOR=0 / PROD=0 / LEVEL_CORRECTOR=0 mean off (a non-empty test used to arm Nav2, the Tracer base and the supervisor)

MAP_NAME=${1:?usage: [PEER=<laptop IP>] nav_live.sh MAP_NAME}
MAP=${MAP_DIR:-$HOME/maps}/$MAP_NAME
IMAGE=${IMAGE:-socialtech:robot}
HOST_IP=${LIVOX_HOST_IP:-192.168.1.50}
LIDAR_IP=${LIVOX_LIDAR_IP:-192.168.1.126}
RMEM_WANT=67108864
die() { echo "FAIL: $*" >&2; exit 1; }

PEER=${PEER:-}   # the laptop's IP address (laptop: ip -br addr), for rviz2 there; empty = no laptop, DDS on loopback
if [[ -n $PEER ]]; then
  DDS_IFACE=${DDS_IFACE:-$(ip -o route get "$PEER" 2>/dev/null | sed -n 's/.* dev \([^ ]*\).*/\1/p' || true)}
  [[ -n $DDS_IFACE ]] || die "no route to PEER='$PEER' (it must be the laptop's IP address, not a host name; or set DDS_IFACE)"
else
  DDS_IFACE=${DDS_IFACE:-lo}
fi
for f in map.yaml report.json vs/run/alidarState.txt; do [[ -f $MAP/$f ]] || die "$MAP/$f missing: is $MAP_NAME a ops/make_map.sh output?"; done
docker image inspect "$IMAGE" >/dev/null 2>&1 || die "image $IMAGE not found"
fresh=0; why=$(image_freshness "$IMAGE" "$REPO") || fresh=$?
case $fresh in
  1) if [[ -n ${OVERLAY:-} || -n ${STALE_IMAGE_OK:-} ]]; then echo "WARN: $why" >&2
     else die "$why: build or ship the image (ops/build_image.sh robot, ops/ship_image.sh), or OVERLAY=1 (launch files, scripts, config, Nav2 parameters and behavior trees from this checkout), or STALE_IMAGE_OK=1"; fi ;;
  2) echo "WARN: cannot compare the image with this checkout: $why" >&2 ;;
esac
[[ -z ${LEVEL_CORRECTOR:-} ]] || { [[ -f $MAP/map.ply ]] && grep -q '^# \[' "$MAP/map.yaml"; } || die "LEVEL_CORRECTOR needs $MAP/map.ply and the T_map_world rows of $MAP/map.yaml (both written by ops/make_map.sh): regenerate the map, or run with LEVEL_CORRECTOR=0"
[[ -z $(docker ps -q) ]] || die "a container is running (docker ps): one heavy job at a time"
ip -4 -o addr show | grep -q " $HOST_IP/" || die "no interface has $HOST_IP (the Livox link): check the ethernet cable"
ping -c1 -W2 "$LIDAR_IP" >/dev/null 2>&1 || die "Livox $LIDAR_IP does not answer: power or ethernet cable"
[[ -z $PEER ]] || ping -c1 -W2 "$PEER" >/dev/null 2>&1 || echo "WARN: $PEER does not answer ping (rviz2 may not see the robot)" >&2
(($(sysctl -n net.core.rmem_max) >= RMEM_WANT)) || echo "WARN: net.core.rmem_max below $RMEM_WANT: scans may be lost (sudo sysctl -w net.core.rmem_max=$RMEM_WANT)" >&2

if [[ -n ${SUPERVISOR:-} ]]; then   # validated first: an error must not come after the "a goal MOVES THE ROBOT" banner
  [[ -z ${HEAVY_DDS:-} ]] || die "HEAVY_DDS=1 puts the driver on a loopback-only DDS URI that the supervisor (default URI) is not known to receive the raw lidar and IMU on: untested together, use SUPERVISOR=0 with HEAVY_DDS=1"
  if [[ ! -f $MAP/prior/map.xyz || ! -f $MAP/prior/map_manifest.json ]]; then   # none, or an interrupted one (the manifest is written last): built now from the cleaned cloud of the map
    [[ -f $MAP/map_clean.ply && -f $MAP/report.json ]] || die "$MAP has no prior/map.xyz and no map_clean.ply to build it from: make the map again with ops/make_map.sh (REUSE_SLAM=1 redoes it from the existing SLAM run in seconds), or SUPERVISOR=0 to run without the supervisor"
    echo "no 3D prior in $MAP/prior: building it from map_clean.ply"
    docker run --rm --network none --user "$(id -u):$(id -g)" -e HOME=/tmp -v "$MAP:/m" "$IMAGE" python3 -B /opt/socialtech/lib/gn_supervisor/prepare_map.py --map-dir /m --out /m/prior >"$MAP/prior.log" 2>&1 \
      || die "the 3D prior of $MAP could not be built ($(tail -1 "$MAP/prior.log")): see $MAP/prior.log (a map made before pipeline 2 needs REUSE_SLAM=1 ops/make_map.sh; or SUPERVISOR=0)"
  fi
  echo "SUPERVISOR: gn_supervisor holds Nav2 when gn_loc cannot be trusted; it starts holding until gn_loc confirms the 2D Pose Estimate (state: /gn_supervisor/state)"
fi
if [[ -n ${NAV:-} ]]; then
  if [[ -n ${BASE:-} ]]; then
    [[ $(cat /sys/class/net/can_tracer/operstate 2>/dev/null) == up ]] || die "can_tracer is not UP: the base will not answer (host/setup_host.sh --check)"
    python3 "$BRINGUP/base_state.py" || die "the base is not ready (see above)"
    echo "NAV=1 BASE=1: Nav2 AND THE TRACER BASE ARE ON: a goal MOVES THE ROBOT. Keep a hand on the power button."
  else
    echo "NAV=1: Nav2 on, base OFF: goals plan and drive nothing. Push the robot by hand."
  fi
fi
RUN=${NAV_RUNS:-$HOME/nav_runs}/$(date +%Y%m%d_%H%M%S)   # persists after the container is gone: ROS node logs (log/) and, with NAV and not PROD, the bag (bag/)
mkdir -p "$RUN/log"
if [[ -n ${NAV:-} && -z ${PROD:-} ]]; then   # the DEV recorder writes raw /livox/lidar (~14 GB/h): a full disk breaks the node logs and the supervisor's work directory too
  free_gb=$(df --output=avail -BG "$RUN" | tail -1 | tr -dc 0-9)
  (( free_gb >= ${MIN_FREE_GB:-20} )) || die "only ${free_gb} GB free in $RUN (the bag needs MIN_FREE_GB=${MIN_FREE_GB:-20}); free space, or PROD=1 to run without recording"
fi
{   # what ran: the image (its tag moves; the label is its commit), this checkout, and whether OVERLAY=1 built socialtech_bringup and socialtech_nav from it
  echo "checkout: $(git -C "$REPO" describe --always --dirty 2>/dev/null || echo unknown)"
  { git -C "$REPO" status --porcelain 2>/dev/null || true; } | sed 's/^/  /'   # the board's checkout is often a plain copy, not a repository: git fails and set -e/pipefail would end the script
  img_id=$(docker image inspect -f '{{.Id}}' "$IMAGE" 2>/dev/null || true)
  echo "image: $IMAGE ${img_id:-unknown} revision $(docker image inspect -f '{{index .Config.Labels "org.opencontainers.image.revision"}}' "$IMAGE" 2>/dev/null || echo unknown)"
  echo "supervisor: $([[ -n ${SUPERVISOR:-} ]] && echo on || echo off); overlay: $([[ -n ${OVERLAY:-} ]] && echo "bringup+nav from this checkout" || echo none)"
} > "$RUN/provenance.txt"
echo "localizing on $MAP (Ctrl-C to stop). Logs$([[ -n ${NAV:-} && -z ${PROD:-} ]] && echo " and bag") in $RUN. Keep the robot still ~5 s, then 2D Pose Estimate in rviz2."
run=(/opt/socialtech/lib/socialtech_bringup/run_live); ovl=()
if [[ -n ${OVERLAY:-} ]]; then   # socialtech_bringup and socialtech_nav of this checkout, built over the image's at start-up; /tmp/ov/install first on the ament path, so its run_live, launch files and config win
  ovl=(-v "$REPO/src:/src:ro")
  run=(bash -c 'mkdir -p /tmp/ov && cd /tmp/ov && colcon build --base-paths /src --packages-select socialtech_bringup socialtech_nav --merge-install --install-base /tmp/ov/install >/tmp/ov/build.log 2>&1 || { cat /tmp/ov/build.log >&2; exit 1; }
    source /tmp/ov/install/setup.bash && exec /tmp/ov/install/lib/socialtech_bringup/run_live')
fi
tty=(); [[ -t 0 ]] && tty=(-t)   # no terminal (a launcher script): docker refuses -t; SIGTERM to the docker client stops the stack as Ctrl-C does
exec docker run --rm --init "${tty[@]}" --network host --user "$(id -u):$(id -g)" -e HOME=/tmp -e ROS_LOG_DIR=/runs/log -e NAV="${NAV:-}" -e PROD="${PROD:-}" -e LEVEL_CORRECTOR="${LEVEL_CORRECTOR:-}" -e BASE="${BASE:-}" -e HEAVY_DDS="${HEAVY_DDS:-}" -e SUPERVISOR="${SUPERVISOR:-}" -e GYR_COV="${GYR_COV:-0.01}" -e FILTER_SIZE_MAP="${FILTER_SIZE_MAP:-}" -e POINT_FILTER_NUM="${POINT_FILTER_NUM:-}" \
  -e DDS_IFACE="$DDS_IFACE" -e PEER="$PEER" -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-53}" -e LIVOX_HOST_IP="$HOST_IP" -e LIVOX_LIDAR_IP="$LIDAR_IP" \
  -v "$MAP:/map:ro" -v "$RUN:/runs" "${ovl[@]}" -w /tmp "$IMAGE" "${run[@]}"
