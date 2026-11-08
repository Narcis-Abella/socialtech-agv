#!/bin/bash
# Laptop side: runs the full-chain simulation (simulated Mid-360 + real FAST-LIO2) on the AGX; No sudo; refuses when the board is busy.
#   agx_full.sh start <lio|fid|sup|lat|clock|nav> <name> [args of the script]   lio: lio_sim.sh [leg v w]   fid: fid_sim.sh [seconds]   clock: clock_sim.sh [seconds] (CPU of an idle sim-time node against a wall-time one)   lat: lat_sim.sh [reps] [idle_s] (does a search disturb FAST-LIO; needs ~/lat_input on the board)   sup: sup_sim.sh [cover_s] [--push-after S | --move-covered M]   nav: nav_full_sim.sh <scenario.json or auto[:N] (N pairs on the chain's own map)> [reps] (Nav2 + trials.py)
#   agx_full.sh wait <remote dir>        blocks until it has ended and prints the last lines of its result
#   agx_full.sh fetch <remote dir> <local dir>
# Ships tools/ and src/ of this checkout and the committed sim repo (SIM_REPO, ~/Documents/GitHub/hospital-agv-sim). On the board: ~/livox_pat (ours.lvm, loyola.tris), ~/greloc/build (sup only).
# env (all forwarded to the container when set): SUPERVISOR=1 (nav: also gn_supervisor), REC_GCM (nav: trials.py records the global costmap), LIDAR_Z (0.57), SIM_WAIT, NAV_TIMEOUT, GYR_COV, NAV2_OVERLAY (nav: a file of tools/nav_sim/params merged last), CLEARANCE (m, 0.6: start/goal clearance of auto scenarios), DIAG=1 (nav only: one look at the scan and the local costmap instead of trials), DEADBAND (rad/s, 0.08: the median of the three real sessions 0.06 / 0.08 / 0.18), RUN_TIMEOUT (1800 s), IMAGE (socialtech:sim-navtest: a `sim` image with socialtech_livox_sim and socialtech_simreal; the checkout's versions are built over it), SCAN_EXTRA=1 (add the calibrated dropout and jumps; on for fid), SEED, SPAWN, PROD=1 (sup, lat: the deployed configuration, viz:=false), CPUSET (cores of the container, e.g. 0-5), SETTLE (sup: s in MONITOR before the cover), PROCMON=1 (sup: per-process CPU/RAM, see procmon.py).
set -eo pipefail
KEY=${AGX_KEY:-~/.nvsdkm/.ssh/id_rsa_sdkm}; HOST=${AGX_HOST:-jetson@socialtech-agx-1}   # the AGX is the only board with the sim image
ssh_() { ssh -i "$KEY" -o ConnectTimeout=10 "$HOST" "$@"; }
repo=$(cd "$(dirname "$0")/../.." && pwd)
case "$1" in
start)
  kind=${2:?lio|fid|sup|lat|clock|nav}; name=${3:?name}; shift 3
  case $kind in lio) script=lio_sim.sh ;; nav) script=nav_full_sim.sh ;; fid) script=fid_sim.sh ;; sup) script=sup_sim.sh ;; lat) script=lat_sim.sh ;; clock) script=clock_sim.sh ;; *) echo "kind: lio|fid|sup|lat|clock|nav" >&2; exit 1 ;; esac
  sim=${SIM_REPO:-$HOME/Documents/GitHub/hospital-agv-sim}
  [ -z "$(git -C "$sim" status --porcelain)" ] || { echo "the sim repo has uncommitted changes" >&2; exit 1; }
  hash=$(git -C "$sim" rev-parse --short HEAD)
  read -r nct load <<< "$(ssh_ 'echo $(docker ps -q | wc -l) $(cut -d" " -f1 /proc/loadavg)')"
  { [ "$nct" = 0 ] && awk -v l="$load" 'BEGIN{exit !(l < 2)}'; } || { echo "the AGX is busy (containers=$nct load=$load)" >&2; exit 1; }
  remote=liosim/${name}_$(date +%y%m%d_%H%M%S)
  ssh_ "mkdir -p $remote/out $remote/runs ~/sim_src/hospital-agv-sim_$hash"
  tar -C "$repo" -c tools src | ssh_ "tar -x -C $remote"
  ssh_ "[ -f ~/sim_src/hospital-agv-sim_$hash/package.xml ]" || git -C "$sim" archive HEAD | ssh_ "tar -x -C ~/sim_src/hospital-agv-sim_$hash"
  args="$*"; lat_mount=; [ "$kind" != lat ] || lat_mount='-v $HOME/lat_input:/lat_input:ro'
  ssh -f -n -i "$KEY" "$HOST" "cd $remote && setsid nohup sh -c 'timeout ${RUN_TIMEOUT:-1800} docker run --rm --name fullsim ${CPUSET:+--cpuset-cpus=$CPUSET} --network host --user root -e ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-54} -e SIM_SRC=/simsrc ${SCAN_EXTRA:+-e SCAN_EXTRA=$SCAN_EXTRA} ${DEADBAND:+-e DEADBAND=$DEADBAND} ${SUPERVISOR:+-e SUPERVISOR=$SUPERVISOR} ${LEVEL_CORRECTOR:+-e LEVEL_CORRECTOR=$LEVEL_CORRECTOR} ${REC_GCM:+-e REC_GCM=$REC_GCM} ${LIVE_PEER:+-e LIVE_PEER=$LIVE_PEER} ${LIDAR_Z:+-e LIDAR_Z=$LIDAR_Z} ${SIM_WAIT:+-e SIM_WAIT=$SIM_WAIT} ${PROCMON:+-e PROCMON=$PROCMON} ${PROD:+-e PROD=$PROD} ${SETTLE:+-e SETTLE=$SETTLE} ${NAV_TIMEOUT:+-e NAV_TIMEOUT=$NAV_TIMEOUT} ${GYR_COV:+-e GYR_COV=$GYR_COV} ${NAV2_OVERLAY:+-e NAV2_OVERLAY=$NAV2_OVERLAY} -e DIAG=${DIAG:-0} -e CLEARANCE=${CLEARANCE:-0.6} -e SEED=${SEED:-1} ${SPAWN:+-e SPAWN=\"$SPAWN\"} -v \$PWD/tools:/tools:ro -v \$PWD/tools:/greloc/tools:ro -v \$PWD/src:/src:ro -v \$HOME/sim_cache/sg:/sg:ro -v \$HOME/greloc/build:/greloc/build:ro -v \$PWD/runs:/runs -v \$PWD/out:/out -v \$HOME/sim_src/hospital-agv-sim_$hash:/simsrc:ro -v ${PAT_DIR:-\$HOME/livox_pat}:/pat:ro ${lat_mount} ${IMAGE:-socialtech:sim-navtest} bash /tools/nav_sim/$script /out $args > run.log 2>&1; echo rc=\$? >> run.log' > /dev/null 2>&1" > /dev/null 2>&1   # the local ssh must not keep the caller's pipe (R=$(agx_full.sh start ...)) open until the run ends
  echo "$remote" ;;
wait)
  until ssh_ "grep -q '^rc=' ${2:?remote dir}/run.log" 2> /dev/null; do sleep 20; done
  ssh_ "tail -14 $2/run.log | cut -c1-220" ;;
fetch)
  mkdir -p "${3:?local dir}"
  ssh_ "tar -C ${2:?remote dir} -c out run.log" | tar -x -C "$3" ;;
*) sed -n 2,5p "$0"; exit 1 ;;
esac
