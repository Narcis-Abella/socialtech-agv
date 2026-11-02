#!/bin/bash
# gn_supervisor in the full simulation: the simulator with a real FAST-LIO2, the robot's own ops/nav_live.launch.py (driver off, sim time) with gn_loc and the supervisor,
# then the covered-lidar scenario (sup_cover.py). usage: sup_sim.sh <out dir> [cover_s] [sup_cover.py options: --push-after S | --move-covered M]
# mounts: repo tools/ at /tools and /greloc/tools, ops/ at /ops, docker/ros_loc/gn_loc at /locsrc (optional), GRELOC_BUILD (register_v2, libsmall_gicp.so) at /greloc/build, out at /out, as lio_sim.sh (SIM_SRC, /pat, /navsrc)
# env: ROS_DOMAIN_ID (55), SPAWN, SEED, SIM_WAIT, PROD=1 (the robot as deployed: viz:=false, no rviz aid), SETTLE (s in MONITOR before the cover, 10), PROCMON=1 (per-process CPU/RAM of the whole container to $out/procmon.csv; procmon.py report separates the stack from the simulator)
set -eo pipefail
out=${1:?out dir}; cover_s=${2:-20}; settle_s=${SETTLE:-10}; shift 2 || true
source "${ROS_ENV:-/ros_env.sh}"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-55}
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces></General><Discovery><Peers><Peer address="localhost"/></Peers><MaxAutoParticipantIndex>30</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
source /ops/flags.sh; normalize_flags PROD   # PROD=0 means off, as in ops/nav_live.sh
source /tools/nav_sim/sim_up.sh
stack_up
if [ -n "$PROCMON" ]; then python3 /tools/nav_sim/procmon.py sample "$out/procmon.csv" 1 & mon=$!; fi
python3 /tools/nav_sim/sup_cover.py "$out/scenario.json" "$cover_s" "$settle_s" "$@" | tee "$out/scenario.txt"
[ -z "$mon" ] || { kill $mon; wait $mon 2> /dev/null || true; python3 /tools/nav_sim/procmon.py report "$out/procmon.csv" -s "$out/scenario.json" | tee "$out/procmon.txt"; }
