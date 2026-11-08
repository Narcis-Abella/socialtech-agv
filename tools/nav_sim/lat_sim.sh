#!/bin/bash
# Does a relocalizer search disturb FAST-LIO? Same stack as sup_sim.sh, in MONITOR; lat_ab.py runs the search worker by hand (plain and under nice) and measures the latency and gaps of /Odometry.
# usage: lat_sim.sh <out dir> [reps] [idle_s]   the search input is mounted at /lat_input (a runs/gn_supervisor/reloc*/input of an earlier sup run); CPUSET of agx_full.sh limits the cores of the container
# mounts: repo tools/ at /tools and /greloc/tools, ops/ at /ops, docker/ros_loc/gn_loc at /locsrc (optional), GRELOC_BUILD (register_v2, libsmall_gicp.so, libcorr_search.so) at /greloc/build, out at /out, as lio_sim.sh (SIM_SRC, /pat, /navsrc)
# env: ROS_DOMAIN_ID (55), SPAWN, SEED, SIM_WAIT, PROD=1 (viz:=false: the deployed configuration)
set -eo pipefail
out=${1:?out dir}; reps=${2:-4}; idle_s=${3:-30}
source "${ROS_ENV:-/ros_env.sh}"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-55}
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces></General><Discovery><Peers><Peer address="localhost"/></Peers><MaxAutoParticipantIndex>30</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
set_cyclone_uri   # this export replaced the URI of ros_env.sh: ask again for the 10 MB receive buffer (docker/cyclone_uri.sh)
source /ops/flags.sh; normalize_flags PROD   # PROD=0 means off, as in ops/nav_live.sh
source /tools/nav_sim/sim_up.sh
stack_up
python3 /tools/nav_sim/lat_ab.py "$out/lat.json" /lat_input "$reps" "$idle_s" --z-band $(python3 -c "print($SENSOR_H - 0.5, $SENSOR_H + 0.5)") | tee "$out/lat.txt"
