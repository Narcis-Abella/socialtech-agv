#!/bin/bash
# Does a relocalizer search disturb FAST-LIO? Same stack as sup_sim.sh, in MONITOR; lat_ab.py runs the search worker by hand (plain and under nice) and measures the latency and gaps of /Odometry.
# usage: lat_sim.sh <out dir> [reps] [idle_s]   the search input is mounted at /lat_input (a runs/gn_supervisor/reloc*/input of an earlier sup run); CPUSET of agx_full.sh limits the cores of the container
# mounts: repo tools/ at /tools and /greloc/tools, ops/ at /ops, docker/ros_loc/gn_loc at /locsrc (optional), GRELOC_BUILD (register_v2, libsmall_gicp.so) at /greloc/build, out at /out, as lio_sim.sh (SIM_SRC, /pat, /navsrc)
# env: ROS_DOMAIN_ID (55), SPAWN, SEED, SIM_WAIT
set -eo pipefail
out=${1:?out dir}; reps=${2:-4}; idle_s=${3:-30}
source "${ROS_ENV:-/ros_env.sh}"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-55}
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces></General><Discovery><Peers><Peer address="localhost"/></Peers><MaxAutoParticipantIndex>30</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
source /tools/nav_sim/sim_up.sh
if [ -d /locsrc ]; then   # gn_loc newer than the image's (gn_loc/scan_fit, which the supervisor confirms with): overlay
  mkdir -p /tmp/locws && (cd /tmp/locws && colcon build --base-paths /locsrc --packages-select gn_loc --merge-install --install-base /tmp/loc_install > "$out/build_loc.log" 2>&1)
  source /tmp/loc_install/setup.bash
fi
fastlio_yaml "$out/fastlio.yaml"
python3 /tools/sim_realism/tris_to_prior.py /pat/loyola.tris "$out/prior.xyz" 0.2 0.1 > "$out/prior.log"
mkdir -p "$out/runs"
export SUPERVISOR=1 SUP_PRIOR="$out/prior.xyz"
chain_map; map=$CHAIN_MAP
ros2 launch /ops/nav_live.launch.py livox_config:=/dev/null fastlio_params:="$out/fastlio.yaml" map:="$map" overrides:=/ops/live_overrides.yaml \
  sensor_height:=$SENSOR_H level_qx:=0.0 level_qy:=0.0 level_qz:=0.0 level_qw:=1.0 driver:=false sim_time:=true > "$out/launch.log" 2>&1 &
sleep 10
if [ -n "$PROCMON" ]; then python3 /tools/nav_sim/procmon.py sample "$out/procmon.csv" 1 & mon=$!; fi
python3 /tools/nav_sim/lat_ab.py "$out/lat.json" /lat_input "$reps" "$idle_s" --z-band $(python3 -c "print($SENSOR_H - 0.5, $SENSOR_H + 0.5)") | tee "$out/lat.txt"
[ -z "$mon" ] || { kill $mon; wait $mon 2> /dev/null || true; }
