#!/bin/bash
# gn_supervisor in the full simulation: the simulator with a real FAST-LIO2, the robot's own ops/nav_live.launch.py (driver off, sim time) with gn_loc and the supervisor,
# then the covered-lidar scenario (sup_cover.py). usage: sup_sim.sh <out dir> [cover_s]
# mounts: repo tools/ at /tools and /greloc/tools, ops/ at /ops, docker/ros_loc/gn_loc at /locsrc (optional), GRELOC_BUILD (register_v2, libsmall_gicp.so) at /greloc/build, out at /out, as lio_sim.sh (SIM_SRC, /pat, /navsrc)
# env: ROS_DOMAIN_ID (55), SPAWN, SEED, SIM_WAIT, SUP_ARGS (extra "-p" arguments appended to nothing: the supervisor runs with the launch file's own parameters)
set -eo pipefail
out=${1:?out dir}; cover_s=${2:-20}
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
map=$(ros2 pkg prefix socialtech_nav)/share/socialtech_nav/maps/loyola.yaml
ros2 launch /ops/nav_live.launch.py livox_config:=/dev/null fastlio_params:="$out/fastlio.yaml" map:="$map" overrides:=/ops/live_overrides.yaml \
  sensor_height:=0.28 level_qx:=0.0 level_qy:=0.0 level_qz:=0.0 level_qw:=1.0 driver:=false sim_time:=true > "$out/launch.log" 2>&1 &
sleep 10
python3 /tools/nav_sim/sup_cover.py "$out/scenario.json" "$cover_s" | tee "$out/scenario.txt"
