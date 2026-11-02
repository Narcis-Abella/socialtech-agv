#!/bin/bash
# Nav2 on the full-chain simulation, inside the `sim` image: the robot's own ops/nav_robot.launch.py (FAST-LIO2 on the simulated Mid-360, planar_odom, scan_median, gn_loc,
# Nav2 with the real robot's overrides, base off: the simulator's base model drives) and trials.py on a scenario. usage: nav_full_sim.sh <out dir> <scenario.json> [reps]
# mounts and env as sup_sim.sh; SUPERVISOR=1 also runs gn_supervisor (needs /greloc/build and the prior); SCAN_EXTRA=0 the chain alone; LOCALIZER=amcl for AMCL.
set -eo pipefail
out=${1:?out dir}; scenario=${2:?scenario.json}; reps=${3:-1}
source "${ROS_ENV:-/ros_env.sh}"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-57}
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces></General><Discovery><Peers><Peer address="localhost"/></Peers><MaxAutoParticipantIndex>30</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
# scenario `auto[:N]`: N start/goal pairs sampled on the chain's own map (the legacy scenarios were made on loyola.pgm: on this map some goals are in closed rooms)
if [[ $scenario == auto* ]]; then
  mkdir -p "$out"; n=${scenario#auto}; n=${n#:}
  python3 /tools/sim_realism/tris_to_map.py /pat/loyola.tris "$out/loyola_chain" -13.0,0.0 > "$out/map.log"
  (cd /tools/nav_sim && python3 make_scenario.py "$out/loyola_chain.yaml" "$out/scenario.json" "${n:-6}" 20261102)
  scenario=$out/scenario.json
fi
# FAST-LIO2 does not survive a teleport: the robot is spawned at the first start of the scenario and the trials chain from where the previous one ended (trials.py CHAIN=1)
export SPAWN=$(python3 -c "import json,sys; print(*json.load(open(sys.argv[1]))['trials'][0]['start'])" "$scenario") CHAIN=${CHAIN:-1}
source /tools/nav_sim/sim_up.sh
if [ -d /locsrc ]; then
  mkdir -p /tmp/locws && (cd /tmp/locws && colcon build --base-paths /locsrc --packages-select gn_loc --merge-install --install-base /tmp/loc_install > "$out/build_loc.log" 2>&1)
  source /tmp/loc_install/setup.bash
fi
chain_map
fastlio_yaml "$out/fastlio.yaml"
share=$(ros2 pkg prefix socialtech_nav)/share/socialtech_nav
python3 /ops/merge_params.py "$share/config/nav2_params.yaml" /ops/nav2_robot_overrides.yaml > "$out/nav2_params.yaml"
if [ "${SUPERVISOR:-0}" != 0 ]; then
  python3 /tools/sim_realism/tris_to_prior.py /pat/loyola.tris "$out/prior.xyz" 0.2 0.1 > "$out/prior.log"; mkdir -p "$out/runs"; export SUP_PRIOR="$out/prior.xyz"
fi
if [ "${SCAN_EXTRA:-1}" != 0 ]; then
  export SCAN_TOPIC=/scan_chain
  ros2 run socialtech_simreal sim_scan_model $SIM -p seed:=${SEED:-1} -p noise_scale:=0.0 -p in_topic:=/scan_chain -p out_topic:=/scan > "$out/scan_extra.log" 2>&1 &
fi
ros2 launch /ops/nav_robot.launch.py livox_config:=/dev/null fastlio_params:="$out/fastlio.yaml" map:="$CHAIN_MAP" overrides:=/ops/live_overrides.yaml \
  sensor_height:=$SENSOR_H level_qx:=0.0 level_qy:=0.0 level_qz:=0.0 level_qw:=1.0 nav2_params:="$out/nav2_params.yaml" driver:=false sim_time:=true base:=false > "$out/launch.log" 2>&1 &
sleep 12
# the robot starts at the spawn pose: tell gn_loc (the operator's 2D Pose Estimate), as trials.py does for every trial
python3 - "$sx" "$sy" "$sw" <<'P' > "$out/init.log" 2>&1
import math, sys, rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
x, y, w = (float(v) for v in sys.argv[1:4])
rclpy.init(); n = rclpy.create_node('init_pose', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
pub = n.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
m = PoseWithCovarianceStamped(); m.header.frame_id = 'map'; m.pose.pose.position.x, m.pose.pose.position.y = x, y
m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(w / 2), math.cos(w / 2); c = [0.] * 36; c[0] = c[7] = .1 ** 2; c[35] = .1 ** 2; m.pose.covariance = c
for _ in range(10):
    m.header.stamp = n.get_clock().now().to_msg(); pub.publish(m); rclpy.spin_once(n, timeout_sec=.5)
P
python3 /ops/nav_start_when_localized.py > "$out/start.log" 2>&1
timeout "${NAV_TIMEOUT:-180}" bash -c 'until [ "$(ros2 lifecycle get /bt_navigator 2>/dev/null | tail -1)" = "active [3]" ]; do sleep 2; done' || { echo "bt_navigator not active: see $out/launch.log" >&2; exit 1; }
sleep 5
python3 /tools/nav_sim/trials.py "$scenario" "$out" "$reps" --ros-args -p use_sim_time:=true 2>&1 | tee "$out/trials.log"
