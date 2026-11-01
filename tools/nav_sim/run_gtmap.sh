#!/bin/bash
# Surveys the simulated world from the poses of a survey file (gt_mapper.py), simulation only (no Nav2), inside the `sim` image. usage: run_gtmap.sh <survey.json> <out dir> [unused]
#   env: SIM_SRC, SIM_ARGS, ROS_DOMAIN_ID (53), SIM_WAIT (45 s), ROS_ENV.  Writes <out dir>/scans.json; build the map offline with gt_map.py.
set -eo pipefail
survey=${1:?survey.json}; out=${2:?out dir}
source "${ROS_ENV:-/ros_env.sh}"
mkdir -p "$out"
trap 'kill $(jobs -p) 2> /dev/null || true' EXIT
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-53}
if ! ros2 pkg prefix hospital-agv-sim > /dev/null 2>&1; then
  mkdir -p /tmp/ws/src && ln -sfn "${SIM_SRC:?hospital-agv-sim is not in the image: set SIM_SRC}" /tmp/ws/src/hospital-agv-sim && cd /tmp/ws
  colcon build --merge-install --install-base /tmp/sim_install > "$out/build.log" 2>&1
  source /tmp/sim_install/setup.bash
fi
ros2 launch hospital-agv-sim simulation.launch.py headless:=true $SIM_ARGS > "$out/sim.log" 2>&1 &
sleep "${SIM_WAIT:-45}"
python3 /tools/nav_sim/gt_mapper.py "$survey" "$out/scans.json" --ros-args -p use_sim_time:=true 2>&1 | tee "$out/survey.log"
echo done
