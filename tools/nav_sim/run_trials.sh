#!/bin/bash
# Runs a scenario file against Nav2 in the simulation, inside the `sim` image on the board. usage: run_trials.sh <scenario.json> <out dir> [reps]
#   mounts: the repo tools/ at /tools, the output dir at /out.   env: PARAMS (nav2_params.yaml instead of the packaged one), SIM_SRC (hospital-agv-sim checkout to build),
#   SIM_ARGS (extra simulation.launch.py arguments: world:=, spawn_x:=, noise_seed:=, odom_source:= ...), ROS_DOMAIN_ID (53), SIM_WAIT (45 s), NAV_TIMEOUT (180 s), plus the env of trials.py
set -eo pipefail
scenario=${1:?scenario.json}; out=${2:?out dir}; reps=${3:-1}
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
ros2 launch socialtech_nav nav_sim.launch.py ${PARAMS:+params_file:=$PARAMS} > "$out/nav.log" 2>&1 &
if ! timeout "${NAV_TIMEOUT:-180}" bash -c 'until [ "$(ros2 lifecycle get /bt_navigator 2>/dev/null | tail -1)" = "active [3]" ]; do sleep 2; done'; then
  echo "bt_navigator not active after ${NAV_TIMEOUT:-180} s: see $out/sim.log and $out/nav.log" >&2
  exit 1
fi
sleep 5
ps -eo pid,args | grep -E 'nav2|opennav|gz|noisy|bridge' | grep -v grep | cut -c1-160 > "$out/ps.txt"
python3 /tools/nav_bench/cost_sampler.py sample "$out/cost.csv" --every 1.0 &
sampler=$!
python3 /tools/nav_sim/trials.py "$scenario" "$out" "$reps" --ros-args -p use_sim_time:=true 2>&1 | tee "$out/trials.log"
kill $sampler 2>/dev/null || true
python3 /tools/nav_bench/cost_sampler.py summarize "$out/cost.csv" > "$out/cost_summary.txt" 2>&1 || true
echo done
