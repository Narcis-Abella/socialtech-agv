#!/bin/bash
# Evaluation of Nav2 in the simulation, inside the `sim` image on the board. usage: run_eval.sh <out dir> <repetitions>
#   mounts: the repo tools/ at /tools, the output dir at /out.   env: PARAMS (a nav2_params.yaml instead of the packaged one), SIM_SRC (a hospital-agv-sim checkout to build when the image has none), SIM_ARGS (extra simulation.launch.py arguments, e.g. odom_source:=ground_truth), ROS_DOMAIN_ID (53)
set -eo pipefail
out=${1:?out dir}; reps=${2:-3}
source /ros_env.sh
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-53}
if ! ros2 pkg prefix hospital-agv-sim > /dev/null 2>&1; then
  mkdir -p /tmp/ws/src && ln -sfn "${SIM_SRC:?hospital-agv-sim is not in the image: set SIM_SRC}" /tmp/ws/src/hospital-agv-sim && cd /tmp/ws
  colcon build --merge-install --install-base /tmp/sim_install > "$out/build.log" 2>&1
  source /tmp/sim_install/setup.bash
fi
map=$(ros2 pkg prefix socialtech_nav)/share/socialtech_nav/maps/loyola.yaml
ros2 launch hospital-agv-sim simulation.launch.py headless:=true $SIM_ARGS > "$out/sim.log" 2>&1 &
sleep 45
ros2 launch socialtech_nav nav_sim.launch.py ${PARAMS:+params_file:=$PARAMS} > "$out/nav.log" 2>&1 &
until [ "$(ros2 lifecycle get /bt_navigator 2>/dev/null | tail -1)" = "active [3]" ]; do sleep 2; done
sleep 5
ps -eo pid,args | grep -E 'nav2|opennav|gz|noisy|bridge' | grep -v grep | cut -c1-160 > "$out/ps.txt"
python3 /tools/nav_bench/cost_sampler.py sample "$out/cost.csv" --every 1.0 &
sampler=$!
for r in $(seq 1 "$reps"); do
  python3 /tools/nav_sim/eval_goals.py /tools/nav_sim/goals.yaml "$map" "$out/run$r.json" --ros-args -p use_sim_time:=true > "$out/run$r.log" 2>&1 || echo "run $r failed (see run$r.log)"
done
kill $sampler 2>/dev/null || true
python3 /tools/nav_bench/cost_sampler.py summarize "$out/cost.csv" > "$out/cost_summary.txt" 2>&1 || true
echo done
