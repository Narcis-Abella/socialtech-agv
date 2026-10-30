#!/bin/bash
# Runs a scenario file against Nav2 in the simulation, inside the `sim` image on the board. usage: run_trials.sh <scenario.json> <out dir> [reps]
#   mounts: the repo tools/ at /tools, the output dir at /out.   env: PARAMS (nav2_params.yaml instead of the packaged one), MERGE (overlay yamls, space-separated, merged into it first in order, may add plugin blocks), SET (comma-separated path=value overrides of it, e.g. "amcl.ros__parameters.alpha1=0.05,amcl.ros__parameters.alpha2=0.05"), MAP (map yaml instead of the packaged loyola.yaml), SIM_SRC (hospital-agv-sim checkout to build),
#   SIM_REALISM=1 (realism models of socialtech_simreal; REALISM_SEED, REALISM_ARGS), SIM_ARGS (extra simulation.launch.py arguments: world:=, spawn_x:=, noise_seed:=, odom_source:= ...), ROS_DOMAIN_ID (53), SIM_WAIT (45 s), NAV_TIMEOUT (180 s), EXTRA (command started after Nav2 is active), plus RUNNER (nav_sim/trials.py; nav_sim/plan_bench.py for the planner benchmark) and the env of the runner
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
[ -n "${SIM_REALISM:-}" ] && export SIM_REALISM=1   # hospital-agv-sim 'sim-realism': hands /cmd_vel, /odom, /scan and /imu/data to socialtech_simreal (use SIM_ARGS "odom_source:=ground_truth")
ros2 launch hospital-agv-sim simulation.launch.py headless:=true $SIM_ARGS > "$out/sim.log" 2>&1 &
[ -z "${SIM_REALISM:-}" ] || ros2 launch socialtech_simreal sim_realism.launch.py seed:=${REALISM_SEED:--1} $REALISM_ARGS > "$out/realism.log" 2>&1 &   # REALISM_ARGS: outlier_scale:=0 noise_scale:=1 kick:=0.05 deadband:=0.1
sleep "${SIM_WAIT:-45}"
params=${PARAMS:-$(ros2 pkg prefix socialtech_nav)/share/socialtech_nav/config/nav2_params.yaml}
if [ -n "$MERGE" ]; then   # overlay that may add plugin blocks (params_merge.py), applied before SET
  i=0
  for f in $MERGE; do   # several overlays, space-separated, applied in order
    i=$((i + 1))
    python3 /tools/nav_sim/params_merge.py "$params" "$f" "$out/params_merged$i.yaml"
    params=$out/params_merged$i.yaml
  done
fi
if [ -n "$SET" ]; then   # parameter variant: comma-separated path=value pairs applied to the base file (params_variant.py keeps types and refuses unknown keys)
  python3 /tools/nav_sim/params_variant.py "$params" "$out/params_variant.yaml" ${SET//,/ }
  params=$out/params_variant.yaml
fi
ros2 launch socialtech_nav nav_sim.launch.py params_file:=$params ${MAP:+map:=$MAP} > "$out/nav.log" 2>&1 &
if ! timeout "${NAV_TIMEOUT:-180}" bash -c 'until [ "$(ros2 lifecycle get /bt_navigator 2>/dev/null | tail -1)" = "active [3]" ]; do sleep 2; done'; then
  echo "bt_navigator not active after ${NAV_TIMEOUT:-180} s: see $out/sim.log and $out/nav.log" >&2
  exit 1
fi
sleep 5
[ -z "$EXTRA" ] || { eval "$EXTRA" > "$out/extra.log" 2>&1 & }   # EXTRA: a command started once Nav2 is active (a helper node under test, e.g. speed_guard.py); the trap at the top stops it
ros2 param list /controller_server > "$out/declared_controller.txt" 2>&1   # to check that an overlay's keys are parameters of the plugin
ps -eo pid,args | grep -E 'nav2|opennav|gz|noisy|bridge' | grep -v grep | cut -c1-160 > "$out/ps.txt"
python3 /tools/nav_bench/cost_sampler.py sample "$out/cost.csv" --every 1.0 &
sampler=$!
python3 /tools/${RUNNER:-nav_sim/trials.py} "$scenario" "$out" "$reps" --ros-args -p use_sim_time:=true 2>&1 | tee "$out/trials.log"
kill $sampler 2>/dev/null || true
python3 /tools/nav_bench/cost_sampler.py summarize "$out/cost.csv" > "$out/cost_summary.txt" 2>&1 || true
echo done
