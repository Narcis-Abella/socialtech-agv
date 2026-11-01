#!/bin/bash
# Runs dyn_probe.py in the sim image: the dyn_hall world plus a dynamic and a kinematic copy of the walker. Called like run_trials.sh (agx_run.sh ENTRY=nav_sim/dyn_probe.sh): <scenario, unused> <out dir>   env: SIM_SRC
set -eo pipefail
out=${2:?out dir}; mkdir -p "$out"
source "${ROS_ENV:-/ros_env.sh}"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-53}
mkdir -p /tmp/ws/src && ln -sfn "${SIM_SRC:?}" /tmp/ws/src/hospital-agv-sim && cd /tmp/ws
colcon build --merge-install --install-base /tmp/sim_install > "$out/build.log" 2>&1
source /tmp/sim_install/setup.bash
python3 - <<'PY'
m = '<model name="{n}"><pose>12 {y} 0.85 0 0 0</pose><link name="link">{extra}<collision name="collision"><geometry><box><size>0.5 0.5 1.7</size></box></geometry></collision><visual name="visual"><geometry><box><size>0.5 0.5 1.7</size></box></geometry></visual></link></model>'
s = open("/tools/nav_sim/worlds/dyn_hall/world.sdf").read()
extra = m.format(n="b_wdyn", y=-32, extra="<gravity>false</gravity>") + m.format(n="b_wkin", y=-34, extra="<kinematic>true</kinematic>")
open("/tmp/probe_world.sdf", "w").write(s.replace("</world>", extra + "</world>"))
PY
ros2 launch hospital-agv-sim simulation.launch.py headless:=true world:=/tmp/probe_world.sdf spawn_x:=2.0 spawn_y:=0.0 spawn_yaw:=0.0 noise_seed:=1 > "$out/sim.log" 2>&1 &
trap 'kill $(jobs -p) 2> /dev/null || true' EXIT
sleep 45
cd /tools/nav_sim && python3 dyn_probe.py b_walker,b_wdyn,b_wkin --ros-args -p use_sim_time:=true 2>&1 | tee "$out/probe.log"
echo done
