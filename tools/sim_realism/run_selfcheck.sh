#!/bin/bash
# inside the sim-nav2 image: build socialtech_simreal (overlay /ws) and the simulator (/sim), run the realism models in the loop and the two self-checks
source /ros_env.sh; export ROS_DOMAIN_ID=88 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
mkdir -p /out && cd /ws && colcon build --packages-select socialtech_simreal --cmake-args -DBUILD_TESTING=ON > /out/build_simreal.log 2>&1; echo "simreal build rc=$?"
colcon test --packages-select socialtech_simreal --event-handlers console_direct+ 2>&1 | grep -E "PASSED|FAILED|tests passed" | head -3
source /ws/install/setup.bash
mkdir -p /tmp/wsim/src && ln -sfn /sim /tmp/wsim/src/hospital-agv-sim && cd /tmp/wsim && colcon build --merge-install --install-base /tmp/sim_install > /out/build_sim.log 2>&1; echo "sim build rc=$?"; source /tmp/sim_install/setup.bash
export SIM_REALISM=1
ros2 launch hospital-agv-sim simulation.launch.py headless:=true odom_source:=ground_truth ${WORLD:+world:=$WORLD} ${SPAWN:-} > /out/sim.log 2>&1 &
ros2 launch socialtech_simreal sim_realism.launch.py seed:=7 > /out/realism.log 2>&1 &
sleep 50; ros2 topic list | grep -E "^/(cmd_vel|cmd_vel_gz|odom|odom_lio|odom_gt_raw|scan|scan_raw|imu/data|imu_raw)$" | tr "\n" " "; echo
grep -h "model:\|wheel odometry:\|scan model" /out/realism.log | head -4
echo "=== sim_selfcheck, Gazebo alone (/cmd_vel_gz)"; CMD_TOPIC=/cmd_vel_gz python3 /tools/sim_realism/sim_selfcheck.py /tools/sim_realism/calibration.json 2>&1 | grep -E "real-time|linear|angular moving"
echo "=== sim_selfcheck, whole chain (/cmd_vel)"; python3 /tools/sim_realism/sim_selfcheck.py /tools/sim_realism/calibration.json 2>&1 | tail -16
if [ -z "$SKIP_SCAN" ]; then echo "=== scan_selfcheck (40 s, robot still)"; python3 /tools/sim_realism/scan_selfcheck.py 40 /tools/sim_realism/calibration.json 2>&1 | tail -14; fi
kill $(jobs -p) 2>/dev/null; sleep 2
