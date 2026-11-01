#!/bin/bash
# The simulated robot with a REAL FAST-LIO2 in the loop, inside the `sim` image on a board. usage: lio_sim.sh <out dir> [drive args: leg_m v w]
#   Sensors: socialtech_livox_sim (non-repetitive Mid-360 from the ground-truth pose, CustomMsg /livox/lidar) and sim_imu_model (ICM-40609 noise, in g, 200 Hz, /livox/imu, frame livox_frame);
#   base/wheel models of socialtech_simreal; FAST-LIO2 with the robot's own parameter edits (ops/nav_live.sh); planar_odom (odom -> base_footprint, level = identity: the simulated floor is level);
#   /scan from scan_median on /cloud_registered_body as on the robot.  Result: out/lio.csv (FAST-LIO2 vs truth) and lio_eval.py report.
# mounts: tools/ at /tools, optionally docker/ros_nav at /navsrc, SIM_SRC (hospital-agv-sim, branch sim-realism) at /simsrc, PAT (dir with ours.lvm = the unit's pattern model and loyola.tris) at /pat, out at /out.
# env: SPAWN ("-13.0 0.0 0.0" x y yaw), ROS_DOMAIN_ID (54), SIM_WAIT (30), DRIVE ("5 0.4 0.5"), SEED (1)
set -eo pipefail
out=${1:?out dir}; shift || true
source "${ROS_ENV:-/ros_env.sh}"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-54}
source /tools/nav_sim/sim_up.sh
fastlio_yaml "$out/fastlio.yaml"
ros2 run fast_lio fastlio_mapping --ros-args --params-file "$out/fastlio.yaml" -p use_sim_time:=true > "$out/fastlio.log" 2>&1 &
ros2 run planar_odom planar_odom $SIM -p sensor_height:=0.28 -p level:="[0.0, 0.0, 0.0, 1.0]" > "$out/planar.log" 2>&1 &
ros2 run scan_median scan_median_node --ros-args --params-file /tools/nav_bench/config/scan.yaml -p quantile:=0.0 -p use_sim_time:=true -r cloud_in:=/cloud_registered_body -r scan:=/scan > "$out/scan.log" 2>&1 &
sleep 8   # FAST-LIO2 takes gravity from the first IMU samples: the robot stays still until the drive starts
python3 /tools/nav_sim/lio_eval.py record "$out/lio.csv" 80 > "$out/rec.log" 2>&1 &
rec=$!
python3 /tools/nav_sim/lio_drive.py ${DRIVE:-5 0.4 0.5} > "$out/drive.log" 2>&1
sleep 3; kill $rec 2>/dev/null || true; wait $rec 2>/dev/null || true
python3 /tools/nav_sim/lio_eval.py report "$out/lio.csv" | tee "$out/report.txt"
