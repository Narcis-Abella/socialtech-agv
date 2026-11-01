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
mkdir -p "$out"
trap 'kill $(jobs -p) 2> /dev/null || true' EXIT
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-54}
if ! ros2 pkg prefix hospital-agv-sim > /dev/null 2>&1; then
  mkdir -p /tmp/ws/src && ln -sfn "${SIM_SRC:?hospital-agv-sim is not in the image: set SIM_SRC}" /tmp/ws/src/hospital-agv-sim && cd /tmp/ws
  colcon build --merge-install --install-base /tmp/sim_install > "$out/build.log" 2>&1
  source /tmp/sim_install/setup.bash
fi
if [ -d /navsrc ]; then   # socialtech_simreal / socialtech_livox_sim newer than the image's: built into an overlay
  mkdir -p /tmp/navws && cd /tmp/navws
  colcon build --base-paths /navsrc --packages-select socialtech_simreal --merge-install --install-base /tmp/nav_install > "$out/build_nav.log" 2>&1
  source /tmp/nav_install/setup.bash
fi
read -r sx sy sw <<< "${SPAWN:--13.0 0.0 0.0}"
SIM="--ros-args -p use_sim_time:=true"
export SIM_REALISM=1 SIM_LIO=1
ros2 launch hospital-agv-sim simulation.launch.py headless:=true odom_source:=ground_truth spawn_x:=$sx spawn_y:=$sy spawn_yaw:=$sw > "$out/sim.log" 2>&1 &
ros2 run socialtech_simreal sim_base_model $SIM -p seed:=${SEED:-1} > "$out/base.log" 2>&1 &
ros2 run socialtech_simreal sim_wheel_odom $SIM -p seed:=${SEED:-1} > "$out/wheel.log" 2>&1 &
ros2 run socialtech_simreal sim_imu_model $SIM -p seed:=${SEED:-1} -p rate_hz:=200.0 -p acc_scale:=0.10197162 -p out_topic:=/livox/imu -p frame_id:=livox_frame > "$out/imu.log" 2>&1 &   # 1/9.80665: the Livox driver publishes g
sleep "${SIM_WAIT:-30}"   # the model is spawned 0.3 m above the floor and falls: FAST-LIO2 would take gravity from the fall (the robot on the floor starts at rest)
ros2 run socialtech_livox_sim livox_mid360_sim $SIM -p model_file:=/pat/ours.lvm -p scene_file:=/pat/loyola.tris -p pose_topic:=/odom_gt_raw -p custom_topic:=/livox/lidar -p frame_id:=livox_frame -p seed:=${SEED:-1} -p jitter_std:=0.0005 > "$out/livox.log" 2>&1 &
# FAST-LIO2: the robot's edits of the upstream mid360.yaml (ops/nav_live.sh: gyr_cov 0.01, acc_cov 1.0, blind 0.1, filter_size_map 0.25, point_filter_num 2)
sed -e "s|pcd_save_en:.*|pcd_save_en: false|" -e "s|map_en:.*|map_en: false|" -e "s|path_en:.*|path_en: false|" \
    -e "s|^\( *\)gyr_cov:.*|\1gyr_cov: ${GYR_COV:-0.01}|" -e "s|^\( *\)acc_cov:.*|\1acc_cov: 1.0|" -e "s|^\( *\)blind:.*|\1blind: 0.1|" \
    -e "s|^\( *\)filter_size_map:.*|\1filter_size_map: ${FILTER_SIZE_MAP:-0.25}|" -e "s|^\( *\)point_filter_num:.*|\1point_filter_num: ${POINT_FILTER_NUM:-2}|" \
    "$(ros2 pkg prefix fast_lio)/share/fast_lio/config/mid360.yaml" > "$out/fastlio.yaml"
ros2 run fast_lio fastlio_mapping --ros-args --params-file "$out/fastlio.yaml" -p use_sim_time:=true > "$out/fastlio.log" 2>&1 &
ros2 run planar_odom planar_odom $SIM -p sensor_height:=0.28 -p level:="[0.0, 0.0, 0.0, 1.0]" > "$out/planar.log" 2>&1 &
ros2 run scan_median scan_median_node --ros-args --params-file /tools/nav_bench/config/scan.yaml -p quantile:=0.0 -p use_sim_time:=true -r cloud_in:=/cloud_registered_body -r scan:=/scan > "$out/scan.log" 2>&1 &
sleep 8   # FAST-LIO2 takes gravity from the first IMU samples: the robot stays still until the drive starts
python3 /tools/nav_sim/lio_eval.py record "$out/lio.csv" 80 > "$out/rec.log" 2>&1 &
rec=$!
python3 /tools/nav_sim/lio_drive.py ${DRIVE:-5 0.4 0.5} > "$out/drive.log" 2>&1
sleep 3; kill $rec 2>/dev/null || true; wait $rec 2>/dev/null || true
python3 /tools/nav_sim/lio_eval.py report "$out/lio.csv" | tee "$out/report.txt"
