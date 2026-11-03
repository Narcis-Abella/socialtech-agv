# sourced by lio_sim.sh, sup_sim.sh, lat_sim.sh and clock_sim.sh (inside the `sim` image): the simulator with the realism models and the simulated Mid-360, no localization yet.
#   needs: $out; mounts /tools, optionally /navsrc (docker/ros_nav newer than the image's), SIM_SRC (hospital-agv-sim 'sim-realism'), /pat (ours.lvm, loyola.tris)
#   env: SPAWN ("-13.0 0.0 0.0" x y yaw), SEED (1), SIM_WAIT (30 s: the model falls 0.3 m at spawn and FAST-LIO2 must not take its gravity from that)
mkdir -p "$out"
trap 'kill $(jobs -p) 2> /dev/null || true' EXIT
if [ -d /navsrc ]; then   # socialtech_simreal / socialtech_livox_sim newer than the image's: built into an overlay
  mkdir -p /tmp/navws && (cd /tmp/navws && colcon build --base-paths /navsrc --packages-select socialtech_simreal socialtech_livox_sim --merge-install --install-base /tmp/nav_install > "$out/build_nav.log" 2>&1)
  source /tmp/nav_install/setup.bash
fi
if ! ros2 pkg prefix hospital-agv-sim > /dev/null 2>&1; then
  mkdir -p /tmp/ws/src && ln -sfn "${SIM_SRC:?hospital-agv-sim is not in the image: set SIM_SRC}" /tmp/ws/src/hospital-agv-sim
  (cd /tmp/ws && colcon build --merge-install --install-base /tmp/sim_install > "$out/build.log" 2>&1)
  source /tmp/sim_install/setup.bash
fi
read -r sx sy sw <<< "${SPAWN:--13.0 0.0 0.0}"
SIM="--ros-args -p use_sim_time:=true"
# the Mid-360 sits at the real robot's height (hospital-agv-sim LIDAR_Z); the body FAST-LIO2 and planar_odom work in is the IMU, extrinsic_T (-0.011, -0.02329, 0.04412) below it
export SIM_REALISM=1 SIM_LIO=1 LIDAR_Z=${LIDAR_Z:-0.57}
SENSOR_H=$(python3 -c "print(round($LIDAR_Z - 0.04412, 5))")
ros2 launch hospital-agv-sim simulation.launch.py headless:=true odom_source:=ground_truth spawn_x:=$sx spawn_y:=$sy spawn_yaw:=$sw > "$out/sim.log" 2>&1 &
ros2 run socialtech_simreal sim_base_model $SIM -p seed:=${SEED:-1} -p deadband:=${DEADBAND:-0.08} > "$out/base.log" 2>&1 &
ros2 run socialtech_simreal sim_wheel_odom $SIM -p seed:=${SEED:-1} > "$out/wheel.log" 2>&1 &
ros2 run socialtech_simreal sim_imu_model $SIM -p seed:=${SEED:-1} -p rate_hz:=200.0 -p acc_scale:=0.10197162 -p out_topic:=/livox/imu -p frame_id:=livox_frame > "$out/imu.log" 2>&1 &   # 1/9.80665: the Livox driver publishes g
sleep "${SIM_WAIT:-30}"
ros2 run socialtech_livox_sim livox_mid360_sim $SIM -p model_file:=/pat/ours.lvm -p sensor_xyz:="[0.0,0.0,$LIDAR_Z]" -p scene_file:=/pat/loyola.tris -p pose_topic:=/odom_gt_raw -p custom_topic:=/livox/lidar -p frame_id:=livox_frame -p seed:=${SEED:-1} -p jitter_std:=0.0005 > "$out/livox.log" 2>&1 &

# FAST-LIO2 parameters: the robot's edits of the upstream mid360.yaml (ops/nav_live.sh: gyr_cov 0.01, acc_cov 1.0, blind 0.1, filter_size_map 0.25, point_filter_num 2) -> $1
fastlio_yaml() {
  sed -e "s|pcd_save_en:.*|pcd_save_en: false|" -e "s|map_en:.*|map_en: false|" -e "s|path_en:.*|path_en: false|" \
      -e "s|^\( *\)gyr_cov:.*|\1gyr_cov: ${GYR_COV:-0.01}|" -e "s|^\( *\)acc_cov:.*|\1acc_cov: 1.0|" -e "s|^\( *\)blind:.*|\1blind: 0.1|" \
      -e "s|^\( *\)filter_size_map:.*|\1filter_size_map: ${FILTER_SIZE_MAP:-0.25}|" -e "s|^\( *\)point_filter_num:.*|\1point_filter_num: ${POINT_FILTER_NUM:-2}|" \
      "$(ros2 pkg prefix fast_lio)/share/fast_lio/config/mid360.yaml" > "$1"
}

# FAST-LIO2 + planar_odom + pointcloud_to_laserscan exactly as on the robot, on the simulated sensors; waits 8 s so FAST-LIO2 takes gravity from the still IMU
lio_nodes() {
  fastlio_yaml "$out/fastlio.yaml"
  ros2 run fast_lio fastlio_mapping --ros-args --params-file "$out/fastlio.yaml" -p use_sim_time:=true > "$out/fastlio.log" 2>&1 &
  ros2 run planar_odom planar_odom $SIM -p sensor_height:=$SENSOR_H -p level:="[0.0, 0.0, 0.0, 1.0]" > "$out/planar.log" 2>&1 &
  # The real /scan has surface jumps (the 3D band collapsed to 2D, furniture) and dropout (21-27 % outlier beams, 2-8 % dropout by range) that this clean world and its static scene do not produce:
  # SCAN_EXTRA=1 adds exactly those from the calibrated model (noise_scale 0: the Gaussian core already comes from the real chain). Off by default: calibrated on a furnished room, in this empty
  # world its "nearer" jumps are phantom obstacles around the robot (Nav2 completed 0 of 4 trials with it, 4 of 4 without). fid_sim.sh turns it on to compare with the calibration.
  local scan_out=/scan
  if [ "${SCAN_EXTRA:-0}" != 0 ]; then
    scan_out=/scan_chain
    ros2 run socialtech_simreal sim_scan_model $SIM -p seed:=${SEED:-1} -p noise_scale:=0.0 -p in_topic:=/scan_chain -p out_topic:=/scan > "$out/scan_extra.log" 2>&1 &
  fi
  ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args --params-file /tools/nav_bench/config/scan.yaml -p use_sim_time:=true -r cloud_in:=/cloud_registered_body -r scan:=$scan_out > "$out/scan.log" 2>&1 &
  sleep 8
}

# The 2D map the chain localizes on: made from the triangles the simulated Mid-360 sees, in the band /scan uses (the legacy loyola.pgm was surveyed with the simulator's 2D lidar
# and sits 0.1-0.3 m off what the chain sees: a constant offset of GN against the truth). Seeded at the spawn. Sets $CHAIN_MAP (yaml).
chain_map() {
  mkdir -p "$out"
  [ -f "$out/loyola_chain.yaml" ] || python3 /tools/sim_realism/tris_to_map.py /pat/loyola.tris "$out/loyola_chain" "${1:-$sx,$sy}" > "$out/map.log"
  CHAIN_MAP="$out/loyola_chain.yaml"
}

# The robot's own ops/nav_live.launch.py (driver off, sim time) with gn_loc and the supervisor over the simulator, 10 s to come up (sup_sim.sh, lat_sim.sh). PROD (normalised by the caller): viz:=false.
# mounts: docker/ros_loc/gn_loc at /locsrc (optional), ops/ at /ops, GRELOC_BUILD (register_v2, libsmall_gicp.so, libcorr_search.so) at /greloc/build
stack_up() {
  if [ -d /locsrc ]; then   # gn_loc newer than the image's (gn_loc/scan_fit, which the supervisor confirms with): overlay
    mkdir -p /tmp/locws && (cd /tmp/locws && colcon build --base-paths /locsrc --packages-select gn_loc --merge-install --install-base /tmp/loc_install > "$out/build_loc.log" 2>&1)
    source /tmp/loc_install/setup.bash
  fi
  fastlio_yaml "$out/fastlio.yaml"
  python3 /tools/sim_realism/tris_to_prior.py /pat/loyola.tris "$out/prior.xyz" 0.2 0.1 > "$out/prior.log"
  mkdir -p "$out/runs"
  export SUPERVISOR=1 SUP_PRIOR="$out/prior.xyz"
  chain_map; map=$CHAIN_MAP
  ros2 launch /ops/nav_live.launch.py livox_config:=/dev/null fastlio_params:="$out/fastlio.yaml" map:="$map" overrides:=/ops/live_overrides.yaml \
    sensor_height:=$SENSOR_H level_qx:=0.0 level_qy:=0.0 level_qz:=0.0 level_qw:=1.0 driver:=false sim_time:=true ${PROD:+viz:=false} > "$out/launch.log" 2>&1 &
  sleep 10
}
