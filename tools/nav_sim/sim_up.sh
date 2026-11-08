# sourced by lio_sim.sh, sup_sim.sh, lat_sim.sh and clock_sim.sh (inside the `sim` image): the simulator with the realism models and the simulated Mid-360, no localization yet.
#   needs: $out; mounts /tools, optionally /src (the checkout's src/, newer than the image's), SIM_SRC (hospital-agv-sim 'sim-realism'), /pat (ours.lvm, loyola.tris)
#   env: SPAWN ("-13.0 0.0 0.0" x y yaw), SEED (1), SIM_WAIT (30 s: the model falls 0.3 m at spawn and FAST-LIO2 must not take its gravity from that)
mkdir -p "$out"
trap 'kill $(jobs -p) 2> /dev/null || true' EXIT
# src_overlay NAME PKG...: those of OUR packages from the mounted /src (the checkout's src/) built into an overlay /tmp/NAME_install and sourced: the run tests the checkout, not the image's copy.
# OVERLAY_CMAKE="--cmake-args ..." adds CMake arguments. Used here and by the scripts that source this one.
src_overlay() {
  local name=$1; shift
  mkdir -p "/tmp/ws_$name" && (cd "/tmp/ws_$name" && colcon build --base-paths /src --packages-select "$@" --merge-install --install-base "/tmp/${name}_install" ${OVERLAY_CMAKE:-} > "$out/build_$name.log" 2>&1) || return 1
  source "/tmp/${name}_install/setup.bash"
}
if [ -d /src ]; then   # socialtech_simreal / socialtech_livox_sim / socialtech_costmap / planar_odom newer than the image's
  src_overlay ours socialtech_simreal socialtech_livox_sim socialtech_costmap planar_odom socialtech_bringup
fi
# The robot's own run scripts and defaults (the image's, or the overlay of /src): flags (normalize_flags), the FAST-LIO2 parameter edits (fastlio_yaml), the launch files and config.
ros2 pkg prefix socialtech_bringup >/dev/null 2>&1 || { echo "sim_up: the image has no socialtech_bringup (built before it existed): rebuild the sim image, or mount the checkout's src/ at /src" >&2; exit 1; }
BRINGUP=$(ros2 pkg prefix socialtech_bringup)/lib/socialtech_bringup; BRINGUP_SHARE=$(ros2 pkg prefix socialtech_bringup)/share/socialtech_bringup
source "$BRINGUP/flags.sh"; source "$BRINGUP/fastlio_config.sh"
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

# FAST-LIO2 + planar_odom + pointcloud_to_laserscan exactly as on the robot, on the simulated sensors; waits 8 s so FAST-LIO2 takes gravity from the still IMU
lio_nodes() {
  fastlio_yaml "$out/fastlio.yaml"
  ros2 run fast_lio fastlio_mapping --ros-args --params-file "$out/fastlio.yaml" -p use_sim_time:=true > "$out/fastlio.log" 2>&1 &
  ros2 run planar_odom planar_odom $SIM -p sensor_height:=$SENSOR_H -p level:="[0.0, 0.0, 0.0, 1.0]" -p lidar_in_body:="[-0.011, -0.02329, 0.04412]" > "$out/planar.log" 2>&1 &
  # The real /scan has surface jumps (the 3D band collapsed to 2D, furniture) and dropout (21-27 % outlier beams, 2-8 % dropout by range) that this clean world and its static scene do not produce:
  # SCAN_EXTRA=1 adds exactly those from the calibrated model (noise_scale 0: the Gaussian core already comes from the real chain). Off by default: calibrated on a furnished room, in this empty
  # world its "nearer" jumps are phantom obstacles around the robot (Nav2 completed 0 of 4 trials with it, 4 of 4 without). fid_sim.sh turns it on to compare with the calibration.
  local scan_out=/scan
  if [ "${SCAN_EXTRA:-0}" != 0 ]; then
    scan_out=/scan_chain
    ros2 run socialtech_simreal sim_scan_model $SIM -p seed:=${SEED:-1} -p noise_scale:=0.0 -p in_topic:=/scan_chain -p out_topic:=/scan > "$out/scan_extra.log" 2>&1 &
  fi
  ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args --params-file $BRINGUP_SHARE/config/scan.yaml -p use_sim_time:=true -p target_frame:=lidar_level -p min_height:=$(python3 -c "print(0.10 - $LIDAR_Z)") -p max_height:=$(python3 -c "print(1.80 - $LIDAR_Z)") -r cloud_in:=/cloud_registered_body -r scan:=$scan_out > "$out/scan.log" 2>&1 &
  sleep 8
}

# The 2D map the chain localizes on: made from the triangles the simulated Mid-360 sees, in the band /scan uses (the legacy loyola.pgm was surveyed with the simulator's 2D lidar
# and sits 0.1-0.3 m off what the chain sees: a constant offset of GN against the truth). Seeded at the spawn. Sets $CHAIN_MAP (yaml).
chain_map() {
  mkdir -p "$out"
  [ -f "$out/loyola_chain.yaml" ] || python3 /tools/sim_realism/tris_to_map.py /pat/loyola.tris "$out/loyola_chain" "${1:-$sx,$sy}" > "$out/map.log"
  CHAIN_MAP="$out/loyola_chain.yaml"
}

# The robot's own nav_live.launch.py (socialtech_bringup; driver off, sim time) with gn_loc and the supervisor over the simulator, 10 s to come up (sup_sim.sh, lat_sim.sh). PROD (normalised by the caller): viz:=false.
# mounts: src/ at /src (optional)
stack_up() {
  if [ -d /src ]; then   # gn_loc newer than the image's (gn_loc/scan_fit, which the supervisor confirms with): overlay
    src_overlay loc gn_loc
    OVERLAY_CMAKE="--cmake-args -DGN_SUPERVISOR_PYTHON_ONLY=ON" src_overlay sup gn_supervisor   # the Python of the checkout; the worker's binaries stay the image's (the launch finds them along the ament path)
  fi
  fastlio_yaml "$out/fastlio.yaml"
  python3 /tools/sim_realism/tris_to_prior.py /pat/loyola.tris "$out/prior.xyz" 0.2 0.1 > "$out/prior.log"
  mkdir -p "$out/runs"
  export SUPERVISOR=1 SUP_PRIOR="$out/prior.xyz"
  chain_map; map=$CHAIN_MAP
  ros2 launch socialtech_bringup nav_live.launch.py livox_config:=/dev/null fastlio_params:="$out/fastlio.yaml" map:="$map" \
    sensor_height:=$SENSOR_H level_qx:=0.0 level_qy:=0.0 level_qz:=0.0 level_qw:=1.0 driver:=false sim_time:=true ${PROD:+viz:=false} > "$out/launch.log" 2>&1 &
  sleep 10
}
