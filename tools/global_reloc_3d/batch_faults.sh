#!/usr/bin/env bash
# NX host: robustness cases on eco01 c04 (known-good window). Each: name, env overrides. Evaluator-free: the expectation is the terminal state/reason, and that no pose is published unless accepted.
set -u
R=/home/jetson/work/reloc-online-20261009; B=/home/jetson/work/global-reloc-3d-20261009; C=$B/claude-v1
BAG=/home/jetson/rosbags/custom/mapping_eco_-1_01_custom; YAML=/home/jetson/map_out/eco01_1x/map.yaml; XYZ=$B/isolated-v16/map/map.xyz; OFF=307.899419
T=$1
run() { name=$1; shift; env NAME=$name BAG=$BAG MAPYAML=$YAML MAPXYZ=$XYZ OFFSET=$OFF TAG=$T "$@" bash $R/tools/global_reloc_3d/run_online.sh > /dev/null 2>&1
  m=$(ls $R/runs/$name-$T/work/attempt*/metrics*.json 2>/dev/null | tail -1)
  echo "$name-$T $(python3 -c "import json;d=json.load(open('$m'));print(d['state'],d['reason'],'pose_published=',d['initialpose'] is not None,'t=',round(d['total_s']['attempt_end_from_first_input'],1))" 2>&1 | tail -1)"; }
RELAY='--spec /livox/lidar,livox_ros_driver2/msg/CustomMsg,%s,0 --spec /livox/imu,sensor_msgs/msg/Imu,%s,%s'
REMAP='--remap /livox/lidar:=/livox/lidar_raw /livox/imu:=/livox/imu_raw'
run f01_lidar_missing PLAY_ARGS_EXTRA="--topics /livox/imu"
run f02_imu_missing PLAY_ARGS_EXTRA="--topics /livox/lidar"
run f03_lidar_drop30 RELAY_SPEC="$(printf -- "$RELAY" 0.3 0 0)" PLAY_ARGS_EXTRA="$REMAP"
run f04_imu_drop30 RELAY_SPEC="$(printf -- "$RELAY" 0 0.3 0)" PLAY_ARGS_EXTRA="$REMAP"
run f05_imu_stamp_shift RELAY_SPEC="$(printf -- "$RELAY" 0 0 1000)" PLAY_ARGS_EXTRA="$REMAP"
run f06_short_acquisition LAUNCH_EXTRA="acquisition_limit_s:=5.0"
run f07_gicp_binary_false REGISTER_BIN=/bin/false
run f08_gicp_binary_missing REGISTER_BIN=/nonexistent
run f09_lio_exits LAUNCH_EXTRA="fastlio_exe:=/bin/false"
run f10_lio_hangs LAUNCH_EXTRA="fastlio_exe:=/tools/global_reloc_3d/hang_exe.sh"
run f11_amcl_absent LAUNCH_EXTRA="with_amcl:=false" PLAY_S=130
run f12_deadline25 LAUNCH_EXTRA="deadline_s:=25.0"
run f13_deadline40 LAUNCH_EXTRA="deadline_s:=40.0"
echo BATCH_DONE
