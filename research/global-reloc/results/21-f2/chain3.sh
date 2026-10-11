#!/usr/bin/env bash
# candidate with input-stamp integrity (method files: see tools-sha256-f2c.txt)
export R=/home/jetson/work/reloc-f2c-20261010
cd $R
T=tools/global_reloc_3d
B=/home/jetson/work/global-reloc-3d-20261009
BAG=/home/jetson/rosbags/custom/mapping_eco_-1_01_custom; YAML=/home/jetson/map_out/eco01_1x/map.yaml; XYZ=$B/isolated-v16/map/map.xyz
bash $T/batch_faults_f2.sh f2c > batch-f2c.log 2>&1
bash $T/batch_faults_s2s3.sh g2c > batch-g2c.log 2>&1
for c in cases-f1 cases-f1b cases-sn; do bash $T/batch_s2s3.sh $c.csv r2c >> batch-r2c.log 2>&1; done
# note 15 relay cases (mode s3) with the corrected relay: frozen F1 method code, then this candidate
RELAY="--spec /livox/lidar,livox_ros_driver2/msg/CustomMsg,%s,0 --spec /livox/imu,sensor_msgs/msg/Imu,%s,%s"
REMAP="--remap /livox/lidar:=/livox/lidar_raw /livox/imu:=/livox/imu_raw"
O=/home/jetson/work/reloc-f2-20261010/f1ref
for root in $O $R; do
  R=$root NAME=f03_lidar_drop30 BAG=$BAG MAPYAML=$YAML MAPXYZ=$XYZ OFFSET=307.899419 TAG=fx RELAY_SPEC="$(printf -- "$RELAY" 0.3 0 0)" PLAY_ARGS_EXTRA="$REMAP" bash $root/tools/global_reloc_3d/run_online.sh > /dev/null 2>&1
  R=$root NAME=f05_imu_stamp_shift BAG=$BAG MAPYAML=$YAML MAPXYZ=$XYZ OFFSET=307.899419 TAG=fx RELAY_SPEC="$(printf -- "$RELAY" 0 0 1000)" PLAY_ARGS_EXTRA="$REMAP" bash $root/tools/global_reloc_3d/run_online.sh > /dev/null 2>&1
done
R=$R NAME=mem-eco01-c00 BAG=$BAG MAPYAML=$YAML MAPXYZ=$XYZ OFFSET=3.399710 TAG=m2 MODE=s2s3 KEEP_PLAYING=1 PLAY_S=515 LAUNCH_EXTRA="s3_auto_authorize:=true" bash $R/tools/global_reloc_3d/run_online.sh > /dev/null 2>&1
touch chain3a.done
bash $T/batch_restarts.sh cases-restarts.csv rs2 0 199 > batch-rs2.log 2>&1
touch chain3.done
