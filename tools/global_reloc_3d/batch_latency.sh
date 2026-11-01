#!/usr/bin/env bash
# NX host: how long does the gated output stay non-zero after AMCL's pose has left 0.30 m / 8 deg? (note 22)
# eco01 c04, mode s2s3: LOCALIZED at ~28.6 s after the first input, the platform then drives on. Faults enter FAST-LIO2 through the relay
# AFTER LOCALIZED, at different strengths and moments; controls without a fault are part of the batch.
# Written before the runs:
#   - latency_s (permit_exposure.py) of every run that is revoked: expected <= 1.5 s, and never above 2.5 s = MAP_FRESH_S (2.0)
#     plus the worst gate closing time measured so far (0.41 s). A run above 2.5 s is a failure of the change.
#   - controls: LOCALIZED kept, withheld_in_doubt == 0 after the first second, no failed check.
#   - a fault FAST-LIO2 absorbs (pose stays inside the limits) may keep LOCALIZED: that is not a failure; latency_s must be 0.
# usage: R=<run root> batch_latency.sh TAG [name-prefix filter]
set -u
R=${R:?run root}; export R
B=/home/jetson/work/global-reloc-3d-20261009
BAG=/home/jetson/rosbags/custom/mapping_eco_-1_01_custom; YAML=/home/jetson/map_out/eco01_1x/map.yaml; XYZ=$B/isolated-v16/map/map.xyz; OFF=307.899419
REF=/home/jetson/map_out/eco01_1x/vs/run/alidarState.txt
T=$1; ONLY=${2:-}
A="s3_auto_authorize:=true"
ANY='LOCALIZED\(None\)/(LOCALIZED|LOCALIZATION_LOST)\(.*'            # the invariants and the exposure decide
LOST='LOCALIZED\(None\)/LOCALIZATION_LOST\(.*'
SAFE='(LOCALIZED|HANDOFF_FAILED|REJECTED|TIMEOUT|ERROR)\(.*'
EVAL="docker run --rm --network none -v $R/tools/global_reloc_3d:/t:ro -v $R/runs:/runs:ro -v $(dirname $REF):/refdir:ro -v $(dirname $YAML):/m:ro socialtech:mapper-verify python3 -B"
run() { name=$1; expect=$2; ctl=$3; evid=$4; shift 4; [ -n "$ONLY" ] && [[ $name != $ONLY* ]] && return
  env NAME=$name BAG=$BAG MAPYAML=$YAML MAPXYZ=$XYZ OFFSET=$OFF TAG=$T MODE=s2s3 LAUNCH_EXTRA="$A" "$@" bash $R/tools/global_reloc_3d/run_online.sh > /dev/null 2>&1
  d=$R/runs/$name-$T
  $EVAL /t/evaluate_online.py --run-dir /runs/$name-$T --reference /refdir/$(basename $REF) --map-yaml /m/$(basename $YAML) > $d/evaluation.json 2> $d/evaluation.err
  python3 -B $R/tools/global_reloc_3d/f2_check.py "$d" "$expect" $R/table-$T.csv "$evid" "$ctl"
  $EVAL /t/permit_exposure.py --run-dir /runs/$name-$T --reference /refdir/$(basename $REF) --map-yaml /m/$(basename $YAML) >> $R/exposure-$T.jsonl 2>> $R/exposure-$T.err; }
RL='--spec /livox/lidar,livox_ros_driver2/msg/CustomMsg,%s --spec /livox/imu,sensor_msgs/msg/Imu,%s'
relay() { printf -- "$RL" "$1" "$2"; }
REMAP='--remap /livox/lidar:=/livox/lidar_raw /livox/imu:=/livox/imu_raw'
FED="odom>50 and relay['lidar']['max_matched_subscribers']>=2 and relay['imu']['max_matched_subscribers']>=2"
ALT="$FED and relay['imu']['altered']>300"
imu() { name=$1; expect=$2; spec=$3; plays=${4:-80}; run $name "$expect" k40 "$ALT" KEEP_PLAYING=lost PLAY_S=$plays RELAY_SPEC="$(relay 0,0 0,0,$spec)" PLAY_ARGS_EXTRA="$REMAP"; }

# controls
run k00_nominal "${LOST%/*}/LOCALIZED\(None\)" - "odom>50" KEEP_PLAYING=1 PLAY_S=100 STOP_AT=80
run k40_relay_passthrough "${LOST%/*}/LOCALIZED\(None\)" - "$FED and all(relay[k]['forwarded']==relay[k]['received'] for k in relay)" KEEP_PLAYING=1 PLAY_S=100 STOP_AT=80 RELAY_SPEC="$(relay 0,0 0,0)" PLAY_ARGS_EXTRA="$REMAP"
# gyro bias 0.5 rad/s (the fault of h28) at five moments after LOCALIZED
imu k10_gyro_0p5_from_31 "$LOST" after=31,gyrobias=0.5
imu k11_gyro_0p5_from_35 "$LOST" after=35,gyrobias=0.5
imu k12_gyro_0p5_from_38p5 "$LOST" after=38.5,gyrobias=0.5
imu k13_gyro_0p5_from_42 "$LOST" after=42,gyrobias=0.5
imu k14_gyro_0p5_from_47 "$LOST" after=47,gyrobias=0.5 95
# strength
imu k20_gyro_0p3_from_35 "$ANY" after=35,gyrobias=0.3
imu k21_gyro_0p4_from_35 "$ANY" after=35,gyrobias=0.4
imu k22_gyro_0p7_from_35 "$LOST" after=35,gyrobias=0.7
imu k23_gyro_1p0_from_35 "$LOST" after=35,gyrobias=1.0
imu k24_gyro_2p0_from_35 "$LOST" after=35,gyrobias=2.0
imu k25_gyro_1p0_from_42 "$LOST" after=42,gyrobias=1.0
# other shapes: a transient, the accelerometer, a negative bias
imu k30_gyro_1p0_35_to_37_transient "$ANY" after=35,until=37,gyrobias=1.0
imu k31_acc_x2_from_35 "$ANY" after=35,accscale=2.0
imu k32_acc_x4_from_35 "$ANY" after=35,accscale=4.0
imu k33_gyro_m0p5_from_35 "$LOST" after=35,gyrobias=-0.5
# the cases of note 21 that the change touches: divergence that starts before LOCALIZED, a foreign pose, h27 itself
imu d08_gyro_0p5_from_21_after_pose "$SAFE" after=21,gyrobias=0.5
imu d07_gyro_0p5_from_18_at_handover "$SAFE" after=18,gyrobias=0.5
imu h27_lio_diverges_in_s2 '(ERROR|TIMEOUT|REJECTED|HANDOFF_FAILED)\(.*' after=4,gyrobias=0.5
run h32_amcl_moved_by_foreign_initialpose "$LOST" k00 "inj_rc==[0]" KEEP_PLAYING=lost PLAY_S=75 ROGUE_POSE_AT=35
# repeats of the reference case: the spread between identical runs
for i in 2 3; do imu k11_gyro_0p5_from_35_r$i "$LOST" after=35,gyrobias=0.5; done
