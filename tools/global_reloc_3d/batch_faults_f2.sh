#!/usr/bin/env bash
# NX host: Phase-2 fault matrix in mode s2s3 on eco01 c04 (nominal: S2 accepts at ~19 s, LOCALIZED at ~24 s, then the robot drives on).
# Every case carries the outcome expected BEFORE it was run (regex on ATTEMPT(reason)/SUPERVISION(reason)); f2_check.py adds the
# invariants (pose only after ACCEPTED, gate output only while LOCALIZED, bounded close time, orphans, evaluator class).
# Times are seconds after the bag starts (first sensor input is within ~0.1 s of it).
# usage: R=<run root> batch_faults_f2.sh TAG [name-prefix filter]
set -u
R=${R:?run root}; export R
B=/home/jetson/work/global-reloc-3d-20261009
BAG=/home/jetson/rosbags/custom/mapping_eco_-1_01_custom; YAML=/home/jetson/map_out/eco01_1x/map.yaml; XYZ=$B/isolated-v16/map/map.xyz; OFF=307.899419
REF=/home/jetson/map_out/eco01_1x/vs/run/alidarState.txt
T=$1; ONLY=${2:-}
A="s3_auto_authorize:=true"
SAFE='(LOCALIZED|HANDOFF_FAILED|REJECTED|TIMEOUT|ERROR)\(.*'          # any terminal state; the invariants decide
LOC='LOCALIZED\(None\)/'
run() { name=$1; expect=$2; shift 2; [ -n "$ONLY" ] && [[ $name != $ONLY* ]] && return
  env NAME=$name BAG=$BAG MAPYAML=$YAML MAPXYZ=$XYZ OFFSET=$OFF TAG=$T MODE=s2s3 LAUNCH_EXTRA="$A" "$@" bash $R/tools/global_reloc_3d/run_online.sh > /dev/null 2>&1
  d=$R/runs/$name-$T
  docker run --rm --network none -v $R/tools/global_reloc_3d:/t:ro -v $R/runs:/runs:ro -v "$(dirname $REF)":/refdir:ro -v "$(dirname $YAML)":/m:ro socialtech:mapper-verify \
    python3 -B /t/evaluate_online.py --run-dir /runs/$name-$T --reference /refdir/$(basename $REF) --map-yaml /m/$(basename $YAML) > $d/evaluation.json 2> $d/evaluation.err
  python3 -B $R/tools/global_reloc_3d/f2_check.py "$d" "$expect" $R/table-$T.csv; }
RL='--spec /livox/lidar,livox_ros_driver2/msg/CustomMsg,%s --spec /livox/imu,sensor_msgs/msg/Imu,%s'
relay() { printf -- "$RL" "$1" "$2"; }
REMAP='--remap /livox/lidar:=/livox/lidar_raw /livox/imu:=/livox/imu_raw'
POST="KEEP_PLAYING=lost PLAY_S=75"       # run on after LOCALIZED until it is revoked or 75 s of bag
EPOCH=-1780000000                        # header stamps fall back to a boot-relative sensor clock

run h00_nominal_supervised "${LOC}LOCALIZED\(None\)" KEEP_PLAYING=1 PLAY_S=100 STOP_AT=70
# --- sensors
run h01_no_sensors_no_clock 'TIMEOUT\(no_sensor_input\)' NO_PLAY=1 PLAY_S=70
run h02_no_sensors_clock_only 'TIMEOUT\(no_sensor_input\)' PLAY_ARGS_EXTRA="--remap /livox/lidar:=/x/lidar /livox/imu:=/x/imu"
run h03_all_input_stops_in_imu_gate 'TIMEOUT\(imu_not_stationary\)' KILL_PLAY_AT=1 KILL_PLAY_SIG=STOP
run h04_all_input_stops_in_s2 'ERROR\(lio_stalled\)' KILL_PLAY_AT=5 KILL_PLAY_SIG=STOP
run h05_input_gap_2s_in_s2 "$SAFE" KILL_PLAY_AT=5 KILL_PLAY_SIG=STOP CONT_PLAY_AT=7
run h06_input_gap_10s_in_s2_then_back 'ERROR\(lio_stalled\)' KILL_PLAY_AT=5 KILL_PLAY_SIG=STOP CONT_PLAY_AT=15 KEEP_PLAYING=1 PLAY_S=100 STOP_AT=40
run h07_input_stops_after_localized "${LOC}LOCALIZATION_LOST\(odometry_stalled\)" $POST KILL_PLAY_AT=35 KILL_PLAY_SIG=STOP
run h08_input_gap_10s_after_localized "${LOC}LOCALIZATION_LOST\(odometry_stalled\)" KEEP_PLAYING=1 PLAY_S=100 STOP_AT=60 KILL_PLAY_AT=35 KILL_PLAY_SIG=STOP CONT_PLAY_AT=45
run h09_lidar_blackout_2s_in_s2 "$SAFE" RELAY_SPEC="$(relay 1,0,after=5,until=7 0,0)" PLAY_ARGS_EXTRA="$REMAP"
run h10_imu_lost_after_localized "${LOC}LOCALIZATION_LOST\(.*" $POST RELAY_SPEC="$(relay 0,0 1,0,after=35)" PLAY_ARGS_EXTRA="$REMAP"
run h11_lidar_lost_after_localized "${LOC}LOCALIZATION_LOST\(odometry_stalled\)" $POST RELAY_SPEC="$(relay 1,0,after=35 0,0)" PLAY_ARGS_EXTRA="$REMAP"
# --- time stamps
run h12_imu_duplicated_20pct "$SAFE" $POST RELAY_SPEC="$(relay 0,0 0,0,dup=0.2)" PLAY_ARGS_EXTRA="$REMAP"
run h13_imu_reversed_5pct "$SAFE" $POST RELAY_SPEC="$(relay 0,0 0,0,swap=0.05)" PLAY_ARGS_EXTRA="$REMAP"
run h14_lidar_duplicated_20pct "$SAFE" $POST RELAY_SPEC="$(relay 0,0,dup=0.2 0,0)" PLAY_ARGS_EXTRA="$REMAP"
run h15_lidar_reversed_20pct "$SAFE" $POST RELAY_SPEC="$(relay 0,0,swap=0.2 0,0)" PLAY_ARGS_EXTRA="$REMAP"
run h16_imu_epoch_jump_in_s2 '(ERROR|TIMEOUT|REJECTED)\(.*' RELAY_SPEC="$(relay 0,0 0,$EPOCH,after=5)" PLAY_ARGS_EXTRA="$REMAP"
run h17_both_epoch_jump_in_s2 '(ERROR|TIMEOUT|REJECTED)\(.*' RELAY_SPEC="$(relay 0,$EPOCH,after=5 0,$EPOCH,after=5)" PLAY_ARGS_EXTRA="$REMAP"
run h18_both_epoch_jump_in_imu_gate "$SAFE" RELAY_SPEC="$(relay 0,$EPOCH,after=1 0,$EPOCH,after=1)" PLAY_ARGS_EXTRA="$REMAP"
run h19_both_epoch_jump_after_localized "${LOC}LOCALIZATION_LOST\(.*" $POST RELAY_SPEC="$(relay 0,$EPOCH,after=35 0,$EPOCH,after=35)" PLAY_ARGS_EXTRA="$REMAP"
run h20_imu_stamp_plus_1000s_after_localized "${LOC}LOCALIZATION_LOST\(.*" $POST RELAY_SPEC="$(relay 0,0 0,1000,after=35)" PLAY_ARGS_EXTRA="$REMAP"
# --- /clock
run h21_clock_back_in_s2_search "$SAFE" CLOCK_JUMP_AT=8 CLOCK_JUMP_SEC=1000
run h22_clock_forward_in_s2_search "$SAFE" CLOCK_JUMP_AT=8 CLOCK_JUMP_SEC=2000000000
run h23_clock_back_after_localized "${LOC}.*" $POST CLOCK_JUMP_AT=35 CLOCK_JUMP_SEC=1000
run h24_clock_forward_after_localized "${LOC}.*" $POST CLOCK_JUMP_AT=35 CLOCK_JUMP_SEC=2000000000
# --- FAST-LIO2
run h25_lio_killed_after_localized "${LOC}LOCALIZATION_LOST\(fastlio_exited_-9\)" $POST KILL_LIO_AT=35 KILL_LIO_SIG=KILL
run h26_lio_frozen_after_localized "${LOC}LOCALIZATION_LOST\(odometry_stalled\)" $POST KILL_LIO_AT=35 KILL_LIO_SIG=STOP
run h27_lio_diverges_in_s2 '(ERROR|TIMEOUT|REJECTED|HANDOFF_FAILED)\(.*' RELAY_SPEC="$(relay 0,0 0,0,after=4,gyrobias=0.5)" PLAY_ARGS_EXTRA="$REMAP"
run h28_lio_diverges_after_localized "${LOC}LOCALIZATION_LOST\(.*" $POST RELAY_SPEC="$(relay 0,0 0,0,after=35,gyrobias=0.5)" PLAY_ARGS_EXTRA="$REMAP"
run h29_lio_acc_x3_after_localized "${LOC}LOCALIZATION_LOST\(.*" $POST RELAY_SPEC="$(relay 0,0 0,0,after=35,accscale=3)" PLAY_ARGS_EXTRA="$REMAP"
# --- AMCL
run h30_amcl_killed_after_localized "${LOC}LOCALIZATION_LOST\(amcl_silent\)" $POST KILL_AMCL_AT=35
run h31_amcl_frozen_after_localized "${LOC}LOCALIZATION_LOST\(amcl_silent\)" $POST KILL_AMCL_AT=35 KILL_AMCL_SIG=STOP
run h32_amcl_moved_by_foreign_initialpose "${LOC}LOCALIZATION_LOST\(amcl_incoherent\)" $POST ROGUE_POSE_AT=35
run h33_amcl_killed_in_handoff 'HANDOFF_FAILED\(amcl_not_confirmed\)' KILL_AMCL_AT=21 PLAY_S=130
# --- supervisor and gate
run h34_node_killed_after_localized "${LOC}LOCALIZED\(None\)" SIG_NODE_AT=35 KEEP_PLAYING=1 PLAY_S=100 STOP_AT=45
run h35_node_frozen_after_localized "${LOC}LOCALIZED\(None\)" SIG_NODE_AT=35 SIG_NODE_SIG=STOP KEEP_PLAYING=1 PLAY_S=100 STOP_AT=45
run h36_node_frozen_5s_then_resumes "${LOC}LOCALIZATION_LOST\(supervisor_stalled\)" SIG_NODE_AT=35 SIG_NODE_SIG=STOP CONT_NODE_AT=40 KEEP_PLAYING=1 PLAY_S=100 STOP_AT=50
run h37_gate_killed_after_localized "${LOC}LOCALIZED\(None\)" KILL_GATE_AT=35 KEEP_PLAYING=1 PLAY_S=100 STOP_AT=45
run h38_second_permit_publisher "${LOC}LOCALIZED\(None\)" ROGUE_PERMIT_AT=1 KEEP_PLAYING=1 PLAY_S=100 STOP_AT=45
run h39_foreign_permit_after_node_death "${LOC}LOCALIZED\(None\)" ROGUE_PERMIT_AT=30 SIG_NODE_AT=35 KEEP_PLAYING=1 PLAY_S=100 STOP_AT=45
echo BATCH_DONE
