#!/usr/bin/env bash
# NX host: fault and watchdog cases in mode s2s3 on eco01 c04 (S2 accepts there at ~18 s when nothing is injected).
# Evaluator-free: the expectation is the terminal state/reason and that /initialpose is published only after ACCEPTED.
# usage: R=<run root> batch_faults_s2s3.sh TAG [name-prefix filter]
set -u
R=${R:?run root}; export R
B=/home/jetson/work/global-reloc-3d-20261009
BAG=/home/jetson/rosbags/custom/mapping_eco_-1_01_custom; YAML=/home/jetson/map_out/eco01_1x/map.yaml; XYZ=$B/isolated-v16/map/map.xyz; OFF=307.899419
T=$1; ONLY=${2:-}
S23="s3_auto_authorize:=true"; X1="s2_exposure_s:=1.0"     # 1 s S2 can never pass the temporal rule: forces the S3 path
run() { name=$1; shift; [ -n "$ONLY" ] && [[ $name != $ONLY* ]] && return
  env NAME=$name BAG=$BAG MAPYAML=$YAML MAPXYZ=$XYZ OFFSET=$OFF TAG=$T MODE=s2s3 "$@" bash $R/tools/global_reloc_3d/run_online.sh > /dev/null 2>&1
  d=$R/runs/$name-$T
  echo "$name-$T $(python3 - "$d" <<'PY' 2>&1 | tail -1
import glob, json, sys
d = sys.argv[1]
out = []
for m in sorted(glob.glob(d + '/work*/attempt*/metrics*.json')):
    j = json.load(open(m))
    out.append(f"{m.split('/')[-3]}/{m.split('/')[-2]}:{j['state']}({j['reason']}) stage={j.get('accepted_stage')} pose={j['initialpose'] is not None} t={j['total_s']['attempt_end_from_first_input']:.1f}")
orphans = open(d + '/orphans.txt').read().strip() if glob.glob(d + '/orphans.txt') else '?'
print(' | '.join(out) or 'NO_METRICS', '| orphans:', orphans)
PY
)"; }
RELAY='--spec /livox/lidar,livox_ros_driver2/msg/CustomMsg,%s,0 --spec /livox/imu,sensor_msgs/msg/Imu,%s,%s'
REMAP='--remap /livox/lidar:=/livox/lidar_raw /livox/imu:=/livox/imu_raw'
run g00_nominal LAUNCH_EXTRA="$S23"
run g01_lidar_missing LAUNCH_EXTRA="$S23" PLAY_ARGS_EXTRA="--topics /livox/imu"
run g02_imu_missing LAUNCH_EXTRA="$S23" PLAY_ARGS_EXTRA="--topics /livox/lidar"
run g03_lidar_drop30 LAUNCH_EXTRA="$S23" RELAY_SPEC="$(printf -- "$RELAY" 0.3 0 0)" PLAY_ARGS_EXTRA="$REMAP"
run g04_imu_drop30 LAUNCH_EXTRA="$S23" RELAY_SPEC="$(printf -- "$RELAY" 0 0.3 0)" PLAY_ARGS_EXTRA="$REMAP"
run g05_imu_stamp_shift LAUNCH_EXTRA="$S23" RELAY_SPEC="$(printf -- "$RELAY" 0 0 1000)" PLAY_ARGS_EXTRA="$REMAP"
run g06_lio_exits_at_start LAUNCH_EXTRA="$S23 fastlio_exe:=/bin/false"
run g07_lio_hangs_at_start LAUNCH_EXTRA="$S23 fastlio_exe:=/tools/global_reloc_3d/hang_exe.sh"
run g08_lio_killed_in_s2_search LAUNCH_EXTRA="$S23" KILL_LIO_AT=11 KILL_LIO_SIG=KILL
run g09_lio_frozen_in_s2_search LAUNCH_EXTRA="$S23" KILL_LIO_AT=11 KILL_LIO_SIG=STOP
run g10_lio_frozen_in_s2_accum LAUNCH_EXTRA="$S23" KILL_LIO_AT=6.5 KILL_LIO_SIG=STOP
run g11_lio_killed_late_forced_s3 LAUNCH_EXTRA="$S23 $X1" KILL_LIO_AT=17 KILL_LIO_SIG=KILL
run g12_lio_killed_after_handoff LAUNCH_EXTRA="$S23" KILL_LIO_AT=21.5 KILL_LIO_SIG=KILL PLAY_S=130
run g13_gicp_binary_false LAUNCH_EXTRA="$S23" REGISTER_BIN=/bin/false
run g14_gicp_binary_missing LAUNCH_EXTRA="$S23" REGISTER_BIN=/nonexistent
run g15_gicp_never_returns LAUNCH_EXTRA="$S23" REGISTER_BIN=/tools/global_reloc_3d/slow_exe.sh
run g16_deadline15_s2_search LAUNCH_EXTRA="$S23 deadline_s:=15.0"
run g17_deadline35_s3 LAUNCH_EXTRA="$S23 $X1 deadline_s:=35.0"
run g18_amcl_absent LAUNCH_EXTRA="$S23 with_amcl:=false" PLAY_S=130
run g19_s3_never_authorized LAUNCH_EXTRA="$X1"
run g20_s3_authorized_by_topic LAUNCH_EXTRA="$X1" AUTH_AT=20
run g21_s3_grant_too_early_ignored LAUNCH_EXTRA="$X1" AUTH_AT=1
run g22_s3_grant_too_late LAUNCH_EXTRA="$X1" AUTH_AT=40
run g23_node_sigkill_restart LAUNCH_EXTRA="$S23" KILL_NODE_AT=10 NODE2_ARGS="-p s3_auto_authorize:=true" PLAY_S=130
run g24_second_attempt_after_motion LAUNCH_EXTRA="$S23" MAX_ATTEMPTS=2 OFFSET=247.5 PLAY_S=160 KEEP_PLAYING=1
echo BATCH_DONE
