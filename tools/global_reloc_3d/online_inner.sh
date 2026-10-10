#!/usr/bin/env bash
# Inside socialtech:reloc-online. One end-to-end replay: relocalizer node + AMCL, input = bag only (no reference, no initial pose).
# Env: BAG MAPYAML MAPXYZ OUT OFFSET (bag offset where the certified stationary start begins) [PRE=2.5 s of stationary IMU before it]
#      [PLAY_S=100] [MODE=s3] [REGISTER_BIN] [LAUNCH_EXTRA="with_amcl:=false ..."] [RELAY_SPEC="--spec topic,type,drop,shift ..." + PLAY_ARGS_EXTRA remaps] [BAG_TOPICS] [H=0.53 provisional] [MAX_ATTEMPTS=1] [LEAD=12 s node/AMCL bring-up before the bag starts] [PLAY_ARGS_EXTRA] [KILL_LIO_AT KILL_LIO_SIG] [AUTH_AT] [KILL_NODE_AT NODE2_ARGS]
set -eo pipefail
tools=/tools
source /ros_env.sh
set -u
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:?unique id per run}
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
mkdir -p "$OUT"
# Method/evaluator separation: refuse to run if a reference trajectory is visible in this container.
if find /map /prior "$(dirname "$MAPYAML")" "$(dirname "$MAPXYZ")" -name 'alidarState*' 2>/dev/null | grep -q .; then
  echo "reference trajectory visible in the method container: aborting" | tee "$OUT/reference_visible.txt"; exit 3
fi
python3 $tools/nav_bench/overlay.py set_initial_pose=false > "$OUT/overrides.yaml"
python3 -B $tools/global_reloc_3d/proc_cost.py sample "$OUT/cost.csv" > "$OUT/cost.log" 2>&1 & cost=$!
ros2 launch $tools/global_reloc_3d/launch/reloc_online.launch.py map:="$MAPYAML" map_xyz:="$MAPXYZ" work_dir:="$OUT/work" overrides:="$OUT/overrides.yaml" \
    mode:=${MODE:-s3} sensor_height:=${H:-0.53} max_attempts:=${MAX_ATTEMPTS:-1} register_binary:=${REGISTER_BIN:-$tools/global_reloc_3d/register_v2} ${LAUNCH_EXTRA:-} > "$OUT/launch.log" 2>&1 &
launch=$!
python3 $tools/nav_bench/record_poses.py /amcl_pose amcl "$OUT/amcl.tum" --idle 0 > "$OUT/rec_amcl.log" 2>&1 & rec_a=$!
python3 $tools/nav_bench/record_poses.py /Odometry odom "$OUT/odom.tum" --idle 0 > "$OUT/rec_odom.log" 2>&1 & rec_o=$!
python3 -B $tools/global_reloc_3d/topic_loss.py "$OUT/witness_imu.json" > "$OUT/witness.log" 2>&1 & witness=$!
sleep ${LEAD:-12}
if [ -n "${RELAY_SPEC:-}" ]; then   # fault injection: the bag publishes <topic>_raw, the relay forwards (with the fault) to <topic>
  # shellcheck disable=SC2086
  python3 -B $tools/global_reloc_3d/fault_relay.py $RELAY_SPEC > "$OUT/relay.log" 2>&1 & relay=$!
fi
start=$(python3 -c "print(max(0.,$OFFSET-${PRE:-2.5}))")
date +%s.%N > "$OUT/play_wall_start.txt"
ros2 bag play "$BAG" --clock --rate 1.0 --start-offset $start --playback-duration ${PLAY_S:-100} ${PLAY_ARGS_EXTRA:-} > "$OUT/play.log" 2>&1 &
play=$!
if [ -n "${KILL_NODE_AT:-}" ]; then   # restart test: SIGKILL the relocalizer mid-attempt, check FAST-LIO2 died with it, start a fresh node
  ( sleep $KILL_NODE_AT; np=$(pgrep -f 'reloc_node.py --ros-args' | head -1); kill -KILL $np; sleep 3
    pgrep -c fastlio_mapping > "$OUT/fastlio_after_kill.txt" || true
    python3 -B $tools/global_reloc_3d/reloc_node.py --ros-args -p use_sim_time:=true -p map_xyz:="$MAPXYZ" -p work_dir:="$OUT/work2" -p mode:=${MODE:-s3} \
      -p register_binary:=$tools/global_reloc_3d/register_v2 ${NODE2_ARGS:-} > "$OUT/node2.log" 2>&1 ) &
fi
if [ -n "${KILL_LIO_AT:-}" ]; then    # mid-run FAST-LIO2 fault: KILL = process dies, STOP = process hangs (stays alive, silent)
  ( sleep $KILL_LIO_AT; lp=$(pgrep -x fastlio_mapping | head -1); echo "lio pid=$lp sig=${KILL_LIO_SIG:-KILL} at $(date +%s.%N)" > "$OUT/lio_fault.txt"; kill -${KILL_LIO_SIG:-KILL} $lp ) &
fi
if [ -n "${AUTH_AT:-}" ]; then        # external supervisor stand-in: grants S3 on the real topic AUTH_AT s after the bag starts
  ( sleep $AUTH_AT; date +%s.%N > "$OUT/auth_wall.txt"; ros2 topic pub -t ${AUTH_TIMES:-3} -r 5 /global_reloc/s3_authorize std_msgs/msg/Bool "data: true" > "$OUT/auth.log" 2>&1 ) &
fi
# Stop when the attempt reached a terminal state and (if localized) AMCL had its confirmation window, or the replay ended.
for _ in $(seq $(( ${PLAY_S:-100} + 10 ))); do
  sleep 1
  kill -0 $play 2>/dev/null || { echo "loop: bag play ended at $SECONDS s"; break; }
  m=$(ls $OUT/work/attempt*/metrics.json 2>/dev/null | tail -1 || true)
  if [ -n "$m" ] && [ "${KEEP_PLAYING:-0}" = 0 ]; then echo "loop: terminal metrics at $SECONDS s"; break; fi
done
sleep 2
kill -TERM $play $rec_a $rec_o $cost $witness ${relay:-} 2>/dev/null || true
wait $play $rec_a $rec_o $cost $witness ${relay:-} 2>/dev/null || true
# ask the relocalizer itself first: it writes metrics_interrupted.json and stops its FAST-LIO2 child (ros2 launch forwards SIGTERM late)
node_pid=$(pgrep -f 'reloc_node.py --ros-args' | head -1 || true)
[ -n "$node_pid" ] && kill -TERM $node_pid 2>/dev/null && sleep 3
kill -TERM $launch 2>/dev/null || true
for _ in $(seq 25); do kill -0 $launch 2>/dev/null || break; sleep 1; done
kill -KILL $launch 2>/dev/null || true
# children left behind after the whole stack was asked to stop (expected: 0 0 0)
echo "fastlio=$(pgrep -xc fastlio_mapping) search=$(pgrep -fc search_v2.py) register=$(pgrep -fc register_v2)" > "$OUT/orphans.txt" || true
python3 $tools/global_reloc_3d/proc_cost.py summarize "$OUT/cost.csv" > "$OUT/cost_summary.json" || true
ls "$OUT/work"/attempt*/metrics*.json
