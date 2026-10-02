#!/bin/bash
# AMCL-only replay: the /scan, /tf and /Odometry recorded by a full run (run_bench_a.sh with RECORD=1) played at RATE x into map_server + AMCL and scored like a
# full run. Fixed inputs and no FAST-LIO2 in the loop: this is where AMCL parameters are tuned. Run inside the robot image.
# usage: run_amcl_only.sh <inputs_bag> <map.yaml> <alidarState.txt> <out_dir> [key=value ...]    (key=value: AMCL parameter overrides, overlay.py)
# env:   RATE (5)  DIST (1.0 m) and DYAW (20 deg): initial-pose offset  ROS_DOMAIN_ID (79)
#        WINDOW: WIN_T0 (header time, epoch s, of the pose to start from), WIN_START (bag offset, s) and WIN_DUR (s) replay only that stretch of the bag, with the
#        initial pose taken from the reference at WIN_T0 (use DIST=0 DYAW=0 and a small INIT_STD): reproduces one failure in isolation
#        INIT_STD (m) and INIT_YAW_STD (deg, 15): publish /initialpose with that covariance instead of the set_initial_pose parameter, which gives AMCL a ZERO
#        covariance (all particles on one pose, so only the motion noise can pull the filter to the true pose)
set -eo pipefail
bag=$1; map=$2; ref=$3; out=$4; shift 4
tools=$(dirname "$(readlink -f "$0")")
source /ros_env.sh
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-79}
mkdir -p "$out"

python3 "$tools/overlay.py" "$@" ${INIT_STD:+set_initial_pose=false} > "$out/overrides.yaml"
read -r x y yaw < <(python3 "$tools/pose_eval.py" init --ref "$ref" --map "$map" --dist "${DIST:-1.0}" --dyaw "${DYAW:-20}" ${WIN_T0:+--at-time $WIN_T0})
echo "initial pose (map frame): $x $y $yaw; overrides: $*" | tee "$out/initial_pose.txt"

setsid ros2 launch "$tools/launch/amcl_only.launch.py" map:="$map" init_x:="$x" init_y:="$y" init_yaw:="$yaw" overrides:="$out/overrides.yaml" > "$out/launch.log" 2>&1 &
launch=$!
python3 "$tools/record_poses.py" /amcl_pose amcl "$out/amcl.tum" --idle 0 > "$out/rec_amcl.log" 2>&1 & rec_a=$!
python3 "$tools/record_poses.py" /Odometry odom "$out/odom.tum" --idle 0 > "$out/rec_odom.log" 2>&1 & rec_o=$!
sleep 8   # lifecycle bring-up
if [ -n "${INIT_STD:-}" ]; then
  msg=$(python3 "$tools/pose_eval.py" initpose --ref "$ref" --map "$map" --dist "${DIST:-1.0}" --dyaw "${DYAW:-20}" --std "$INIT_STD" --yaw-std "${INIT_YAW_STD:-15}" ${WIN_T0:+--at-time $WIN_T0})
  timeout 30 ros2 topic pub --once -w 1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "$msg" > "$out/initpose.log" 2>&1 \
    || { kill -KILL -- "-$launch" 2>/dev/null; echo "FAIL: /initialpose not delivered in 30 s (AMCL never became active; see $out/launch.log)"; exit 1; }
fi
# shellcheck disable=SC2086
ros2 bag play "$bag" --clock --rate "${RATE:-5}" --topics /scan /tf /Odometry ${WIN_START:+--start-offset $WIN_START} ${WIN_DUR:+--playback-duration $WIN_DUR} > "$out/play.log" 2>&1
sleep 3; kill -TERM "$rec_a" "$rec_o" 2>/dev/null || true; wait "$rec_a" "$rec_o" 2>/dev/null || true   # see run_bench_a.sh: AMCL is silent while the robot is still
# the launch runs in its own session (setsid): signal the whole group, or a KILLed launch leaves its nodes behind to clash with the next run on the same ROS domain
kill -TERM -- "-$launch" 2>/dev/null || true   # background jobs ignore SIGINT; see run_bench_a.sh
for _ in $(seq 20); do kill -0 "$launch" 2>/dev/null || break; sleep 1; done
kill -KILL -- "-$launch" 2>/dev/null || true

[ -s "$out/amcl.tum" ] || { echo "FAIL: AMCL published no pose (see $out/launch.log)"; exit 1; }
python3 "$tools/pose_eval.py" report --est "$out/amcl.tum" --odom "$out/odom.tum" --ref "$ref" --map "$map" > "$out/report.json"
