#!/bin/bash
# AMCL-only replay: the /scan, /tf and /Odometry recorded by a full run (run_bench_a.sh with RECORD=1) played at RATE x into map_server + AMCL and scored like a
# full run. Fixed inputs and no FAST-LIO2 in the loop: this is where AMCL parameters are tuned. Run inside the robot image.
# usage: run_amcl_only.sh <inputs_bag> <map.yaml> <alidarState.txt> <out_dir> [key=value ...]    (key=value: AMCL parameter overrides, overlay.py)
# env:   RATE (5)  DIST (1.0 m) and DYAW (20 deg): initial-pose offset  ROS_DOMAIN_ID (79)
set -eo pipefail
bag=$1; map=$2; ref=$3; out=$4; shift 4
tools=$(dirname "$(readlink -f "$0")")
source /ros_env.sh
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-79}
mkdir -p "$out"

python3 "$tools/overlay.py" "$@" > "$out/overrides.yaml"
read -r x y yaw < <(python3 "$tools/pose_eval.py" init --ref "$ref" --map "$map" --dist "${DIST:-1.0}" --dyaw "${DYAW:-20}")
echo "initial pose (map frame): $x $y $yaw; overrides: $*" | tee "$out/initial_pose.txt"

ros2 launch "$tools/launch/amcl_only.launch.py" map:="$map" init_x:="$x" init_y:="$y" init_yaw:="$yaw" overrides:="$out/overrides.yaml" > "$out/launch.log" 2>&1 &
launch=$!
python3 "$tools/record_poses.py" /amcl_pose amcl "$out/amcl.tum" > "$out/rec_amcl.log" 2>&1 & rec_a=$!
python3 "$tools/record_poses.py" /Odometry odom "$out/odom.tum" > "$out/rec_odom.log" 2>&1 & rec_o=$!
sleep 8   # lifecycle bring-up
ros2 bag play "$bag" --clock --rate "${RATE:-5}" --topics /scan /tf /Odometry > "$out/play.log" 2>&1
wait "$rec_a" "$rec_o" || true
kill -TERM "$launch" 2>/dev/null || true   # background jobs ignore SIGINT; see run_bench_a.sh
for _ in $(seq 20); do kill -0 "$launch" 2>/dev/null || break; sleep 1; done
kill -KILL "$launch" 2>/dev/null || true

[ -s "$out/amcl.tum" ] || { echo "FAIL: AMCL published no pose (see $out/launch.log)"; exit 1; }
python3 "$tools/pose_eval.py" report --est "$out/amcl.tum" --odom "$out/odom.tum" --ref "$ref" --map "$map" > "$out/report.json"
