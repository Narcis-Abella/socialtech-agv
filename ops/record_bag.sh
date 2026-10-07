#!/bin/bash
# Record a Livox Mid-360 bag (CustomMsg lidar + IMU) with the robot's own driver config, then check it for lost scans.
#   ops/record_bag.sh [NAME]                 Ctrl-C stops (needs a tty: ssh -t)
#   DURATION=30 ops/record_bag.sh smoke      stops by itself after 30 s (counted from the start of the recording)
# The first STATIC_SECS (default 5) are announced as "STILL": the robot must not move so FAST-LIO starts from rest.
set -euo pipefail

NAME=${1:-bag_$(date +%Y%m%d_%H%M%S)}
BAG_DIR=${BAG_DIR:-$HOME/rosbags/records}
IMAGE=${IMAGE:-socialtech:robot}
HOST_IP=${LIVOX_HOST_IP:-192.168.1.50}
LIDAR_IP=${LIVOX_LIDAR_IP:-192.168.1.126}
MIN_FREE_GB=${MIN_FREE_GB:-20}
RMEM_WANT=67108864  # same limit as host/setup_host.sh
REPO=$(cd "$(dirname "$0")/.." && pwd)

die() { echo "FAIL: $*" >&2; exit 1; }
warn() { echo "WARN: $*" >&2; }
udp_rcvbuf_errors() { awk '/^Udp:/ { if (!h) { for (i = 1; i <= NF; i++) if ($i == "RcvbufErrors") c = i; h = 1 } else print $c }' /proc/net/snmp; }

[[ $NAME =~ ^[A-Za-z0-9_.-]+$ ]] || die "bag name '$NAME': only letters, digits, _ . -"
mkdir -p "$BAG_DIR"
[[ ! -e $BAG_DIR/$NAME ]] || die "$BAG_DIR/$NAME already exists: pick another name"
docker image inspect "$IMAGE" >/dev/null 2>&1 || die "image $IMAGE not found (docker images)"
ip -4 -o addr show | grep -q " $HOST_IP/" || die "no interface has $HOST_IP (the Livox link): check the ethernet cable, then ip -br addr"
ping -c1 -W2 "$LIDAR_IP" >/dev/null 2>&1 || die "Livox $LIDAR_IP does not answer: power, ethernet cable, or its IP is not $LIDAR_IP (LIVOX_LIDAR_IP=...)"
pgrep -f '[l]ivox_ros_driver2_node' >/dev/null && die "a Livox driver is already running: stop it first (it owns the UDP ports)"
free_gb=$(df --output=avail -BG "$BAG_DIR" | tail -1 | tr -dc 0-9)
((free_gb >= MIN_FREE_GB)) || die "only ${free_gb} GB free in $BAG_DIR (need $MIN_FREE_GB)"
(($(sysctl -n net.core.rmem_max) >= RMEM_WANT)) \
  || warn "net.core.rmem_max is $(sysctl -n net.core.rmem_max), want $RMEM_WANT: scans may be lost silently. Fix (needs sudo): sudo sysctl -w net.core.rmem_max=$RMEM_WANT"

CFG=$(mktemp -d)
trap 'rm -rf "$CFG"' EXIT
LIVOX_HOST_IP=$HOST_IP LIVOX_LIDAR_IP=$LIDAR_IP "$REPO/ops/livox_config.sh" >"$CFG/MID360_config.json"

err_before=$(udp_rcvbuf_errors)
echo "recording $BAG_DIR/$NAME ${DURATION:+for ${DURATION}s }(Ctrl-C to stop)"
rc=0
docker run --rm --init -t --network host --user "$(id -u):$(id -g)" \
  -e HOME=/tmp -e ROS_LOG_DIR=/tmp/roslog -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST -e DDS_IFACE=lo \
  -e NAME="$NAME" -e DURATION="${DURATION:-0}" -e STATIC_SECS="${STATIC_SECS:-5}" \
  -v "$CFG:/cfg:ro" -v "$BAG_DIR:/bags" -w /tmp "$IMAGE" bash -c '
    set -m  # job control: background jobs keep default signals (else they inherit SIGINT ignored and nothing can stop the recorder)
    ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args -r __node:=livox_lidar_publisher \
      -p xfer_format:=1 -p multi_topic:=0 -p data_src:=0 -p publish_freq:=10.0 -p output_data_type:=0 \
      -p frame_id:=livox_frame -p lvx_file_path:=/tmp/none.lvx -p user_config_path:=/cfg/MID360_config.json \
      -p cmdline_input_bd_code:=livox0000000001 >/tmp/driver.log 2>&1 &
    drv=$!
    for t in /livox/lidar /livox/imu; do
      # echo --once exits at once on a topic that does not exist yet, and the driver only creates them ~3 s after the lidar answers
      for _ in $(seq 20); do ros2 topic list | grep -qx "$t" && break; sleep 1; done
      timeout 10 ros2 topic echo --once --no-arr "$t" >/dev/null 2>&1 \
        || { echo "FAIL: no data on $t after 30 s (Livox powered? driver log follows)" >&2; tail -20 /tmp/driver.log >&2; kill $drv; exit 2; }
    done
    # Background + stdin from /dev/null: with -t the recorder reads the keyboard and a background process group gets SIGTTIN (hang).
    ros2 bag record -s mcap -o /bags/"$NAME" --topics /livox/lidar /livox/imu </dev/null &
    rec=$!
    trap "kill -INT $rec" INT TERM
    for _ in $(seq 20); do [ -e /bags/"$NAME"/"$NAME"_0.mcap ] && break; sleep 0.5; done
    echo "STILL: recording. Do NOT touch or move the robot for ${STATIC_SECS}s (FAST-LIO initialises at rest)"
    sleep "$STATIC_SECS"
    echo "GO: you can move the robot now. Ctrl-C ends the recording."
    if [ "$DURATION" -gt 0 ]; then (sleep "$DURATION"; kill -INT $rec) & fi
    wait $rec; wait $rec
    kill -TERM $drv; wait $drv  # the driver ignores SIGINT; SIGTERM makes it deinit the SDK cleanly
    exit 0  # the verdict on the bag is bag_check.py (host side), not the exit codes of these signals
  ' || rc=$?
((rc == 0 || rc == 130)) || die "recording container exited with $rc"

err_after=$(udp_rcvbuf_errors)
((err_after == err_before)) || warn "kernel dropped $((err_after - err_before)) UDP datagrams (RcvbufErrors) during the recording: raise net.core.rmem_max"
du -sh "$BAG_DIR/$NAME"
PY=${OPS_PYTHON:-$HOME/venv-ops/bin/python3}
[[ -x $PY ]] || die "bag recorded, but $PY is missing, so it was not checked. One-time setup: python3 -m venv --system-site-packages ~/venv-ops && ~/venv-ops/bin/pip install rosbags==0.11.5"
"$PY" "$REPO/tools/slam_bench/bag_check.py" "$BAG_DIR/$NAME"
