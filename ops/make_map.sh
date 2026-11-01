#!/bin/bash
# Bag -> 2D map: checks the bag, then runs tools/slam_bench/make_map.sh in the `mapper` image (Voxel-SLAM -> PLY -> PGM + YAML).
#   ops/make_map.sh BAG_NAME [MAP_NAME]     BAG_NAME is a folder in ~/rosbags/records; the map goes to ~/maps/MAP_NAME (default: BAG_NAME)
# Replays the bag at PLAY_RATE (default 2): a 3-minute bag takes ~2 min plus the global stage; if the run warns that scans were lost, relaunch with PLAY_RATE=1. Keep the board otherwise idle.
#   REUSE_SLAM=1 ops/make_map.sh BAG_NAME MAP_NAME   redoes everything after the SLAM of an existing ~/maps/MAP_NAME (seconds, no replay): the way to regenerate an old map with the current pipeline.
set -euo pipefail

BAG_NAME=${1:?usage: make_map.sh BAG_NAME [MAP_NAME]}
MAP_NAME=${2:-$BAG_NAME}
BAG_ROOT=${BAG_DIR:-$HOME/rosbags/records}
MAP_ROOT=${MAP_DIR:-$HOME/maps}
IMAGE=${IMAGE:-socialtech:mapper}
MIN_FREE_GB=${MIN_FREE_GB:-5}
REPO=$(cd "$(dirname "$0")/.." && pwd)
PY=${OPS_PYTHON:-$HOME/venv-ops/bin/python3}

die() { echo "FAIL: $*" >&2; exit 1; }

[[ $BAG_NAME =~ ^[A-Za-z0-9_.-]+$ && $MAP_NAME =~ ^[A-Za-z0-9_.-]+$ ]] || die "names: only letters, digits, _ . -"
[[ -f $BAG_ROOT/$BAG_NAME/metadata.yaml ]] || die "$BAG_ROOT/$BAG_NAME has no metadata.yaml: not a finished bag (was the recording killed?)"
reuse=; [[ ${REUSE_SLAM:-} != 1 ]] || reuse=--reuse-slam
if [[ -n $reuse ]]; then [[ -s $MAP_ROOT/$MAP_NAME/vs/run/alidarState.txt ]] || die "REUSE_SLAM=1 needs $MAP_ROOT/$MAP_NAME/vs/run/alidarState.txt of an earlier run"
else [[ ! -e $MAP_ROOT/$MAP_NAME ]] || die "$MAP_ROOT/$MAP_NAME already exists: pick another MAP_NAME (or REUSE_SLAM=1 to rebuild it without the replay)"; fi
docker image inspect "$IMAGE" >/dev/null 2>&1 || die "image $IMAGE missing. Build it (long): ops/build_image.sh mapper"
[[ -z $(docker ps -q) ]] || die "a container is running (docker ps): one heavy job at a time"
mkdir -p "$MAP_ROOT"
free_gb=$(df --output=avail -BG "$MAP_ROOT" | tail -1 | tr -dc 0-9)
((free_gb >= MIN_FREE_GB)) || die "only ${free_gb} GB free in $MAP_ROOT (need $MIN_FREE_GB)"
[[ -x $PY ]] || die "$PY missing (bag_check): sudo host/setup_host.sh, or see notes/operador_runbook.md"
"$PY" "$REPO/tools/slam_bench/bag_check.py" "$BAG_ROOT/$BAG_NAME" || die "bag_check failed: a bag that lost scans does not map reliably"

echo "mapping $BAG_ROOT/$BAG_NAME -> $MAP_ROOT/$MAP_NAME"
docker run --rm --init --network host --user "$(id -u):$(id -g)" -e HOME=/tmp -e ROS_LOG_DIR=/tmp/roslog \
  -e ROS_DOMAIN_ID=78 -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST -e DDS_IFACE=lo \
  -e PLAY_RATE="${PLAY_RATE:-2}" -e CEIL="${CEIL:-1.80}" -e MAKE_MAP_COMMIT="$(git -C "$REPO" rev-parse --short HEAD 2>/dev/null || echo unknown)" \
  -v "$BAG_ROOT:/bags:ro" -v "$MAP_ROOT:/out" -v "$REPO/tools/slam_bench:/tools:ro" "$IMAGE" \
  /tools/make_map.sh $reuse "/bags/$BAG_NAME" "/out/$MAP_NAME"
echo "map: $MAP_ROOT/$MAP_NAME/map.yaml  (report: $MAP_ROOT/$MAP_NAME/report.json)"
