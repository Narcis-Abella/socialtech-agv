#!/bin/bash
# Build one stage of docker/Dockerfile as socialtech:<target> (robot | mapper). Long (10+ min, needs internet): run it detached,
# the log is ~/build_<target>.log and its last line is "rc=<exit code>".
#   ops/build_image.sh mapper
#   ssh -f -n HOST 'setsid nohup ~/socialtech-agv-int/ops/build_image.sh mapper </dev/null >/dev/null 2>&1 &'
set -uo pipefail
TARGET=${1:?usage: build_image.sh robot|mapper}
[[ $TARGET =~ ^(robot|mapper)$ ]] || { echo "unknown target '$TARGET' (robot|mapper)" >&2; exit 2; }
REPO=$(cd "$(dirname "$0")/.." && pwd)
LOG=$HOME/build_$TARGET.log

echo "start $(date -Is) $(git -C "$REPO" rev-parse --short HEAD) target=$TARGET" >"$LOG"
DOCKER_BUILDKIT=1 docker build --progress=plain --target "$TARGET" -t "socialtech:$TARGET" "$REPO/docker" >>"$LOG" 2>&1
rc=$?
echo "rc=$rc" >>"$LOG"
exit $rc
