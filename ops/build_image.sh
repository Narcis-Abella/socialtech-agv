#!/bin/bash
# Build one stage of docker/Dockerfile (robot | mapper | sim) from the repository root. Long (10+ min, needs internet): run it detached,
# the log is ~/build_<target>.log and its last line is "rc=<exit code>".
#   ops/build_image.sh mapper
#   ssh -f -n HOST 'setsid nohup <checkout>/ops/build_image.sh mapper </dev/null >/dev/null 2>&1 &'
# The image is tagged socialtech:<target>-<7 hex of the commit>[-dirty] (kept: the newest KEEP=3 per target) and socialtech:<target> is moved to it; the full commit
# (+ "-dirty") is the label org.opencontainers.image.revision. Rollback = move the alias back: docker tag socialtech:<target>-<old sha> socialtech:<target>
set -uo pipefail
TARGET=${1:?usage: build_image.sh robot|mapper|sim}
[[ $TARGET =~ ^(robot|mapper|sim)$ ]] || { echo "unknown target '$TARGET' (robot|mapper|sim)" >&2; exit 2; }
REPO=$(cd "$(dirname "$0")/.." && pwd)
LOG=$HOME/build_$TARGET.log
source "$REPO/ops/image_revision.sh"

rev=$(checkout_revision "$REPO"); dirty=; ! checkout_dirty "$REPO" || dirty=-dirty
[[ -n $rev ]] || echo "WARN: no commit known for $REPO (no .git and no filled REVISION): the image is tagged unknown" >&2
tag=$TARGET-${rev:0:7}$dirty; [[ -n $rev ]] || tag=$TARGET-unknown

echo "start $(date -Is) ${rev:0:7}$dirty target=$TARGET" >"$LOG"
DOCKER_BUILDKIT=1 docker build --progress=plain --target "$TARGET" -t "socialtech:$tag" -t "socialtech:$TARGET" \
  --label "org.opencontainers.image.revision=${rev}${dirty}" -f "$REPO/docker/Dockerfile" "$REPO" >>"$LOG" 2>&1
rc=$?
if ((rc == 0)); then   # older tags of this target beyond KEEP (docker lists the newest first); one in use or still the alias just stays
  docker images socialtech --format '{{.Tag}}' | grep -E "^$TARGET-([0-9a-f]{7}(-dirty)?|unknown)$" | tail -n +$((${KEEP:-3} + 1)) | while read -r t; do
    docker rmi "socialtech:$t" >>"$LOG" 2>&1 || true
  done
fi
echo "rc=$rc" >>"$LOG"
exit $rc
