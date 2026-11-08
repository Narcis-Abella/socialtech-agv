#!/bin/bash
# Build one stage of docker/Dockerfile (robot | mapper | sim) from the repository root. Long (10+ min, needs internet): run it detached,
# the log is ~/build_<target>.log and its last line is "rc=<exit code>".
#   ops/build_image.sh mapper
#   ssh -f -n HOST 'setsid nohup <checkout>/ops/build_image.sh mapper </dev/null >/dev/null 2>&1 &'
# The image is tagged socialtech:<target>-<7 hex of the commit>[-dirty] (kept: the last KEEP=3 builds of this script per target) and socialtech:<target> is moved to it; the full commit
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
if ((rc == 0)); then   # the older tags of this target beyond KEEP go. Docker's own order is no use: cached layers give every image the same Created, so the history of THIS script's builds is kept
  state=${XDG_STATE_HOME:-$HOME/.local/state}/socialtech; mkdir -p "$state"; hist=$state/images-$TARGET      # one tag per line, the newest last; a tag it does not know (built by hand, earlier) is never removed
  { grep -vxF "$tag" "$hist" 2>/dev/null || true; echo "$tag"; } >"$hist.new" && mv "$hist.new" "$hist"
  head -n -"${KEEP:-3}" "$hist" | while read -r t; do docker rmi "socialtech:$t" >>"$LOG" 2>&1 || true; done      # one in use, or the alias's, only loses its name
  tail -n "${KEEP:-3}" "$hist" >"$hist.new" && mv "$hist.new" "$hist"
fi
echo "rc=$rc" >>"$LOG"
exit $rc
