#!/bin/bash
# Run: bash ops/test_image_revision.sh   (no Docker, no board: a fake docker and ssh record what build_image.sh and ship_image.sh ask of them)
here=$(cd "$(dirname "$0")/.." && pwd)
t=$(mktemp -d /tmp/test_image_revision.XXXXXX); trap 'rm -r "$t"' EXIT
fail=0; check() { if ! eval "$2"; then echo "FAIL: $1"; fail=1; fi; }
source "$here/ops/image_revision.sh"

# --- checkout_revision / checkout_dirty
mkdir "$t/repo" && git -C "$t/repo" init -q && echo a > "$t/repo/f" && git -C "$t/repo" add f && git -C "$t/repo" -c user.name=t -c user.email=t@t commit -qm x
head=$(git -C "$t/repo" rev-parse HEAD)
check "a repository reports its HEAD" '[[ $(checkout_revision "$t/repo") == "$head" ]]'
check "a clean repository is not dirty" '! checkout_dirty "$t/repo"'
echo b >> "$t/repo/f"; check "an edited repository is dirty" 'checkout_dirty "$t/repo"'
mkdir "$t/repo/copy"; echo '$Format:%H$' > "$t/repo/copy/REVISION"
check "a folder inside another repository is not that repository (unfilled REVISION: unknown)" '[[ -z $(checkout_revision "$t/repo/copy") ]] && ! checkout_dirty "$t/repo/copy"'
mkdir "$t/plain"; echo "$head" > "$t/plain/REVISION"
check "a plain copy reads the filled REVISION" '[[ $(checkout_revision "$t/plain") == "$head" ]]'
mkdir "$t/plain2"; echo '$Format:%H$' > "$t/plain2/REVISION"
check "an unfilled REVISION is unknown" '[[ -z $(checkout_revision "$t/plain2") ]]'
check "no REVISION at all is unknown" '[[ -z $(checkout_revision "$t") ]]'
# git archive fills REVISION in (the way a board gets a copy)
git -C "$t/repo" checkout -q -- f; echo '$Format:%H$' > "$t/repo/REVISION"; echo '/REVISION export-subst' > "$t/repo/.gitattributes"
git -C "$t/repo" add REVISION .gitattributes && git -C "$t/repo" -c user.name=t -c user.email=t@t commit -qm y
mkdir "$t/arch" && git -C "$t/repo" archive HEAD | tar -x -C "$t/arch"
check "git archive writes the commit into REVISION" '[[ $(checkout_revision "$t/arch") == $(git -C "$t/repo" rev-parse HEAD) ]]'

# --- fake docker / ssh
mkdir "$t/bin" "$t/home"
cat > "$t/bin/docker" <<'F'
#!/bin/bash
echo "docker $*" >> "$FAKE_LOG"
case "$1" in
  images) printf '%s\n' ${FAKE_TAGS:-};;
  image) [[ $2 == inspect ]] && echo "${FAKE_REV:-}";;
  build) ;;
esac
F
cat > "$t/bin/ssh" <<'F'
#!/bin/bash
echo "ssh $*" >> "$FAKE_LOG"
[[ $* == *"image inspect"* ]] && exit ${FAKE_REMOTE_HAS:-1}
exit 0
F
chmod +x "$t/bin/docker" "$t/bin/ssh"
run() { FAKE_LOG=$t/log PATH=$t/bin:$PATH HOME=$t/home "$@"; }

# --- build_image.sh in a copy of ops/ + docker/Dockerfile stand-in made from a plain REVISION folder
mkdir -p "$t/co/ops" "$t/co/docker"; cp "$here/ops/build_image.sh" "$here/ops/image_revision.sh" "$t/co/ops/"; : > "$t/co/docker/Dockerfile"; echo "$head" > "$t/co/REVISION"
for sha in 1111111 2222222 3333333 4444444; do      # four builds of one target, KEEP=2
  printf '%s%033d\n' "$sha" 0 > "$t/co/REVISION"; : > "$t/log"
  KEEP=2 run bash "$t/co/ops/build_image.sh" robot
  [[ $sha == 1111111 ]] && b=$(grep '^docker build' "$t/log") && cp "$t/log" "$t/log1"
done
check "build tags <target>-<sha7> and moves the alias" '[[ $b == *"-t socialtech:robot-1111111 -t socialtech:robot "* ]]'
check "build labels the full commit" '[[ $b == *"--label org.opencontainers.image.revision=1111111$(printf %033d 0) "* ]]'
check "build uses the repository root as the context and docker/Dockerfile" '[[ $b == *"-f $t/co/docker/Dockerfile $t/co" ]]'
check "build keeps the last KEEP builds: the third build removed the first, the fourth the second, nothing else" '[[ $(grep -c "^docker rmi" "$t/log1") == 0 ]] && ! grep -q "rmi socialtech:robot-4444444" "$t/log" && grep -q "rmi socialtech:robot-2222222" "$t/log" && ! grep -q "rmi socialtech:robot-3333333" "$t/log"'
check "the history keeps the last two tags, newest last" '[[ $(cat "$t/home/.local/state/socialtech/images-robot" | tr "\n" " ") == "robot-3333333 robot-4444444 " ]]'
mkdir -p "$t/home/.local/state/socialtech"; printf 'robot-9999999\n' > "$t/home/.local/state/socialtech/images-sim"
printf '%s%033d\n' "5555555" 0 > "$t/co/REVISION"; : > "$t/log"; KEEP=3 run bash "$t/co/ops/build_image.sh" sim
check "a tag the history does not know is never removed" '! grep -q "^docker rmi" "$t/log"'
check "build log ends with rc=0" '[[ $(tail -1 "$t/home/build_robot.log") == rc=0 ]]'

# --- ship_image.sh
: > "$t/log"; FAKE_REV=$head run bash "$here/ops/ship_image.sh" robot jetson@nano 2>&1 | tail -2 > "$t/out"
check "ship sends the image of the label's commit and moves the alias" 'grep -q "docker save socialtech:robot-${head:0:7}" "$t/log" && grep -q "ssh .*jetson@nano docker tag socialtech:robot-${head:0:7} socialtech:robot" "$t/log"'
: > "$t/log"; FAKE_REV=$head FAKE_REMOTE_HAS=0 run bash "$here/ops/ship_image.sh" robot jetson@nano >/dev/null 2>&1
check "ship does not send what the board already has" '! grep -q "docker save" "$t/log" && grep -q "docker tag" "$t/log"'
: > "$t/log"; FAKE_REV=${head}-dirty run bash "$here/ops/ship_image.sh" robot jetson@nano >/dev/null 2>&1
check "ship names a dirty image with its suffix" 'grep -q "docker save socialtech:robot-${head:0:7}-dirty" "$t/log"'
FAKE_REV= run bash "$here/ops/ship_image.sh" robot jetson@nano >/dev/null 2>&1; rc=$?; check "ship refuses an image without a revision label" '((rc != 0))'


# --- image_freshness (the fake docker answers `image inspect` with FAKE_REV)
g() { git -C "$t/repo" -c user.name=t -c user.email=t@t "$@"; }
mkdir -p "$t/repo/src" "$t/repo/docs" "$t/repo/ops"; echo 1 > "$t/repo/src/a"; echo 1 > "$t/repo/docs/d"; echo 1 > "$t/repo/ops/nav_live.sh"; echo 1 > "$t/repo/ops/other.sh"
g add -A && g commit -qm base; base=$(g rev-parse HEAD)
fresh() { FAKE_LOG=$t/log FAKE_REV=$1 PATH=$t/bin:$PATH bash -c 'source "$1"; image_freshness img "$2"' _ "$here/ops/image_revision.sh" "$t/repo"; }
fresh $base >/dev/null; check "same commit, clean tree: current" '[[ $? == 0 ]]'
echo 2 > "$t/repo/docs/d"; g commit -qam docs; fresh $base >/dev/null; check "a docs-only commit leaves the image current" '[[ $? == 0 ]]'
echo 2 > "$t/repo/ops/other.sh"; g commit -qam other; fresh $base >/dev/null; check "another ops script leaves it current" '[[ $? == 0 ]]'
echo 2 > "$t/repo/src/a"; g commit -qam src; msg=$(fresh $base); rc=$?; check "a src/ commit makes it stale, with the reason" '[[ $rc == 1 && $msg == *differ* ]]'
g checkout -q "$base" -- src; fresh $base >/dev/null; check "back to the image's src: current again" '[[ $? == 0 ]]'
echo 3 > "$t/repo/ops/nav_live.sh"; fresh $base >/dev/null; check "an uncommitted edit of the launcher counts" '[[ $? == 1 ]]'
g checkout -q -- ops/nav_live.sh; fresh ${base}-dirty >/dev/null; check "a -dirty label is read as its commit" '[[ $? == 0 ]]'
fresh "" >/dev/null; check "no label: cannot tell" '[[ $? == 2 ]]'
mkdir "$t/copy"; echo "$(g rev-parse HEAD)" > "$t/copy/REVISION"; fresh2() { FAKE_LOG=$t/log FAKE_REV=$1 PATH=$t/bin:$PATH bash -c 'source "$1"; image_freshness img "$2"' _ "$here/ops/image_revision.sh" "$t/copy"; }
fresh2 "$(g rev-parse HEAD)" >/dev/null; check "a plain copy at the image's commit: current" '[[ $? == 0 ]]'
fresh2 "$base" >/dev/null; check "a plain copy at another commit: stale (no history to compare)" '[[ $? == 1 ]]'

((fail)) || echo ok
exit $fail
