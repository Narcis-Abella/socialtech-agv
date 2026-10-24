#!/bin/bash
# Laptop side: sends a list of jobs to the AGX and runs them there one after another, detached, so a sleeping or disconnected laptop does not stop them. Needs no sudo. Each job waits for an idle board.
#   agx_queue.sh submit <queue name> <spec file>    ships tools/ and the sim commit once, writes the jobs, starts the queue; prints the remote dir
#   agx_queue.sh status <remote dir>                what has started / finished
#   agx_queue.sh fetch <remote dir> <local dir>     copies every job's out/ and run.log into <local dir>/<job name>/
# spec file: blocks separated by a blank line. First line "trials <name> <scenario.json under tools/> [reps]" (run_trials.sh) or "real <name> <script under nav_sim/>" (real_factorial.sh, record_real_inputs.sh ...);
# the next lines are KEY=VALUE for the container (as the env of agx_run.sh / agx_real.sh; a value may hold spaces and semicolons). Lines starting with # are ignored.
# Container env values are taken literally (docker --env-file): write BAGS=a b c without quotes.
# env: AFTER=<remote dir of another queue> (wait for its QUEUE-DONE first), DRY=1 (build the files, send nothing), IMAGE (socialtech:sim-main), SIM_REPO (~/Documents/GitHub/hospital-agv-sim, committed), RUN_TIMEOUT (7200 s per job)
set -eo pipefail
KEY=~/.nvsdkm/.ssh/id_rsa_sdkm; HOST=jetson@socialtech-agx-1
ssh_() { ssh -i "$KEY" -o ConnectTimeout=10 "$HOST" "$@"; }
send() { ssh_ "$@"; }   # receives a tar stream on stdin
[ -n "$DRY" ] && { ssh_() { echo "[dry] ssh $*" >&2; }; send() { cat > /dev/null; echo "[dry] send $*" >&2; }; }   # DRY=1: nothing is sent; the generated files stay in the printed temp dir
repo=$(cd "$(dirname "$0")/../.." && pwd)
case "$1" in
submit)
  q=${2:?queue name}; spec=${3:?spec file}
  sim=${SIM_REPO:-$HOME/Documents/GitHub/hospital-agv-sim}
  [ -z "$(git -C "$sim" status --porcelain)" ] || { echo "the sim repo has uncommitted changes" >&2; exit 1; }
  hash=$(git -C "$sim" rev-parse --short HEAD)
  remote=nav2runs/q_${q}_$(date +%y%m%d_%H%M%S)
  ssh_ "mkdir -p $remote ~/sim_src/hospital-agv-sim_$hash ~/nav_inputs"
  tar -C "$repo" -c tools | send "tar -x -C $remote"
  ssh_ "[ -f ~/sim_src/hospital-agv-sim_$hash/package.xml ]" || git -C "$sim" archive HEAD | send "tar -x -C ~/sim_src/hospital-agv-sim_$hash"
  tmp=$(mktemp -d); n=0; : > "$tmp/queue.sh"
  echo '#!/bin/bash' >> "$tmp/queue.sh"
  echo "cd \$(dirname \"\$0\")" >> "$tmp/queue.sh"
  [ -n "$AFTER" ] && echo "until grep -q QUEUE-DONE \$HOME/$AFTER/progress.log; do sleep 30; done" >> "$tmp/queue.sh"   # start only after another queue has finished
  # split the spec into one directory per job
  block=""; flush() {
    [ -z "$block" ] && return
    n=$((n + 1)); kind=$(echo "$block" | head -1 | cut -d' ' -f1); name=$(echo "$block" | head -1 | cut -d' ' -f2); arg=$(echo "$block" | head -1 | cut -d' ' -f3); reps=$(echo "$block" | head -1 | cut -d' ' -f4)
    mkdir -p "$tmp/$name"; echo "$block" | tail -n +2 | grep -v '^#' | grep . > "$tmp/$name/user.env" || true
    case "$kind" in
      trials) { echo "SIM_SRC=/simsrc"; echo "ROS_DOMAIN_ID=53"; cat "$tmp/$name/user.env"; echo "SIM_COMMIT=$hash"; echo "REPO_COMMIT=$(git -C "$repo" rev-parse --short HEAD)$([ -n "$(git -C "$repo" status --porcelain)" ] && echo -dirty)"; } > "$tmp/$name/env.list"
        cmd="timeout ${RUN_TIMEOUT:-7200} docker run --rm --name nav2run --network host --env-file $name/env.list -e DDS_IFACE=lo -v \$PWD/tools:/tools:ro -v \$PWD/$name/out:/out -v \$HOME/sim_src/hospital-agv-sim_$hash:/simsrc:ro ${IMAGE:-socialtech:sim-main} /tools/nav_sim/run_trials.sh /tools/$arg /out ${reps:-1}";;
      real) cp "$tmp/$name/user.env" "$tmp/$name/env.list"
        cmd="timeout ${RUN_TIMEOUT:-7200} docker run --rm --name nav2run --network host --env-file $name/env.list -v \$PWD/tools:/tools:ro -v \$PWD/$name/out:/out -v \$HOME/nav_out:/navout:ro -v \$HOME/nav_data:/data:ro -v \$HOME/rosbags:/bags:ro -v \$HOME/nav_inputs:/inputs ${IMAGE:-socialtech:sim-main} /tools/nav_sim/$arg /out";;
      *) echo "unknown job kind: $kind" >&2; exit 1;;
    esac
    {
      echo "mkdir -p $name/out"
      echo "until [ \"\$(docker ps -q | wc -l)\" = 0 ] && awk -v l=\"\$(cut -d' ' -f1 /proc/loadavg)\" 'BEGIN{exit !(l < 2)}'; do sleep 20; done"
      echo "echo \"\$(date +%H:%M:%S) started $name\" >> progress.log"
      echo "$cmd > $name/run.log 2>&1; echo rc=\$? >> $name/run.log"
      echo "echo \"\$(date +%H:%M:%S) finished $name \$(grep -h '^rc=' $name/run.log)\" >> progress.log"
    } >> "$tmp/queue.sh"
    block=""
  }
  while IFS= read -r line || [ -n "$line" ]; do
    case "$line" in "#"*) continue;; "") flush;; *) block="${block:+$block
}$line";; esac
  done < "$spec"
  flush
  echo "echo QUEUE-DONE >> progress.log" >> "$tmp/queue.sh"
  tar -C "$tmp" -c . | send "tar -x -C $remote"
  [ -n "$DRY" ] && { echo "dry run: $tmp"; exit 0; }
  ssh -f -n -i "$KEY" "$HOST" "cd $remote && chmod +x queue.sh && setsid nohup ./queue.sh > queue.log 2>&1 &" > /dev/null 2>&1
  echo "$remote  ($n jobs)" ;;
status) ssh_ "cat ${2:?remote dir}/progress.log 2>/dev/null; docker ps --format '{{.Names}} {{.Status}}'" ;;
fetch)
  mkdir -p "${3:?local dir}"
  for j in $(ssh_ "cd ${2:?remote dir} && ls -d */ | grep -v '^tools/' | tr -d /"); do mkdir -p "$3/$j"; ssh_ "tar -C $2/$j -c out run.log env.list 2>/dev/null" | tar -x -C "$3/$j" 2>/dev/null || true; done ;;
*) sed -n 2,10p "$0"; exit 1 ;;
esac
