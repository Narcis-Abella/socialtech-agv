#!/bin/bash
# Laptop side: runs a scenario on the AGX in the sim image, or fetches the results. No sudo; refuses when the board is busy.
#   agx_run.sh start <name> <scenario.json under tools/> <reps> [KEY=VALUE ...]   ships tools/ and the sim commit, starts run_trials.sh in the background, prints the remote dir
#   agx_run.sh wait <remote dir>                                                  blocks until the run has ended and prints its last lines
#   agx_run.sh fetch <remote dir> <local dir>                                     copies the output (series, logs, cost) to the laptop
# env: IMAGE (socialtech:sim-main), SIM_REPO (~/Documents/GitHub/hospital-agv-sim, must be committed: its short hash names the copy on the board), RUN_TIMEOUT (7200 s)
# KEY=VALUE pairs go to the container (PARAMS, SIM_ARGS, INIT_SIGMA_XY, TRIAL_TIMEOUT ...); PARAMS and scenario paths are relative to /tools or absolute in the container.
set -eo pipefail
KEY=~/.nvsdkm/.ssh/id_rsa_sdkm; HOST=jetson@socialtech-agx-1
ssh_() { ssh -i "$KEY" -o ConnectTimeout=10 "$HOST" "$@"; }
repo=$(cd "$(dirname "$0")/../.." && pwd)
case "$1" in
start)
  name=${2:?name}; scen=${3:?scenario}; reps=${4:?reps}; shift 4
  sim=${SIM_REPO:-$HOME/Documents/GitHub/hospital-agv-sim}
  [ -z "$(git -C "$sim" status --porcelain)" ] || { echo "the sim repo has uncommitted changes" >&2; exit 1; }
  hash=$(git -C "$sim" rev-parse --short HEAD)
  read -r nct load <<< "$(ssh_ 'echo $(docker ps -q | wc -l) $(cut -d" " -f1 /proc/loadavg)')"
  { [ "$nct" = 0 ] && awk -v l="$load" 'BEGIN{exit !(l < 2)}'; } || { echo "the AGX is busy (containers=$nct load=$load)" >&2; exit 1; }
  remote=nav2runs/${name}_$(date +%y%m%d_%H%M%S)
  ssh_ "mkdir -p $remote/out ~/sim_src/hospital-agv-sim_$hash"
  tar -C "$repo" -c tools | ssh_ "tar -x -C $remote"
  ssh_ "[ -f ~/sim_src/hospital-agv-sim_$hash/package.xml ]" || git -C "$sim" archive HEAD | ssh_ "tar -x -C ~/sim_src/hospital-agv-sim_$hash"
  { echo "SIM_SRC=/simsrc"; echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-53}"; printf '%s\n' "$@"; echo "SIM_COMMIT=$hash"; echo "REPO_COMMIT=$(git -C "$repo" rev-parse --short HEAD)"; } | ssh_ "cat > $remote/env.list"
  ssh -f -n -i "$KEY" "$HOST" "cd $remote && setsid nohup sh -c 'timeout ${RUN_TIMEOUT:-7200} docker run --rm --name nav2run --network host --env-file env.list -e DDS_IFACE=lo -v \$PWD/tools:/tools:ro -v \$PWD/out:/out -v \$HOME/sim_src/hospital-agv-sim_$hash:/simsrc:ro ${IMAGE:-socialtech:sim-main} /tools/nav_sim/run_trials.sh /tools/$scen /out $reps > run.log 2>&1; echo rc=\$? >> run.log; docker kill nav2run > /dev/null 2>&1' > /dev/null 2>&1 &"
  echo "$remote" ;;
wait)
  until ssh_ "grep -q '^rc=' ${2:?remote dir}/run.log" 2> /dev/null; do sleep 20; done
  ssh_ "tail -4 $2/run.log | cut -c1-200" ;;
fetch)
  mkdir -p "${3:?local dir}"
  ssh_ "tar -C ${2:?remote dir} -c out run.log env.list" | tar -x -C "$3" ;;
*) sed -n 2,7p "$0"; exit 1 ;;
esac
