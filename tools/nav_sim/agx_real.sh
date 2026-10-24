#!/bin/bash
# Laptop side: runs real_sweeps.sh on the AGX (needs the recorded inputs in ~/nav_out and the maps in ~/nav_data there). Refuses when the board is busy. Prints the remote dir;
# use agx_run.sh wait / fetch on it. usage: agx_real.sh start <name>      env: IMAGE (socialtech:sim-nav2), PAR, RATE, BAGS, DATASETS (record_real_inputs.sh), VARIANTS (real_factorial.sh subset), COND_LIST (real_factorial.sh initial-pose conditions), SCRIPT (real_sweeps.sh; real_factorial.sh)
set -eo pipefail
KEY=~/.nvsdkm/.ssh/id_rsa_sdkm; HOST=jetson@socialtech-agx-1
ssh_() { ssh -i "$KEY" -o ConnectTimeout=10 "$HOST" "$@"; }
[ "$1" = start ] || { sed -n 2,4p "$0"; exit 1; }
repo=$(cd "$(dirname "$0")/../.." && pwd)
read -r nct load <<< "$(ssh_ 'echo $(docker ps -q | wc -l) $(cut -d" " -f1 /proc/loadavg)')"
{ [ "$nct" = 0 ] && awk -v l="$load" 'BEGIN{exit !(l < 2)}'; } || { echo "the AGX is busy (containers=$nct load=$load)" >&2; exit 1; }
remote=nav2runs/${2:?name}_$(date +%y%m%d_%H%M%S)
ssh_ "mkdir -p $remote/out ~/nav_inputs"
tar -C "$repo" -c tools | ssh_ "tar -x -C $remote"
ssh -f -n -i "$KEY" "$HOST" "cd $remote && setsid nohup sh -c 'docker run --rm --name nav2run --network host -e PAR=${PAR:-4} -e RATE=${RATE:-5} -e BAGS=\"${BAGS:-eco01 eco03}\" -e DATASETS=\"${DATASETS:-}\" -e VARIANTS=\"${VARIANTS:-}\" -e COND_LIST=\"${COND_LIST:-}\" -v \$PWD/tools:/tools:ro -v \$PWD/out:/out -v \$HOME/nav_out:/navout:ro -v \$HOME/nav_data:/data:ro -v \$HOME/rosbags:/bags:ro -v \$HOME/nav_inputs:/inputs ${IMAGE:-socialtech:sim-nav2} /tools/nav_sim/${SCRIPT:-real_sweeps.sh} /out > run.log 2>&1; echo rc=\$? >> run.log' > /dev/null 2>&1 &" > /dev/null 2>&1
echo "$remote"
