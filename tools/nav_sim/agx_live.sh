#!/bin/bash
# Laptop side: runs a scenario on the AGX with DDS on its wifi, so rviz2 on this laptop shows the Nav2 stack live (PEER=<AGX IP> rviz_laptop.sh in another terminal). No sudo.
#   LAPTOP=<laptop LAN IP> agx_live.sh start <scenario.json under tools/> <env file>   env file: KEY=VALUE lines as in an agx_queue.sh block (MAP, SIM_ARGS, MERGE ...)
#   agx_live.sh stop
# env: LAPTOP (required for start), DDS_IFACE (AGX interface, wlP1p1s0), IMAGE (socialtech:sim-nav2), SIM_REPO (~/Documents/GitHub/hospital-agv-sim, committed). Results land in <remote dir>/out.
set -eo pipefail
KEY=~/.nvsdkm/.ssh/id_rsa_sdkm; HOST=jetson@socialtech-agx-1
ssh_() { ssh -i "$KEY" -o ConnectTimeout=10 "$HOST" "$@"; }
repo=$(cd "$(dirname "$0")/../.." && pwd)
case "$1" in
start)
  scen=${2:?scenario under tools/}; envf=${3:?env file}; : "${LAPTOP:?set LAPTOP to the LAN IP of this laptop}"
  sim=${SIM_REPO:-$HOME/Documents/GitHub/hospital-agv-sim}; hash=$(git -C "$sim" rev-parse --short HEAD)
  remote=nav2runs/live_$(date +%y%m%d_%H%M%S)
  ssh_ "mkdir -p $remote/out && [ -f ~/sim_src/hospital-agv-sim_$hash/package.xml ]"
  tar -C "$repo" -c tools | ssh_ "tar -x -C $remote"
  scp -q -i "$KEY" "$envf" "$HOST:$remote/user.env"
  iface=${DDS_IFACE:-wlP1p1s0}
  ssh_ "cd $remote && { echo SIM_SRC=/simsrc; echo ROS_DOMAIN_ID=53; echo DDS_IFACE=$iface; echo 'CYCLONEDDS_URI=<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$iface\"/></Interfaces></General><Discovery><Peers><Peer address=\"$LAPTOP\"/></Peers><MaxAutoParticipantIndex>30</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'; cat user.env; } > env.list"
  ssh -f -n -i "$KEY" "$HOST" "cd $remote && setsid nohup sh -c 'docker run --rm --name nav2run --network host --env-file env.list -v \$PWD/tools:/tools:ro -v \$PWD/out:/out -v \$HOME/sim_src/hospital-agv-sim_$hash:/simsrc:ro ${IMAGE:-socialtech:sim-nav2} /tools/nav_sim/run_trials.sh /tools/$scen /out 1 > run.log 2>&1; echo rc=\$? >> run.log' > /dev/null 2>&1 &" > /dev/null 2>&1
  echo "$remote" ;;
stop) ssh_ "docker stop nav2run" ;;
*) sed -n 2,5p "$0"; exit 1 ;;
esac
