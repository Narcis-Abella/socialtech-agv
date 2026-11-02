#!/bin/bash
# usage (inside the container, this directory mounted at /g): harness.sh INSTALL_DIR N OUT
# N SIGINT stops of gn_loc + the Nav2 lifecycle manager (+ synthetic scans and TF, SYNTH=1; bond on both sides, BOND=1). HOG=n busy loops squeeze the stop.
# A stop is clean when the pre-shutdown hook ran and nothing aborted, segfaulted or had to be killed.
. /ros_env.sh >/dev/null 2>&1
. /g/$1/setup.bash
export ROS_DOMAIN_ID=77 CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"lo\"/></Interfaces></General><Discovery><Peers><Peer address=\"localhost\"/></Peers></Discovery></Domain></CycloneDDS>"
mkdir -p /g/$3; : > /g/$3/summary.txt; bad=0; ok=0
hogs=(); trap 'kill "${hogs[@]}" 2>/dev/null' EXIT
for h in $(seq 1 ${HOG:-0}); do ( while :; do :; done ) & hogs+=($!); done
for i in $(seq 1 $2); do
  log=/g/$3/run$i.log
  timeout -s INT -k 40 9 ros2 launch /g/harness.launch.py > $log 2>&1; rc=$?     # foreground: a SIGINT to a background job of a non-interactive shell is ignored
  hook=$(grep -c 'Running gn_loc rcl preshutdown' $log)
  fail=$(grep -c 'terminate called\|Segmentation\|core dumped\|process has died.*exit code -\(6\|11\)' $log)
  if [ "$fail" -gt 0 ] || [ "$rc" = 137 ] || [ "$hook" = 0 ]; then bad=$((bad+1)); else ok=$((ok+1)); fi
  echo "run $i: rc=$rc hook=$hook failures=$fail" >> /g/$3/summary.txt
done
echo "BAD STOPS $bad / $2 (clean $ok)" | tee -a /g/$3/summary.txt
