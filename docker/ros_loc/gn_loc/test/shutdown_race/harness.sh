#!/bin/bash
# usage: harness.sh INSTALL_DIR N OUT   (inside the container, /g mounted). A real SIGINT (timeout in the foreground), as Ctrl-C / docker stop deliver it.
. /ros_env.sh >/dev/null 2>&1
. /g/$1/setup.bash
export ROS_DOMAIN_ID=77 CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"lo\"/></Interfaces></General><Discovery><Peers><Peer address=\"localhost\"/></Peers></Discovery></Domain></CycloneDDS>"
mkdir -p /g/$3; ab=0; ok=0
hogs=(); for h in $(seq 1 ${HOG:-0}); do ( while :; do :; done ) & hogs+=($!); done   # HOG=n: n busy loops, to squeeze the shutdown (cpuset of the container)
for i in $(seq 1 $2); do
  log=/g/$3/run$i.log
  timeout -s INT -k 40 9 ros2 launch /g/harness.launch.py > $log 2>&1
  if grep -q "terminate called" $log; then ab=$((ab+1)); else ok=$((ok+1)); fi
  echo "run $i: $(grep -c 'terminate called' $log) abort, shutdown-seen=$(grep -c 'rcl preshutdown\|process has finished\|has died' $log) bond-active=$(grep -c 'connected with bond' $log)" >> /g/$3/summary.txt
done
kill "${hogs[@]}" 2>/dev/null
echo "ABORTS $ab / $2 (clean $ok)" | tee -a /g/$3/summary.txt
