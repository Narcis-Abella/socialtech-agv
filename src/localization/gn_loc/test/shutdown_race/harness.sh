#!/bin/bash
# usage (inside the container, this directory mounted at /g): harness.sh INSTALL_DIR N OUT
# N stops of gn_loc + the Nav2 lifecycle manager + synthetic scans and TF (bond on both sides, BOND=1). HOG=n busy loops squeeze the stop.
# Each stop waits until synth.py sees gn_loc TRACKING (else the run is INVALID, not clean), waits a random 0-1.5 s, then sends SIGINT to the
# whole launch process group (what Ctrl-C does). MODE=lcshutdown first takes gn_loc ACTIVE -> FINALIZED with `ros2 lifecycle set /gn_loc shutdown`.
# Env passed through to the nodes: SCAN_HZ (synth), GN_SCAN_DELAY_MS (instrumented builds only).
# A stop is a HANG when launch had to escalate gn_loc past SIGINT (it sends SIGTERM, then SIGKILL) or the harness had to SIGKILL the group.
. /ros_env.sh >/dev/null 2>&1
. /g/$1/setup.bash
export ROS_DOMAIN_ID=77 CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"lo\"/></Interfaces></General><Discovery><Peers><Peer address=\"localhost\"/></Peers></Discovery></Domain></CycloneDDS>"
set_cyclone_uri   # this export replaced the URI of ros_env.sh: ask again for the 10 MB receive buffer (docker/cyclone_uri.sh)
set -m   # job control: the launch gets its own process group and keeps SIGINT (a background job of a non-interactive shell ignores it)
out=/g/$3; mkdir -p $out; : > $out/summary.txt
hogs=(); trap 'kill "${hogs[@]}" 2>/dev/null' EXIT
for h in $(seq 1 ${HOG:-0}); do ( while :; do :; done ) & hogs+=($!); done
for i in $(seq 1 $2); do
  log=$out/run$i.log
  ros2 launch /g/harness.launch.py > $log 2>&1 &
  pid=$!
  ready=0; for t in $(seq 1 600); do grep -q SYNTH_TRACKING $log && { ready=1; break; }; sleep 0.1; done   # 60 s
  lc=-
  if [ $ready = 1 ] && [ "${MODE:-sigint}" = lcshutdown ]; then
    timeout 60 ros2 lifecycle set /gn_loc shutdown > $out/lc$i.txt 2>&1; lc=$?
    sleep 1
  fi
  sleep $(awk -v s=$RANDOM 'BEGIN{srand(s); printf "%.2f", rand()*1.5}')
  t0=$(date +%s.%N)
  kill -INT -- -$pid
  hang=0; for t in $(seq 1 400); do kill -0 $pid 2>/dev/null || break; sleep 0.1; done   # 40 s
  if kill -0 $pid 2>/dev/null; then hang=1; kill -KILL -- -$pid; fi
  wait $pid; rc=$?
  dt=$(awk -v a=$t0 -v b=$(date +%s.%N) 'BEGIN{printf "%.1f", b-a}')
  g=$(grep -a 'gn_loc_node-1\]\|gn_loc_node-1:' $log)
  hook=$(echo "$g" | grep -c 'Running gn_loc rcl preshutdown')
  abort=$(echo "$g" | grep -c 'terminate called\|exit code -6')
  segv=$(echo "$g" | grep -c 'Segmentation\|exit code -11\|core dumped')
  esc=$(grep -a 'gn_loc_node-1' $log | grep -c "escalating to 'SIG\(TERM\|KILL\)'")
  exitline=$(grep -a 'gn_loc_node-1\]: process has' $log | head -1 | sed 's/.*process has finished cleanly.*/0/; s/.*exit code \(-*[0-9]*\).*/\1/')
  other=$(grep -a 'terminate called\|exit code -\(6\|11\)\|Segmentation' $log | grep -vc 'gn_loc_node-1')
  echo "run $i: ready=$ready lc=$lc rc=$rc dt=${dt} hook=$hook abort=$abort segv=$segv escalated=$esc harness_kill=$hang other_fail=$other exit=$exitline" >> $out/summary.txt
done
awk '{for(k=3;k<=NF;k++){split($k,a,"="); v[a[1]]=a[2]}
      n++; if(v["ready"]!=1){inv++; next}
      if(v["abort"]>0)ab++; if(v["segv"]>0)sg++; if(v["escalated"]>0||v["harness_kill"]>0)hg++; if(v["hook"]>0)hk++; if(v["other_fail"]>0)ot++
      if(v["abort"]==0&&v["segv"]==0&&v["escalated"]==0&&v["harness_kill"]==0)cl++}
     END{printf "TOTAL stops=%d invalid(not TRACKING)=%d valid=%d clean=%d abort=%d segv=%d hang=%d hook_ran=%d other_node_fail=%d\n", n, inv, n-inv, cl, ab, sg, hg, hk, ot}' $out/summary.txt | tee -a $out/summary.txt
