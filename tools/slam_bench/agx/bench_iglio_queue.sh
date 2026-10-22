#!/bin/bash
# iG-LIO on all benchmark sequences (PointCloud2 bags), one run each, real-time playback. Waits for the FAST-LIO2 queue.
cd ~
log=~/bench_out/queue.log
until grep -q "fastlio queue done" $log 2>/dev/null; do sleep 10; done
run() {
  echo "$(date -Is) start iglio $1" >> $log
  ~/bench_iglio.sh "$1" "$2" "$3" "$4" > ~/bench_out/iglio_$1.log 2>&1
  echo "$(date -Is) end iglio $1 rc=$? poses=$(wc -l < ~/bench_out/iglio_$1/odom.tum)" >> $log
}
run tiers_office1 tiers_IndoorOffice1 /mid360/livox/lidar /mid360/livox/imu
run tiers_office2 tiers_IndoorOffice2 /mid360/livox/lidar /mid360/livox/imu
for s in Corridor01 Dynamic01 Sha-turn01 Elevator01; do run m3dgr_$s m3dgr_$s /livox/mid360/lidar /livox/mid360/imu; done
echo "$(date -Is) iglio queue done" >> $log
