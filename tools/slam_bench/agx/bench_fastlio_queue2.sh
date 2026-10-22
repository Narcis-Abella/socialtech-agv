#!/bin/bash
cd ~
log=~/bench_out/queue.log
run() { echo "$(date -Is) start fastlio $1" >> $log; ~/bench_fastlio.sh "$1" "$2" "$3" "$4" > ~/bench_out/fastlio_$1.log 2>&1; echo "$(date -Is) end fastlio $1 rc=$? poses=$(wc -l < ~/bench_out/fastlio_$1/odom.tum)" >> $log; }
for s in Corridor01 Dynamic01 Sha-turn01 Elevator01; do run m3dgr_$s m3dgr_${s}_custom /livox/mid360/lidar /livox/mid360/imu; done
echo "$(date -Is) fastlio queue done" >> $log
