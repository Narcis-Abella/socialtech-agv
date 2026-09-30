#!/bin/bash
# FAST-LIO2 then iG-LIO on all six benchmark sequences at 0.5x playback (real time under load dropped messages and made iG-LIO diverge).
cd ~
log=~/bench_out/queue.log
runf() { echo "$(date -Is) start fastlio r05_$1" >> $log; ~/bench_fastlio.sh "r05_$1" "$2" "$3" "$4" > ~/bench_out/fastlio_r05_$1.log 2>&1; echo "$(date -Is) end fastlio r05_$1 rc=$? poses=$(wc -l < ~/bench_out/fastlio_r05_$1/odom.tum)" >> $log; }
runi() { echo "$(date -Is) start iglio r05_$1" >> $log; ~/bench_iglio.sh "r05_$1" "$2" "$3" "$4" > ~/bench_out/iglio_r05_$1.log 2>&1; echo "$(date -Is) end iglio r05_$1 rc=$? poses=$(wc -l < ~/bench_out/iglio_r05_$1/odom.tum)" >> $log; }
T=/mid360/livox; M=/livox/mid360
runf tiers_office1 tiers_IndoorOffice1_custom $T/lidar $T/imu
runf tiers_office2 tiers_IndoorOffice2_custom $T/lidar $T/imu
for s in Corridor01 Dynamic01 Sha-turn01 Elevator01; do runf m3dgr_$s m3dgr_${s}_custom $M/lidar $M/imu; done
runi tiers_office1 tiers_IndoorOffice1 $T/lidar $T/imu
runi tiers_office2 tiers_IndoorOffice2 $T/lidar $T/imu
for s in Corridor01 Dynamic01 Sha-turn01 Elevator01; do runi m3dgr_$s m3dgr_$s $M/lidar $M/imu; done
echo "$(date -Is) iglio queue done" >> $log
