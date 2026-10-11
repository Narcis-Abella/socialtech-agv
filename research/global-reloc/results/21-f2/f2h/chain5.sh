#!/usr/bin/env bash
export R=/home/jetson/work/reloc-f2h-20261011
cd $R
T=tools/global_reloc_3d
B=/home/jetson/work/global-reloc-3d-20261009
BAG=/home/jetson/rosbags/custom/mapping_eco_-1_01_custom; YAML=/home/jetson/map_out/eco01_1x/map.yaml; XYZ=$B/isolated-v16/map/map.xyz
bash $T/batch_faults_f2.sh f2h > batch-f2h.log 2>&1
bash $T/batch_faults_s2s3.sh g2h > batch-g2h.log 2>&1
for c in cases-f1 cases-f1b cases-sn; do bash $T/batch_s2s3.sh $c.csv r2h >> batch-r2h.log 2>&1; done
R=$R NAME=mem-eco01-c00 BAG=$BAG MAPYAML=$YAML MAPXYZ=$XYZ OFFSET=3.399710 TAG=m4 MODE=s2s3 KEEP_PLAYING=1 PLAY_S=515 LAUNCH_EXTRA="s3_auto_authorize:=true" bash $R/tools/global_reloc_3d/run_online.sh > /dev/null 2>&1
bash $T/batch_restarts.sh cases-restarts50.csv rs50h 0 49 > batch-rs50h.log 2>&1
touch chain5a.done
