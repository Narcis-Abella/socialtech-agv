#!/usr/bin/env bash
# Final tree of note 22 (supervision every 0.5 s by a persistent worker, freshness rule, tail rule). One experiment at a time.
export R=/home/jetson/work/reloc-f2l-20261011
cd $R
T=tools/global_reloc_3d
B=/home/jetson/work/global-reloc-3d-20261009
BAG=/home/jetson/rosbags/custom/mapping_eco_-1_01_custom; YAML=/home/jetson/map_out/eco01_1x/map.yaml; XYZ=$B/isolated-v16/map/map.xyz
# 1. targeted latency batch (24 runs)
bash $T/batch_latency.sh y1 > batch-y1.log 2>&1
touch chain6a.done
# 2. F1 regressions and negatives (S2, S3, cross map, holes)
for c in cases-f1 cases-f1b cases-sn; do bash $T/batch_s2s3.sh $c.csv r2l >> batch-r2l.log 2>&1; done
# 3. the cases of the note-21 matrix that reach the hand-over or go beyond it (41 of 56); the other 15 end before /initialpose,
#    on code this change does not touch
for n in h00 h40 h05 h07 h08 h09 h10 h11 h19 h20 h42 h43 d01 d02 d03 d05 d06 d07 d08 d09 d10 h21 h22 h23 h24 h25 h26 h27 h44 h45 h28 h29 h30 h31 h32 h33 h34 h35 h36 h37 h38 h39; do
  bash $T/batch_faults_f2.sh f2l $n >> batch-f2l.log 2>&1
done
touch chain6b.done
# 4. supervision on the other maps and windows: restart cases kept running after LOCALIZED (no fault)
for i in 3 10 22 49 14 36 7 31 4 12 44; do
  RUN_ENV="KEEP_PLAYING=1 PLAY_S=92 STOP_AT=85" bash $T/batch_restarts.sh cases-restarts50.csv sv $i $i >> batch-sv.log 2>&1
done
# 5. long no-fault run (memory, CPU, false withholding)
R=$R NAME=mem-eco01-c00 BAG=$BAG MAPYAML=$YAML MAPXYZ=$XYZ OFFSET=3.399710 TAG=m5 MODE=s2s3 KEEP_PLAYING=1 PLAY_S=515 LAUNCH_EXTRA="s3_auto_authorize:=true" bash $R/tools/global_reloc_3d/run_online.sh > /dev/null 2>&1
# 6. gate cell on virtual CAN (gate files unchanged; confirms the bypass stays closed with this tree)
mkdir -p gate-cell; FINAL=gate bash $T/gate_cell_test.sh $R/gate-cell/v3 > gate-cell-v3.log 2>&1
touch chain6.done
