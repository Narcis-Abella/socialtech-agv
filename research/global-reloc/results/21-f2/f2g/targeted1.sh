#!/usr/bin/env bash
export R=/home/jetson/work/reloc-f2g-20261011; cd $R; T=tools/global_reloc_3d
for c in h40 h27 d0 d10 h44 h45 h28 h29 h30 h32 h33; do bash $T/batch_faults_f2.sh t1 $c; done > batch-t1.log 2>&1
for tag in t2 t3; do bash $T/batch_faults_f2.sh $tag h27; done >> batch-t1.log 2>&1
bash $T/batch_s2s3.sh cases-f1.csv t1 > batch-t1-s2s3.log 2>&1
touch targeted1.done
