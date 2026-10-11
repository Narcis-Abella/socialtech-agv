#!/usr/bin/env bash
export R=/home/jetson/work/reloc-f2g-20261011; cd $R; T=tools/global_reloc_3d
for c in h00 d07 d08 h28 h27 d02 d03 h33 h32; do bash $T/batch_faults_f2.sh t4 $c; done > batch-t4.log 2>&1
for tag in t5 t6; do for c in h27 d07 d08; do bash $T/batch_faults_f2.sh $tag $c; done; done >> batch-t4.log 2>&1
bash $T/batch_s2s3.sh cases-f1.csv t4 > batch-t4-s2s3.log 2>&1
touch targeted2.done
