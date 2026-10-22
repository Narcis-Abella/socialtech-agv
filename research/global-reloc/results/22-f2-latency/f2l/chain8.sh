#!/usr/bin/env bash
# after chain7: h25 again (the f2l row is INVALID: a 26.9 s storage stall starved the bag player before FAST-LIO2 was killed)
export R=/home/jetson/work/reloc-f2l-20261011
cd $R
until [ -e chain7.done ]; do sleep 30; done
bash tools/global_reloc_3d/batch_faults_f2.sh f2l2 h25 > batch-f2l2.log 2>&1
touch chain8.done
