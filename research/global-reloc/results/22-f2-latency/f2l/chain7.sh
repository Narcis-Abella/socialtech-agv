#!/usr/bin/env bash
# after chain6: h27 repeated 5 times on the final tree (in y1 its pose was published, correct, and AMCL then never confirmed)
export R=/home/jetson/work/reloc-f2l-20261011
cd $R
until [ -e chain6.done ]; do sleep 30; done
for t in y2 y3 y4 y5 y6; do bash tools/global_reloc_3d/batch_latency.sh $t h27 > batch-$t.log 2>&1; done
touch chain7.done
