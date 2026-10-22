#!/usr/bin/env bash
R=/home/jetson/work/reloc-f3-20261011; export R; cd $R
T=tools/global_reloc_3d
for t in b1 b2 b3; do bash $T/batch_s2s3.sh cases-f3.csv $t >> batch-$t.log 2>&1; done
for c in 4 2; do DOCKER_CPUS=$c bash $T/batch_s2s3.sh cases-f3.csv q$c >> batch-q$c.log 2>&1; done
touch chain9.done
export DOCKER_CPUS=2
bash $T/batch_latency.sh f3q2 k00 >> batch-f3q2.log 2>&1
bash $T/batch_latency.sh f3q2r k11_gyro_0p5_from_35 >> batch-f3q2r.log 2>&1
touch chain9b.done
