#!/bin/sh
# usage (host): variants.sh INSTALL  — runs the squeezed shutdown test for the two discriminating variants; results in ~/gnabort/<INSTALL>_A, _B
cd ~/gnabort
for v in A C B; do
  case $v in A) synth=0; bond=1 ;; B) synth=1; bond=0 ;; C) synth=1; bond=1 ;; esac
  rm -f $1_$v/summary.txt $1_$v/run*.log
  docker run --rm --cpuset-cpus 0-1 -e HOG=3 -e SYNTH=$synth -e BOND=$bond --user $(id -u):$(id -g) -e HOME=/tmp -v $PWD:/g socialtech:robot-navtest bash /g/harness.sh $1 20 $1_$v > $1_$v.run 2>&1
done
echo done > variants.done
