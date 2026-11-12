#!/bin/bash
# run dbg_e2 for several seeds in parallel: bash run_dbg_e2_multi.sh <fault> <seed>...
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p3_capture
f=$1; shift
for s in "$@"; do
  ( r=$(bash $D/run_dbg_e2.sh $s $f | grep -E 'outcome|t_to_safe|hit_first|min_clear' | tr '\n' ' '); echo "seed $s: $r" ) &
done
wait
