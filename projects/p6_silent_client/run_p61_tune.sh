#!/bin/bash
# P6.1 tuning series on tune seeds, one run at a time: filter noise inflation and rate random walk
# at the nominal vision condition (10 mm x range/2.7, 1 deg, 2% outliers, 5% flips, 1 dropout/min).
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p6_silent_client
R="bash $D/run_p6.sh vis_eval.py --conds nom:10,1,0.02,0.05,1 --n 25 --workers 8"
for c in "a:--r-inf 1" "b:--r-inf 3" "c:--r-inf 7" "d:--r-inf 3 --sig-w 2e-6"; do
  tag=${c%%:*}; args=${c#*:}
  echo "=== $tag: $args"
  $R $args --out $D/p61_tune_$tag.json | grep -v -E '^  [0-9]+/'
done
