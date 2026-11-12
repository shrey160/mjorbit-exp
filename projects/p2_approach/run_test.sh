#!/bin/bash
# B6 test set (seed 1000), run once: chosen r1/599 + shield and the shielded benchmarks (SPEC_2 18)
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p2_approach
for U in u1 none; do
bash $D/../../tools/rlrun.sh python ../projects/p2_approach/evaluate.py --env residual --split test \
  --unc $U --n 1000 --nav --raw-epoch --epoch-only --shield --cw-deadband 0.002 \
  --checkpoint ../projects/p2_approach/logs/r1/model_599.pt \
  --out ../projects/p2_approach/results_test_r1_599_${U}.json 2>&1 \
  | grep -v -E 'Module |lock file' > $D/eval_test_r1_599_${U}.log
done
echo ALLDONE
