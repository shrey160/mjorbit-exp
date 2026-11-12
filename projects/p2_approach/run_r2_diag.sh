#!/bin/bash
# r2/599 with u1 split into nav-only and dynamics-only parts, validation 200 (SPEC_2 19)
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p2_approach
for U in u1nav u1dyn; do
  bash $D/../../tools/rlrun.sh python ../projects/p2_approach/evaluate.py --env residual --base gs \
    --unc $U --nav --no-cw --epoch-only --shield-only --cw-deadband 0.002 \
    --checkpoint ../projects/p2_approach/logs/r2/model_599.pt \
    --out ../projects/p2_approach/results_r2_diag_${U}.json 2>&1 \
    | grep -v -E 'Module |lock file' > $D/eval_r2_diag_$U.log
done
