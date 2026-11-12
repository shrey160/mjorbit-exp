#!/bin/bash
# r2 (residual on glideslope base) checkpoints, validation u1, shielded (SPEC_2 19)
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p2_approach
for it in "$@"; do
  bash $D/../../tools/rlrun.sh python ../projects/p2_approach/evaluate.py --env residual --base gs \
    --unc u1 --nav --no-cw --epoch-only --shield-only --cw-deadband 0.002 \
    --checkpoint ../projects/p2_approach/logs/r2/model_$it.pt \
    --out ../projects/p2_approach/results_val_r2_${it}_u1.json 2>&1 \
    | grep -v -E 'Module |lock file' > $D/eval_r2_$it.log
done
echo ALLDONE
