#!/bin/bash
# Evaluate r1 residual checkpoints on validation under u1 (SPEC_2 16)
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p2_approach
for it in "$@"; do
  bash $D/../../tools/rlrun.sh python ../projects/p2_approach/evaluate.py --env residual \
    --unc u1 --nav --skip-pd --cw-deadband 0.002 \
    --checkpoint ../projects/p2_approach/logs/r1/model_$it.pt \
    --out ../projects/p2_approach/results_val_r1_${it}_u1.json 2>&1 \
    | grep -v -E 'Module |lock file' > $D/eval_r1_$it.log
done
