#!/bin/bash
# B3 evaluation: filtered classical controllers vs PPO u1/2099, validation, u1 and none.
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p2_approach
for u in u1 none; do
  bash $D/../../tools/rlrun.sh python ../projects/p2_approach/evaluate.py --env maneuver \
    --unc $u --kf --nav --skip-pd --cw-deadband 0.002 0.005 \
    --checkpoint ../projects/p2_approach/logs/u1/model_2099.pt \
    --out ../projects/p2_approach/results_val_nav9_$u.json 2>&1 \
    | grep -v -E 'Module |lock file' > $D/eval_nav9_$u.log
done
