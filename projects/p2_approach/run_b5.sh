#!/bin/bash
# B5: every 20 s controller with and without the shield, validation (SPEC_2 17)
# usage: run_b5.sh <unc> <n> <tag> [extra evaluate args]
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p2_approach
U=$1; N=$2; T=$3; shift 3
bash $D/../../tools/rlrun.sh python ../projects/p2_approach/evaluate.py --env residual \
  --unc $U --n $N --nav --raw-epoch --epoch-only --shield --cw-deadband 0.002 \
  --checkpoint ../projects/p2_approach/logs/r1/model_599.pt \
  --out ../projects/p2_approach/results_b5_${T}_${U}.json "$@" 2>&1 \
  | grep -v -E 'Module |lock file' > $D/eval_b5_${T}_${U}.log
bash $D/../../tools/rlrun.sh python ../projects/p2_approach/evaluate.py --env maneuver \
  --unc $U --n $N --epoch-only --shield-only --cw-deadband 0.002 \
  --checkpoint ../projects/p2_approach/logs/u1/model_2099.pt \
  --out ../projects/p2_approach/results_b5_${T}_ppo_${U}.json "$@" 2>&1 \
  | grep -v -E 'Module |lock file' > $D/eval_b5_${T}_ppo_${U}.log
