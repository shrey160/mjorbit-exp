#!/bin/bash
# Glideslope hybrid (k 0.03, v_max 0.1, v_min 0.003, switch 3 m, CW arrive 440 s; chosen on
# validation) vs CW + nav9, r1/599 and direct PPO, all shielded, on the fresh val2 split
# (seed 2000), 1000 cases, u1 and none; plus dt 0.1 at 200 cases (SPEC_2 19)
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p2_approach
GS=0.03,0.1,0.003,3,440
run() {  # tag unc n extra...
  T=$1; U=$2; N=$3; shift 3
  bash $D/../../tools/rlrun.sh python ../projects/p2_approach/evaluate.py --env residual --split val2 \
    --unc $U --n $N --nav --epoch-only --shield-only --cw-deadband 0.002 --glideslope $GS \
    --checkpoint ../projects/p2_approach/logs/r1/model_599.pt \
    --out ../projects/p2_approach/results_gs_${T}_${U}.json "$@" 2>&1 \
    | grep -v -E 'Module |lock file' > $D/eval_gs_${T}_${U}.log
  bash $D/../../tools/rlrun.sh python ../projects/p2_approach/evaluate.py --env maneuver --split val2 \
    --unc $U --n $N --nav --no-cw --epoch-only --shield-only --cw-deadband 0.002 \
    --checkpoint ../projects/p2_approach/logs/u1/model_2099.pt \
    --out ../projects/p2_approach/results_gs_${T}_ppo_${U}.json "$@" 2>&1 \
    | grep -v -E 'Module |lock file' > $D/eval_gs_${T}_ppo_${U}.log
}
run val2 u1 1000
run val2 none 1000
run val2dt01 u1 200 --physics-dt 0.1
echo ALLDONE
