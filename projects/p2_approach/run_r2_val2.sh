#!/bin/bash
# r2/599 vs its glideslope base (a=0) and CW + nav9, all shielded, fresh val2 split (SPEC_2 19)
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p2_approach
run() {  # tag unc n extra...
  T=$1; U=$2; N=$3; shift 3
  bash $D/../../tools/rlrun.sh python ../projects/p2_approach/evaluate.py --env residual --base gs \
    --split val2 --unc $U --n $N --nav --epoch-only --shield-only --cw-deadband 0.002 \
    --checkpoint ../projects/p2_approach/logs/r2/model_599.pt \
    --out ../projects/p2_approach/results_r2_${T}_${U}.json "$@" 2>&1 \
    | grep -v -E 'Module |lock file' > $D/eval_r2_${T}_${U}.log
}
run val2 u1 1000
run val2 none 1000
run val2dt01 u1 200 --physics-dt 0.1
echo ALLDONE
