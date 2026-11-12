#!/bin/bash
# E3 whole-mission Monte Carlo: bash run_e3.sh <split> <n> [workers]
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p3_capture
S=$1; N=$2; W=${3:-20}
R="bash $D/../../tools/rlrun.sh python ../projects/p3_capture/evaluate_e3.py"
$R A --n $N --split $S --out $D/e3_handoff_$S.json 2>&1 | grep -v -E 'Module |lock file|Warp|warp'
$R B --handoff $D/e3_handoff_$S.json --workers $W --out $D/results_e3_$S.json 2>&1 | grep -v -E 'Module |lock file'
