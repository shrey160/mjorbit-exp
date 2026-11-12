#!/bin/bash
# E3 regression after the validation fixes: first 200 validation missions, stage B only
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p3_capture
cd $D/../../tools
SEEDS=$(seq 6000000 6000199 | tr '\n' ' ')
bash rlrun.sh python ../projects/p3_capture/evaluate_e3.py B --handoff $D/e3_handoff_validation.json --workers 20 --seeds $SEEDS --out $D/results_e3_regress.json 2>&1 | grep -v -E 'Module |lock file'
