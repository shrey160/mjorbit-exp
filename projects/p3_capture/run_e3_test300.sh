#!/bin/bash
# E3 test split, cut to the first 300 missions (decided 11 Nov 22:25 before any outcome was
# looked at; 6 missions done): stage B only, resumes from the checkpoint
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p3_capture
cd $D/../../tools
SEEDS=$(seq 7000000 7000299 | tr '\n' ' ')
bash rlrun.sh python ../projects/p3_capture/evaluate_e3.py B --handoff $D/e3_handoff_test.json --workers 20 --seeds $SEEDS --out $D/results_e3_test.json 2>&1 | grep -v -E 'Module |lock file'
