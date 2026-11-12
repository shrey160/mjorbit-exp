#!/bin/bash
# E3 stage B only (resumable via <out>.partial.jsonl): bash run_e3B.sh <split> [workers]
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p3_capture
S=$1; W=${2:-20}
cd $D/../../tools
bash rlrun.sh python ../projects/p3_capture/evaluate_e3.py B --handoff $D/e3_handoff_$S.json --workers $W --out $D/results_e3_$S.json 2>&1 | grep -v -E 'Module |lock file'
