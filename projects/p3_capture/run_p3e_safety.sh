#!/bin/bash
# E2 safety evaluation: bash run_p3e_safety.sh <tag> [evaluate_p3e_safety.py args...]
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p3_capture
tag=$1; shift
bash $D/../../tools/rlrun.sh python ../projects/p3_capture/evaluate_p3e_safety.py "$@" \
  --out $D/results_p3e_safety_$tag.json 2>&1 | grep -v -E 'Module |lock file'
