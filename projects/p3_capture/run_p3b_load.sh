#!/bin/bash
# 3B grip load test (SPEC_3 section 6)
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p3_capture
bash $D/../../tools/rlrun.sh python ../projects/p3_capture/loadtest_p3b.py --n 10 \
  --out $D/results_p3b_load${TAG}.json "$@" 2>&1 | grep -v -E 'Module |lock file|Warp|CUDA|Devices|cuda:0|Kernel cache|/home' > $D/eval_p3b_load${TAG}.log
echo ALLDONE
