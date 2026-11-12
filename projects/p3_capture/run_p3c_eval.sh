#!/bin/bash
# 3C evaluation: tag, then evaluate_p3c args (SPEC_3 section 7)
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p3_capture
T=$1; shift
bash $D/../../tools/rlrun.sh python ../projects/p3_capture/evaluate_p3c.py --out $D/results_p3c_$T.json "$@" 2>&1 \
  | grep -v -E 'Module |lock file|Warp|CUDA|Devices|cuda:0|Kernel cache|/home' > $D/eval_p3c_$T.log
tail -20 $D/eval_p3c_$T.log
