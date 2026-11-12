#!/bin/bash
# 3D variants on 40 matched validation cases (SPEC_3 section 8)
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p3_capture
bash $D/../../tools/rlrun.sh python ../projects/p3_capture/evaluate_p3d.py --n 40 \
  --out $D/results_p3d_v3.json "$@" 2>&1 | grep -v -E 'Module |lock file|Warp|CUDA|Devices|cuda:0|Kernel cache|/home' > $D/eval_p3d_v3.log
echo ALLDONE
