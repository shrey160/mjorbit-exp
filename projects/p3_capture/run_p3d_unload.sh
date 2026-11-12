#!/bin/bash
# 3D momentum management variants on 40 matched validation cases (SPEC_3 section 9)
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p3_capture
bash $D/../../tools/rlrun.sh python ../projects/p3_capture/evaluate_p3d.py --n 40 \
  --variants id/align id/align/unload=true id/align/w_max_deg=0.2 id/fixed/unload=true \
  --out $D/results_p3d_unload.json "$@" 2>&1 | grep -v -E 'Module |lock file|Warp|CUDA|Devices|cuda:0|Kernel cache|/home' > $D/eval_p3d_unload.log
echo ALLDONE
