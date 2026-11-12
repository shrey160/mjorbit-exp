#!/bin/bash
# 3E: grasp a spinning client, free-flying, validation (SPEC_3 section 10)
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p3_capture
F='Module |lock file|Warp|CUDA|Devices|cuda:0|Kernel cache|/home'
for S in "$@"; do
bash $D/../../tools/rlrun.sh python ../projects/p3_capture/evaluate_p3b.py --n 100 --mode free-flying --spin $S \
  --out $D/results_p3e_grasp_s$S.json 2>&1 | grep -v -E "$F" > $D/eval_p3e_grasp_s$S.log
echo "== spin $S"; grep -E '^(success |grasped|other|push_dv_mm_s_max|peak_contact|bus_att|approach|final_slip|bar_shift|clamp)' $D/eval_p3e_grasp_s$S.log
grep -A3 "^failures" $D/eval_p3e_grasp_s$S.log | head -4
done
