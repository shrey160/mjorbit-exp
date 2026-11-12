#!/bin/bash
# 3A baseline: u1, 200 validation cases, path-margin variants (SPEC_3 section 3)
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p3_capture
E="bash $D/../../tools/rlrun.sh python ../projects/p3_capture/evaluate_p3a.py --n 200"
F='Module |lock file|Warp|CUDA|Devices|cpu"|cuda:0|Kernel cache|/home'
$E --unc u1 --out $D/results_p3a_u1_base.json 2>&1 | grep -v -E "$F" > $D/eval_p3a_u1_base.log
$E --unc u1 --gkw seg_margin=3 r_safe=10 --out $D/results_p3a_u1_m3.json 2>&1 | grep -v -E "$F" > $D/eval_p3a_u1_m3.log
$E --unc u1 --gkw seg_margin=4 r_safe=12 --out $D/results_p3a_u1_m4.json 2>&1 | grep -v -E "$F" > $D/eval_p3a_u1_m4.log
echo ALLDONE
