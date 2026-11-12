#!/bin/bash
# 3A: aligned client and inertially fixed client, u1 and none, guidance m4 (SPEC_3 section 3)
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p3_capture
E="bash $D/../../tools/rlrun.sh python ../projects/p3_capture/evaluate_p3a.py --n 200 --gkw seg_margin=4 r_safe=12"
F='Module |lock file|Warp|CUDA|Devices|cpu"|cuda:0|Kernel cache|/home'
$E --unc u1 --yaw aligned --out $D/results_p3a_u1_aligned.json 2>&1 | grep -v -E "$F" > $D/eval_p3a_u1_aligned.log
$E --unc u1 --client-rot inertial --out $D/results_p3a_u1_inertial.json 2>&1 | grep -v -E "$F" > $D/eval_p3a_u1_inertial.log
$E --unc none --out $D/results_p3a_none_m4.json 2>&1 | grep -v -E "$F" > $D/eval_p3a_none_m4.log
$E --unc none --client-rot inertial --out $D/results_p3a_none_inertial.json 2>&1 | grep -v -E "$F" > $D/eval_p3a_none_inertial.log
echo ALLDONE
