#!/bin/bash
# 3A re-run on the rectangular client with a grapple bar (SPEC_3 section 3b), guidance m4
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p3_capture
E="bash $D/../../tools/rlrun.sh python ../projects/p3_capture/evaluate_p3a.py --n 200 --gkw seg_margin=4 r_safe=12"
F='Module |lock file|Warp|CUDA|Devices|cpu"|cuda:0|Kernel cache|/home'
for c in "u1 random lvlh" "u1 aligned lvlh" "u1 random inertial" "none random lvlh" "none random inertial"; do
  set -- $c
  T=v2_$1_$2_$3
  $E --unc $1 --yaw $2 --client-rot $3 --out $D/results_p3a_$T.json 2>&1 | grep -v -E "$F" > $D/eval_p3a_$T.log
done
echo ALLDONE
