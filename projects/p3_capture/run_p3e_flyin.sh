#!/bin/bash
# 3E E1b: fly-in from the spin-axis hold + grasp, 100 validation cases per spin (SPEC_3 11)
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p3_capture
F='Module |lock file|Warp|CUDA|Devices|cuda:0|Kernel cache|/home'
for S in 0 0.5 1 2; do
bash $D/../../tools/rlrun.sh python ../projects/p3_capture/evaluate_p3b.py --n 100 --mode free-flying --spin $S \
  --set flyin=true --out $D/results_p3e_flyin_s$S.json 2>&1 | grep -v -E "$F" > $D/eval_p3e_flyin_s$S.log
done
echo ALLDONE
