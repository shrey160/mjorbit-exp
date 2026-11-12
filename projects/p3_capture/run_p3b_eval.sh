#!/bin/bash
# 3B classical grasp: both bus modes, timestep sensitivity, validation (SPEC_3 section 5)
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p3_capture
E="bash $D/../../tools/rlrun.sh python ../projects/p3_capture/evaluate_p3b.py --n 200"
F='Module |lock file'
$E --mode free-flying --out $D/results_p3b_ff_v2.json 2>&1 | grep -v -E "$F" > $D/eval_p3b_ff_v2.log
$E --mode free-floating --out $D/results_p3b_float_v2.json 2>&1 | grep -v -E "$F" > $D/eval_p3b_float_v2.log
$E --mode free-flying --dt 0.001 --out $D/results_p3b_ff_dt001_v2.json 2>&1 | grep -v -E "$F" > $D/eval_p3b_ff_dt001_v2.log
$E --mode free-flying --dt 0.005 --out $D/results_p3b_ff_dt005_v2.json 2>&1 | grep -v -E "$F" > $D/eval_p3b_ff_dt005_v2.log
echo ALLDONE
