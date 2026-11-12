#!/bin/bash
# square grapple bar: 3B grasp (200, both modes) and 3C identification (24)
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p3_capture
F='Module |lock file|Warp|CUDA|Devices|cuda:0|Kernel cache|/home'
for M in free-flying free-floating; do
bash $D/../../tools/rlrun.sh python ../projects/p3_capture/evaluate_p3b.py --n 200 --mode $M --set scene=p3b_scene_sq.xml \
  --out $D/results_p3b_sq_$M.json 2>&1 | grep -v -E "$F" > $D/eval_p3b_sq_$M.log
done
bash $D/run_p3c_eval.sh sq --n 24 --set scene=p3b_scene_sq.xml > /dev/null
echo ALLDONE
