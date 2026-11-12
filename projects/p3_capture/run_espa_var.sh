#!/bin/bash
# ESPA geometry variants, 3B grasp of a still client (push debug): bash run_espa_var.sh
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p3_capture
cd $D/../../tools
for v in d125 d145 sh0; do
  (bash rlrun.sh python ../projects/p3_capture/evaluate_p3b.py --n 4 --workers 4 --set scene=p3e_espa_v_$v.xml 2>&1 | grep -E "^success |push_dv|bus_att|bar_shift" | sed "s/^/$v /") &
done
wait
