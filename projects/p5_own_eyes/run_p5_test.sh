#!/bin/bash
# P5.6: Project 5 test split, run ONCE, settings frozen (nav_anom 60, fly-in a_max 1.5e-3, k = 2, drift filter).
# Test seeds 7,000,000-024 (TUNE + 2,000,000). Same chain as p5_cam_fix_stow.json / p5_fault_est.json.
# bash run_p5_test.sh nominal|faults
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p5_own_eyes
R="bash $D/run_p5cam.sh cam_eval.py --conds nom:60,0.05,1,0.3 --cam-fix --n 25 --start 2000000 --workers 8"
case $1 in
  nominal) $R --out $D/p5_test_nom.json ;;
  faults)  $R --faults client_kick thruster_weak thruster_stuck spin_axis_err --out $D/p5_test_fault.json ;;
esac
