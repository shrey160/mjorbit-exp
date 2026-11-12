#!/bin/bash
# E2 threshold calibration on the tune split (monitors record only)
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p3_capture
bash $D/run_p3e_safety.sh tune_nominal --n 100 --no-act --split tune --workers 16 | tail -8
for f in client_kick thruster_weak thruster_stuck squeeze_loss spin_axis_err; do
  echo "== $f"
  bash $D/run_p3e_safety.sh tune_${f}_rec --n 40 --no-act --split tune --fault $f --workers 16 | grep -E "outcomes|detect_rate|detect_reasons|false_al|delay|min_clear|hit_force"
done
