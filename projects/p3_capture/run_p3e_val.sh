#!/bin/bash
# E2 validation: each fault with monitors acting vs record-only (= no monitor), matched seeds
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p3_capture
N=${N:-100}
for f in none client_kick thruster_weak thruster_stuck spin_axis_err; do
  ff=$f; [ "$f" = none ] && ff=""
  for m in act rec; do
    extra=""; [ $m = rec ] && extra="--no-act"
    echo "== $f $m"
    bash $D/run_p3e_safety.sh val_${f}_$m --n $N --split validation --fault "$ff" $extra --workers 16 \
      | grep -E "^outcomes|detect_rate|detect_reasons|false_al|delay|t_to_safe|min_clear|hit_force"
  done
done
