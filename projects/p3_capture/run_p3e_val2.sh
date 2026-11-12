#!/bin/bash
# E2: thruster faults on the fresh val2 split (seeds 4000000+) after the re-pointing fix
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p3_capture
for f in thruster_weak thruster_stuck; do
  for m in act rec; do
    extra=""; [ $m = rec ] && extra="--no-act"
    echo "== $f $m"
    bash $D/run_p3e_safety.sh val2_${f}_$m --n 100 --split val2 --fault $f $extra --workers ${W:-8} \
      | grep -E "^outcomes|detect_rate|detect_reasons|false_al|delay|t_to_safe|min_clear|hit_force"
  done
done
