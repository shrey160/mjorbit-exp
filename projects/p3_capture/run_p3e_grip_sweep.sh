#!/bin/bash
# E2b: how loose must the jaw brakes be before the 3C loads defeat the grip? (record only)
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p3_capture
for L in ${LS:-0.05 0.1 0.2 0.3 0.4}; do
  echo "== loose $L mm"
  bash $D/run_p3e_grip.sh sweep_$L --n 16 --loose $L --no-act --split tune --workers 8 --set spin_deg_s=2 impose_tumble=false \
    | grep -E "retained|lost|alarm|shift|slip|pad_off|clamp|mass"
done
