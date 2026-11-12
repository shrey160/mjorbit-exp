#!/bin/bash
# E2b: grip monitor acting (safe the stack) vs record only, random loose-jaw fault 0.05-0.3 mm
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p3_capture
SPLIT=${SPLIT:-tune}; N=${N:-32}; W=${W:-8}; EXTRA=${EXTRA:-}
for m in act rec; do
  x=""; [ "$m" = rec ] && x="--no-act"
  echo "== $m"
  bash $D/run_p3e_grip.sh ${SPLIT}_$m --n $N --loose -1 $x --split $SPLIT --workers $W $EXTRA \
    | grep -E "retained|lost|alarm|safed|shift_mm_max_max|mass"
done
