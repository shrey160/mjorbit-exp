#!/bin/bash
# E2b: re-clamp response (jaw brake released, drive squeezes again) on the same cases
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p3_capture
SPLIT=${SPLIT:-tune}; N=${N:-32}; W=${W:-8}; EXTRA=${EXTRA:-}
bash $D/run_p3e_grip.sh ${SPLIT}_reclamp --n $N --loose -1 --split $SPLIT --workers $W --set grip_response=reclamp $EXTRA \
  | grep -E "retained|lost|alarm|safed|shift_mm_max_max|mass"
