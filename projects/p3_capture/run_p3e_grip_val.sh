#!/bin/bash
# E2b validation (seeds 210000+): nominal false alarms, then the loose-jaw fault with no
# action / safe / re-clamp, client spinning at 2 deg/s; plus nominal under 3C's imposed tumble
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p3_capture
N=${N:-50}; W=${W:-8}
S="spin_deg_s=2 impose_tumble=false"
run() { tag=$1; shift; echo "== $tag"; bash $D/run_p3e_grip.sh val_$tag --n $N --split validation --workers $W "$@" \
  | grep -E "^retained|^lost|^alarms|alarm_kinds|^safed|shift_mm_max_max|slip_vel|mass"; }
run nominal_spin2 --loose 0 --set $S
run nominal_tumble --loose 0
run loose_rec --loose -1 --no-act --set $S
run loose_safe --loose -1 --set $S
run loose_reclamp --loose -1 --set $S grip_response=reclamp
