#!/bin/bash
# E2 validation, both parts in parallel (pre-contact: 16 workers; grip: 8)
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p3_capture
bash $D/run_p3e_grip_val.sh > $D/e2_grip_val.log 2>&1 &
bash $D/run_p3e_val.sh > $D/e2_val.log 2>&1
wait
cat $D/e2_val.log; echo; echo "#### grip"; cat $D/e2_grip_val.log
