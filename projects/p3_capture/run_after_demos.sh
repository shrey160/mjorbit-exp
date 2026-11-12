#!/bin/bash
# Chain (12 Nov): after run_after_val.sh finishes (p3b identity re-check + demos_v1), run the
# ESPA E3 test split once (300), but only if the p3b re-check came out 20/20 identical.
# Log: p3_capture/after_demos.log
P3=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p3_capture
LOG=$P3/../p4_vision/after_val.log
until grep -q "== done" $LOG; do sleep 60; done
if grep -q "20/20 identical" $LOG; then
  echo "== p3b identity OK; ESPA E3 test 300 $(date)"
  bash $P3/run_e3_espa.sh test 300
  echo "== test done $(date)"
else
  echo "== p3b identity re-check did not pass; test split NOT run $(date)"
fi
