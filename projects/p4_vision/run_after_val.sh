#!/bin/bash
# Chain (12 Nov): wait for the ESPA E3 validation to finish, re-check p3b bit-identity after
# the step_hook was added to grasp_sim.py (20 E3 missions), then generate the first 500
# P4 demonstrations. Logs: p4_vision/after_val.log
P3=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p3_capture
P4=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p4_vision
until [ -f $P3/results_e3_espa_validation.json ]; do sleep 60; done
echo "== validation finished $(date)"
echo "== p3b identity re-check (step_hook added)"
bash $P3/run_e3_wiring.sh 2>&1 | tail -3
echo "== demos_v1: 500 episodes $(date)"
bash $P4/run_p4.sh gen_demos.py --split demo --n 500 --workers 20 --out /home/shrey31/p4_data/demos_v1
du -sh /home/shrey31/p4_data/demos_v1
echo "== done $(date)"
