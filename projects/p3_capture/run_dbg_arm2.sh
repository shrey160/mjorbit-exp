#!/bin/bash
# bash run_dbg_arm2.sh <scene,scene,...> <seed,seed,...>
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p3_capture
cd $D/../../tools
bash rlrun.sh python $D/dbg_arm2_3c.py "$1" "$2" 2>&1 | grep -v -E 'Module |lock file|bogus|WARN'
