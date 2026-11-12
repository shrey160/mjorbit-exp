#!/bin/bash
# ESPA 3B grasp (still client), GraspCfg variants: bash run_espa_ab.sh <tag> <n> k=v ...
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p3_capture
TAG=$1; N=$2; shift 2
cd $D/../../tools
bash rlrun.sh python ../projects/p3_capture/evaluate_p3b.py --n $N --workers 4 --set scene=p3e_espa_scene.xml "$@" --out $D/results_espa_ab_$TAG.json 2>&1 | grep -v -E 'Module |lock file' | grep -E "success |push_dv|bus_att|peak_contact|final_bar|approach_track"
