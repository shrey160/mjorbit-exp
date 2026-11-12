#!/bin/bash
# ESPA servicer smoke tests (SPEC_3 section 14): 3B grasp of a still client, then a 2 deg/s
# fly-in + grasp, a few validation cases each. bash run_espa_smoke.sh [n] [workers]
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p3_capture
N=${1:-4}; W=${2:-4}
cd $D/../../tools
echo "== 3B grasp, still client"
bash rlrun.sh python ../projects/p3_capture/evaluate_p3b.py --n $N --workers $W --set scene=p3e_espa_scene.xml --out $D/results_espa_smoke_3b.json 2>&1 | grep -v -E 'Module |lock file'
echo "== fly-in + grasp, 2 deg/s"
bash rlrun.sh python ../projects/p3_capture/evaluate_p3b.py --n $N --workers $W --spin 2 --set scene=p3e_espa_scene.xml flyin=true --out $D/results_espa_smoke_flyin.json 2>&1 | grep -v -E 'Module |lock file'
