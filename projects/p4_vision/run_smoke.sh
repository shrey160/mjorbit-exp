#!/bin/bash
# P4 smoke test: regenerate the camera scene, record N demo episodes, contact sheets
# bash run_smoke.sh <tag> [n] [workers]
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p4_vision
O=/home/shrey31/p4_data/$1
rm -rf $O $D/sheets
bash $D/run_p4.sh make_scene.py
bash $D/run_p4.sh gen_demos.py --split demo --n ${2:-4} --workers ${3:-4} --out $O
bash $D/run_sheets.sh $O 8
du -sh $O
