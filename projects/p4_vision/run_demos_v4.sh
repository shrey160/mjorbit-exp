#!/bin/bash
# P4.4b (SPEC_4 4.4): demos_v4 on the arm2 camera scene, images at 10 Hz (--img-steps 10),
# queued one after the other (one run at a time): 125 clean episodes (seeds 8000000-124, the
# demos_v3 cases), then 125 DART episodes on new seeds (8000125-249). 8 workers (thermal).
# Resumable: re-run the same command. Log: p4_vision/demos_v4.log
P4=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p4_vision
OUT=/home/shrey31/p4_data
N=${1:-125}
export P4_SCENE=p4_espa_arm2_cam_scene.xml
echo "== clean $N $(date)"
bash $P4/run_p4.sh gen_demos.py --split demo --start 0 --n $N --workers 8 --img-steps 10 --out $OUT/demos_v4_clean
echo "== dart $N $(date)"
bash $P4/run_p4.sh gen_demos.py --split demo --start $N --n $N --workers 8 --img-steps 10 --dart --out $OUT/demos_v4_dart
echo "== done $(date)"
du -sh $OUT/demos_v4_clean $OUT/demos_v4_dart
