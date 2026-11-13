#!/bin/bash
# arm2 camera-scene smoke (13 Nov): record 4 demo episodes on p4_espa_arm2_cam_scene.xml,
# check the physics matches the plain arm2 scene, make contact sheets of the images.
P4=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p4_vision
OUT=/home/shrey31/p4_data/smoke_arm2
export P4_SCENE=p4_espa_arm2_cam_scene.xml
rm -rf $OUT
bash $P4/run_p4.sh gen_demos.py --split demo --n 4 --workers 4 --out $OUT | tail -6
echo "== physics vs plain arm2 scene"
bash $P4/run_p4.sh check_cam_physics.py $OUT p3e_espa_arm2_scene.xml | tail -5
echo "== contact sheets"
mkdir -p $P4/sheets
for f in $OUT/ep_*.npz; do bash $P4/run_p4.sh contact_sheet.py $f $P4/sheets/arm2_$(basename $f .npz).png 8 | tail -2; done
