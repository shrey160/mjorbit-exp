#!/bin/bash
# P4.8 (13 Nov): how much camera-mount (extrinsic calibration) error does the classical grasp on
# the P4.3 v3 estimate tolerate? Mount error +-M mm and +-M/10 deg per axis, per sensor, per episode
# (the network was trained at 1 mm, 0.1 deg). 25 validation seeds each, spin 0-2 deg/s as P4.3,
# arm2 camera scene, monitors on, 8 CPU workers, one run after the other.
# Usage: bash run_mount_sweep.sh [mm="3 5 10"] [N=25]     Log: p4_vision/mount_sweep.log
P4=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p4_vision
MMS=${1:-"3 5 10"}; N=${2:-25}
CK=/home/shrey31/p4_data/pose_v3/best.pt
export P4_SCENE=p4_espa_arm2_cam_scene.xml
for M in $MMS; do
  MM=$(python3 -c "print($M/1000)"); DEG=$(python3 -c "print($M/10)")
  echo "== mount +-$M mm, +-$DEG deg, estimate, $N seeds, $(date)"
  bash $P4/run_p4.sh eval_est.py --mode est --ckpt $CK --split validation --n $N --workers 8 \
      --mount-m $MM --mount-deg $DEG --out $P4/p48_mount${M}_est.json | tail -34
done
echo "== done $(date)"
