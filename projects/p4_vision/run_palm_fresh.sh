#!/bin/bash
# P4.8 confirmation on validation seeds not used so far (6000100-124), mount +-10 mm, 1 deg:
# the classical grasp on the v3 estimate alone, then with the gated palm v2 correction.
P4=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p4_vision
export P4_SCENE=p4_espa_arm2_cam_scene.xml
bash $P4/run_p4.sh eval_est.py --mode est --ckpt /home/shrey31/p4_data/pose_v3/best.pt --split validation --start 100 --n 25 \
    --workers 8 --mount-m 0.010 --mount-deg 1.0 --out $P4/p48_mount10_est_fresh.json | tail -3
PV=v2 TAG=_v2_fresh bash $P4/run_palm_build.sh eval 10 25 --palm-no-rot --start 100 | tail -3
echo "== fresh done $(date)"
