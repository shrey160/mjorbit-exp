#!/bin/bash
# Regression after the stow-pose change of 15 Nov (arm2 scenes): 25 validation seeds each, one after another.
#   1. classical grasp on the true bar pose           -> p48_oracle_stow.json   (before: p48_oracle_palmscene.json)
#   2. v3 + gated palm v2, mounts +-10 mm, 1 deg      -> p48_mount10_palm_v2_stow.json (before: p48_mount10_palm_v2.json)
P4=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p4_vision
export P4_SCENE=p4_espa_arm2_cam_scene.xml OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
bash $P4/run_p4.sh eval_est.py --mode oracle --split validation --n 25 --workers 8 --out $P4/p48_oracle_stow.json | tail -3
bash $P4/run_p4.sh cmp_runs.py $P4/p48_oracle_palmscene.json $P4/p48_oracle_stow.json | tail -4
PV=v2 TAG=_v2_stow bash $P4/run_palm_build.sh eval 10 25 --palm-no-rot | tail -3
bash $P4/run_p4.sh cmp_runs.py $P4/p48_mount10_palm_v2.json $P4/p48_mount10_palm_v2_stow.json | tail -4
echo "== stow regression done $(date)"
