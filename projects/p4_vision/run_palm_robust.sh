#!/bin/bash
# P4.8 Step 2 (15 Nov): palm-camera robustness, +-10 mm, 1 deg mounts, 25 seeds per run, CPU, 8 workers.
# One run after another. Usage: bash run_palm_robust.sh holdout|test
P4=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p4_vision
D=/home/shrey31/p4_data
export P4_SCENE=p4_espa_arm2_cam_scene.xml OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
R="bash $P4/run_p4.sh"
EST="--mode est --ckpt $D/pose_v3/best.pt --n 25 --workers 8 --mount-m 0.010 --mount-deg 1.0"
case $1 in
  holdout)  # held-out sun directions on the fresh validation seeds 6000100-124
    $R eval_est.py $EST --split validation --start 100 --holdout-sun --out $P4/p48_mount10_est_holdout.json | tail -3
    PV=v2 TAG=_v2_holdout bash $P4/run_palm_build.sh eval 10 25 --palm-no-rot --start 100 --holdout-sun | tail -3 ;;
  test)     # the test split, ONCE: seeds 7000000-24. Do not tune on it afterwards.
    $R eval_est.py $EST --split test --out $P4/p48_test_mount10_est.json | tail -3
    PV=v2 TAG=_v2_test bash $P4/run_palm_build.sh eval 10 25 --palm-no-rot --split test | tail -3
    mv $P4/p48_mount10_palm_v2_test.json $P4/p48_test_mount10_palm_v2.json ;;
  *) echo "usage: holdout|test"; exit 1 ;;
esac
echo "== $1 done $(date)"
