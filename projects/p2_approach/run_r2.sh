#!/bin/bash
# B4b residual PPO, run r2 on the glideslope base (SPEC_2 19)
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p2_approach
bash $D/../../tools/rlrun.sh python ../projects/p2_approach/train.py --env residual --unc u1 \
  --stage a3 --num-envs 2048 --steps-per-env 16 --gamma 0.99 --lam 0.99 --w-shape 0.1 \
  --w-fuel 4 --std-min 0.02 --init-std 0.15 --zero-init --max-iterations 600 --run-name r2 --base gs \
  "$@" > $D/train_r2.log 2>&1
