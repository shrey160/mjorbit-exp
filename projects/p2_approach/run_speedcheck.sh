#!/bin/bash
# Torch-filter check: 20 training iterations (timing) and r1/599 re-evaluation.
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p2_approach
R=$D/../../tools/rlrun.sh
bash $R python ../projects/p2_approach/train.py --env residual --unc u1 --stage a3 \
  --num-envs 2048 --steps-per-env 16 --gamma 0.99 --lam 0.99 --w-shape 0.1 --w-fuel 4 \
  --std-min 0.02 --init-std 0.15 --zero-init --max-iterations 20 --run-name speedcheck \
  2>&1 | grep -E "Iteration time|done in|Error|Traceback" > $D/speedcheck.log
bash $R python ../projects/p2_approach/evaluate.py --env residual --unc u1 --skip-pd \
  --cw-deadband 0.002 --checkpoint ../projects/p2_approach/logs/r1/model_599.pt \
  --out ../projects/p2_approach/results_val_r1_599_u1_torchkf.json 2>&1 \
  | grep -E "^(metric|success |collisions|dv_mean|overlap|paired)|Error|Traceback" >> $D/speedcheck.log
