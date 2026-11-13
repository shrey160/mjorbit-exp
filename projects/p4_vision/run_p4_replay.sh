#!/bin/bash
# Re-export the two P4.3 browser replays with a checkpoint (13 Nov: epoch 23, best_final.pt).
P4=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p4_vision
CK=${1:-/home/shrey31/p4_data/pose_v2/best_final.pt}
R="bash $P4/run_p4.sh"
$R export_p4_replay.py --ckpt $CK --seed 6000001 --out $P4/replay/fast_spin.json
$R export_p4_replay.py --ckpt $CK --seed 6000014 --holdout-sun --out $P4/replay/heldout_sun.json
