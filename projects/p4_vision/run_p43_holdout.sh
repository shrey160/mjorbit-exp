#!/bin/bash
# P4.3 held-out sun (SPEC_4 section 5): estimate closed loop, 200 validation seeds, sun inside the held-out cone.
P4=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p4_vision
CK=${1:-/home/shrey31/p4_data/pose_v2/best_final.pt}; N=${2:-25}
echo "== estimate, held-out sun, validation $N $(date)"
bash $P4/run_p4.sh eval_est.py --mode est --ckpt $CK --split validation --n $N --workers 8 --holdout-sun --out $P4/p43_val_est_final_holdout.json | tail -24
echo "== done $(date)"
