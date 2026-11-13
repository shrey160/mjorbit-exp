#!/bin/bash
# P4.3 test split, run once (13 Nov): seeds 7,000,000-299, oracle then estimate, 8 CPU workers.
P4=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p4_vision
CK=${1:-/home/shrey31/p4_data/pose_v2/best_final.pt}; N=${2:-25}
R="bash $P4/run_p4.sh"
echo "== oracle, test $N $(date)"
[ -f $P4/p43_test_oracle.json ] || $R eval_est.py --mode oracle --split test --n $N --workers 8 --out $P4/p43_test_oracle.json | tail -16
echo "== estimate, test $N $(date)"
[ -f $P4/p43_test_est_final.json ] || $R eval_est.py --mode est --ckpt $CK --split test --n $N --workers 8 --out $P4/p43_test_est_final.json | tail -24
echo "== done $(date)"
