#!/bin/bash
# P4.3 closed loop (12 Nov): classical grasp on the true pose (oracle) vs on the v2 estimate,
# validation seeds (default 25: small runs for quick experiments), CPU only, 8 workers (thermal limit: no GPU, no big pool).
# Usage: bash run_p43_eval.sh <checkpoint.pt> <tag> [N=25]     Log: p4_vision/p43_eval_<tag>.log
P4=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p4_vision
CK=$1; TAG=$2; N=${3:-25}
R="bash $P4/run_p4.sh"
echo "== oracle, validation $N $(date)"
[ -f $P4/p43_val_oracle.json ] || $R eval_est.py --mode oracle --split validation --n $N --workers 8 --out $P4/p43_val_oracle.json | tail -16
echo "== estimate ($TAG), validation $N $(date)"
$R eval_est.py --mode est --ckpt $CK --split validation --n $N --workers 8 --out $P4/p43_val_est_$TAG.json | tail -24
echo "== done $(date)"
