#!/bin/bash
# P4.3 chain at reduced load (12 Nov: the laptop overheated with GPU training and a 20-worker
# simulation together). One job at a time, fewer CPU workers. Each step is resumable or short.
# Log: p4_vision/p43.log
P3=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p3_capture
P4=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p4_vision
D=/home/shrey31/p4_data
W=8
echo "== p3b identity re-check (pose_est hook) $(date)"
bash $P3/run_e3_wiring.sh $W 2>&1 | tail -2
echo "== pose estimator training (resumes from state.pt) $(date)"
bash $P4/run_p4.sh train_pose.py --pack $D/demos_v1_pack --out $D/pose_v1 --epochs 20 --workers 4
echo "== closed loop, oracle, validation 200 $(date)"
bash $P4/run_p4.sh eval_est.py --mode oracle --split validation --n 200 --workers $W --out $P4/p43_val_oracle.json | tail -30
echo "== closed loop, estimate, validation 200 $(date)"
bash $P4/run_p4.sh eval_est.py --mode est --ckpt $D/pose_v1/best.pt --split validation --n 200 --workers $W --out $P4/p43_val_est.json | tail -40
echo "== done $(date)"
