#!/bin/bash
# v2 smoke test (12 Nov): 4 episodes end to end. Log: p4_vision/smoke_v2.log
P4=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p4_vision
D=/home/shrey31/p4_data/smoke_v2
R="bash $P4/run_p4.sh"
rm -rf $D $D.pack $D.pose
echo "== record 4 episodes $(date)"
$R gen_demos.py --split demo --n 4 --workers 4 --out $D
ls -la $D; du -sh $D
echo "== contact sheet"
F=$(ls $D/ep_*.npz | head -1)
$R contact_sheet.py $F $P4/sheets/smoke_v2.png 8
echo "== pack"
$R pack_data.py $D $D.pack 4
ls -la $D.pack
echo "== mount floor (should be ~1-2 mm now)"
$R mount_floor.py $D.pack
echo "== train 30 steps (GPU, seconds)"
$R train_pose.py --pack $D.pack --out $D.pose --epochs 1 --max-steps 30 --bs 32 --workers 2 --n-train 3
echo "== closed loop, 1 episode on the estimate (CPU)"
$R eval_est.py --mode est --ckpt $D.pose/best.pt --split validation --n 1 --workers 1 --out $D.eval.json | tail -12
echo "== done $(date)"
