#!/bin/bash
# P4.8 (13 Nov): where does the classical grasp stop working as the client spins faster?
# Oracle (true pose) and the P4.3 v3 estimate at fixed spin rates, 25 validation seeds each, arm2
# camera scene, E2 monitors on, 8 CPU workers, one run after the other.
# Usage: bash run_spin_sweep.sh [spins="2 3 4 5 6"] [N=25]    Log: p4_vision/spin_sweep.log
P4=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p4_vision
SPINS=${1:-"2 3 4 5 6"}; N=${2:-25}
CK=/home/shrey31/p4_data/pose_v3/best.pt
export P4_SCENE=p4_espa_arm2_cam_scene.xml
for S in $SPINS; do
  for MODE in oracle est; do
    echo "== spin $S deg/s, $MODE, $N seeds, $(date)"
    EXTRA=""; [ $MODE = est ] && EXTRA="--ckpt $CK"
    bash $P4/run_p4.sh eval_est.py --mode $MODE $EXTRA --split validation --n $N --workers 8 --spin $S \
        --out $P4/p48_spin${S}_$MODE.json | tail -30
  done
done
echo "== done $(date)"
