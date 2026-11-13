#!/bin/bash
# arm2 smoke (13 Nov): old ESPA arm vs arm2, still client and 2 deg/s, same seeds.
# bash run_arm2_smoke.sh [n] [workers]
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p3_capture
N=${1:-8}; W=${2:-4}
cd $D/../../tools
for SPIN in 0 2; do for ARM in espa arm2; do
  echo "== $ARM spin $SPIN $(date)"
  bash rlrun.sh python $D/arm2_smoke.py --arm $ARM --n $N --spin $SPIN --workers $W --out $D/arm2_smoke_${ARM}_s${SPIN}.json 2>&1 | grep -v -E 'Module |lock file|WARN'
done; done
echo "== done $(date)"
