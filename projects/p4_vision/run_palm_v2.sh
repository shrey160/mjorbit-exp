#!/bin/bash
# P4.8 palm v2 (13 Nov): data with the palm light (300 episodes), retrain (10 epochs, CPU), then the
# gated test at 10, 1 and 5 mm mount error, 25 validation seeds each. One step after the other.
P4=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p4_vision
export PV=v2 TAG=_v2
bash $P4/run_palm_build.sh data 300
bash $P4/run_palm_build.sh train 10
for M in 10 1 5; do bash $P4/run_palm_build.sh eval $M 25 --palm-no-rot; done
echo "== all done $(date)"
