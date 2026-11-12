#!/bin/bash
# Continue the 2A curriculum from a trained a1 run: a2 then a3 with a given fuel weight.
# Usage: bash run_a2a3.sh <a1_run> <suffix> <w_fuel> [iters_a2] [iters_a3]
set -e
HERE=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p2_approach
cd /mnt/c/Shrey_Projs/robotics/mjorbit/simulator
PIXI="/home/shrey31/.pixi/bin/pixi run -e rl --locked"
last() { ls "$HERE/logs/$1"/model_*.pt | sed 's/.*model_\([0-9]*\)\.pt/\1 &/' | sort -n | tail -1 | cut -d' ' -f2; }
A1=$1; SUF=$2; WF=$3; I2=${4:-300}; I3=${5:-500}
$PIXI python $HERE/train.py --stage a2 --run-name "a2$SUF" --max-iterations $I2 --w-fuel $WF --resume "$(last $A1)" > "$HERE/logs_a2$SUF.txt" 2>&1
$PIXI python $HERE/train.py --stage a3 --run-name "a3$SUF" --max-iterations $I3 --w-fuel $WF --resume "$(last a2$SUF)" > "$HERE/logs_a3$SUF.txt" 2>&1
echo "done a3$SUF $(basename "$(last a3$SUF)")"
