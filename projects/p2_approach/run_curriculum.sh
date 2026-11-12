#!/bin/bash
# Train the Project 2A curriculum a1 -> a2 -> a3, each stage resuming the previous
# stage's last checkpoint, then print training curves.
# Usage: bash run_curriculum.sh <suffix> [iters_a1] [iters_a2] [iters_a3] [extra train.py args...]
set -e
HERE=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p2_approach
cd /mnt/c/Shrey_Projs/robotics/mjorbit/simulator
PIXI="/home/shrey31/.pixi/bin/pixi run -e rl --locked"
SUF=${1:-v1}; I1=${2:-300}; I2=${3:-300}; I3=${4:-500}; shift 4 || true
EXTRA="$@"
last() { ls "$HERE/logs/$1"/model_*.pt | sed 's/.*model_\([0-9]*\)\.pt/\1 &/' | sort -n | tail -1 | cut -d' ' -f2; }
T="$HERE/train.py $EXTRA"
$PIXI python $T --stage a1 --run-name "a1$SUF" --max-iterations "$I1" > "$HERE/logs_a1$SUF.txt" 2>&1
$PIXI python $T --stage a2 --run-name "a2$SUF" --max-iterations "$I2" --resume "$(last a1$SUF)" > "$HERE/logs_a2$SUF.txt" 2>&1
$PIXI python $T --stage a3 --run-name "a3$SUF" --max-iterations "$I3" --resume "$(last a2$SUF)" > "$HERE/logs_a3$SUF.txt" 2>&1
for s in a1 a2 a3; do echo "== $s$SUF  last: $(basename "$(last $s$SUF)")"; $PIXI python ../projects/p2_approach/tb_summary.py "$HERE/logs/$s$SUF" 2>/dev/null | grep -E "success_ema|collide|dv_success|mean_reward|mean_std"; done
