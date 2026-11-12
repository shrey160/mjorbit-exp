#!/bin/bash
# Train the Project 1B pointing curriculum p1 -> p2 -> p3, each stage resuming the
# previous stage's last checkpoint, then print training curves.
# Usage: bash run_pointing_curriculum.sh [reward v1|v2] [obs v1|v2] [iters_p1] [iters_p2] [iters_p3]
# Run names: p1,p2,p3 (v1/v1); p1v2.. (reward v2, obs v1); p1v3.. (reward v2, obs v2);
# other combinations get a suffix r<reward>o<obs>. Set GAMMA=0.997 (env var) to change
# the discount; a non-default gamma appends g<gamma> to the run names.
set -e
HERE=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p1_detumble
cd /mnt/c/Shrey_Projs/robotics/mjorbit/simulator
PIXI="/home/shrey31/.pixi/bin/pixi run -e rl --locked"
REW=${1:-v1}; OBS=${2:-v1}
case "$REW/$OBS" in
  v1/v1) SUF="" ;; v2/v1) SUF="v2" ;; v2/v2) SUF="v3" ;; *) SUF="r${REW}o${OBS}" ;;
esac
last() { ls "$HERE/logs/$1"/model_*.pt | sed 's/.*model_\([0-9]*\)\.pt/\1 &/' | sort -n | tail -1 | cut -d' ' -f2; }
GAMMA=${GAMMA:-0.99}
[ "$GAMMA" != "0.99" ] && SUF="${SUF}g${GAMMA}"
T="$HERE/train.py --reward $REW --obs $OBS --gamma $GAMMA"

$PIXI python $T --stage p1 --run-name "p1$SUF" --max-iterations "${3:-200}" > "$HERE/logs_p1$SUF.txt" 2>&1
$PIXI python $T --stage p2 --run-name "p2$SUF" --max-iterations "${4:-200}" --resume "$(last p1$SUF)" > "$HERE/logs_p2$SUF.txt" 2>&1
$PIXI python $T --stage p3 --run-name "p3$SUF" --max-iterations "${5:-400}" --resume "$(last p2$SUF)" > "$HERE/logs_p3$SUF.txt" 2>&1
for s in p1 p2 p3; do echo "== $s$SUF  last: $(basename "$(last $s$SUF)")"; $PIXI python "$HERE/tb_summary.py" "$HERE/logs/$s$SUF" 2>/dev/null | grep -E "success_ema|mean_reward|mean_std"; done
