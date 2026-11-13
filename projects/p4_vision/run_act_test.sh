#!/bin/bash
# P4.4d local test (SPEC_4 4.4, user's choice 13 Nov): a SHORT ACT run on the RTX 4060 to check
# that LeRobot trains on demos_v4 and how fast. Not the real training (that is Colab).
# Thermal guard (laptop shut down twice under sustained GPU load, 12 Nov): a watchdog polls the
# GPU every 5 s and kills training at >= TMAX C; nothing else may run alongside.
# Usage: bash run_act_test.sh [steps=600] [batch=8] [TMAX=87] [THI=80] [TLO=72]   Log: p4_vision/act_test.log
# Duty-cycle throttle (13 Nov: unthrottled, the GPU went 74 -> 85 C in about 6 s at 51 W):
# SIGSTOP the trainer at >= THI C, SIGCONT at <= TLO C; the hard kill stays at TMAX.
STEPS=${1:-600}; BS=${2:-8}; TMAX=${3:-87}; THI=${4:-80}; TLO=${5:-72}
P4=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p4_vision
DS=/home/shrey31/p4_data/lerobot/demos_v4
OUT=/home/shrey31/p4_data/act_test
rm -rf $OUT
# train episodes only (the last 15 of each source are validation), from the sidecar index
EPS=$(python3 -c "import json; print('[' + ','.join(str(e['episode']) for e in json.load(open('${DS}_episodes.json')) if e['split'] == 'train') + ']')")
TLOG=$P4/act_test_temp.log
: > $TLOG
bash $P4/lrrun.sh lerobot-train --policy.type=act --policy.device=cuda --policy.push_to_hub=false \
  --policy.chunk_size=20 --policy.n_action_steps=20 --policy.use_amp=true \
  --dataset.repo_id=local/mjorbit_demos_v4 --dataset.root=$DS --dataset.episodes="$EPS" \
  --batch_size=$BS --steps=$STEPS --num_workers=4 --log_freq=25 --save_freq=$STEPS \
  --wandb.enable=false --output_dir=$OUT --job_name=act_test &
TP=$!
PEAK=0; PAUSED=0; NPAUSE=0; TPAUSE=0
while kill -0 $TP 2>/dev/null; do
  T=$(nvidia-smi --query-gpu=temperature.gpu,utilization.gpu,power.draw --format=csv,noheader,nounits | head -1)
  C=${T%%,*}
  echo "$(date +%T) $T paused=$PAUSED" >> $TLOG
  [ "$C" -gt "$PEAK" ] && PEAK=$C
  if [ "$C" -ge "$TMAX" ]; then
    echo "WATCHDOG: GPU ${C} C >= ${TMAX} C, stopping training" | tee -a $TLOG
    pkill -CONT -f "[l]erobot-train"; pkill -f "[l]erobot-train"; sleep 2; pkill -9 -f "[l]erobot-train"
    break
  fi
  if [ $PAUSED = 0 ] && [ "$C" -ge "$THI" ]; then
    pkill -STOP -f "[l]erobot-train"; PAUSED=1; NPAUSE=$((NPAUSE + 1))
  elif [ $PAUSED = 1 ] && [ "$C" -le "$TLO" ]; then
    pkill -CONT -f "[l]erobot-train"; PAUSED=0
  fi
  [ $PAUSED = 1 ] && TPAUSE=$((TPAUSE + 2))
  sleep 2
done
wait $TP 2>/dev/null
echo "== done, GPU peak ${PEAK} C, paused $NPAUSE times, about ${TPAUSE} s paused" | tee -a $TLOG
