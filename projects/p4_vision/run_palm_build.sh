#!/bin/bash
# P4.8 palm-camera correction (13 Nov), one step per call, CPU only, 8 workers.
# Usage: bash run_palm_build.sh scene|smoke|data|train|eval [args]
P4=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p4_vision
D=/home/shrey31/p4_data
export P4_SCENE=p4_espa_arm2_cam_scene.xml
PV=${PV:-v1}  # data / network version: palm_$PV, palm_net_$PV
R="bash $P4/run_p4.sh"
STEP=$1; shift
case $STEP in
  scene)  # regenerate the camera scene with palm_cam, then check the oracle grasp is unchanged
    $R make_scene.py --arm arm2 | tail -2
    $R eval_est.py --mode oracle --split validation --n 25 --workers 8 --out $P4/p48_oracle_palmscene.json | tail -3
    $R cmp_runs.py $P4/p43_val_oracle_arm2.json $P4/p48_oracle_palmscene.json | tail -3 ;;
  smoke)  # 4 episodes and a sheet of palm images
    rm -rf $D/palm_smoke
    $R gen_palm.py --start 900 --n 4 --workers 4 --out $D/palm_smoke | tail -3
    $R palm_sheet.py $D/palm_smoke $P4/sheets/palm_smoke.png | tail -8 ;;
  data)   # N episodes (default 150), resumable
    $R gen_palm.py --start 1000 --n ${1:-150} --workers 8 --out $D/palm_$PV | tail -6 ;;
  train)  # CPU training, 8 threads
    $R train_palm.py --data $D/palm_$PV --out $D/palm_net_$PV --epochs ${1:-20} --threads 8 ;;
  eval)   # mount error M mm (default 10), N seeds (25), then extra eval_est.py args; TAG=_x names the output
          # v3 + palm
    M=${1:-10}; MM=$(python3 -c "print($M/1000)"); DEG=$(python3 -c "print($M/10)")
    $R eval_est.py --mode palm --ckpt $D/pose_v3/best.pt --palm-ckpt $D/palm_net_$PV/best.pt --split validation \
        --n ${2:-25} --workers 8 --mount-m $MM --mount-deg $DEG --out $P4/p48_mount${M}_palm${TAG}.json "${@:3}" | tail -40 ;;
  *) echo "unknown step $STEP"; exit 1 ;;
esac
echo "== $STEP done $(date)"
