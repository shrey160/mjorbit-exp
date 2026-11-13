#!/bin/bash
# P4.4e: ACT closed loop on the arm2 camera scene, validation seeds, CPU (thermal limit: 8 workers, no GPU).
# Usage: bash run_act_eval.sh <pretrained_model dir> <tag> [N=25] [WORKERS=8] [extra eval_act.py args]
# Log: p4_vision/act_eval_<tag>.log, results p4_vision/p44_val_act_<tag>.json
P4=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p4_vision
CK=$1; TAG=$2; N=${3:-25}; W=${4:-8}; shift $(( $# < 4 ? $# : 4 ))
export P4_SCENE=p4_espa_arm2_cam_scene.xml MUJOCO_GL=egl PYOPENGL_PLATFORM=egl
echo "== ACT $TAG ($CK), validation $N, $W workers, $(date)"
bash $P4/lrrun.sh python $P4/eval_act.py --ckpt $CK --split validation --n $N --workers $W \
    --out $P4/p44_val_act_$TAG.json "$@" 2>&1 \
  | grep --line-buffered -v -E 'EGLError|OpenGL/|egl/__init__|Exception ignored|renderer.py|glCheckError|baseplatform|Traceback \(most recent call last\):$'
echo "== done $(date)"
