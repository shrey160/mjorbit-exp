#!/bin/bash
# Project 4 runner: bash run_p4.sh <script.py> [args...]  (rl env, EGL rendering)
# e.g. bash run_p4.sh make_scene.py
#      bash run_p4.sh gen_demos.py --split demo --n 500 --workers 20 --out /home/shrey31/p4_data/demos_v1
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p4_vision
S=$1; shift
cd $D/../../tools
MUJOCO_GL=egl PYOPENGL_PLATFORM=egl bash rlrun.sh python $D/$S "$@" 2>&1 | grep --line-buffered -v -E 'Module |lock file|EGLError|egl/__init__|OpenGL/|Exception ignored|Traceback \(most recent call last\):$|renderer.py|glCheckError|baseplatform'
