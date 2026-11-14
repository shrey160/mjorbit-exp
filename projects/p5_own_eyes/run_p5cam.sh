#!/bin/bash
# Project 5 runner for scripts that render cameras: bash run_p5cam.sh <script.py> [args...]
# (rl env, EGL rendering, CPU networks, one maths thread per worker)
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p5_own_eyes
S=$1; shift
cd $D/../../tools
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
MUJOCO_GL=egl PYOPENGL_PLATFORM=egl bash rlrun.sh python $D/$S "$@" 2>&1 | grep --line-buffered -v -E 'Module |lock file|EGLError|egl/__init__|OpenGL/|Exception ignored|Traceback \(most recent call last\):$|renderer.py|glCheckError|baseplatform'
