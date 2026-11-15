#!/bin/bash
# Project 6 runner: bash run_p6.sh <script.py> [args...]  (rl env, CPU only, one maths thread per worker)
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p6_silent_client
S=$1; shift
cd $D/../../tools
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
bash rlrun.sh python $D/$S "$@" 2>&1 | grep --line-buffered -v -E 'Module |lock file'
