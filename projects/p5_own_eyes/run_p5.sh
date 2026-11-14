#!/bin/bash
# Project 5 runner: bash run_p5.sh <script.py> [args...]  (rl env, CPU only)
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p5_own_eyes
S=$1; shift
cd $D/../../tools
# one maths thread per worker: small matrix products otherwise start several threads each
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
bash rlrun.sh python $D/$S "$@" 2>&1 | grep --line-buffered -v -E 'Module |lock file'
