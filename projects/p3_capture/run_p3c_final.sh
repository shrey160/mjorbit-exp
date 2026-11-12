#!/bin/bash
# 3C final: 100 cases default; sensor-noise x10; grip load test on the square bar
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p3_capture
bash $D/run_p3c_eval.sh final --n 100 > /dev/null
bash $D/run_p3c_eval.sh noise10 --n 100 --set sigma_gyro=1e-4 sigma_dv=1e-5 > /dev/null
TAG=_sq bash $D/run_p3b_load.sh --squeeze 0.03 > /dev/null
echo ALLDONE
