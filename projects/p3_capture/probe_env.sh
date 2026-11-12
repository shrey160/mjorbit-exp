#!/bin/bash
# Project 4 environment check: packages in the rl env, ffmpeg, cores, memory, disk
cd /mnt/c/Shrey_Projs/robotics/mjorbit/tools
bash rlrun.sh python -c "
import importlib.util as u
for m in ('lerobot','av','imageio','imageio_ffmpeg','cv2','PIL','pyarrow','pandas','datasets','torchvision'):
    print(m, bool(u.find_spec(m)))
import torch; print('torch', torch.__version__)
" 2>&1 | grep -E 'True|False|torch'
echo "ffmpeg: $(which ffmpeg)"; nproc; free -g | head -2; df -h /home /mnt/c | tail -2
