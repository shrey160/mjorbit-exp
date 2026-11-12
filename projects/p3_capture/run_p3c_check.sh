#!/bin/bash
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p3_capture
bash $D/../../tools/rlrun.sh python ../projects/p3_capture/check_p3c_physics.py 2>&1 | grep -v -E 'Module |lock file'
