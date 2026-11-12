#!/bin/bash
# B6: 1000-case validation Monte Carlo (u1, none) and a dt 0.1 re-run (SPEC_2 18)
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p2_approach
bash $D/run_b5.sh u1 1000 mc
bash $D/run_b5.sh none 1000 mc
bash $D/run_b5.sh u1 200 dt01 --physics-dt 0.1
echo ALLDONE
