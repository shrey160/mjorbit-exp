#!/bin/bash
# Run a command in the simulator's pixi "rl" env (GPU, torch, rsl-rl) from the simulator root.
# From PowerShell: wsl -d Ubuntu -- bash /mnt/c/Shrey_Projs/robotics/mjorbit/tools/rlrun.sh python ../projects/p2_approach/evaluate.py --env maneuver --unc u1
cd /mnt/c/Shrey_Projs/robotics/mjorbit/simulator
export CMAKE_GENERATOR=Ninja
exec /home/shrey31/.pixi/bin/pixi run -e rl --locked "$@"
