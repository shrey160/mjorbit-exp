#!/bin/bash
# Run a command in the LeRobot pixi env (setup_lerobot_env.sh): bash lrrun.sh python to_lerobot.py ...
cd /home/shrey31/lerobot_px
exec /home/shrey31/.pixi/bin/pixi run "$@"
