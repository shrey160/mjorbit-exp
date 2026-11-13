#!/bin/bash
# P4.4c (SPEC_4 4.4): LeRobot in its own pixi env (the rl env's torch 2.13 is outside LeRobot
# 0.6's 2.7-2.11, and LeRobot must not go into the rl env). Python 3.12, LeRobot 0.6.1 pinned
# (same version on Colab), mujoco 3.7.0 for the closed-loop evaluation, ffmpeg for video.
# Env: /home/shrey31/lerobot_px.  Run a command in it: bash lrrun.sh python ...
set -e
PX=/home/shrey31/.pixi/bin/pixi
D=/home/shrey31/lerobot_px
mkdir -p $D && cd $D
[ -f pixi.toml ] || $PX init .
$PX add "python=3.12.*" pip ffmpeg
# [training] only: [core-scripts] pulls evdev (keyboard teleop), whose wheel fails to build here
$PX run pip install "lerobot[training]==0.6.1" "mujoco==3.7.0" scipy
$PX run python -c 'import importlib.metadata as m, torch, mujoco; print("lerobot", m.version("lerobot"), "torch", torch.__version__, "cuda", torch.cuda.is_available(), "mujoco", mujoco.__version__)'
$PX run which lerobot-train ffmpeg
