#!/bin/bash
# Full demo mission (13 Nov): far approach record + mission with the vision grasp + release.
# Usage: bash run_export_demo.sh [seed=6000000] [range_m=30] [mount_mm=10] [backoff_m=3]
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p3_capture
SEED=${1:-6000000}; RNG=${2:-30}; MM=${3:-10}; BACK=${4:-3}
A=$D/replay/demo_stage_a.json
[ -f $A ] || bash $D/../../tools/rlrun.sh python ../projects/p3_capture/demo_stage_a.py $SEED $RNG $A 2>&1 | grep -v -E 'Module |lock file' | tail -1
MUJOCO_GL=egl PYOPENGL_PLATFORM=egl bash $D/../../tools/rlrun.sh python ../projects/p3_capture/export_demo.py $A $D/replay/demo_full.json $MM $BACK ${@:5} 2>&1 \
  | grep -v -E 'Module |lock file|EGLError|egl/__init__|OpenGL/|Exception ignored|renderer.py|glCheckError|baseplatform|Traceback \(most recent call last\):$'
echo "== done $(date)"
