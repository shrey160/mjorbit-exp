#!/bin/bash
# E3 whole-mission Monte Carlo on the ESPA servicer with the realistic arm (arm2, 13 Nov).
# Stage A only uses the servicer mass and bounding radius, both unchanged from p3e_espa,
# so stage B reuses the ESPA hand-off of the same split (no GPU run).
# Small by default (demonstration project): the first N missions of the split (N=25).
# bash run_e3_arm2.sh <split> [N=25] [workers=8]; stage B resumes from its .partial.jsonl
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p3_capture
S=$1; N=${2:-25}; W=${3:-8}
declare -A BASE=([tune]=5000000 [validation]=6000000 [test]=7000000)
SEEDS=$(seq ${BASE[$S]} $((${BASE[$S]} + N - 1)) | tr '\n' ' ')
cd $D/../../tools
bash rlrun.sh python ../projects/p3_capture/evaluate_e3.py B --handoff $D/e3_handoff_espa_$S.json --workers $W \
  --scene p3e_espa_arm2_scene.xml --seeds $SEEDS --out $D/results_e3_arm2_${S}_n$N.json 2>&1 | grep -v -E 'Module |lock file|bogus'
