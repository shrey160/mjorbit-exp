#!/bin/bash
# E3 whole-mission Monte Carlo on the ESPA servicer (SPEC_3 section 14):
# bash run_e3_espa.sh <split> <n> [workers]; stage B resumes from its .partial.jsonl
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p3_capture
S=$1; N=$2; W=${3:-20}
SC=p3e_espa_scene.xml
R="bash $D/../../tools/rlrun.sh python ../projects/p3_capture/evaluate_e3.py"
cd $D/../../tools
[ -f $D/e3_handoff_espa_$S.json ] || $R A --n $N --split $S --scene $SC --out $D/e3_handoff_espa_$S.json 2>&1 | grep -v -E 'Module |lock file|Warp|warp'
$R B --handoff $D/e3_handoff_espa_$S.json --workers $W --scene $SC --out $D/results_e3_espa_$S.json 2>&1 | grep -v -E 'Module |lock file'
