#!/bin/bash
# Robot-description wiring regression (SPEC_3 section 14): 20 validation missions on p3b,
# stage B only (optional arg: workers, default 20); outcomes must match results_e3_regress.json exactly (cmp_e3.py cmp)
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p3_capture
cd $D/../../tools
SEEDS="6000000 6000001 6000002 6000011 6000015 6000016 6000032 6000033 6000050 6000051 6000057 6000066 6000071 6000085 6000093 6000104 6000134 6000140 6000160 6000173"
rm -f $D/results_e3_wiring.json.partial.jsonl
bash rlrun.sh python ../projects/p3_capture/evaluate_e3.py B --handoff $D/e3_handoff_validation.json --workers ${1:-20} --seeds $SEEDS --out $D/results_e3_wiring.json 2>&1 | grep -v -E 'Module |lock file'
cd $D && python3 cmp_e3.py cmp results_e3_regress.json results_e3_wiring.json
