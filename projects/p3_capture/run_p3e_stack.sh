#!/bin/bash
# 3E: 3C (detumble + identification) for spinning clients, plus the 3C regression (SPEC_3 10)
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p3_capture
bash $D/run_p3c_eval.sh wm_regress --n 40 > /dev/null
for S in 1 2; do
bash $D/run_p3c_eval.sh spin$S --n 40 --set spin_deg_s=$S impose_tumble=false > /dev/null
done
for T in wm_regress spin1 spin2; do echo "== $T"; grep -v '^wrote' $D/eval_p3c_$T.log | tail -21; done
