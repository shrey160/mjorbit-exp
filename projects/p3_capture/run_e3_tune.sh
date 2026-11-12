#!/bin/bash
# E3 tuning pass (200 missions) + E2 kick re-check with the rate-jump monitor, in parallel
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p3_capture
bash $D/run_p3e_safety.sh tune_kick_v2 --n 40 --split tune --fault client_kick --workers 5 > $D/e2_kick_v2.log 2>&1 &
bash $D/run_p3e_safety.sh tune_nominal_v2 --n 40 --split tune --workers 3 > $D/e2_nom_v2.log 2>&1 &
bash $D/run_e3.sh tune 200 16 > $D/e3_tune.log 2>&1
wait
echo "#### E2 kick (tune)"; grep -E "^outcomes|detect_rate|detect_reasons|false_al|delay|min_clear|hit_force|mon_signal_max" $D/e2_kick_v2.log
echo "#### E2 nominal (tune)"; grep -E "^outcomes|false_al|mon_signal_max" $D/e2_nom_v2.log
echo "#### E3 tune"; cat $D/e3_tune.log
