#!/bin/bash
# P6.1 sweep of vision error levels (tune seeds, 25 each, one process), inspection hop on.
# name:pos_mm (x range/2.7),att_deg,p_outlier,p_flip,dropouts_per_min
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p6_silent_client
bash $D/run_p6.sh vis_eval.py --n 25 --workers 8 --conds \
  clean:10,1,0,0,0 nom:10,1,0.02,0.05,1 att2:10,2,0.02,0.05,1 pos30:30,1,0.02,0.05,1 \
  mid:20,1.5,0.05,0.1,2 dirty:10,1,0.1,0.2,3 \
  --out $D/p61_sweep.json
