#!/bin/bash
# Contact sheets for every episode in a demo directory: bash run_sheets.sh <dir> [n_cols]
D=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p4_vision
mkdir -p $D/sheets
for f in $1/ep_*.npz; do
  b=$(basename $f .npz)
  bash $D/run_p4.sh contact_sheet.py $f $D/sheets/$b.png ${2:-8}
done
