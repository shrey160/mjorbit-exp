#!/bin/bash
# Colab upload bundle v3 (13 Nov): demos_v3 (arm2, 125 episodes) in 5 tar shards of 25 episodes, plus the code.
# Output: C:\Shrey_Projs\robotics\mjorbit\tmp\colab_upload_v3\  -> Drive MyDrive/mjorbit_p4/data_v3/
set -e
SRC=/home/shrey31/p4_data/demos_v3
P4=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p4_vision
OUT=/mnt/c/Shrey_Projs/robotics/mjorbit/tmp/colab_upload_v3
mkdir -p $OUT/data_v3 $OUT/code
cd $SRC
ls ep_*.npz | sort -t_ -k2 -n > /tmp/eps_v3.txt
rm -f /tmp/shv3_*
split -l 25 -d -a 2 /tmp/eps_v3.txt /tmp/shv3_
for s in /tmp/shv3_*; do
  n=$(basename $s | sed 's/shv3_//')
  [ -f $OUT/data_v3/demos_v3_$n.tar ] || tar -cf $OUT/data_v3/demos_v3_$n.tar -T $s
  echo "shard $n: $(wc -l < $s) episodes, $(du -h $OUT/data_v3/demos_v3_$n.tar | cut -f1)"
done
(cd $OUT/data_v3 && sha256sum demos_v3_*.tar > SHA256SUMS)
cp $P4/pose_net.py $P4/train_pose.py $P4/pack_data.py $P4/colab_p43_v3.ipynb $OUT/code/
du -sh $OUT
