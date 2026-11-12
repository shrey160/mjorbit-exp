#!/bin/bash
# Colab upload bundle v2 (12 Nov): demos_v2 in 10 tar shards of 50 episodes, plus the code.
# Output: C:\Shrey_Projs\robotics\mjorbit\tmp\colab_upload_v2\  -> Drive MyDrive/mjorbit_p4/data_v2/
set -e
SRC=/home/shrey31/p4_data/demos_v2
P4=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p4_vision
OUT=/mnt/c/Shrey_Projs/robotics/mjorbit/tmp/colab_upload_v2
mkdir -p $OUT/data_v2 $OUT/code
cd $SRC
ls ep_*.npz | sort -t_ -k2 -n > /tmp/eps_v2.txt
rm -f /tmp/shv2_*
split -l 50 -d -a 2 /tmp/eps_v2.txt /tmp/shv2_
for s in /tmp/shv2_*; do
  n=$(basename $s | sed 's/shv2_//')
  [ -f $OUT/data_v2/demos_v2_$n.tar ] || tar -cf $OUT/data_v2/demos_v2_$n.tar -T $s
  echo "shard $n: $(wc -l < $s) episodes, $(du -h $OUT/data_v2/demos_v2_$n.tar | cut -f1)"
done
(cd $OUT/data_v2 && sha256sum demos_v2_*.tar > SHA256SUMS)
cp $P4/pose_net.py $P4/train_pose.py $P4/pack_data.py $P4/colab_p43_v2.ipynb $OUT/code/
du -sh $OUT
