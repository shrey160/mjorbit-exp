#!/bin/bash
# Colab upload bundle (12 Nov): demos_v1 in 10 tar shards of 50 episodes (~1 GB each; the npz
# files are already compressed), plus the training code and the epoch-1 training state.
# Output: C:\Shrey_Projs\robotics\mjorbit\tmp\colab_upload\  -> upload to Drive: MyDrive/mjorbit_p4/
set -e
SRC=/home/shrey31/p4_data/demos_v1
P4=/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p4_vision
OUT=/mnt/c/Shrey_Projs/robotics/mjorbit/tmp/colab_upload
mkdir -p $OUT/data $OUT/code $OUT/pose_v1
cd $SRC
ls ep_*.npz | sort -t_ -k2 -n > /tmp/eps.txt
split -l 50 -d -a 2 /tmp/eps.txt /tmp/shard_
for s in /tmp/shard_*; do
  n=$(basename $s | sed 's/shard_//')
  [ -f $OUT/data/demos_v1_$n.tar ] || tar -cf $OUT/data/demos_v1_$n.tar -T $s
  echo "shard $n: $(wc -l < $s) episodes, $(du -h $OUT/data/demos_v1_$n.tar | cut -f1)"
done
cp $P4/pose_net.py $P4/train_pose.py $P4/pack_data.py $P4/colab_p43.ipynb $OUT/code/
cp /home/shrey31/p4_data/pose_v1/state.pt /home/shrey31/p4_data/pose_v1/best.pt /home/shrey31/p4_data/pose_v1/log.json $OUT/pose_v1/
(cd $OUT/data && sha256sum demos_v1_*.tar > SHA256SUMS)
du -sh $OUT
