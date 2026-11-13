#!/bin/bash
# P4.4d: Colab upload bundle for ACT on demos_v4 (SPEC_4 4.4). One uncompressed tar (the
# videos are already compressed) of the LeRobot dataset plus its sidecar index (with phases),
# and SHA256SUMS. The user uploads tmp/colab_upload_v4/data_v4/* to Drive mjorbit_p4/data_v4/.
set -e
SRC=/home/shrey31/p4_data/lerobot
OUT=/mnt/c/Shrey_Projs/robotics/mjorbit/tmp/colab_upload_v4/data_v4
mkdir -p $OUT
tar -C $SRC -cf $OUT/demos_v4_lerobot.tar demos_v4 demos_v4_episodes.json
cd $OUT && sha256sum demos_v4_lerobot.tar > SHA256SUMS && cat SHA256SUMS && ls -la
