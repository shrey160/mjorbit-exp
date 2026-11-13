"""P4.4c check: decode the first two and last two frames of every episode and count blank ones
(an episode-end frame of seed 8000011 decoded as all zeros). python check_lerobot_ends.py <root>"""
import sys
from pathlib import Path

import numpy as np
from lerobot.datasets import LeRobotDataset

root = Path(sys.argv[1])
ds = LeRobotDataset("local/mjorbit_demos_v4", root=root)
lo, hi = ds.meta.episodes["dataset_from_index"], ds.meta.episodes["dataset_to_index"]
bad = []
for e in range(ds.num_episodes):
    for j in (lo[e], lo[e] + 1, hi[e] - 2, hi[e] - 1):
        x = ds[int(j)]
        for k in ("wrist", "bus", "lidar"):
            if float(x[f"observation.images.{k}"].max()) == 0.0:
                bad.append((e, int(j - lo[e]), k))
print(f"{len(bad)} blank frames out of {ds.num_episodes * 4 * 3} checked")
for b in bad[:40]:
    print("  episode", b[0], "frame", b[1], b[2])
