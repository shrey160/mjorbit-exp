"""P4.4c check: a converted LeRobot dataset against its npz sources. State and action must be
exact; images are compared after video decoding (PSNR per camera, lidar range error in mm).
python check_lerobot.py <dataset_root> [n_frames_per_episode=20]   (LeRobot env)"""
import json
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from to_lerobot import pack_lidar  # noqa: E402

root = Path(sys.argv[1])
n = int(sys.argv[2]) if len(sys.argv) > 2 else 20
from lerobot.datasets import LeRobotDataset  # noqa: E402

ds = LeRobotDataset("local/mjorbit_demos_v4", root=root)
index = json.loads((root.parent / f"{root.name}_episodes.json").read_text())
print(f"{len(ds)} frames, {ds.num_episodes} episodes, features {list(ds.features)}")
from_idx = ds.meta.episodes["dataset_from_index"]
psnr = {"wrist": [], "bus": [], "lidar": []}
lid_err, lid_p95, exact = [], [], True
for e in index:
    src = Path(e["src"]) / f"ep_{e['seed']}.npz"
    with np.load(src) as z:
        d = {k: z[k] for k in ("wrist", "bus", "lidar", "state", "action")}
    for i in np.linspace(0, e["frames"] - 1, n).astype(int):
        x = ds[int(from_idx[e["episode"]]) + int(i)]
        exact &= np.array_equal(x["observation.state"].numpy(), d["state"][i]) and \
            np.array_equal(x["action"].numpy(), d["action"][i])
        ref = {"wrist": d["wrist"][i], "bus": cv2.resize(d["bus"][i], (128, 128), interpolation=cv2.INTER_AREA),
               "lidar": pack_lidar(d["lidar"][i])}
        for k, r in ref.items():
            im = (x[f"observation.images.{k}"].permute(1, 2, 0).numpy() * 255.0).round()
            mse = float(np.mean((im - r.astype(np.float64)) ** 2))
            psnr[k].append(99.0 if mse == 0 else 10 * np.log10(255.0**2 / mse))
            if psnr[k][-1] < 30:
                print(f"LOW {k} episode {e['episode']} seed {e['seed']} frame {i}: {psnr[k][-1]:.1f} dB, decoded mean {im.mean():.1f}, ref mean {r.mean():.1f}")
            if k == "lidar":  # grey level: 0.3-1.8 m over levels 1-255, where returns are
                hit = r[..., 0] > 0
                e_mm = np.abs(im[..., 0][hit] - r[..., 0][hit]) * 1500.0 / 254.0
                lid_err.append(float(np.median(e_mm)))
                lid_p95.append(float(np.percentile(e_mm, 95)))
print("state/action exact:", exact)
for k, v in psnr.items():
    print(f"{k}: PSNR median {np.median(v):.1f} dB, min {np.min(v):.1f}")
print(f"lidar range error after decoding (returns): median {np.median(lid_err):.1f} mm, "
      f"p95 {np.median(lid_p95):.1f} mm (per-frame medians, then median)")
