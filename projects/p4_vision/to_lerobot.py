"""P4.4c (SPEC_4 4.4): convert demos_v4 (gen_demos.py npz, images at 10 Hz) to a LeRobot v3
dataset for ACT. Runs in the LeRobot env: bash lrrun.sh python to_lerobot.py ...

One dataset, clean episodes first, then DART; a sidecar episodes.json maps the dataset
episode index to (seed, dart, split). Per frame (10 fps):
  observation.images.wrist  128x128x3 RGB
  observation.images.bus    128x128x3 RGB (recorded at 192, area-resized: ACT needs one size)
  observation.images.lidar  128x128x3 grey: range 0.3-1.8 m mapped to levels 1-255 (5.9 mm per
                            level, clipped; demos span 0.33-1.6 m), 0 = no return, the same
                            value in all three channels. (A first packing with different data
                            per channel lost 17.6 mm after yuv420p chroma subsampling.)
  observation.state         29-D (gen_demos.py state)
  action                    10-D: expert grip target in the bus frame (3 + rot6) and close flag
  task                      "grasp the grapple bar"
No Hub push; finalize() closes the Parquet writers (required).

python to_lerobot.py --src /home/.../demos_v4_clean /home/.../demos_v4_dart --out /home/.../lerobot/demos_v4 [--max-episodes N]
"""
from __future__ import annotations

import argparse
import json
import shutil
import time
from pathlib import Path

import cv2
import numpy as np

TASK = "grasp the grapple bar"
N_VAL = 15  # last 15 episodes of each source are validation (as P4.3: episodes 110-124)
STATE_NAMES = ([f"q{i}" for i in range(6)] + [f"qd{i}" for i in range(6)] + ["jaw_u", "jaw_l"]
               + ["grip_x", "grip_y", "grip_z"] + [f"grip_r6_{i}" for i in range(6)]
               + ["grip_vx", "grip_vy", "grip_vz", "grip_wx", "grip_wy", "grip_wz"])
ACTION_NAMES = ["tgt_x", "tgt_y", "tgt_z"] + [f"tgt_r6_{i}" for i in range(6)] + ["close"]


def pack_lidar(L):
    """uint16 range image in mm (0 = no return) -> uint8 HxWx3."""
    r = L.astype(np.float32)
    g = np.where(r > 0, np.clip(1.0 + (r - 300.0) / 1500.0 * 254.0, 1, 255), 0).round().astype(np.uint8)
    return np.repeat(g[..., None], 3, axis=2)


def features():
    img = dict(dtype="video", shape=(128, 128, 3), names=["height", "width", "channels"])
    return {"observation.images.wrist": img, "observation.images.bus": dict(img),
            "observation.images.lidar": dict(img),
            "observation.state": dict(dtype="float32", shape=(29,), names=STATE_NAMES),
            "action": dict(dtype="float32", shape=(10,), names=ACTION_NAMES)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", nargs="+", required=True, help="demo dirs (clean first, then DART)")
    ap.add_argument("--out", required=True)
    ap.add_argument("--max-episodes", type=int, default=0, help="per source (smoke test)")
    ap.add_argument("--vcodec", default="h264")
    ap.add_argument("--crf", type=int, default=10)  # 13 Nov: 18 lost 11.8 mm lidar median; 10 gives one level (5.9 mm)
    ap.add_argument("--preset", default="veryfast")
    ap.add_argument("--streaming", type=int, default=1, help="encode while adding frames (1) or via PNGs (0)")
    a = ap.parse_args()
    from lerobot.datasets import LeRobotDataset
    out = Path(a.out)
    if out.exists():
        shutil.rmtree(out)
    # 13 Nov: the default (libsvtav1 CRF 30 via PNG frames) took 1.6 min per episode; h264 with
    # streaming encoding (no PNG round trip) instead. GOP 2 keeps random-access decoding cheap.
    from lerobot.configs.video import RGBEncoderConfig
    enc = RGBEncoderConfig(vcodec=a.vcodec, crf=a.crf, preset=a.preset, g=2)
    ds = LeRobotDataset.create(repo_id="local/mjorbit_demos_v4", fps=10, features=features(), root=out,
                               robot_type="espa_arm2", use_videos=True, rgb_encoder=enc,
                               streaming_encoding=a.streaming, image_writer_threads=4)
    index, t0 = [], time.time()
    for src in a.src:
        fs = sorted(Path(src).glob("ep_*.npz"))
        fs = [f for f in fs if not f.name.endswith(".tmp.npz")]
        if a.max_episodes:
            fs = fs[:a.max_episodes]
        for k, f in enumerate(fs):
            with np.load(f) as z:  # read each array once: NpzFile decompresses on every access
                d = {k_: z[k_] for k_ in ("wrist", "bus", "lidar", "state", "action", "t", "img_t", "meta")}
            meta = json.loads(str(d["meta"]))
            assert len(d["img_t"]) == len(d["t"]) and np.allclose(d["img_t"], d["t"]), f"{f}: images not at 10 Hz"
            for i in range(len(d["t"])):
                ds.add_frame({"observation.images.wrist": d["wrist"][i],
                              "observation.images.bus": cv2.resize(d["bus"][i], (128, 128), interpolation=cv2.INTER_AREA),
                              "observation.images.lidar": pack_lidar(d["lidar"][i]),
                              "observation.state": d["state"][i].astype(np.float32),
                              "action": d["action"][i].astype(np.float32), "task": TASK})
            ds.save_episode()
            index.append(dict(episode=len(index), seed=meta["seed"], dart=bool(meta.get("dart")), frames=len(d["t"]),
                              split="val" if k >= len(fs) - N_VAL else "train", src=str(src)))
            print(f"  {len(index)} episodes ({Path(src).name}), {(time.time() - t0) / 60:.1f} min", flush=True)
    ds.finalize()
    (out.parent / f"{out.name}_episodes.json").write_text(json.dumps(index, indent=1))
    n_tr = sum(e["split"] == "train" for e in index)
    print(f"wrote {out}: {len(index)} episodes ({n_tr} train), {sum(e['frames'] for e in index)} frames, "
          f"{(time.time() - t0) / 60:.1f} min")


if __name__ == "__main__":
    main()
