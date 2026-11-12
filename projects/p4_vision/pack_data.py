"""P4.3: unpack a demo set (ep_*.npz, v2 format) into flat memmaps for fast random access in
training. One packed frame per image frame, with the state and labels recorded at that image's
time (img_state, img_priv, img_phase).
python pack_data.py <demo_dir> <out_dir> [workers]
out: wrist.npy (N,128,128,3) u8, bus.npy (N,192,192,3) u8, lidar.npy (N,128,128) u16 mm,
     state.npy (N,29) f32, priv.npy (N,39) f32, phase.npy (N,) i8, ep.npy (N,) i32 episode
     index, index.json (episode files, offsets, shapes)"""
import json
import sys
from multiprocessing import Pool
from pathlib import Path

import numpy as np

IMG = ("wrist", "bus", "lidar")


def _shape(f):
    with np.load(f) as z:
        return len(z["img_t"]), {k: z[k].shape[1:] for k in IMG}


def _load(f):
    with np.load(f) as z:
        return {**{k: z[k] for k in IMG}, "state": z["img_state"], "priv": z["img_priv"], "phase": z["img_phase"]}


def main():
    src, out = Path(sys.argv[1]), Path(sys.argv[2])
    W = int(sys.argv[3]) if len(sys.argv) > 3 else 16
    out.mkdir(parents=True, exist_ok=True)
    files = sorted(src.glob("ep_*.npz"), key=lambda p: int(p.stem.split("_")[1]))
    with Pool(W) as pool:
        sh = pool.map(_shape, files)
    off = np.concatenate([[0], np.cumsum([s[0] for s in sh])])
    N, shp = int(off[-1]), sh[0][1]
    om = lambda name, dt, s: np.lib.format.open_memmap(out / f"{name}.npy", "w+", dt, (N, *s))  # noqa: E731
    mm = dict(wrist=om("wrist", np.uint8, shp["wrist"]), bus=om("bus", np.uint8, shp["bus"]),
              lidar=om("lidar", np.uint16, shp["lidar"]), state=om("state", np.float32, (29,)),
              priv=om("priv", np.float32, (39,)), phase=om("phase", np.int8, ()))
    ep = om("ep", np.int32, ())
    with Pool(W) as pool:
        for k, z in enumerate(pool.imap(_load, files, chunksize=2)):
            s = slice(off[k], off[k + 1])
            for key, a in mm.items():
                a[s] = z[key]
            ep[s] = k
            if (k + 1) % 50 == 0:
                print(f"  {k + 1}/{len(files)}", flush=True)
    for a in [*mm.values(), ep]:
        a.flush()
    (out / "index.json").write_text(json.dumps(dict(files=[f.name for f in files], offsets=off.tolist(), N=N,
                                                    shapes={k: list(v) for k, v in shp.items()})))
    print(f"packed {len(files)} episodes, {N} image frames -> {out}")


if __name__ == "__main__":
    main()
