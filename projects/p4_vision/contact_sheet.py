"""Contact sheet of a demo episode (v2 format): wrist camera (top), bus camera (middle), bus
flash lidar (bottom, near = bright, no return = dark red) at evenly spaced image frames, shown
at 128 px, plus checks: expert target vs grip point (10 Hz rows), and lidar range at the bar
centre's pixel vs the true distance (settle frames, arm stowed).
python contact_sheet.py <episode.npz> <out.png> [n_cols]"""
import json
import sys

import numpy as np
from PIL import Image, ImageDraw

PH = ("settle", "reach", "approach", "close", "hold")
LIDAR_FOVY = 45.0
S = 128  # display size per tile
z = np.load(sys.argv[1])
ncol = int(sys.argv[3]) if len(sys.argv) > 3 else 8
W, B, L = z["wrist"], z["bus"], z["lidar"]
ph, t = z["img_phase"], z["img_t"]
T = len(t)
idx = np.linspace(0, T - 1, ncol).astype(int)
sheet = Image.new("RGB", (ncol * S, 3 * S + 14), (40, 40, 40))
d = ImageDraw.Draw(sheet)


def lidar_rgb(r):
    m = r.astype(np.float32) / 1000.0
    g = np.clip(1.0 - (m - 0.3) / 3.0, 0.08, 1.0) * 255
    rgb = np.stack([g, g, g], -1).astype(np.uint8)
    rgb[r == 0] = (90, 0, 0)
    return Image.fromarray(rgb)


tile = lambda im: im.resize((S, S), Image.BILINEAR)  # noqa: E731
for j, i in enumerate(idx):
    sheet.paste(tile(Image.fromarray(W[i])), (j * S, 14))
    sheet.paste(tile(Image.fromarray(B[i])), (j * S, 14 + S))
    sheet.paste(lidar_rgb(L[i]).resize((S, S), Image.NEAREST), (j * S, 14 + 2 * S))
    d.text((j * S + 2, 1), f"{t[i]:.0f}s {PH[ph[i]]}", fill=(255, 255, 255))
sheet = sheet.resize((sheet.width * 2, sheet.height * 2), Image.NEAREST)
sheet.save(sys.argv[2])
meta = json.loads(str(z["meta"]))
st, ac, rph = z["state"], z["action"], z["phase"]
lag = np.linalg.norm(ac[:, :3] - st[:, 14:17], axis=1)
print("meta:", {k: meta[k] for k in ("seed", "spin", "axis", "success", "push_dv", "bar_shift_mm", "t_grasp")})
print("sun", np.round(meta["dr"]["sun_dir"], 2), "I", round(meta["dr"]["sun_I"], 2), "amb", round(meta["dr"]["ambient"], 3),
      "mount", meta["dr"].get("mount_m"), "m /", meta["dr"].get("mount_deg"), "deg")
print(f"rows {len(rph)} (10 Hz), image frames {T}; shapes wrist {W.shape[1:]}, bus {B.shape[1:]}, lidar {L.shape[1:]}")
print("target-vs-grip distance per phase (m):", {PH[p]: round(float(np.median(lag[rph == p])), 4) for p in np.unique(rph)})
n = L.shape[1]
f = (n / 2) / np.tan(np.radians(LIDAR_FOVY) / 2)
errs = []
for i in np.flatnonzero(ph == 0):
    p = z["img_priv"][i, 30:33]  # bar centre in the lidar camera frame (camera looks along -z)
    u, v = n / 2 + f * p[0] / -p[2], n / 2 - f * p[1] / -p[2]
    ui, vi = int(u), int(v)
    if 0 <= ui < n and 0 <= vi < n and L[i, vi, ui]:
        errs.append(L[i, vi, ui] / 1000.0 - np.linalg.norm(p))
print(f"lidar: valid {np.mean(L > 0) * 100:.0f}% of pixels; at the bar-centre pixel, range minus true distance (m): "
      f"median {np.median(errs) if errs else float('nan'):+.3f}, n={len(errs)} (the bar's near face is 2.5 cm in front of its centre)")
