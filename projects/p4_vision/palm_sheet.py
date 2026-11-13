"""Sheet of palm-camera images from a gen_palm.py directory: one row per episode, 8 frames
spread over the episode; prints the label ranges.   python palm_sheet.py <dir> <out.png>"""
import json
import sys
from pathlib import Path

import numpy as np

fs = sorted(Path(sys.argv[1]).glob("ep_*.npz"))[:6]
rows, allrel = [], []
for f in fs:
    z = np.load(f)
    palm, rel, ph = z["palm"], z["rel"], z["phase"]
    m = json.loads(str(z["meta"]))
    idx = np.linspace(0, len(palm) - 1, 8).astype(int)
    rows.append(np.concatenate([palm[i] for i in idx], 1))
    allrel.append(rel)
    print(f.name, "frames", len(palm), m["outcome"], m["abort_reason"], "bias mm", m["bias_mm"],
          "rel mm at frames:", [tuple(np.round(rel[i, :3] * 1000).astype(int)) for i in idx[[0, 3, 7]]])
rel = np.concatenate(allrel)
print("label range mm: x", np.round(rel[:, 0].min() * 1000), np.round(rel[:, 0].max() * 1000),
      "y", np.round(rel[:, 1].min() * 1000), np.round(rel[:, 1].max() * 1000),
      "z", np.round(rel[:, 2].min() * 1000), np.round(rel[:, 2].max() * 1000))
img = np.clip(np.concatenate(rows, 0).astype(np.float32) * 1.5, 0, 255).astype(np.uint8)
try:
    from PIL import Image
    Image.fromarray(img).save(sys.argv[2])
except ImportError:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.imsave(sys.argv[2], img)
print("wrote", sys.argv[2], img.shape)
