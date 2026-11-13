"""Add the per-frame phase (from the source npz) to a to_lerobot.py sidecar index, so that
val_act.py can report errors per phase without the npz files (Colab).
python add_phase_index.py <dataset root>"""
import json
import sys
from pathlib import Path

import numpy as np

root = Path(sys.argv[1])
f = root.parent / f"{root.name}_episodes.json"
index = json.loads(f.read_text())
for e in index:
    with np.load(Path(e["src"]) / f"ep_{e['seed']}.npz") as z:
        e["phase"] = z["phase"].astype(int).tolist()
    assert len(e["phase"]) == e["frames"], e["episode"]
f.write_text(json.dumps(index))
print(f"{f}: phase added to {len(index)} episodes")
