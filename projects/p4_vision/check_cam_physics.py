"""Check (SPEC_4 P4.0): the camera scene plus recorder gives the same physics as the plain
ESPA scene. Re-runs the episodes of a demo directory on p3e_espa_scene.xml without any hook
and compares outcome metrics with the recorded episodes.jsonl exactly.
python check_cam_physics.py <demo_dir> [plain_scene]   (arm2: p3e_espa_arm2_scene.xml)"""
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "p3_capture"))
from grasp_sim import GraspCfg, GraspSim  # noqa: E402

KEYS = ("success", "outcome", "push_dv", "bar_shift_mm", "t_grasp", "collision")
ms = [json.loads(ln) for ln in (Path(sys.argv[1]) / "episodes.jsonl").read_text().splitlines() if ln.strip()]
sim = GraspSim(GraspCfg(scene=sys.argv[2] if len(sys.argv) > 2 else "p3e_espa_scene.xml", hold_s=5.0, mon=True, mon_act=True))
same = 0
for m in ms:
    r = sim.run(np.random.default_rng(m["seed"]), spin_deg_s=m["spin"], spin_axis=m["axis"])
    diff = {k: (m.get(k), r.get(k)) for k in KEYS if str(m.get(k)) != str(r.get(k))}
    same += not diff
    print(m["seed"], "IDENTICAL" if not diff else f"DIFF {diff}")
print(f"{same}/{len(ms)} identical")
