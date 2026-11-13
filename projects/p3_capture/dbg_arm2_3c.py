"""Debug (13 Nov): run E3 mission(s) from the ESPA validation hand-off on scene variants
in one process, print the stage, grip outcome and pad-drop details."""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "p2_approach"))
import evaluate_e3 as e3  # noqa: E402

H = json.loads(Path(__file__).with_name("e3_handoff_espa_validation.json").read_text())
cases = {c["seed"]: c for c in H["cases"]}
seeds = [int(s) for s in sys.argv[2].split(",")]
for scene in sys.argv[1].split(","):
    for s in seeds:
        r = e3._mission(cases[s], mon_act=True, scene=scene)
        keep = {k: v for k, v in r.items() if k in ("stage", "g_outcome", "gm_pad_drops", "grip_abort", "grip_abort_phase",
                                                     "grip_detect", "wall_s", "success", "outcome")
                or k.startswith(("c_", "d_"))}
        print(scene, s, json.dumps(keep, default=str)[:600], flush=True)
