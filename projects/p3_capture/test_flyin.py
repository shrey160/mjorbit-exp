"""Run fly-in + grasp for one seed per spin axis (debugging aid, SPEC_3 section 11)."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import numpy as np  # noqa: E402
from grasp_sim import GraspCfg, GraspSim  # noqa: E402

spin = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
seed = int(sys.argv[2]) if len(sys.argv) > 2 else 0
keys = ("success", "grasped", "flyin_s", "flyin_legs", "flyin_min_clear", "flyin_sat_frac",
        "flyin_end_err_m", "flyin_end_vrel_mm_s", "flyin_end_att_deg", "push_dv", "bar_shift_mm",
        "other_contact", "final_slip" if False else "final_rel_speed")
for ax in (0, 1, 2):
    sim = GraspSim(GraspCfg(flyin=True))
    r = sim.run(np.random.default_rng(seed), spin_deg_s=spin, spin_axis=ax)
    print(f"spin {spin} axis {ax}: " + ", ".join(f"{k}={r.get(k)!r:.8}" if isinstance(r.get(k), float)
                                                 else f"{k}={r.get(k)}" for k in keys))
