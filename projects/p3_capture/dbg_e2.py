"""E2 debugging aid: one case, print the abort outcome and first unintended contact."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))
import numpy as np  # noqa: E402
from grasp_sim import GraspCfg, GraspSim  # noqa: E402
seed, fault = int(sys.argv[1]), sys.argv[2]
act = len(sys.argv) < 4 or sys.argv[3] != "rec"
spin = float(np.random.default_rng([seed, 31]).uniform(0.0, 2.0))
sim = GraspSim(GraspCfg(flyin=True, mon=True, mon_act=act, fault_kind=fault))
r = sim.run(np.random.default_rng(seed), spin_deg_s=spin)
for k in ("outcome", "fault_phase", "fault_mag", "t_fault", "detect_reason", "t_detect", "abort_phase",
          "t_to_safe", "hit_first", "hit_force_max", "min_clear"):
    print(f"{k:14s} {r.get(k)}")
