"""Print a coarse trace of one 3B grasp episode (debugging aid)."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import numpy as np  # noqa: E402
from grasp_sim import GraspCfg, GraspSim  # noqa: E402

mode = sys.argv[1] if len(sys.argv) > 1 else "free-flying"
seed = int(sys.argv[2]) if len(sys.argv) > 2 else 0
scene = sys.argv[3] if len(sys.argv) > 3 else "p3b_scene.xml"
s = GraspSim(GraspCfg(mode=mode, scene=scene))
r = s.run(np.random.default_rng(seed), record=True)
print({k: v for k, v in r.items() if k != "trace"})
for t, ph, pg, pb, q, v in r["trace"][::int(sys.argv[4]) if len(sys.argv) > 4 else 200]:
    print(f"{t:6.1f} {ph:9s} grip-bar {np.round(pg - pb, 3)} client pos {np.round(q[15:18], 4)}"
          f" client vel mm/s {np.round(v[14:17] * 1000, 2)}")
