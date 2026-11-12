"""Does the reward prefer the frugal baseline? Discounted return of each baseline under
the training reward (stage a3 validation cases, w_fuel as given).

    pixi run -e rl --locked python ../projects/p2_approach/diag_returns.py [w_fuel] [gamma]
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import mjorbit  # noqa: F401,E402

# isort: split
import numpy as np  # noqa: E402

from approach_env import HOLD, ApproachEnv, ApproachEnvCfg, baseline_force, sample_starts  # noqa: E402

w_fuel = float(sys.argv[1]) if len(sys.argv) > 1 else 0.2
gamma = float(sys.argv[2]) if len(sys.argv) > 2 else 0.998
n = 64
cfg = ApproachEnvCfg(num_envs=n, auto_reset=False, w_fuel=w_fuel, gamma_shaping=gamma)
p0, v0 = sample_starts(np.random.default_rng(100), n, 20.0, 50.0, 60.0, 0.02)
d0 = np.linalg.norm(p0 - HOLD, axis=1)
env = ApproachEnv(cfg)
for name, kw in (("planned T400", dict(plan_s=400.0, d0=d0)),
                 ("planned T250", dict(plan_s=250.0, d0=d0)),
                 ("fast vmax0.2", dict(v_max=0.2)),
                 ("fast vmax0.4", dict(v_max=0.4))):
    env.set_initial_conditions(p0, v0)
    G = np.zeros(n)
    U = np.zeros(n)
    disc = 1.0
    alive = np.ones(n, bool)
    for k in range(env.max_episode_length):
        _, r, d, _ = env.step_world_force(baseline_force(env, **kw))
        r = r.cpu().numpy()
        G += alive * disc * r
        U += alive * r
        disc *= gamma
        alive &= ~d.cpu().numpy()
    print(f"{name:14s} discounted return {G.mean():8.3f}   undiscounted {U.mean():8.3f}   "
          f"dv {env.ep_dv.mean():.3f}")
