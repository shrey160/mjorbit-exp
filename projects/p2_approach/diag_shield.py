"""When does the B5 shield intervene, and when does it find no certified option?
Residual a=0 (= CW epoch + nav9) behind the shield, validation, u1 (SPEC_2 17).

    python ../projects/p2_approach/diag_shield.py [--n 200] [--unc u1]
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import mjorbit  # noqa: F401,E402

# isort: split
import numpy as np  # noqa: E402
import torch  # noqa: E402
from approach_env import HOLD, UNCERTAINTY, sample_starts  # noqa: E402
from residual_env import ResidualEnv, ResidualEnvCfg  # noqa: E402
from shield import CANDIDATES, Shield  # noqa: E402
from train import STAGES  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=200)
    ap.add_argument("--unc", default="u1")
    args = ap.parse_args()
    cfg = ResidualEnvCfg(num_envs=args.n, auto_reset=False, device="cuda",
                         **UNCERTAINTY[args.unc], **STAGES["a3"])
    p0, v0 = sample_starts(np.random.default_rng(100), args.n, cfg.r_min, cfg.r_max,
                           cfg.cone_deg, cfg.v0_max)
    env = ResidualEnv(cfg)
    env.shield = Shield(env, lambda: env.kf)
    env.set_initial_conditions(p0, v0, unc_seed=5100)
    zero = torch.zeros(args.n, 3, device="cuda")
    rows = []  # (epoch, world, choice, any_safe, |p|, |p-hold|, closing speed, nominal margin)
    for k in range(env.max_episode_length):
        _, _, p, v = env.lvlh_state()
        r = np.linalg.norm(p, axis=1)
        close = -np.sum(p * v, axis=1) / r
        env.step(zero)
        sh = env.shield
        m, ch = sh.last_margins, sh.last_choice
        safe = (m >= 0).any(axis=0)
        for i in np.flatnonzero((ch != 0) | ~safe):
            rows.append((k, i, ch[i], safe[i], r[i], np.linalg.norm(p[i] - HOLD), close[i], m[0, i]))
    rows = np.array(rows, dtype=float)
    print(f"{len(rows)} flagged epochs in {len(np.unique(rows[:, 1]))} episodes")
    for lab, sel in (("intervened, safe option found", (rows[:, 2] != 0) & (rows[:, 3] == 1)),
                     ("no certified option", rows[:, 3] == 0)):
        s = rows[sel]
        if not len(s):
            continue
        print(f"\n{lab}: {len(s)} epochs, {len(np.unique(s[:, 1]))} episodes")
        print(f"  time (s)      min/med/max {s[:, 0].min() * 20:.0f} / {np.median(s[:, 0]) * 20:.0f}"
              f" / {s[:, 0].max() * 20:.0f}")
        print(f"  range |p| (m) min/med/max {s[:, 4].min():.2f} / {np.median(s[:, 4]):.2f} / "
              f"{s[:, 4].max():.2f}")
        print(f"  |p - hold|    median {np.median(s[:, 5]):.2f} m; closing speed median "
              f"{np.median(s[:, 6]) * 100:.1f} cm/s; nominal margin median {np.median(s[:, 7]):.2f} m")
        print("  chosen: " + ", ".join(f"{CANDIDATES[c]} {int((s[:, 2] == c).sum())}"
                                      for c in range(len(CANDIDATES))))


if __name__ == "__main__":
    main()
