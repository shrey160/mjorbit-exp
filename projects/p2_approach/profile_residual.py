"""Time ResidualEnv steps vs ManeuverEnv and break down the filter cost (2048 worlds).

    python ../projects/p2_approach/profile_residual.py [--n 2048] [--steps 10]
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import mjorbit  # noqa: F401,E402

# isort: split
import numpy as np  # noqa: E402
import torch  # noqa: E402
from approach_env import UNCERTAINTY  # noqa: E402
from maneuver_env import ManeuverEnv, ManeuverEnvCfg  # noqa: E402
from nav_filter import BiasKalman, BiasKalmanTorch  # noqa: E402
from residual_env import ResidualEnv, ResidualEnvCfg  # noqa: E402
from train import STAGES  # noqa: E402


def time_env(env, steps):
    a = torch.zeros(env.num_envs, 3, device=env.device)
    env.step(a)
    t0 = time.perf_counter()
    for _ in range(steps):
        env.step(a)
    return (time.perf_counter() - t0) / steps


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=2048)
    ap.add_argument("--steps", type=int, default=10)
    args = ap.parse_args()
    kw = dict(num_envs=args.n, device="cuda", **UNCERTAINTY["u1"], **STAGES["a3"])
    # numpy vs torch filter on identical inputs (with resets mid-way)
    rng = np.random.default_rng(1)
    N, n_ = 64, 0.0011
    a = BiasKalman(N, n_, 0.05, 0.001)
    b = BiasKalmanTorch(N, n_, 0.05, 0.001)
    z0 = rng.normal(0, 10, (N, 6))
    a.reset(np.arange(N), z0)
    b.reset(np.arange(N), z0)
    for k in range(300):
        u = rng.normal(0, 3e-3, (N, 3)) * (rng.random((N, 1)) < 0.2)
        z = rng.normal(0, 10, (N, 6))
        a.predict(1.0, u=u)
        b.predict(1.0, u=u)
        na, nb = a.update(z), b.update(z)
        if k == 150:
            a.reset(np.arange(0, N, 3), z)
            b.reset(np.arange(0, N, 3), z)
    dx = np.abs(a.x - b.x).max() / np.abs(a.x).max()
    dP = np.abs(a.P - b.P).max() / np.abs(a.P).max()
    print(f"numpy vs torch filter after 300 steps: rel max diff x {dx:.1e}, P {dP:.1e}, "
          f"NIS {np.abs(na - nb).max():.1e}")
    m = ManeuverEnv(ManeuverEnvCfg(**kw))
    print(f"ManeuverEnv step: {time_env(m, args.steps) * 1000:.0f} ms")
    del m
    r = ResidualEnv(ResidualEnvCfg(**kw))
    print(f"ResidualEnv step: {time_env(r, args.steps) * 1000:.0f} ms")
    for torch_kf in (True,):
        kf = r.kf
        tlog = []
        nl = [(np.zeros((args.n, 3)), np.zeros((args.n, 6)), None)] * 20
        t0 = time.perf_counter()
        for _ in range(5):
            r.nav_log = list(nl)
            r._nav_update()
        t1 = time.perf_counter()
        for _ in range(5):
            r._cw_command()
        t2 = time.perf_counter()
        for _ in range(5):
            r._compute_obs()
        t3 = time.perf_counter()
        print(f"torch filter epoch (20 x predict+update) {(t1 - t0) / 5 * 1000:.0f} ms, "
              f"CW command {(t2 - t1) / 5 * 1000:.0f} ms, full obs {(t3 - t2) / 5 * 1000:.0f} ms")
    return
    u = np.random.default_rng(0).normal(0, 1e-3, (args.n, 3))
    z = np.concatenate([r.meas_p, r.meas_v], axis=1)
    reps = 20
    t0 = time.perf_counter()
    for _ in range(reps):
        kf.predict(1.0, u=u)
    tp = (time.perf_counter() - t0) / reps
    t0 = time.perf_counter()
    for _ in range(reps):
        kf.update(z)
    tu = (time.perf_counter() - t0) / reps
    t0 = time.perf_counter()
    for _ in range(reps):
        r._cw_command()
    tc = (time.perf_counter() - t0) / reps
    print(f"per second of sim: predict {tp * 1000:.1f} ms, update {tu * 1000:.1f} ms; "
          f"per epoch x20 = {(tp + tu) * 20 * 1000:.0f} ms; CW command {tc * 1000:.1f} ms")


if __name__ == "__main__":
    main()
