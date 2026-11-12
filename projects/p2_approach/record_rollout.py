"""Record Project 2A trajectories (LVLH) for the browser replay.

    pixi run -e rl --locked python ../projects/p2_approach/record_rollout.py \
        --checkpoint ../projects/p2_approach/logs/<run>/model_<it>.pt --out <file.npz> [--cases 0 1 2]

Runs the policy, the planned-400 s baseline and the fast baseline from the same
validation start states and saves p, v, F (LVLH) per second for each.
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
from approach_env import HOLD, ApproachEnv, ApproachEnvCfg, baseline_force, sample_starts  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402
from cw_baseline import CWTargeting1Hz, CWTargetingEpoch  # noqa: E402
from maneuver_env import ManeuverEnv, ManeuverEnvCfg  # noqa: E402
from train import build_train_cfg  # noqa: E402


def record(env, p0, v0, act):
    obs = env.set_initial_conditions(p0, v0)
    T = env.max_episode_length
    n = env.num_envs
    P = np.zeros((T + 1, n, 3))
    V = np.zeros((T + 1, n, 3))
    F = np.zeros((T, n, 3))
    _, _, P[0], V[0] = env.lvlh_state()
    dead = np.zeros(n, bool)
    for k in range(T):
        C, _, _, _ = env.lvlh_state()
        kind, a = act(env, obs)
        obs, _, d, _ = env.step(a) if kind == "lvlh" else env.step_world_force(a)
        Fw = np.asarray(env.data.ctrl[:, 0:3], float)
        F[k] = np.einsum("nji,nj->ni", C, Fw)
        _, _, P[k + 1], V[k + 1] = env.lvlh_state()
        P[k + 1][dead] = P[k][dead]
        dead |= d.cpu().numpy()
    dv = np.cumsum(np.abs(F).sum(axis=2), axis=0) / env.cfg.mass
    return P, V, F, dv


def record_epoch(env, p0, v0, act):
    """Per-second trace from the maneuver-epoch env."""
    obs = env.set_initial_conditions(p0, v0)
    _, _, p_init, v_init = env.lvlh_state()
    env.trace = []
    for _ in range(env.max_episode_length):
        kind, a = act(env, obs)
        obs, _, _, _ = env.step(a) if kind == "lvlh" else env.step_world_dv(a)
    tr, env.trace = env.trace, None
    P = np.concatenate([p_init[None], np.stack([t[0] for t in tr])])
    V = np.concatenate([v_init[None], np.stack([t[1] for t in tr])])
    F = np.stack([t[2] for t in tr])
    dv = np.cumsum(np.abs(F).sum(axis=2), axis=0) / env.cfg.mass
    return P, V, F, dv


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--checkpoint", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--cases", type=int, nargs="*", default=[0, 1, 2, 3])
    ap.add_argument("--env", choices=["thrust", "maneuver"], default="thrust")
    args = ap.parse_args()
    p_all, v_all = sample_starts(np.random.default_rng(100), 200, 20.0, 50.0, 60.0, 0.02)
    idx = np.array(args.cases)
    # pad to >= 2 worlds (nworld=1 uses unbatched buffers)
    k = max(2, idx.size)
    sel = np.resize(idx, k)
    p0, v0 = p_all[sel], v_all[sel]
    d0 = np.linalg.norm(p0 - HOLD, axis=1)
    env = ApproachEnv(ApproachEnvCfg(num_envs=k, auto_reset=False))
    penv = env
    if args.env == "maneuver":
        penv = ManeuverEnv(ManeuverEnvCfg(num_envs=k, auto_reset=False))
    gamma = 0.99 if args.env == "maneuver" else 0.998
    runner = OnPolicyRunner(penv, build_train_cfg(0, gamma), log_dir=None, device="cuda")
    runner.load(args.checkpoint)
    pol = runner.get_inference_policy(device="cuda")
    out = {}
    cw = CWTargeting1Hz(env, t_arrive=480.0)
    jobs = [("policy", penv, lambda e, o: ("lvlh", pol(o))),
            ("planned", env, lambda e, o: ("world", baseline_force(e, plan_s=400.0, d0=d0))),
            ("cw", env, lambda e, o: ("world", cw()))]
    with torch.no_grad():
        for name, en, fn in jobs:
            rec = record_epoch if isinstance(en, ManeuverEnv) else record
            P, V, F, dv = rec(en, p0, v0, fn)
            out[f"{name}_p"], out[f"{name}_v"], out[f"{name}_F"], out[f"{name}_dv"] = \
                P[:, :idx.size], V[:, :idx.size], F[:, :idx.size], dv[:, :idx.size]
            print(f"{name:8s} final dv per case: {np.round(dv[-1, :idx.size], 3)}")
    np.savez(args.out, cases=idx, hold=HOLD, checkpoint=args.checkpoint, **out)
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
