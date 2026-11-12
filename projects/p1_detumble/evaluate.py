"""Evaluate a detumble policy against the saturated damping baseline (SPEC section 8).

Both controllers run on exactly the same initial spins/attitudes, on the GPU env,
with the deterministic policy (mean action). Run from the simulator repo root:

    pixi run -e rl --locked python ../projects/p1_detumble/evaluate.py \
        --checkpoint ../projects/p1_detumble/logs/s3/model_<it>.pt --split test

Splits use disjoint seed ranges: validation = 100, test = 1000. Never tune on test.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import mjorbit  # noqa: E402,F401

# isort: split

import numpy as np  # noqa: E402
import torch  # noqa: E402
from detumble_env import DetumbleEnv, DetumbleEnvCfg, random_quats, sample_spins  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402
from train import STAGES, build_train_cfg  # noqa: E402

SPLIT_SEEDS = {"validation": 100, "test": 1000}


def wilson(k: int, n: int, z: float = 1.96) -> tuple[float, float]:
    if n == 0:
        return (0.0, 1.0)
    p = k / n
    den = 1 + z * z / n
    c = (p + z * z / (2 * n)) / den
    h = z * np.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return (max(0.0, c - h), min(1.0, c + h))


def rollout(env: DetumbleEnv, w0, quat, controller) -> dict:
    cfg = env.cfg
    obs = env.set_initial_conditions(w0, quat)
    T = env.max_episode_length
    speeds = np.zeros((env.num_envs, T))
    acts_hist = np.zeros((env.num_envs, T, 3))
    failed = np.zeros(env.num_envs, dtype=bool)
    with torch.inference_mode():
        for k in range(T):
            a = controller(obs, env)
            a = a.clamp(-1, 1)
            obs, _, _, _ = env.step(a)
            w = env.body_rates()
            s = np.linalg.norm(w, axis=1)
            failed |= ~np.isfinite(s) | (s > cfg.term_speed)
            speeds[:, k] = np.nan_to_num(s, nan=np.inf)
            acts_hist[:, k] = a.cpu().numpy()
    below = speeds < cfg.success_rate
    hold = int(round(cfg.success_hold_s / cfg.control_dt))
    success = below[:, -hold:].all(axis=1) & ~failed
    # settle time: start of the final unbroken run below threshold (inf if not at rest at end)
    rev = below[:, ::-1]
    run_len = np.where(rev.all(axis=1), T, np.argmin(rev, axis=1))
    settle = np.where(below[:, -1], (T - run_len) * cfg.control_dt, np.inf)
    effort = np.abs(acts_hist).sum(axis=(1, 2)) * cfg.tau_max * cfg.control_dt
    floor = np.linalg.norm(env.inertia * w0, axis=1)  # |H0|, lower bound on effort
    sat = (np.abs(acts_hist) > 0.999).any(axis=2).mean(axis=1)
    dact = np.abs(np.diff(acts_hist, axis=1)).sum(axis=2).mean(axis=1)
    return dict(success=success, failed=failed, settle=settle, final=speeds[:, -1],
                effort=effort, floor=floor, sat=sat, dact=dact)


def summarize(name: str, r: dict) -> dict:
    n = r["success"].size
    k = int(r["success"].sum())
    lo, hi = wilson(k, n)
    ok = r["success"]
    st = r["settle"][ok]
    out = {
        "controller": name,
        "episodes": n,
        "success": k / n,
        "success_ci95": [lo, hi],
        "failures_spun_up_or_nonfinite": int(r["failed"].sum()),
        "not_at_rest_at_end": int((~ok & ~r["failed"]).sum()),
        "settle_s_median": float(np.median(st)) if st.size else None,
        "settle_s_p95": float(np.quantile(st, 0.95)) if st.size else None,
        "settle_s_max": float(st.max()) if st.size else None,
        "final_speed_max": float(np.max(r["final"])),
        # effort compared only among successes, so quick failures can't look cheap
        "effort_Nms_mean_successes": float(r["effort"][ok].mean()) if ok.any() else None,
        "effort_over_floor_mean_successes": float((r["effort"] / r["floor"])[ok].mean())
        if ok.any() else None,
        "saturated_step_frac_mean": float(r["sat"].mean()),
        "action_change_mean": float(r["dact"].mean()),
    }
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--checkpoint", default=None, help="omit to evaluate the baseline only")
    ap.add_argument("--stage", choices=sorted(STAGES), default="s3",
                    help="initial-spin distribution to test on")
    ap.add_argument("--split", choices=sorted(SPLIT_SEEDS), default="validation")
    ap.add_argument("--episodes", type=int, default=200)
    ap.add_argument("--K", type=float, default=2.0, help="baseline damping gain (1/s)")
    ap.add_argument("--device", default="cuda")
    ap.add_argument("--out", default=None, help="write JSON results here")
    ap.add_argument("--tau-max", type=float, default=0.01,
                    help="per-axis torque limit, N*m (must match training)")
    args = ap.parse_args()

    st = STAGES[args.stage]
    env = DetumbleEnv(DetumbleEnvCfg(num_envs=args.episodes, auto_reset=False,
                                     device=args.device, seed=SPLIT_SEEDS[args.split],
                                     tau_max=args.tau_max, **st))
    rng = np.random.default_rng(SPLIT_SEEDS[args.split])
    w0 = sample_spins(rng, args.episodes, st["spin_mode"], st["spin_min"], st["spin_max"])
    quat = random_quats(rng, args.episodes)

    # Feasibility: any controller needs at least |H0| / (sqrt(3) * tau_max) seconds to
    # remove the momentum (largest torque vector the per-axis box allows). If that
    # exceeds episode - hold time, success is impossible. This bound is optimistic,
    # so cases just under it may still be infeasible.
    h0 = np.linalg.norm(env.inertia * w0, axis=1)
    t_min = h0 / (np.sqrt(3.0) * args.tau_max)
    budget = env.cfg.episode_length_s - env.cfg.success_hold_s
    n_inf = int((t_min > budget).sum())
    print(f"tau_max={args.tau_max} N*m: lower-bound stop time median {np.median(t_min):.1f} s,"
          f" max {t_min.max():.1f} s; provably infeasible (> {budget:.0f} s): {n_inf}/{args.episodes}")

    inertia = torch.as_tensor(env.inertia, dtype=torch.float32, device=args.device)
    w_scale, tau_max = env.cfg.w_scale, env.cfg.tau_max

    def baseline(obs, _env):
        w = obs["policy"][:, 0:3] * w_scale
        return -args.K * inertia * w / tau_max

    rb = rollout(env, w0, quat, baseline)
    results = [summarize(f"damping K={args.K:g}", rb)]
    overlap = None

    if args.checkpoint:
        runner = OnPolicyRunner(env, build_train_cfg(0), log_dir=None, device=args.device)
        runner.load(args.checkpoint)
        policy = runner.get_inference_policy(device=args.device)
        rp = rollout(env, w0, quat, lambda o, _e: policy(o))
        results.append(summarize("PPO policy", rp))
        sb, sp = rb["success"], rp["success"]
        overlap = {"both": int((sb & sp).sum()), "baseline_only": int((sb & ~sp).sum()),
                   "policy_only": int((~sb & sp).sum()), "neither": int((~sb & ~sp).sum())}

    print(f"\nsplit={args.split} stage={args.stage} ({st}) episodes={args.episodes}")
    keys = [k for k in results[0] if k != "controller"]
    print(f"{'metric':36s}" + "".join(f"{r['controller']:>22s}" for r in results))
    for k in keys:
        cells = []
        for r in results:
            v = r[k]
            if isinstance(v, list):
                cells.append(f"[{v[0]:.3f}, {v[1]:.3f}]")
            elif isinstance(v, float):
                cells.append(f"{v:.4g}")
            else:
                cells.append(str(v))
        print(f"{k:36s}" + "".join(f"{c:>22s}" for c in cells))
    if overlap:
        print(f"\nper-case success overlap: {overlap}")
    if args.out:
        Path(args.out).write_text(json.dumps(
            {"split": args.split, "stage": args.stage, "checkpoint": args.checkpoint,
             "tau_max": args.tau_max, "provably_infeasible": n_inf, "overlap": overlap,
             "results": results}, indent=2))
        print(f"\nwrote {args.out}")


if __name__ == "__main__":
    main()
