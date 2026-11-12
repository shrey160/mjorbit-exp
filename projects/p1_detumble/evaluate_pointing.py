"""Evaluate a Project 1B pointing policy against a rate-limited attitude controller.

Both controllers run on exactly the same spins, attitudes and targets, on the GPU env,
with the deterministic policy (mean action). Run from the simulator repo root:

    pixi run -e rl --locked python ../projects/p1_detumble/evaluate_pointing.py \
        --checkpoint ../projects/p1_detumble/logs/p3/model_<it>.pt --split validation

Omit --checkpoint to evaluate only the baseline (used to tune its gains on validation).

Baseline (uses the true inertia I, which the policy does not see):
    angle, axis   from the error quaternion (body frame)
    w_ref         = axis * min(Kp * angle, sqrt(2 * beta * alpha * angle))
                    alpha = tau_max / max(I) is the weakest-axis deceleration, so the
                    sqrt term caps speed at what can still be braked in the remaining angle
    tau           = clip(Kd * I * (w_ref - w), +-tau_max)
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
from detumble_env import random_quats, sample_spins  # noqa: E402
from evaluate import SPLIT_SEEDS, wilson  # noqa: E402
from pointing_env import error_angle, error_quat, qexp, qmul  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402
from train import STAGES, build_train_cfg, make_env  # noqa: E402


def make_cases(stage: str, split: str, n: int, cfg) -> tuple[np.ndarray, ...]:
    st = STAGES[stage]
    rng = np.random.default_rng(SPLIT_SEEDS[split])
    w0 = sample_spins(rng, n, cfg.spin_mode, cfg.spin_min, cfg.spin_max)
    quat = random_quats(rng, n)
    axis = rng.normal(size=(n, 3))
    axis /= np.linalg.norm(axis, axis=1, keepdims=True)
    ang = rng.uniform(st["err_min"], st["err_max"], n)
    q_tgt = qmul(quat, qexp(axis, ang))
    return w0, quat, q_tgt


def rollout(env, w0, quat, q_tgt, controller) -> dict:
    cfg = env.cfg
    obs = env.set_pointing_conditions(w0, quat, q_tgt)
    T = env.max_episode_length
    n = env.num_envs
    speeds = np.zeros((n, T))
    angles = np.zeros((n, T))
    acts_hist = np.zeros((n, T, 3))
    failed = np.zeros(n, dtype=bool)
    with torch.inference_mode():
        for k in range(T):
            a = controller(obs).clamp(-1, 1)
            obs, _, _, _ = env.step(a)
            s = np.linalg.norm(env.body_rates(), axis=1)
            _, ang = env.pointing_error()
            failed |= ~np.isfinite(s) | (s > cfg.term_speed)
            speeds[:, k] = np.nan_to_num(s, nan=np.inf)
            angles[:, k] = np.nan_to_num(ang, nan=np.pi)
            acts_hist[:, k] = a.cpu().numpy()
    ok = (angles < np.radians(cfg.success_angle_deg)) & (speeds < cfg.success_rate)
    hold = int(round(cfg.success_hold_s / cfg.control_dt))
    success = ok[:, -hold:].all(axis=1) & ~failed
    rev = ok[:, ::-1]
    run_len = np.where(rev.all(axis=1), T, np.argmin(rev, axis=1))
    settle = np.where(ok[:, -1], (T - run_len) * cfg.control_dt, np.inf)
    effort = np.abs(acts_hist).sum(axis=(1, 2)) * cfg.tau_max * cfg.control_dt
    sat = (np.abs(acts_hist) > 0.999).any(axis=2).mean(axis=1)
    dact = np.abs(np.diff(acts_hist, axis=1)).sum(axis=2).mean(axis=1)
    return dict(success=success, failed=failed, settle=settle, final_angle=angles[:, -1],
                final_speed=speeds[:, -1], max_angle=angles.max(axis=1),
                effort=effort, sat=sat, dact=dact)


def summarize(name: str, r: dict, init_angle: np.ndarray) -> dict:
    n = r["success"].size
    k = int(r["success"].sum())
    lo, hi = wilson(k, n)
    ok = r["success"]
    st = r["settle"][ok]
    big = init_angle > np.radians(120)
    return {
        "controller": name,
        "episodes": n,
        "success": k / n,
        "success_ci95": [lo, hi],
        "success_init_err_gt_120deg": float(ok[big].mean()) if big.any() else None,
        "failures_spun_up_or_nonfinite": int(r["failed"].sum()),
        "settle_s_median": float(np.median(st)) if st.size else None,
        "settle_s_p95": float(np.quantile(st, 0.95)) if st.size else None,
        "settle_s_max": float(st.max()) if st.size else None,
        "final_angle_deg_p95": float(np.degrees(np.quantile(r["final_angle"], 0.95))),
        "final_angle_deg_max": float(np.degrees(r["final_angle"].max())),
        "final_speed_max": float(r["final_speed"].max()),
        "overshoot_deg_mean": float(np.degrees(np.maximum(r["max_angle"] - init_angle, 0)).mean()),
        "effort_Nms_mean_successes": float(r["effort"][ok].mean()) if ok.any() else None,
        "saturated_step_frac_mean": float(r["sat"].mean()),
        "action_change_mean": float(r["dact"].mean()),
    }


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--checkpoint", default=None)
    ap.add_argument("--stage", choices=["p1", "p2", "p3"], default="p3")
    ap.add_argument("--split", choices=sorted(SPLIT_SEEDS), default="validation")
    ap.add_argument("--episodes", type=int, default=200)
    ap.add_argument("--tau-max", type=float, default=0.01)
    ap.add_argument("--Kp", type=float, default=0.3, help="baseline angle gain (1/s)")
    ap.add_argument("--Kd", type=float, default=2.0, help="baseline rate gain (1/s)")
    ap.add_argument("--beta", type=float, default=0.5, help="fraction of braking authority budgeted")
    ap.add_argument("--device", default="cuda")
    ap.add_argument("--out", default=None)
    ap.add_argument("--obs", choices=["v1", "v2"], default="v1",
                    help="observation version the checkpoint was trained with")
    args = ap.parse_args()

    env = make_env(args.stage, obs=args.obs, num_envs=args.episodes, auto_reset=False,
                   device=args.device, seed=SPLIT_SEEDS[args.split], tau_max=args.tau_max)
    w0, quat, q_tgt = make_cases(args.stage, args.split, args.episodes, env.cfg)
    init_angle = error_angle(error_quat(quat, q_tgt))

    # Feasibility lower bound: rest-to-rest slew of `angle` with the most favourable
    # inertia and the largest box-corner torque magnitude (very optimistic).
    alpha_best = np.sqrt(3.0) * args.tau_max / env.inertia.min()
    t_lb = 2.0 * np.sqrt(init_angle / alpha_best)
    budget = env.cfg.episode_length_s - env.cfg.success_hold_s
    print(f"initial error median {np.degrees(np.median(init_angle)):.0f} deg, max"
          f" {np.degrees(init_angle.max()):.0f} deg; optimistic slew-time bound max"
          f" {t_lb.max():.1f} s vs {budget:.0f} s budget; provably infeasible:"
          f" {int((t_lb > budget).sum())}")

    I = torch.as_tensor(env.inertia, dtype=torch.float32, device=args.device)
    alpha = args.tau_max / float(env.inertia.max())
    w_scale, tau_max = env.cfg.w_scale, args.tau_max

    def baseline(obs):
        o = obs["policy"]
        v = o[:, 0:3]
        w = o[:, 3:6] * w_scale
        sin_half = v.norm(dim=1).clamp(max=1.0)
        angle = 2.0 * torch.asin(sin_half)
        axis = v / sin_half.clamp(min=1e-9)[:, None]
        speed = torch.minimum(args.Kp * angle, torch.sqrt(2.0 * args.beta * alpha * angle))
        w_ref = axis * speed[:, None]
        return args.Kd * I * (w_ref - w) / tau_max

    rb = rollout(env, w0, quat, q_tgt, baseline)
    name = f"rate-limited PD Kp={args.Kp:g} Kd={args.Kd:g} b={args.beta:g}"
    results = [summarize(name, rb, init_angle)]
    overlap = None
    if args.checkpoint:
        runner = OnPolicyRunner(env, build_train_cfg(0), log_dir=None, device=args.device)
        runner.load(args.checkpoint)
        policy = runner.get_inference_policy(device=args.device)
        rp = rollout(env, w0, quat, q_tgt, lambda o: policy(o))
        results.append(summarize("PPO policy", rp, init_angle))
        sb, sp = rb["success"], rp["success"]
        overlap = {"both": int((sb & sp).sum()), "baseline_only": int((sb & ~sp).sum()),
                   "policy_only": int((~sb & sp).sum()), "neither": int((~sb & ~sp).sum())}

    print(f"\nsplit={args.split} stage={args.stage} episodes={args.episodes}")
    print(f"{'metric':32s}" + "".join(f"{r['controller'][:30]:>32s}" for r in results))
    for k in [k for k in results[0] if k != "controller"]:
        cells = []
        for r in results:
            v = r[k]
            if isinstance(v, list):
                cells.append(f"[{v[0]:.3f}, {v[1]:.3f}]")
            elif isinstance(v, float):
                cells.append(f"{v:.4g}")
            else:
                cells.append(str(v))
        print(f"{k:32s}" + "".join(f"{c:>32s}" for c in cells))
    if overlap:
        print(f"\nper-case success overlap: {overlap}")
    if args.out:
        Path(args.out).write_text(json.dumps(
            {"split": args.split, "stage": args.stage, "checkpoint": args.checkpoint,
             "obs_version": args.obs,
             "baseline": {"Kp": args.Kp, "Kd": args.Kd, "beta": args.beta},
             "overlap": overlap, "results": results}, indent=2))
        print(f"\nwrote {args.out}")


if __name__ == "__main__":
    main()
