"""B1 checks for nav_filter.BiasKalman (SPEC_2 section 15).

    python ../projects/p2_approach/check_nav.py [--n 200] [--unc u1]

1. Discretisation: Phi9 with b = 0 equals the 6-state CW STM; constant-accel response
   matches a fine numerical integration.
2. Closed loop on validation cases (seed 100) with CW-epoch guidance flown through the
   filter, matched uncertainty draws. Reports filter consistency (mean NEES of p,v and
   of b; expected 6 and 3), NIS (expected 6) and bias-estimate error vs time.
   Done-when (SPEC_2 14, B1): bias error < 30 % of the true bias after 200 s.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import mjorbit  # noqa: F401,E402

# isort: split
import numpy as np  # noqa: E402
from approach_env import UNCERTAINTY, sample_starts  # noqa: E402
from cw_baseline import CWTargetingEpoch, cw_stm6  # noqa: E402
from maneuver_env import ManeuverEnv, ManeuverEnvCfg  # noqa: E402
from nav_filter import NavController, _Disc, cw_A9  # noqa: E402
from train import STAGES  # noqa: E402


def check_disc(n):
    d = _Disc(n, 20.0)
    e1 = np.abs(d.Phi[:6, :6] - cw_stm6(n, 20.0)).max()
    # numerical integration of x' = A x with b constant
    A = cw_A9(n)
    x = np.zeros(9)
    x[6:] = [1e-5, -2e-5, 3e-5]
    h = 1e-3
    for _ in range(20000):
        k1 = A @ x
        k2 = A @ (x + h / 2 * k1)
        k3 = A @ (x + h / 2 * k2)
        k4 = A @ (x + h * k3)
        x = x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    x0 = np.zeros(9)
    x0[6:] = [1e-5, -2e-5, 3e-5]
    e2 = np.abs(d.Phi @ x0 - x).max()
    print(f"[disc] Phi9 vs CW STM max diff {e1:.2e}; forced response vs RK4 {e2:.2e}")


def run(env, p0, v0, useed, label, **nav_kw):
    nav = NavController(env, lambda vw: CWTargetingEpoch(vw, t_arrive=480.0, deadband=0.002),
                        mode="dv", record=True, **nav_kw)
    env.set_initial_conditions(p0, v0, unc_seed=useed)
    succ = np.zeros(env.num_envs, bool)
    dv = np.zeros(env.num_envs)
    for _ in range(env.max_episode_length):
        env.step_world_dv(nav())
        info = env.last_done_info
        if info is not None:
            succ[info["idx"]] = info["success"]
            dv[info["idx"]] = info["dv"]
    if env.nav_log is not None:  # last epoch's measurements
        env.nav_log = None
    H = nav.hist
    T = len(H)
    per = 1.0 if nav.nav_1hz else env.cfg.control_dt
    t = (np.arange(T) + 1) * per
    nees_pv = np.array([np.mean(_nees(x, P, xt, slice(0, 6))) for x, P, xt, _ in H])
    nees_b = (np.array([np.mean(_nees(x, P, xt, slice(6, 9))) for x, P, xt, _ in H])
              if nav.kf.estimate_bias else None)
    nis = np.array([np.mean(h[3]) for h in H])
    berr = np.array([np.linalg.norm(x[:, 6:] - xt[:, 6:], axis=1)
                     / np.linalg.norm(xt[:, 6:], axis=1) for x, P, xt, _ in H])
    perr = np.array([np.linalg.norm(x[:, :3] - xt[:, :3], axis=1) for x, P, xt, _ in H])
    verr = np.array([np.linalg.norm(x[:, 3:6] - xt[:, 3:6], axis=1) for x, P, xt, _ in H])
    print(f"\n== {label}: success {succ.mean():.1%}, dv mean (succ) "
          f"{dv[succ].mean() if succ.any() else float('nan'):.3f}")
    print(f"   mean NIS {nis.mean():.2f} (6)  mean NEES p,v {nees_pv.mean():.2f} (6)"
          + (f"  NEES b {nees_b.mean():.2f} (3)" if nees_b is not None else ""))
    print(f"   {'t (s)':>6s} {'|p err| med':>12s} {'|v err| med':>12s} "
          f"{'bias err/|b| med':>17s} {'p90':>6s} {'<30%':>6s} {'NEES pv':>8s}")
    for ts in (20, 60, 100, 200, 300, 400, 480, 520, 600):
        k = min(int(np.searchsorted(t, ts)), T - 1)
        print(f"   {t[k]:6.0f} {np.median(perr[k]) * 100:10.1f}cm {np.median(verr[k]) * 1000:9.2f}mm/s"
              f" {np.median(berr[k]):17.2f} {np.percentile(berr[k], 90):6.2f} "
              f"{(berr[k] < 0.3).mean():6.0%} {nees_pv[k]:8.2f}")
    return dict(t=t, nees_pv=nees_pv, nees_b=nees_b, nis=nis, berr=berr, succ=succ, dv=dv)


def _nees(x, P, xt, sl):
    e = (x - xt)[:, sl]
    return np.einsum("ni,nij,nj->n", e, np.linalg.inv(P[:, sl, sl]), e)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=200)
    ap.add_argument("--unc", default="u1")
    ap.add_argument("--device", default="cuda")
    ap.add_argument("--only", default=None, help="comma list of variants to run")
    args = ap.parse_args()
    st = STAGES["a3"]
    cfg = ManeuverEnvCfg(num_envs=args.n, auto_reset=False, device=args.device,
                         **UNCERTAINTY[args.unc], **st)
    p0, v0 = sample_starts(np.random.default_rng(100), args.n, cfg.r_min, cfg.r_max,
                           cfg.cone_deg, cfg.v0_max)
    env = ManeuverEnv(cfg)
    check_disc(env.mean_motion)
    variants = {
        "9-state, 1 Hz nav, bias-compensated CW": dict(nav_1hz=True, use_bias=True),
        "9-state, 1 Hz nav, CW ignores bias": dict(nav_1hz=True, use_bias=False),
        "6-state, 1 Hz nav": dict(nav_1hz=True, use_bias=False, estimate_bias=False),
        "9-state, 1 meas/epoch, bias-compensated CW": dict(nav_1hz=False, use_bias=True),
        "6-state, 1 meas/epoch (finite-burn prediction)": dict(nav_1hz=False, use_bias=False,
                                                                 estimate_bias=False),
        "6-state, 1 meas/epoch, old filter tuning (sigma_w 3e-5)": dict(
            nav_1hz=False, use_bias=False, estimate_bias=False, sigma_w=3e-5),
    }
    for i, (lab, kw) in enumerate(variants.items()):
        if args.only and str(i) not in args.only.split(","):
            continue
        run(env, p0, v0, 5100, lab, **kw)


if __name__ == "__main__":
    main()
