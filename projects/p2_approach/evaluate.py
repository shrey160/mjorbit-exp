"""Evaluate a Project 2A policy against the baselines on matched cases (GPU envs).

    pixi run -e rl --locked python ../projects/p2_approach/evaluate.py \
        [--checkpoint logs/<run>/model_<it>.pt] [--env thrust|maneuver] \
        [--split validation|test] [--physics-dt 0.1] [--out results.json]

Validation seed 100 for every choice; test seed 1000 once (SPEC_2 section 2).
Baselines: planned-PD T 400 s, fast PD v_max 0.2, CW targeting (1 Hz), and for
--env maneuver also CW targeting flown with 20 s epoch dv decisions (the benchmark).
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import mjorbit  # noqa: F401,E402

# isort: split
import numpy as np  # noqa: E402
import torch  # noqa: E402
from approach_env import (HOLD, UNCERTAINTY, ApproachEnv, ApproachEnvCfg,  # noqa: E402
                          baseline_force, sample_starts)
from cw_baseline import CWTargeting1Hz, CWTargetingEpoch, KFController  # noqa: E402
from guidance import GlideslopeEpoch  # noqa: E402
from maneuver_env import ManeuverEnv, ManeuverEnvCfg  # noqa: E402
from nav_filter import NavController  # noqa: E402
from shield import NavTracker, Shield  # noqa: E402
from residual_env import ResidualEnv, ResidualEnvCfg  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402
from train import STAGES, build_train_cfg  # noqa: E402

SPLIT_SEEDS = {"validation": 100, "test": 1000, "val2": 2000}  # val2: fresh, after the test run


def wilson(k: int, n: int, z: float = 1.96):
    p = k / n
    den = 1 + z * z / n
    c = (p + z * z / (2 * n)) / den
    h = z * np.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return (round(c - h, 3), round(c + h, 3))


def rollout(env, p0, v0, act, unc_seed=None):
    """act(env, obs) -> ('lvlh', actions tensor) | ('world', force) | ('dv', world dv)."""
    n = env.num_envs
    obs = env.set_initial_conditions(p0, v0, unc_seed=unc_seed)
    done = np.zeros(n, bool)
    keys = ("success", "collided", "dv", "min_dist", "close_max", "final_err", "final_speed")
    res = {k: np.full(n, np.nan) for k in keys}
    settle_env = np.full(n, np.nan)
    ok_hist = np.zeros((env.max_episode_length, n), bool)
    dact = np.zeros(n)
    prev = np.zeros((n, 3))
    for k in range(env.max_episode_length):
        kind, a = act(env, obs)
        if kind == "lvlh":
            obs, _, _, _ = env.step(a)
        elif kind == "dv":
            obs, _, _, _ = env.step_world_dv(a)
        else:
            obs, _, _, _ = env.step_world_force(a)
        live = ~done
        dact += live * np.linalg.norm(env._last_actions - prev, axis=1)
        prev = env._last_actions.copy()
        _, _, p, v = env.lvlh_state()
        ok_hist[k] = (np.linalg.norm(p - HOLD, axis=1) < env.cfg.pos_tol) & \
            (np.linalg.norm(v, axis=1) < env.cfg.vel_tol)
        info = env.last_done_info
        if info is not None:
            new = ~done[info["idx"]]
            ids = info["idx"][new]
            for key in keys:
                res[key][ids] = info[key][new]
            if "settle" in info:
                settle_env[ids] = info["settle"][new]
            done[ids] = True
    if np.isfinite(settle_env).any():
        settle = settle_env  # epoch env tracks per-second settle time itself
    else:
        settle = np.full(n, np.nan)
        for i in range(n):
            bad = np.flatnonzero(~ok_hist[:, i])
            settle[i] = ((bad[-1] + 2) if bad.size else 1) * env.cfg.control_dt
    res["success"] = res["success"] == 1
    res["collided"] = res["collided"] == 1
    res["settle"] = np.where(res["success"], settle, np.nan)
    res["act_change"] = dact / env.max_episode_length
    sh = getattr(env, "shield", None)
    if sh is not None:
        res["shield_count"] = sh.count.astype(float)
        res["shield_unsafe"] = sh.unsafe_flown.astype(float)
        res["shield_choice"] = sh.choice_hist.sum(axis=0)
        res["shield_margin"] = sh.min_margin.copy()
    return res


def summarize(name, r):
    s = r["success"]
    n = s.size
    g = lambda k: r[k][s]  # noqa: E731
    return {
        "controller": name, "episodes": n, "success": float(s.mean()),
        "success_ci95": wilson(int(s.sum()), n), "collisions": int(r["collided"].sum()),
        "settle_s_median": float(np.median(g("settle"))) if s.any() else None,
        "settle_s_max": float(g("settle").max()) if s.any() else None,
        "dv_mean_successes": float(g("dv").mean()) if s.any() else None,
        "dv_p95_successes": float(np.percentile(g("dv"), 95)) if s.any() else None,
        "dv_p99_successes": float(np.percentile(g("dv"), 99)) if s.any() else None,
        "final_err_m_max": float(g("final_err").max()) if s.any() else None,
        "final_speed_max": float(g("final_speed").max()) if s.any() else None,
        "min_dist_m": float(np.nanmin(r["min_dist"])),
        "close_speed_max_inside_10m": float(np.nanmax(r["close_max"])),
        "action_change_mean": float(r["act_change"].mean()),
        **({"shield_interventions_mean": float(r["shield_count"].mean()),
            "shield_episodes_intervened": float((r["shield_count"] > 0).mean()),
            "shield_epochs_no_safe_option": int(r["shield_unsafe"].sum()),
            "shield_choice_totals": "/".join(str(int(c)) for c in r["shield_choice"]),
            "shield_pred_margin_min": float(np.min(r["shield_margin"]))}
           if "shield_count" in r else {}),
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--checkpoint", default=None)
    ap.add_argument("--env", choices=["thrust", "maneuver", "residual"], default="thrust",
                    help="env the policy runs in (maneuver = 20 s dv decisions)")
    ap.add_argument("--split", choices=list(SPLIT_SEEDS), default="validation")
    ap.add_argument("--stage", choices=sorted(STAGES), default="a3")
    ap.add_argument("--n", type=int, default=200)
    ap.add_argument("--physics-dt", type=float, default=1.0)
    ap.add_argument("--deadzone", type=float, default=0.0)
    ap.add_argument("--cw-arrive", type=float, default=480.0)
    ap.add_argument("--skip-pd", action="store_true", help="skip the PD baselines")
    ap.add_argument("--unc", choices=sorted(UNCERTAINTY), default="none",
                    help="uncertainty preset (SPEC_2 13); draws matched across controllers")
    ap.add_argument("--kf", action="store_true",
                    help="also run the classical controllers behind a CW Kalman filter")
    ap.add_argument("--nav", action="store_true",
                    help="also run the classical controllers behind the 9-state bias filter "
                         "(SPEC_2 15): 1 Hz nav, bias-compensated guidance")
    ap.add_argument("--cw-deadband", type=float, nargs="*", default=[0.002],
                    help="CW re-targeting deadband(s), m/s")
    ap.add_argument("--glideslope", nargs="*", default=None, metavar="K,VMAX,VMIN[,RSWITCH,TARRIVE]",
                    help="also run glideslope guidance + nav9 (SPEC_2 19) for each parameter set")
    ap.add_argument("--base", choices=["cw", "gs"], default="cw",
                    help="residual env base guidance (SPEC_2 19)")
    ap.add_argument("--no-cw", action="store_true", help="skip the CW epoch nav9 runs")
    ap.add_argument("--epoch-only", action="store_true", help="skip the 1 Hz controllers")
    ap.add_argument("--shield", action="store_true",
                    help="also run every 20 s controller behind the B5 shield (SPEC_2 17)")
    ap.add_argument("--shield-only", action="store_true", help="only the shielded runs")
    ap.add_argument("--shield-sigma", type=float, default=3.0)
    ap.add_argument("--shield-horizon", type=float, default=300.0)
    ap.add_argument("--shield-drift", action="store_true",
                    help="backup = one stop then free drift (passive safety) instead of "
                         "stop and station-keep")
    ap.add_argument("--raw-epoch", action="store_true",
                    help="with --nav, also run epoch CW on raw measurements")
    ap.add_argument("--device", default="cuda")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    st = STAGES[args.stage]
    unc = UNCERTAINTY[args.unc]
    useed = SPLIT_SEEDS[args.split] + 5000
    cfg = ApproachEnvCfg(num_envs=args.n, auto_reset=False, device=args.device,
                         physics_dt=args.physics_dt, deadzone=args.deadzone, **unc, **st)
    p0, v0 = sample_starts(np.random.default_rng(SPLIT_SEEDS[args.split]), args.n,
                           cfg.r_min, cfg.r_max, cfg.cone_deg, cfg.v0_max)
    d0 = np.linalg.norm(p0 - HOLD, axis=1)
    env = ApproachEnv(cfg)
    print(f"split={args.split} stage={args.stage} n={args.n} physics_dt={args.physics_dt} "
          f"unc={args.unc}; "
          f"initial distance to hold median {np.median(d0):.1f} m, max {d0.max():.1f} m")

    runs = {}
    if not args.skip_pd and not args.epoch_only:
        runs["PD planned T400"] = rollout(env, p0, v0, lambda e, o: (
            "world", baseline_force(e, plan_s=400.0, d0=d0)), useed)
        runs["PD fast vmax0.2"] = rollout(env, p0, v0, lambda e, o: (
            "world", baseline_force(e, v_max=0.2)), useed)
    if not args.epoch_only:  # 1 Hz controllers (ApproachEnv)
        for db in args.cw_deadband:
            cw1 = CWTargeting1Hz(env, t_arrive=args.cw_arrive, deadband=db)
            runs[f"CW 1Hz db{db * 1000:g}"] = rollout(env, p0, v0, lambda e, o, c=cw1: (
                "world", c()), useed)
        bkey = f"CW 1Hz db{args.cw_deadband[0] * 1000:g}"
        if args.kf:
            k = KFController(env, lambda vw: (lambda: baseline_force(vw, plan_s=400.0, d0=d0)))
            runs["PD planned KF"] = rollout(env, p0, v0, lambda e, o, c=k: ("world", c()), useed)
            for db in args.cw_deadband:
                k = KFController(env, lambda vw, db=db: CWTargeting1Hz(vw, t_arrive=args.cw_arrive,
                                                                      deadband=db))
                runs[f"CW 1Hz KF db{db * 1000:g}"] = rollout(env, p0, v0, lambda e, o, c=k: (
                    "world", c()), useed)
        if args.nav:
            k = NavController(env, lambda vw: (lambda: baseline_force(vw, plan_s=400.0, d0=d0)))
            runs["PD planned nav9"] = rollout(env, p0, v0, lambda e, o, c=k: ("world", c()), useed)
            for db in args.cw_deadband:
                k = NavController(env, lambda vw, db=db: CWTargeting1Hz(vw, t_arrive=args.cw_arrive,
                                                                       deadband=db))
                runs[f"CW 1Hz nav9 db{db * 1000:g}"] = rollout(env, p0, v0, lambda e, o, c=k: (
                    "world", c()), useed)
    penv = env

    def epoch_run(label, env_, build, keep_log=False):
        """build() -> (act, nav getter or None). Runs the controller plain and, with
        --shield, again behind the B5 shield (SPEC_2 17) on the same draws."""
        variants = [False, True] if args.shield else [False]
        if args.shield_only:
            variants = [True]
        for shielded in variants:
            act, nav = build()
            if shielded:
                if nav is None:  # controller runs no filter: give the shield its own
                    tr = NavTracker(env_)
                    nav = lambda tr=tr: tr.kf  # noqa: E731
                    act = (lambda e, o, inner=act, tr=tr: (tr(), inner(e, o))[1])
                env_.shield = Shield(env_, nav, t_arrive=args.cw_arrive, n_sigma=args.shield_sigma,
                                     horizon=args.shield_horizon,
                                     backup_hold=not args.shield_drift)
            runs[label + (" +S" if shielded else "")] = rollout(env_, p0, v0, act, useed)
            env_.shield = None
            if not keep_log:
                env_.nav_log = None

    gs_sets = []
    for spec in args.glideslope or []:
        vals = [float(x) for x in spec.split(",")]
        d = dict(k=vals[0], v_max=vals[1], v_min=vals[2])
        if len(vals) == 5:
            d.update(r_switch=vals[3], t_arrive=vals[4])
        gs_sets.append(d)
    if args.env in ("maneuver", "residual"):
        penv = ManeuverEnv(ManeuverEnvCfg(num_envs=args.n, auto_reset=False, device=args.device,
                                          physics_dt=args.physics_dt, **unc, **st))
        for db in args.cw_deadband:
            if not args.nav or args.raw_epoch:
                epoch_run(f"CW epoch db{db * 1000:g}", penv, lambda db=db: (
                    (lambda e, o, c=CWTargetingEpoch(penv, t_arrive=args.cw_arrive, deadband=db):
                     ("dv", c())), None))
            if args.kf:
                k = KFController(penv, lambda vw, db=db: CWTargetingEpoch(
                    vw, t_arrive=args.cw_arrive, deadband=db), mode="dv")
                runs[f"CW epoch KF db{db * 1000:g}"] = rollout(penv, p0, v0, lambda e, o, c=k: (
                    "dv", c()), useed)
            if args.nav and not args.no_cw:
                for lab, n1 in (("nav9", True), ("nav9 1meas", False)):
                    if not n1 and args.shield:
                        continue  # the shield's filter assumes 1 Hz navigation

                    def build(db=db, n1=n1):
                        k = NavController(penv, lambda vw: CWTargetingEpoch(
                            vw, t_arrive=args.cw_arrive, deadband=db), mode="dv", nav_1hz=n1)
                        return (lambda e, o, c=k: ("dv", c())), (lambda k=k: k.kf)
                    epoch_run(f"CW epoch {lab} db{db * 1000:g}", penv, build)
            for gi, gs in enumerate(gs_sets):
                def build_gs(db=db, gs=gs):
                    k = NavController(penv, lambda vw: GlideslopeEpoch(vw, deadband=db, **gs),
                                      mode="dv")
                    return (lambda e, o, c=k: ("dv", c())), (lambda k=k: k.kf)
                tag = "/".join(f"{v:g}" for v in gs.values())
                epoch_run(f"GS {tag}", penv, build_gs)
        bkey = f"CW epoch db{args.cw_deadband[0] * 1000:g}"
        if args.nav:
            bkey = f"CW epoch nav9 db{args.cw_deadband[0] * 1000:g}"
    if args.env == "residual":
        penv = ResidualEnv(ResidualEnvCfg(num_envs=args.n, auto_reset=False, device=args.device,
                                          physics_dt=args.physics_dt, base=args.base,
                                          **unc, **st))
        zero = torch.zeros(args.n, 3, device=args.device)
        epoch_run("Residual a=0", penv, lambda: ((lambda e, o: ("lvlh", zero)),
                                                 (lambda: penv.kf)), keep_log=True)
    if args.checkpoint:
        gamma = 0.998 if args.env == "thrust" else 0.99
        runner = OnPolicyRunner(penv, build_train_cfg(0, gamma), log_dir=None, device=args.device)
        runner.load(args.checkpoint)
        policy = runner.get_inference_policy(device=args.device)
        with torch.no_grad():
            if args.env == "thrust":
                runs["PPO policy"] = rollout(penv, p0, v0, lambda e, o: ("lvlh", policy(o)), useed)
            else:
                nav = (lambda: penv.kf) if args.env == "residual" else None
                epoch_run("PPO policy", penv, lambda: ((lambda e, o: ("lvlh", policy(o))), nav),
                          keep_log=args.env == "residual")

    rows = [summarize(k, r) for k, r in runs.items()]
    keys = []
    for r in rows:
        keys += [k for k in r if k != "controller" and k not in keys]
    print(f"\n{'metric':30s}" + "".join(f"{r['controller'][:21]:>22s}" for r in rows))
    for k in keys:
        cells = []
        for r in rows:
            v = r.get(k, "")
            cells.append(f"{v:>22.4g}" if isinstance(v, float) else f"{str(v):>22s}")
        print(f"{k[:30]:30s}" + "".join(cells))

    out = {"split": args.split, "stage": args.stage, "physics_dt": args.physics_dt,
           "unc": args.unc, "env": args.env, "checkpoint": args.checkpoint,
           "shield": {"n_sigma": args.shield_sigma, "horizon": args.shield_horizon,
                      "backup": "stop+drift" if args.shield_drift else "stop+station-keep"}
           if (args.shield or args.shield_only) else None, "results": rows}
    out["paired"] = {}
    pairs = [(bkey + suf, "PPO policy" + suf) for suf in ("", " +S")]
    pairs += [("Residual a=0" + suf, "PPO policy" + suf) for suf in ("", " +S")]
    for bk, pk in pairs:
        if bk not in runs or pk not in runs:
            continue
        b, p = runs[bk], runs[pk]
        sb, sp = b["success"], p["success"]
        both = sb & sp
        diff = p["dv"][both] - b["dv"][both]
        rec = {"benchmark": bk,
               "overlap": {"both": int(both.sum()), "baseline_only": int((sb & ~sp).sum()),
                           "policy_only": int((~sb & sp).sum()), "neither": int((~sb & ~sp).sum())},
               "policy_minus_benchmark_dv_mean": float(diff.mean()) if both.any() else None,
               "policy_lower_frac": float((diff < 0).mean()) if both.any() else None}
        out["paired"][f"{pk} vs {bk}"] = rec
        print(f"\n{pk} vs {bk}: {rec}")
    if args.out:
        Path(args.out).write_text(json.dumps(out, indent=2))
        print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
