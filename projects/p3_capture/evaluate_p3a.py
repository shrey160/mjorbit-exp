"""Evaluate Project 3A controllers on matched cases (SPEC_3 section 3).

    python ../projects/p3_capture/evaluate_p3a.py [--unc u1] [--yaw random|aligned]
        [--client-rot lvlh|inertial] [--n 200] [--split validation|test] [--out f.json]

Cases: start states as Project 2 (20-50 m, 60 deg cone around -V-bar), client yaw
uniform +-180 deg, uncertainty draws matched across controllers. Validation seed 100,
test seed 1000 (unused).
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "p2_approach"))

import mjorbit  # noqa: F401,E402

# isort: split
import numpy as np  # noqa: E402
from approach_env import UNCERTAINTY, sample_starts  # noqa: E402
from capture_env import CaptureEnv, CaptureEnvCfg  # noqa: E402
from capture_guidance import CaptureGuidance  # noqa: E402
from evaluate import wilson  # noqa: E402
from nav_filter import NavController  # noqa: E402

SPLIT_SEEDS = {"validation": 100, "test": 1000}


def rollout(env, p0, v0, yaw, ctrl_factory, useed):
    n = env.num_envs
    env.set_initial_conditions(p0, v0, unc_seed=useed, yaw=yaw)
    ctrl, guid = ctrl_factory(env)
    keys = ("success", "collided", "dv", "min_dist", "final_err", "final_speed", "settle")
    res = {k: np.full(n, np.nan) for k in keys}
    done = np.zeros(n, bool)
    corridor_viol = np.zeros(n, np.int64)
    phases = np.zeros((n, 3), np.int64)
    tan = guid.tan_cone
    for _ in range(env.max_episode_length):
        dv = ctrl()
        # corridor check on the true state at each decision (within 5 m of the feature)
        _, _, p, _ = env.lvlh_state()
        geo = env.client_geometry()
        d = p - geo["f"]
        along = np.sum(d * geo["nf"], axis=1)
        lat = np.linalg.norm(d - along[:, None] * geo["nf"], axis=1)
        near = np.linalg.norm(d, axis=1) < 5.0
        corridor_viol += ~done & near & ((along <= 0) | (lat > tan * along + 0.3))
        phases[np.arange(n), guid.phase] += ~done
        env.step_world_dv(dv)
        info = env.last_done_info
        if info is not None:
            new = ~done[info["idx"]]
            ids = info["idx"][new]
            for k in keys:
                res[k][ids] = info[k][new]
            done[ids] = True
    res["success"] = res["success"] == 1
    res["collided"] = res["collided"] == 1
    res["corridor_viol"] = corridor_viol
    res["phase_epochs"] = phases.sum(axis=0)
    env.nav_log = None
    return res


def summarize(name, r, yaw):
    s = r["success"]
    g = lambda k: r[k][s]  # noqa: E731
    face_away = np.abs(np.angle(np.exp(1j * yaw))) > np.pi / 2
    return {
        "controller": name, "episodes": int(s.size), "success": float(s.mean()),
        "success_ci95": wilson(int(s.sum()), s.size), "collisions": int(r["collided"].sum()),
        "success_face_towards": float(s[~face_away].mean()) if (~face_away).any() else None,
        "success_face_away": float(s[face_away].mean()) if face_away.any() else None,
        "settle_s_median": float(np.median(g("settle"))) if s.any() else None,
        "settle_s_max": float(g("settle").max()) if s.any() else None,
        "dv_mean_successes": float(g("dv").mean()) if s.any() else None,
        "dv_p95_successes": float(np.percentile(g("dv"), 95)) if s.any() else None,
        "final_err_m_max": float(g("final_err").max()) if s.any() else None,
        "final_speed_max": float(g("final_speed").max()) if s.any() else None,
        "min_clearance_m": float(np.nanmin(r["min_dist"])),
        "episodes_with_corridor_violation": float((r["corridor_viol"] > 0).mean()),
        "phase_epochs_route/waypoint/corridor": "/".join(str(int(x)) for x in r["phase_epochs"]),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--unc", choices=sorted(UNCERTAINTY), default="none")
    ap.add_argument("--yaw", choices=["random", "aligned"], default="random")
    ap.add_argument("--client-rot", choices=["lvlh", "inertial"], default="lvlh")
    ap.add_argument("--n", type=int, default=200)
    ap.add_argument("--split", choices=list(SPLIT_SEEDS), default="validation")
    ap.add_argument("--episode-s", type=float, default=1200.0)
    ap.add_argument("--device", default="cuda")
    ap.add_argument("--gkw", nargs="*", default=[], metavar="NAME=VALUE",
                    help="CaptureGuidance parameter overrides, e.g. seg_margin=3")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    seed = SPLIT_SEEDS[args.split]
    cfg = CaptureEnvCfg(num_envs=args.n, auto_reset=False, device=args.device,
                        yaw_mode=args.yaw, client_rot=args.client_rot,
                        episode_length_s=args.episode_s, **UNCERTAINTY[args.unc])
    rng = np.random.default_rng(seed)
    p0, v0 = sample_starts(rng, args.n, cfg.r_min, cfg.r_max, cfg.cone_deg, cfg.v0_max)
    yaw = (np.random.default_rng(seed + 1).uniform(-np.pi, np.pi, args.n)
           if args.yaw == "random" else np.zeros(args.n))
    env = CaptureEnv(cfg)
    useed = seed + 5000
    gkw = {k: float(v) for k, v in (x.split("=") for x in args.gkw)}
    print(f"P3A split={args.split} n={args.n} unc={args.unc} yaw={args.yaw} "
          f"client_rot={args.client_rot} episode {args.episode_s:g} s guidance {args.gkw}")

    def raw(e):
        g = CaptureGuidance(e, **gkw)
        return g, g

    def nav(e):
        holder = {}

        def make(view):
            holder["g"] = CaptureGuidance(view, **gkw)
            return holder["g"]
        k = NavController(e, make, mode="dv")
        return k, holder["g"]

    runs = {"Guidance raw meas": rollout(env, p0, v0, yaw, raw, useed),
            "Guidance + nav9": rollout(env, p0, v0, yaw, nav, useed)}
    rows = [summarize(k, r, yaw) for k, r in runs.items()]
    keys = [k for k in rows[0] if k != "controller"]
    print(f"\n{'metric':38s}" + "".join(f"{r['controller']:>22s}" for r in rows))
    for k in keys:
        cells = []
        for r in rows:
            v = r[k]
            cells.append(f"{v:>22.4g}" if isinstance(v, float) else f"{str(v):>22s}")
        print(f"{k[:38]:38s}" + "".join(cells))
    if args.out:
        Path(args.out).write_text(json.dumps({"args": vars(args), "results": rows}, indent=2))
        print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
