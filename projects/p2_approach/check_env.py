"""Check the GPU approach env against the CPU backend.

    pixi run -e rl --locked python ../projects/p2_approach/check_env.py

1. LVLH helpers (approach_env) vs mjorbit's CPU conversions, same chief state.
2. Baseline controller on the GPU env, validation cases (seed 100), vs the CPU
   numbers in SPEC_2 section 4 (planned T 400 s: dv 0.356, settle 428/440 s;
   fixed v_max 0.2: dv 0.678, settle 226/292 s).
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import mjorbit  # noqa: F401,E402

# isort: split
import numpy as np  # noqa: E402

from approach_env import (HOLD, ApproachEnv, ApproachEnvCfg, baseline_force,  # noqa: E402
                          lvlh_frames, lvlh_to_world, sample_starts, world_to_lvlh)


def check_frames() -> float:
    from mjorbit import MjoModel, OrbitInit, mjo_forward
    from mjorbit.constants import GM_EARTH, R_EARTH

    a = R_EARTH + 400.0
    v = np.sqrt(GM_EARTH / a)
    inc = np.radians(51.6)
    model = MjoModel.from_xml_path(str(Path(__file__).with_name("p2_scene.xml")))
    data = model.make_data(orbit=OrbitInit(R_eci=[a, 0, 0], V_eci=[0, v * np.cos(inc),
                                                                   v * np.sin(inc)]))
    mjo_forward(model, data)
    R, V = np.asarray(data.orbit.R_eci)[None], np.asarray(data.orbit.V_eci)[None]
    C, om = lvlh_frames(R, V)
    rng = np.random.default_rng(0)
    worst = 0.0
    for _ in range(20):
        p, vv = rng.normal(size=3) * 30, rng.normal(size=3) * 0.1
        rw_cpu = data.world_position_from_lvlh(p)
        vw_cpu = data.world_velocity_from_lvlh(p, vv)
        rw, vw = lvlh_to_world(C, om, p[None], vv[None])
        p2, v2 = world_to_lvlh(C, om, rw, vw)
        worst = max(worst, np.abs(rw[0] - rw_cpu).max(), np.abs(vw[0] - vw_cpu).max(),
                    np.abs(p2[0] - p).max(), np.abs(v2[0] - vv).max())
    return worst


def run_baseline(env: ApproachEnv, p0, v0, **kw):
    n = env.num_envs
    env.set_initial_conditions(p0, v0)
    d0 = np.linalg.norm(p0 - HOLD, axis=1)
    done = np.zeros(n, bool)
    res = {k: np.full(n, np.nan) for k in ("success", "collided", "dv", "min_dist", "close_max",
                                           "final_err")}
    ok_hist = np.zeros((env.max_episode_length, n), bool)
    for k in range(env.max_episode_length):
        F = baseline_force(env, d0=d0, **kw)
        env.step_world_force(F)
        _, _, p, v = env.lvlh_state()
        ok_hist[k] = (np.linalg.norm(p - HOLD, axis=1) < env.cfg.pos_tol) & \
            (np.linalg.norm(v, axis=1) < env.cfg.vel_tol)
        info = env.last_done_info
        if info is not None:
            new = ~done[info["idx"]]
            ids = info["idx"][new]
            for key in res:
                res[key][ids] = info[key][new]
            done[ids] = True
    settle = np.full(n, np.nan)
    for i in range(n):
        bad = np.flatnonzero(~ok_hist[:, i])
        settle[i] = (bad[-1] + 2) if bad.size else 1
    s = res["success"].astype(bool)
    return dict(success=s.mean(), collided=np.nansum(res["collided"]),
                settle_med=np.median(settle[s]), settle_max=settle[s].max(),
                dv=res["dv"][s].mean(), min_dist=np.nanmin(res["min_dist"]),
                close_max=np.nanmax(res["close_max"]))


def main() -> None:
    worst = check_frames()
    print(f"[1] LVLH helpers vs CPU conversions: worst abs difference {worst:.2e} "
          f"({'PASS' if worst < 1e-9 else 'CHECK'})")

    n = 200
    p0, v0 = sample_starts(np.random.default_rng(100), n, 20.0, 50.0, 60.0, 0.02)
    env = ApproachEnv(ApproachEnvCfg(num_envs=n, auto_reset=False, seed=0))
    for name, kw, ref in (("planned T 400 s", dict(plan_s=400.0), (0.356, 428, 440)),
                          ("fixed v_max 0.2", dict(v_max=0.2), (0.678, 226, 292))):
        r = run_baseline(env, p0, v0, **kw)
        print(f"[2] GPU baseline {name}: " + "  ".join(f"{k}={v:.4g}" for k, v in r.items())
              + f"   | CPU dv {ref[0]}, settle {ref[1]}/{ref[2]} s")


if __name__ == "__main__":
    main()
