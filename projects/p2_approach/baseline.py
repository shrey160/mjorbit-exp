"""Project 2 baseline: velocity-limited LVLH PD with CW feedforward (CPU backend).

Run from the simulator repo root:
    pixi run -e rl --locked python ../projects/p2_approach/baseline.py [--n 200] [--sweep]

Controller (all in LVLH: x radial, y along-track, z orbit normal):
    e      = hold_point - p
    v_ref  = e_hat * min(Kp |e|, sqrt(2 beta a_min |e|), v_max)
    u      = Kd (v_ref - v) - cw(p, v)          (cancel the relative-gravity terms)
    F      = clip(m * C_lvlh_to_world u, +-F_max) per world axis
with cw(p, v) = (3 n^2 x + 2 n vy, -2 n vx, -n^2 z).
"""

from __future__ import annotations

import argparse
import itertools
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from mjorbit import MjoModel, MjoSpec, OrbitInit, mjo_forward, mjo_step
from mjorbit.constants import GM_EARTH, R_EARTH

XML = Path(__file__).with_name("p2_scene.xml")
ALT_KM, INC = 400.0, np.radians(51.6)
A_KM = R_EARTH + ALT_KM
N = float(np.sqrt(GM_EARTH / A_KM**3))
M, F_MAX = 10.0, 0.05
A_MIN = F_MAX / M - 3 * N**2 * 50.0  # per-axis authority left after the worst CW term

# --- task contract (SPEC_2) ---
HOLD = np.array([0.0, -3.0, 0.0])  # m, LVLH, 3 m behind the target on V-bar
KEEP_OUT = 2.0  # m from target centre; entering it is a failure
POS_TOL, VEL_TOL = 0.25, 0.01  # m, m/s
HOLD_S = 60.0  # success condition must hold for the final 60 s
EPISODE_S, CONTROL_DT = 600.0, 1.0
R_MIN, R_MAX, CONE_DEG, V0_MAX = 20.0, 50.0, 60.0, 0.02


@dataclass
class Gains:
    Kp: float = 0.05
    Kd: float = 0.5
    beta: float = 0.8
    v_max: float = 0.2


def orbit() -> OrbitInit:
    v = np.sqrt(GM_EARTH / A_KM)
    return OrbitInit(R_eci=[A_KM, 0.0, 0.0], V_eci=[0.0, v * np.cos(INC), v * np.sin(INC)])


def sample_cases(seed: int, n: int):
    """Start positions on a spherical-shell cap around -V-bar, small LVLH velocities."""
    rng = np.random.default_rng(seed)
    r = rng.uniform(R_MIN, R_MAX, n)
    cos_t = rng.uniform(np.cos(np.radians(CONE_DEG)), 1.0, n)
    sin_t = np.sqrt(1 - cos_t**2)
    phi = rng.uniform(0, 2 * np.pi, n)
    # cone axis = -y; perpendicular plane spanned by x and z
    p = np.stack([r * sin_t * np.cos(phi), -r * cos_t, r * sin_t * np.sin(phi)], axis=1)
    v = rng.uniform(-V0_MAX, V0_MAX, (n, 3))
    return p, v


def cw_accel(p, v):
    return np.array([3 * N**2 * p[0] + 2 * N * v[1], -2 * N * v[0], -N**2 * p[2]])


def control(p, v, g: Gains):
    e = HOLD - p
    d = np.linalg.norm(e)
    speed = min(g.Kp * d, np.sqrt(2 * g.beta * A_MIN * d), g.v_max)
    v_ref = e / d * speed if d > 1e-9 else np.zeros(3)
    return g.Kd * (v_ref - v) - cw_accel(p, v)


def cw_mats(t):
    """CW state-transition blocks (rr, rv, vr, vv) over time t."""
    nt = N * t
    c, s = np.cos(nt), np.sin(nt)
    rr = np.array([[4 - 3 * c, 0, 0], [6 * (s - nt), 1, 0], [0, 0, c]])
    rv = np.array([[s / N, 2 * (1 - c) / N, 0], [-2 * (1 - c) / N, (4 * s - 3 * nt) / N, 0],
                   [0, 0, s / N]])
    vr = np.array([[3 * N * s, 0, 0], [6 * N * (c - 1), 0, 0], [0, 0, -N * s]])
    vv = np.array([[c, 2 * s, 0], [-2 * s, 4 * c - 3, 0], [0, 0, c]])
    return rr, rv, vr, vv


class CWTargeting:
    """Impulsive CW targeting: burn onto the coasting arc that reaches the hold point
    at t_arrive, coast (re-targeting only when the velocity error exceeds a deadband),
    then hand over to the PD hold controller near arrival."""

    def __init__(self, t_arrive=480.0, deadband=0.002, switch_s=10.0):
        self.t_arrive, self.deadband, self.switch_s = t_arrive, deadband, switch_s
        self.burning = False

    def __call__(self, p, v, t, hold_gains):
        t_go = self.t_arrive - t
        if t_go <= self.switch_s:
            return control(p, v, hold_gains)
        rr, rv, _, _ = cw_mats(t_go)
        v_req = np.linalg.solve(rv, HOLD - rr @ p)
        dv = v_req - v
        if self.burning or np.abs(dv).max() > self.deadband:
            self.burning = np.abs(dv).max() > self.deadband * 0.1
            return dv / CONTROL_DT  # accel that closes the error this step (clipped later)
        return np.zeros(3)


def lvlh_basis(data):
    """Columns = LVLH unit axes in world coordinates (C_lvlh_to_world)."""
    return np.stack([data.world_position_from_lvlh(np.eye(3)[i]) for i in range(3)], axis=1)


def rollout(model, p0, v0, g: Gains | None, plan_s: float | None = None, cwt=None):
    """plan_s: if set, v_max = |hold - p0| / plan_s (cruise speed planned per case)."""
    if g is not None and plan_s is not None:
        g = Gains(Kp=g.Kp, Kd=g.Kd, beta=g.beta,
                  v_max=float(np.linalg.norm(HOLD - p0)) / plan_s)
    data = model.make_data(orbit=orbit())
    data.qpos[:] = 0.0
    data.qpos[3] = data.qpos[10] = 1.0
    data.qvel[:] = 0.0
    data.qpos[0:3] = data.world_position_from_lvlh(p0)
    data.qvel[0:3] = data.world_velocity_from_lvlh(p0, v0)
    mjo_forward(model, data)
    decim = int(round(CONTROL_DT / model.opt.timestep))
    steps = int(round(EPISODE_S / CONTROL_DT))
    ok_hist = np.zeros(steps, bool)
    dist_min, dv, sat, close_max = np.inf, 0.0, 0, 0.0
    for k in range(steps):
        pw, vw = np.asarray(data.qpos[0:3]), np.asarray(data.qvel[0:3])
        p = data.lvlh_position_from_world(pw)
        v = data.lvlh_velocity_from_world(pw, vw)
        if cwt is not None:
            F = np.clip(M * lvlh_basis(data) @ cwt(p, v, k * CONTROL_DT, Gains(v_max=0.05)),
                        -F_MAX, F_MAX)
        elif g is None:
            F = np.zeros(3)
        else:
            F = np.clip(M * lvlh_basis(data) @ control(p, v, g), -F_MAX, F_MAX)
        sat += int(np.any(np.abs(F) >= F_MAX * 0.999))
        dv += np.sum(np.abs(F)) / M * CONTROL_DT
        data.ctrl[:] = F
        for _ in range(decim):
            mjo_step(model, data)
        pw, vw = np.asarray(data.qpos[0:3]), np.asarray(data.qvel[0:3])
        p = data.lvlh_position_from_world(pw)
        v = data.lvlh_velocity_from_world(pw, vw)
        r = np.linalg.norm(p)
        dist_min = min(dist_min, r)
        if r < 10.0:
            close_max = max(close_max, -np.dot(p, v) / r)  # closing speed inside 10 m
        ok_hist[k] = np.linalg.norm(p - HOLD) < POS_TOL and np.linalg.norm(v) < VEL_TOL
        if r < KEEP_OUT:
            return dict(success=False, collided=True, settle=np.nan, dist_min=r, dv=dv,
                        final_pos_err=np.linalg.norm(p - HOLD), final_speed=np.linalg.norm(v),
                        sat_frac=sat / (k + 1), close_max=close_max)
    hold_steps = int(HOLD_S / CONTROL_DT)
    bad = np.flatnonzero(~ok_hist)
    settle = (bad[-1] + 2 if bad.size else 1) * CONTROL_DT  # first time ok holds to the end
    success = bool(ok_hist[-hold_steps:].all())
    return dict(success=success, collided=False, settle=settle if success else np.nan,
                dist_min=dist_min, dv=dv, final_pos_err=np.linalg.norm(p - HOLD),
                final_speed=np.linalg.norm(v), sat_frac=sat / steps, close_max=close_max)


def summarize(res):
    s = np.array([r["success"] for r in res])
    col = np.array([r["collided"] for r in res])
    settle = np.array([r["settle"] for r in res])
    st = settle[s]
    f = lambda k: np.array([r[k] for r in res])  # noqa: E731
    return dict(success=s.mean(), collided=col.mean(),
                settle_med=np.median(st) if st.size else np.nan,
                settle_max=st.max() if st.size else np.nan,
                dv_mean=f("dv")[s].mean() if s.any() else np.nan,
                dist_min=f("dist_min").min(), final_err_max=f("final_pos_err")[s].max()
                if s.any() else np.nan, close_max=f("close_max").max(),
                sat=f("sat_frac").mean())


def fmt(d):
    return "  ".join(f"{k}={v:.4g}" if isinstance(v, float) else f"{k}={v}" for k, v in d.items())


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=200)
    ap.add_argument("--seed", type=int, default=100)
    ap.add_argument("--sweep", action="store_true")
    ap.add_argument("--dt", type=float, default=None, help="override physics timestep (s)")
    ap.add_argument("--vmax", type=float, nargs="*", default=[0.2],
                    help="v_max values to run with the frozen Kp/Kd")
    ap.add_argument("--cw", type=float, nargs="*", default=[],
                    help="CW two-impulse targeting with these arrival times (s)")
    ap.add_argument("--plan", type=float, nargs="*", default=[],
                    help="planned transfer times (s): v_max = initial distance / plan")
    args = ap.parse_args()
    model = (MjoModel.from_xml_path(str(XML)) if args.dt is None
             else MjoSpec.from_xml_path(str(XML)).compile(mj_timestep=args.dt))
    p0s, v0s = sample_cases(args.seed, args.n)
    d0 = np.linalg.norm(p0s - HOLD, axis=1)
    print(f"n={N:.4e} rad/s  a_min={A_MIN:.2e} m/s^2  initial distance to hold "
          f"median {np.median(d0):.1f} m, max {d0.max():.1f} m; "
          f"bang-bang time bound max {2 * np.sqrt(d0.max() / A_MIN):.0f} s "
          f"vs {EPISODE_S - HOLD_S:.0f} s budget")

    t0 = time.perf_counter()
    nc = min(args.n, 50)
    free = [rollout(model, p0s[i], v0s[i], None) for i in range(nc)]
    print(f"no control ({nc} cases): {fmt(summarize(free))}  [{time.perf_counter() - t0:.0f} s]")

    grid = [Gains(v_max=vm) for vm in args.vmax]
    if args.sweep:
        grid = [Gains(Kp=kp, Kd=kd, v_max=vm) for kp, kd, vm in
                itertools.product([0.02, 0.05, 0.1], [0.2, 0.5, 2.0], [0.1, 0.2, 0.4])]
    for g in grid:
        t0 = time.perf_counter()
        n = args.n if not args.sweep else min(args.n, 60)
        res = [rollout(model, p0s[i], v0s[i], g) for i in range(n)]
        print(f"Kp={g.Kp} Kd={g.Kd} vmax={g.v_max} ({n}): {fmt(summarize(res))}  "
              f"[{time.perf_counter() - t0:.0f} s]", flush=True)
    for T in args.plan:
        res = [rollout(model, p0s[i], v0s[i], Gains(), plan_s=T) for i in range(args.n)]
        print(f"planned T={T:.0f} s ({args.n}): {fmt(summarize(res))}", flush=True)
    for ta in args.cw:
        res = [rollout(model, p0s[i], v0s[i], None, cwt=CWTargeting(t_arrive=ta))
               for i in range(args.n)]
        print(f"CW targeting arrive {ta:.0f} s ({args.n}): {fmt(summarize(res))}", flush=True)
    lb = 2 * d0 / (EPISODE_S - HOLD_S)
    print(f"rough dv lower bound (accelerate + brake to cover d0 in {EPISODE_S - HOLD_S:.0f} s, "
          f"ignoring v0 and CW): mean {lb.mean():.3f} m/s, max {lb.max():.3f}")


if __name__ == "__main__":
    main()
