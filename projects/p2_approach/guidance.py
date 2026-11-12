"""Glideslope guidance for 20 s epochs (SPEC_2 section 19), after Hablani, Tapper and
Dana-Bashian (JGCD 2002), applied in closed loop each epoch.

The vehicle moves along the straight line from its current (estimated) position to the
hold point, with a commanded closing speed proportional to the remaining range:

    rho_dot = -k rho,   clipped to [v_min, v_max]

Each epoch the next waypoint is the point s = clip(rho (1 - exp(-k dt)), v_min dt, v_max dt)
(at most rho) along that line, and the Δv is the bias-compensated CW velocity that coasts
to that waypoint in t_go = lookahead epochs. Far out the vehicle cruises at v_max; near
the hold point the closing speed falls with the range, so it arrives at about v_min
instead of the 7-9 cm/s of a single CW arc (SPEC_2 17). After arrival the waypoint is
the hold point itself (station keeping, same as CW epoch).
"""

from __future__ import annotations

import numpy as np

from approach_env import HOLD
from nav_filter import cw_forced_mats


def coast_velocity(n: float, p: np.ndarray, target: np.ndarray, t_go: np.ndarray, b=None):
    """Velocity that coasts from p to target in t_go under CW (+ constant accel b)."""
    rr, rv, gam = cw_forced_mats(n, t_go)
    rhs = target - np.einsum("kij,kj->ki", rr, p)
    if b is not None:
        rhs = rhs - np.einsum("kij,kj->ki", gam, b)
    return np.linalg.solve(rv, rhs[..., None])[..., 0]


def glideslope_dv(n, p, v, b, dt=20.0, k=0.02, v_max=0.15, v_min=0.003, lookahead=1,
                  deadband=0.002, r_switch=None, t_arrive=None, t=None):
    """LVLH Δv (N,3) for one epoch of closed-loop glideslope guidance.
    Hybrid (r_switch, t_arrive set): outside r_switch metres from the hold point fly
    bias-compensated CW targeting to the hold point (t_go = t_arrive - t, >= dt), the
    cheap coasting arc; inside it, the glideslope caps the closing speed."""
    e = HOLD[None] - p
    rho = np.linalg.norm(e, axis=1)
    u = e / np.maximum(rho, 1e-9)[:, None]
    t_go = lookahead * dt
    s = np.clip(rho * (1.0 - np.exp(-k * t_go)), v_min * t_go, v_max * t_go)
    s = np.minimum(s, rho)
    wp = p + u * s[:, None]
    dv = coast_velocity(n, p, wp, np.full(p.shape[0], t_go), b) - v
    if r_switch is not None:
        far = rho > r_switch
        if far.any():
            tg = np.maximum(t_arrive - np.broadcast_to(t, rho.shape)[far], dt)
            dv[far] = coast_velocity(n, p[far], HOLD[None], tg,
                                     None if b is None else b[far]) - v[far]
    return np.where(np.abs(dv).max(axis=1, keepdims=True) > deadband, dv, 0.0)


class GlideslopeEpoch:
    """For ManeuverEnv: world Δv per epoch (drop-in for cw_baseline.CWTargetingEpoch)."""

    def __init__(self, env, deadband=0.002, **gs):
        self.env, self.deadband, self.gs = env, deadband, gs

    def __call__(self) -> np.ndarray:
        env = self.env
        cfg = env.cfg
        C, _, p, v = env.measured_state()
        t = env.episode_length_buf.cpu().numpy() * cfg.control_dt
        dv = glideslope_dv(env.mean_motion, p, v, getattr(env, "est_bias", None),
                           dt=cfg.control_dt, deadband=self.deadband, t=t, **self.gs)
        return np.clip(np.einsum("nij,nj->ni", C, dv), -cfg.dv_max, cfg.dv_max)
