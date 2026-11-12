"""Vectorized CW impulsive-targeting baselines (SPEC_2 section 10) for the GPU envs.

CW targeting: from the current LVLH state, the velocity that coasts to the hold point
in t_go seconds is v_req = Φrv(t_go)^-1 (hold - Φrr(t_go) p). Burn the difference,
coast, and re-target only when the velocity error exceeds a deadband.
"""

from __future__ import annotations

import numpy as np

from approach_env import HOLD, ApproachEnv, baseline_force


def cw_mats(n: float, t: np.ndarray):
    """Batched CW blocks Φrr, Φrv (k,3,3) for times t (k,)."""
    nt = n * t
    c, s = np.cos(nt), np.sin(nt)
    z, o = np.zeros_like(t), np.ones_like(t)
    rr = np.stack([np.stack([4 - 3 * c, z, z], -1), np.stack([6 * (s - nt), o, z], -1),
                   np.stack([z, z, c], -1)], -2)
    rv = np.stack([np.stack([s / n, 2 * (1 - c) / n, z], -1),
                   np.stack([-2 * (1 - c) / n, (4 * s - 3 * nt) / n, z], -1),
                   np.stack([z, z, s / n], -1)], -2)
    return rr, rv


def v_required(n: float, p: np.ndarray, t_go: np.ndarray, b=None) -> np.ndarray:
    if b is not None:  # bias-compensated (SPEC_2 15): include the estimated constant accel
        from nav_filter import v_required_bias
        return v_required_bias(n, p, t_go, b)
    rr, rv = cw_mats(n, t_go)
    rhs = HOLD[None] - np.einsum("kij,kj->ki", rr, p)
    return np.linalg.solve(rv, rhs[..., None])[..., 0]


class CWTargeting1Hz:
    """For ApproachEnv (1 s decisions): returns world forces."""

    def __init__(self, env: ApproachEnv, t_arrive=480.0, deadband=0.002, switch_s=10.0):
        self.env, self.t_arrive, self.deadband, self.switch_s = env, t_arrive, deadband, switch_s
        self.burning = np.zeros(env.num_envs, bool)

    def __call__(self) -> np.ndarray:
        env = self.env
        cfg = env.cfg
        C, _, p, v = env.measured_state()
        t = env.episode_length_buf.cpu().numpy() * cfg.control_dt
        t_go = self.t_arrive - t
        far = t_go > self.switch_s
        F = baseline_force(env, v_max=0.05)  # PD hold near/after arrival
        if far.any():
            b = getattr(env, "est_bias", None)
            dv = v_required(env.mean_motion, p[far], t_go[far],
                            None if b is None else b[far]) - v[far]
            big = np.abs(dv).max(axis=1)
            fire = self.burning[far] | (big > self.deadband)
            self.burning[far] = fire & (big > self.deadband * 0.1)
            u = np.where(fire[:, None], dv / cfg.control_dt, 0.0)
            F[far] = np.clip(cfg.mass * np.einsum("kij,kj->ki", C[far], u), -cfg.f_max, cfg.f_max)
        return F


class CWTargetingEpoch:
    """For ManeuverEnv: returns world Δv per epoch. Arrive at t_arrive, then re-target
    the hold point one epoch ahead each epoch (station keeping)."""

    def __init__(self, env, t_arrive=480.0, deadband=0.002):
        self.env, self.t_arrive, self.deadband = env, t_arrive, deadband

    def __call__(self) -> np.ndarray:
        env = self.env
        cfg = env.cfg
        C, _, p, v = env.measured_state()
        t = env.episode_length_buf.cpu().numpy() * cfg.control_dt
        t_go = np.maximum(self.t_arrive - t, cfg.control_dt)
        dv = v_required(env.mean_motion, p, t_go, getattr(env, "est_bias", None)) - v
        dv = np.where(np.abs(dv).max(axis=1, keepdims=True) > self.deadband, dv, 0.0)
        return np.clip(np.einsum("nij,nj->ni", C, dv), -cfg.dv_max, cfg.dv_max)


# --------------------------------------------------------------------- navigation filter
def cw_stm6(n: float, dt: float) -> np.ndarray:
    """6x6 CW state-transition matrix over dt (state = LVLH p, v)."""
    nt = n * dt
    c, s = np.cos(nt), np.sin(nt)
    rr = np.array([[4 - 3 * c, 0, 0], [6 * (s - nt), 1, 0], [0, 0, c]])
    rv = np.array([[s / n, 2 * (1 - c) / n, 0], [-2 * (1 - c) / n, (4 * s - 3 * nt) / n, 0],
                   [0, 0, s / n]])
    vr = np.array([[3 * n * s, 0, 0], [6 * n * (c - 1), 0, 0], [0, 0, -n * s]])
    vv = np.array([[c, 2 * s, 0], [-2 * s, 4 * c - 3, 0], [0, 0, c]])
    return np.block([[rr, rv], [vr, vv]])


class CWKalman:
    """Batched linear Kalman filter on LVLH (p, v) with CW dynamics and the controller's
    own commanded input. Process noise covers the disturbance and the thrust execution
    error: sigma_a^2 = sigma_dist^2 + (rel * |commanded accel|)^2 per axis."""

    def __init__(self, env, sigma_dist=3e-5, rel=0.12):
        cfg = env.cfg
        self.env, self.sigma_dist, self.rel = env, sigma_dist, rel
        sp, sv = max(cfg.nav_pos_sigma, 1e-4), max(cfg.nav_vel_sigma, 1e-5)
        self.R = np.diag([sp**2] * 3 + [sv**2] * 3)
        self.x = np.zeros((env.num_envs, 6))
        self.P = np.tile(self.R, (env.num_envs, 1, 1))

    def reset(self, idx, z):
        self.x[idx] = z[idx]
        self.P[idx] = self.R

    def predict(self, dt: float, u=None, dv=None):
        Phi = cw_stm6(self.env.mean_motion, dt)
        a_mag = np.zeros((self.x.shape[0], 3))
        if dv is not None:
            self.x[:, 3:] += dv
            a_mag = np.abs(dv) / dt
        self.x = self.x @ Phi.T
        if u is not None:
            self.x[:, :3] += 0.5 * u * dt**2
            self.x[:, 3:] += u * dt
            a_mag = np.abs(u)
        q = self.sigma_dist**2 + (self.rel * a_mag) ** 2  # (n,3)
        Q = np.zeros_like(self.P)
        for i in range(3):
            Q[:, i, i] = q[:, i] * dt**4 / 4
            Q[:, i, i + 3] = Q[:, i + 3, i] = q[:, i] * dt**3 / 2
            Q[:, i + 3, i + 3] = q[:, i] * dt**2
        self.P = Phi @ self.P @ Phi.T + Q

    def update(self, z):
        S = self.P + self.R
        K = self.P @ np.linalg.inv(S)
        self.x = self.x + np.einsum("nij,nj->ni", K, z - self.x)
        self.P = (np.eye(6) - K) @ self.P


class _FilteredView:
    """Env proxy whose measured_state() returns the filter estimate."""

    def __init__(self, env, kf):
        self._env, self._kf = env, kf

    def __getattr__(self, name):
        return getattr(self._env, name)

    def measured_state(self):
        C, om, _, _ = self._env.lvlh_state()
        return C, om, self._kf.x[:, :3], self._kf.x[:, 3:]


class KFController:
    """Wrap a classical controller with the CW Kalman filter.
    make_ctrl(view) -> callable returning world force (mode 'force') or world dv ('dv')."""

    def __init__(self, env, make_ctrl, mode="force", **kf_kw):
        self.env, self.mode = env, mode
        self.kf = CWKalman(env, **kf_kw)
        self.ctrl = make_ctrl(_FilteredView(env, self.kf))
        self.last = None

    def __call__(self):
        env = self.env
        z = np.concatenate([env.meas_p, env.meas_v], axis=1)
        step = env.episode_length_buf.cpu().numpy()
        start = step == 0
        if self.last is None:
            self.kf.reset(np.arange(env.num_envs), z)
        else:
            dt = env.cfg.control_dt
            if self.mode == "force":
                self.kf.predict(dt, u=self.last)
            else:
                self.kf.predict(dt, dv=self.last)
            self.kf.update(z)
            if start.any():
                self.kf.reset(np.flatnonzero(start), z)
        out = self.ctrl()
        C, _, _, _ = env.lvlh_state()
        lv = np.einsum("nji,nj->ni", C, out)
        self.last = lv / env.cfg.mass if self.mode == "force" else lv
        return out
