"""Project 2B step B1: bias-estimating relative-navigation filter (SPEC_2 section 15).

State (LVLH, per world): x = [p (3), v (3), b (3)], where b is an unknown constant
acceleration (the u1 "unmodelled disturbance"). Dynamics are CW plus b plus the
controller's commanded acceleration u, held constant over each propagation step:

    d/dt [p, v, b] = A9 [p, v, b] + [0, u, 0] + noise

Discretised exactly with the matrix exponential; process noise by Van Loan.
Noise model:
  - white acceleration noise on v, PSD  sigma_w^2 + (rel * |u|)^2 per axis (covers
    thrust gain and misalignment errors while burning, and anything unmodelled);
  - random walk on b, PSD sigma_brw^2 (lets the bias estimate keep adapting).
estimate_bias=False gives the plain 6-state filter (b pinned at 0) for comparison.

Measurement: z = [p, v] + white noise (nav_pos_sigma, nav_vel_sigma).
"""

from __future__ import annotations

import numpy as np
from scipy.linalg import expm

from approach_env import HOLD


def cw_A9(n: float) -> np.ndarray:
    A = np.zeros((9, 9))
    A[0:3, 3:6] = np.eye(3)
    A[3, 0], A[5, 2] = 3 * n * n, -n * n
    A[3, 4], A[4, 3] = 2 * n, -2 * n
    A[3:6, 6:9] = np.eye(3)
    return A


class _Disc:
    """Cached discretisation for one dt: Phi (9,9) and Q basis matrices."""

    def __init__(self, n: float, dt: float):
        A = cw_A9(n)
        self.Phi = expm(A * dt)
        self.G = self.Phi[:, 6:9].copy()  # response of x to a constant accel held over dt
        self.G[6:9] = 0.0  # a commanded accel moves p, v but is not part of the bias state
        # Van Loan: for Qc = e_i e_i^T (unit PSD on one state), Q = Phi * int(...)
        self.Qbasis = []
        for i in range(9):
            Qc = np.zeros((9, 9))
            Qc[i, i] = 1.0
            M = np.block([[-A, Qc], [np.zeros((9, 9)), A.T]]) * dt
            E = expm(M)
            Q = E[9:, 9:].T @ E[:9, 9:]
            self.Qbasis.append(0.5 * (Q + Q.T))
        self.Qbasis = np.stack(self.Qbasis)  # (9, 9, 9): index = noisy state


class BiasKalman:
    def __init__(self, num_envs: int, mean_motion: float, nav_pos_sigma: float,
                 nav_vel_sigma: float, sigma_w: float = 3e-6, sigma_brw: float = 1e-8,
                 rel: float = 0.12, b0_sigma: float = 2e-5, estimate_bias: bool = True):
        self.n, self.N = mean_motion, num_envs
        self.sigma_w, self.sigma_brw, self.rel = sigma_w, sigma_brw, rel
        self.b0_sigma, self.estimate_bias = b0_sigma, estimate_bias
        sp, sv = max(nav_pos_sigma, 1e-4), max(nav_vel_sigma, 1e-5)
        self.R = np.diag([sp**2] * 3 + [sv**2] * 3)
        self.H = np.zeros((6, 9))
        self.H[:, :6] = np.eye(6)
        self._disc: dict[float, _Disc] = {}
        self.x = np.zeros((num_envs, 9))
        self.P = np.zeros((num_envs, 9, 9))
        self.reset(np.arange(num_envs), np.zeros((num_envs, 6)))

    def disc(self, dt: float) -> _Disc:
        key = round(dt, 9)
        if key not in self._disc:
            self._disc[key] = _Disc(self.n, dt)
        return self._disc[key]

    def reset(self, idx, z) -> None:
        self.x[idx] = 0.0
        self.x[idx, :6] = z[idx]
        self.P[idx] = 0.0
        self.P[idx, :6, :6] = self.R
        if self.estimate_bias:
            for i in range(6, 9):
                self.P[idx, i, i] = self.b0_sigma**2

    @property
    def bias(self) -> np.ndarray:
        return self.x[:, 6:9]

    def predict(self, dt: float, u: np.ndarray | None = None) -> None:
        """Propagate dt seconds with commanded LVLH acceleration u (N,3) held constant."""
        d = self.disc(dt)
        self.x = self.x @ d.Phi.T
        q = np.full((self.N, 3), self.sigma_w**2)
        if u is not None:
            self.x += u @ d.G.T
            q = q + (self.rel * np.abs(u)) ** 2
        Q = np.einsum("ni,ijk->njk", q, d.Qbasis[3:6])
        if self.estimate_bias:
            Q = Q + self.sigma_brw**2 * d.Qbasis[6:9].sum(axis=0)
        self.P = d.Phi @ self.P @ d.Phi.T + Q
        if not self.estimate_bias:
            self.x[:, 6:9] = 0.0
            self.P[:, 6:9, :] = 0.0
            self.P[:, :, 6:9] = 0.0

    def update(self, z: np.ndarray) -> np.ndarray:
        """Measurement update with z = [p, v] (N,6). Returns the normalised innovation
        squared (NIS, expected value 6 for a consistent filter)."""
        H, R = self.H, self.R
        y = z - self.x[:, :6]
        S = self.P[:, :6, :6] + R
        Sinv = np.linalg.inv(S)
        K = self.P[:, :, :6] @ Sinv
        self.x = self.x + np.einsum("nij,nj->ni", K, y)
        I_KH = np.eye(9) - K @ H
        # Joseph form for numerical robustness
        self.P = I_KH @ self.P @ np.transpose(I_KH, (0, 2, 1)) + K @ R @ np.transpose(K, (0, 2, 1))
        return np.einsum("ni,nij,nj->n", y, Sinv, y)

    def nees(self, x_true: np.ndarray, sl=slice(0, 9)) -> np.ndarray:
        """Normalised estimation error squared over the state slice (expected = its size)."""
        e = (self.x - x_true)[:, sl]
        P = self.P[:, sl, sl]
        return np.einsum("ni,nij,nj->n", e, np.linalg.inv(P), e)


class BiasKalmanTorch:
    """Same filter as BiasKalman, batched in torch float64 (GPU). Same interface; x, P
    and bias are returned as numpy copies. run_log() applies a whole epoch of 1 Hz
    (u, z) pairs with one host-device transfer."""

    def __init__(self, num_envs: int, mean_motion: float, nav_pos_sigma: float,
                 nav_vel_sigma: float, sigma_w: float = 3e-6, sigma_brw: float = 1e-8,
                 rel: float = 0.12, b0_sigma: float = 2e-5, estimate_bias: bool = True,
                 device: str = "cuda"):
        import torch
        self.torch, self.dev = torch, torch.device(device)
        self.n, self.N = mean_motion, num_envs
        self.sigma_w, self.sigma_brw, self.rel = sigma_w, sigma_brw, rel
        self.b0_sigma, self.estimate_bias = b0_sigma, estimate_bias
        sp, sv = max(nav_pos_sigma, 1e-4), max(nav_vel_sigma, 1e-5)
        self._R_np = np.diag([sp**2] * 3 + [sv**2] * 3)
        self.R = self._t(self._R_np)
        self.I9 = self._t(np.eye(9))
        self._disc: dict = {}
        self.xt = torch.zeros(num_envs, 9, dtype=torch.float64, device=self.dev)
        self.Pt = torch.zeros(num_envs, 9, 9, dtype=torch.float64, device=self.dev)
        self.reset(np.arange(num_envs), np.zeros((num_envs, 6)))

    def _t(self, a):
        return self.torch.as_tensor(np.asarray(a, float), dtype=self.torch.float64,
                                    device=self.dev)

    def disc(self, dt: float):
        key = round(dt, 9)
        if key not in self._disc:
            d = _Disc(self.n, dt)
            Qb = self.sigma_brw**2 * d.Qbasis[6:9].sum(axis=0) if self.estimate_bias                 else np.zeros((9, 9))
            self._disc[key] = (self._t(d.Phi), self._t(d.G), self._t(d.Qbasis[3:6]), self._t(Qb))
        return self._disc[key]

    @property
    def x(self) -> np.ndarray:
        return self.xt.cpu().numpy()

    @property
    def P(self) -> np.ndarray:
        return self.Pt.cpu().numpy()

    @property
    def bias(self) -> np.ndarray:
        return self.x[:, 6:9]

    def reset(self, idx, z) -> None:
        torch = self.torch
        idx_t = torch.as_tensor(np.asarray(idx), device=self.dev, dtype=torch.long)
        P0 = np.zeros((9, 9))
        P0[:6, :6] = self._R_np
        if self.estimate_bias:
            P0[6:, 6:] = np.eye(3) * self.b0_sigma**2
        x0 = np.zeros((len(idx_t), 9))
        x0[:, :6] = np.asarray(z)[np.asarray(idx)]
        self.xt[idx_t] = self._t(x0)
        self.Pt[idx_t] = self._t(P0)

    def _predict_t(self, dt, u):
        Phi, G, Qw, Qb = self.disc(dt)
        self.xt = self.xt @ Phi.T
        q = self.torch.full((self.N, 3), self.sigma_w**2, dtype=self.torch.float64,
                            device=self.dev)
        if u is not None:
            self.xt = self.xt + u @ G.T
            q = q + (self.rel * u.abs()) ** 2
        Q = self.torch.einsum("ni,ijk->njk", q, Qw) + Qb
        self.Pt = Phi @ self.Pt @ Phi.T + Q
        if not self.estimate_bias:
            self.xt[:, 6:9] = 0.0
            self.Pt[:, 6:9, :] = 0.0
            self.Pt[:, :, 6:9] = 0.0

    def _update_t(self, z):
        y = z - self.xt[:, :6]
        S = self.Pt[:, :6, :6] + self.R
        L = self.torch.linalg.cholesky(S)
        Kt = self.torch.cholesky_solve(self.Pt[:, :6, :], L)  # S^-1 P[:6, :] = K^T
        K = Kt.transpose(1, 2)
        self.xt = self.xt + (K @ y[..., None])[..., 0]
        I_KH = self.I9.expand(self.N, 9, 9).clone()
        I_KH[:, :, :6] -= K
        self.Pt = I_KH @ self.Pt @ I_KH.transpose(1, 2) + K @ self.R @ Kt
        w = self.torch.cholesky_solve(y[..., None], L)[..., 0]
        return (y * w).sum(-1)

    def predict(self, dt: float, u: np.ndarray | None = None) -> None:
        self._predict_t(dt, None if u is None else self._t(u))

    def update(self, z: np.ndarray) -> np.ndarray:
        return self._update_t(self._t(z)).cpu().numpy()

    def run_log(self, dt: float, us: np.ndarray, zs: np.ndarray) -> None:
        """us (T,N,3), zs (T,N,6): predict dt then update, T times."""
        ut, zt = self._t(us), self._t(zs)
        for k in range(ut.shape[0]):
            self._predict_t(dt, ut[k])
            self._update_t(zt[k])


# ------------------------------------------------------------- bias-aware guidance
def cw_forced_mats(n: float, t: np.ndarray):
    """Batched Φrr, Φrv and Γ (response of p to a constant accel) at times t (k,).
    Computed once per distinct t (guidance times repeat across worlds)."""
    A = cw_A9(n)
    tu, inv = np.unique(np.asarray(t, float), return_inverse=True)
    E = expm(A[None] * tu[:, None, None])[inv]
    return E[:, 0:3, 0:3], E[:, 0:3, 3:6], E[:, 0:3, 6:9]


def v_required_bias(n: float, p: np.ndarray, t_go: np.ndarray, b: np.ndarray | None):
    """Velocity that coasts to HOLD in t_go under CW plus constant accel b."""
    rr, rv, gam = cw_forced_mats(n, t_go)
    rhs = HOLD[None] - np.einsum("kij,kj->ki", rr, p)
    if b is not None:
        rhs = rhs - np.einsum("kij,kj->ki", gam, b)
    return np.linalg.solve(rv, rhs[..., None])[..., 0]


# ------------------------------------------------------------- controller wrapper
class _NavView:
    """Env proxy: measured_state() returns the filter estimate; est_bias exposes b."""

    def __init__(self, env, kf: BiasKalman, use_bias: bool):
        self._env, self._kf, self._use_bias = env, kf, use_bias

    def __getattr__(self, name):
        return getattr(self._env, name)

    @property
    def est_bias(self):
        return self._kf.bias if self._use_bias else None

    def measured_state(self):
        C, om, _, _ = self._env.lvlh_state()
        return C, om, self._kf.x[:, :3], self._kf.x[:, 3:6]


class NavController:
    """Wrap a classical controller with BiasKalman.

    mode 'force': ApproachEnv, 1 s steps, one measurement per step (env.meas_p/meas_v).
    mode 'dv':    ManeuverEnv. If env.nav_log is a list the env records a measurement and
                  the commanded acceleration every second, and the filter runs at 1 Hz
                  (nav_1hz=True). Otherwise it predicts the epoch's burn profile second
                  by second and updates once per epoch with env.meas_p/meas_v.
    make_ctrl(view) -> callable returning world force ('force') or world dv ('dv').
    use_bias: give the controller the bias estimate (bias-compensated guidance).
    """

    def __init__(self, env, make_ctrl, mode="force", nav_1hz=True, use_bias=True,
                 record=False, **kf_kw):
        cfg = env.cfg
        self.env, self.mode, self.nav_1hz = env, mode, nav_1hz and mode == "dv"
        self.kf = BiasKalman(env.num_envs, env.mean_motion, cfg.nav_pos_sigma,
                             cfg.nav_vel_sigma, **kf_kw)
        self.ctrl = make_ctrl(_NavView(env, self.kf, use_bias))
        self.last = None
        self.record = record
        self.hist: list = []  # (t, x_est, P_diag, x_true, nis) per update when record
        if self.nav_1hz:
            env.nav_log = []

    def _truth(self):
        _, _, p, v = self.env.lvlh_state()
        return np.concatenate([p, v, self.env._adist], axis=1)

    def _upd(self, z, t):
        nis = self.kf.update(z)
        if self.record:
            xt = self._truth() if t is None else t
            self.hist.append((self.kf.x.copy(), self.kf.P.copy(), xt, nis))

    def __call__(self):
        env = self.env
        cfg = env.cfg
        step = env.episode_length_buf.cpu().numpy()
        z_env = np.concatenate([env.meas_p, env.meas_v], axis=1)
        if self.last is None or (step == 0).all():
            self.kf.reset(np.arange(env.num_envs), z_env)
            if self.nav_1hz:
                env.nav_log.clear()
        elif self.mode == "force":
            self.kf.predict(cfg.control_dt, u=self.last)
            self._upd(z_env, None)
        elif self.nav_1hz:
            for u, z, xt in env.nav_log:
                self.kf.predict(cfg.physics_dt, u=u)
                self._upd(z, xt)
            env.nav_log.clear()
        else:
            # replay the burn profile the env flew (per-world-axis clip at full thrust)
            C, _, _, _ = env.lvlh_state()
            rem = self.last.copy()
            a_step = cfg.f_max / cfg.mass * cfg.physics_dt
            for _ in range(env._decimation):
                d = np.clip(rem, -a_step, a_step)
                rem -= d
                self.kf.predict(cfg.physics_dt, u=np.einsum("nji,nj->ni", C, d) / cfg.physics_dt)
            self._upd(z_env, None)
        out = self.ctrl()
        if self.mode == "force":
            C, _, _, _ = env.lvlh_state()
            self.last = np.einsum("nji,nj->ni", C, out) / cfg.mass
        else:
            self.last = np.asarray(out, float).copy()  # world dv request
        return out
