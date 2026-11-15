"""P5.1: relative-motion filter for the servicer body (SPEC_5 section 5, piece 2).

Input: client pose measurements (position relative to the servicer and attitude, world axes)
at a low rate and with latency, plus the servicer's own commanded thrust. Output at the control
rate: relative position and velocity, client attitude and client angular velocity, propagated
to the current time.

Translation (3 independent axes), state [rho, rho'] with rho = p_client - p_servicer:
    rho'' = -F/m + w,   w white, PSD sig_a^2          (differential gravity ~1e-6 m/s^2: in w)
    z = rho + noise
Attitude and rate, multiplicative filter on the attitude kinematics only (no client inertia;
AAS 18-420), world-frame error [dtheta, dw] with R_true = exp(dtheta) R_est:
    R' = [w]x R,   w' = white, PSD sig_w^2  (random walk: a nutating client's rate drifts)
    dtheta' = w x dtheta + dw
    z = rotvec(R_meas R_est^T) = dtheta + noise

Latency: the filter state lives at the time of the last processed measurement; inputs since
then are buffered and the output is the state propagated through them (mean only).
"""
from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation


def _exp(v):
    return Rotation.from_rotvec(v).as_matrix()


def _skew(v):
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


class RelFilter:
    def __init__(self, dt, sig_a=2e-4, sig_w=2e-5, v0_sig=2e-3, w0_sig=np.radians(3.0), bias_frac=0.0, bias_tau=60.0):
        """bias_frac > 0 (P5.4b): that fraction of the position measurement variance is a slow
        error (first-order Gauss-Markov, correlation time bias_tau) carried as three more states,
        z = rho + b + white. Without it the filter reads the drift of the link error as velocity."""
        self.dt, self.sig_a, self.sig_w = dt, sig_a, sig_w
        self.bias_frac, self.bias_tau = bias_frac, bias_tau
        self.nx = 9 if bias_frac > 0 else 6
        self.v0_sig, self.w0_sig = v0_sig, w0_sig
        self.ready = False

    def init(self, rho, R, sp, sa, v0=None, w0=None):
        """First measurement: position and attitude from it; velocity and rate priors."""
        self.x = np.concatenate([rho, np.zeros(3) if v0 is None else v0])  # [rho, rho']
        self.P = np.diag([sp**2] * 3 + [self.v0_sig**2] * 3)
        if self.nx == 9:  # [rho, rho', b]: the first measurement is rho + b + white
            sb2 = self.bias_frac * sp**2
            self.x = np.concatenate([self.x, np.zeros(3)])
            P = np.zeros((9, 9))
            P[0:6, 0:6] = self.P
            P[6:9, 6:9] = sb2 * np.eye(3)
            P[0:3, 6:9] = P[6:9, 0:3] = -sb2 * np.eye(3)
            self.P = P
            self.sb = np.sqrt(sb2)
        self.R = R.copy()
        self.w = np.zeros(3) if w0 is None else w0.copy()
        self.Pa = np.diag([sa**2] * 3 + [self.w0_sig**2] * 3)
        self.acc = []  # servicer thrust accelerations applied since the filter time (one per dt)
        self.nis_p = self.nis_a = 0.0
        self.ready = True

    # ---- one control step of the servicer has passed with relative acceleration a (= -F/m)
    def push(self, a):
        self.acc.append(np.asarray(a, float).copy())

    def _advance(self, n):
        """Move the filter state forward over the first n buffered steps."""
        dt = self.dt
        nx = self.nx
        A = np.eye(nx)
        A[0:3, 3:6] = dt * np.eye(3)
        q = self.sig_a**2
        Q = np.zeros((nx, nx))
        if nx == 9:
            ab = np.exp(-dt / self.bias_tau)
            A[6:9, 6:9] = ab * np.eye(3)
            Q[6:9, 6:9] = self.sb**2 * (1 - ab**2) * np.eye(3)
        Q[0:3, 0:3] = q * dt**3 / 3 * np.eye(3)
        Q[0:3, 3:6] = Q[3:6, 0:3] = q * dt**2 / 2 * np.eye(3)
        Q[3:6, 3:6] = q * dt * np.eye(3)
        Qa = np.zeros((6, 6))
        qw = self.sig_w**2
        Qa[0:3, 0:3] = qw * dt**3 / 3 * np.eye(3)
        Qa[0:3, 3:6] = Qa[3:6, 0:3] = qw * dt**2 / 2 * np.eye(3)
        Qa[3:6, 3:6] = qw * dt * np.eye(3)
        for a in self.acc[:n]:
            self.x[0:3] += dt * self.x[3:6] + 0.5 * dt**2 * a
            self.x[3:6] += dt * a
            if nx == 9:
                self.x[6:9] *= ab
            self.P = A @ self.P @ A.T + Q
            Fa = np.eye(6)
            Fa[0:3, 0:3] += dt * _skew(self.w)
            Fa[0:3, 3:6] = dt * np.eye(3)
            self.R = _exp(self.w * dt) @ self.R
            self.Pa = Fa @ self.Pa @ Fa.T + Qa
        del self.acc[:n]

    def update(self, rho_m, R_m, sp, sa, age_steps, biased=True):
        """A measurement taken age_steps control steps ago (1-sigma sp [m], sa [rad]).
        biased=False: a position measurement without the slow error (the servicer's cameras)."""
        n = max(len(self.acc) - age_steps, 0)
        self._advance(n)
        nx = self.nx
        Hp = np.zeros((3, nx))
        Hp[:, 0:3] = np.eye(3)
        r2 = sp**2
        if nx == 9 and biased:
            Hp[:, 6:9] = np.eye(3)
            r2 = (1 - self.bias_frac) * sp**2
        y = rho_m - Hp @ self.x
        S = Hp @ self.P @ Hp.T + r2 * np.eye(3)
        K = self.P @ Hp.T @ np.linalg.inv(S)
        self.nis_p = float(y @ np.linalg.solve(S, y))
        self.x = self.x + K @ y
        self.P = (np.eye(nx) - K @ Hp) @ self.P
        H = np.zeros((3, 6))
        H[:, 0:3] = np.eye(3)
        ya = Rotation.from_matrix(R_m @ self.R.T).as_rotvec()
        Sa = H @ self.Pa @ H.T + sa**2 * np.eye(3)
        Ka = self.Pa @ H.T @ np.linalg.inv(Sa)
        self.nis_a = float(ya @ np.linalg.solve(Sa, ya))
        d = Ka @ ya
        self.R = _exp(d[0:3]) @ self.R
        self.w = self.w + d[3:6]
        self.Pa = (np.eye(6) - Ka @ H) @ self.Pa

    def now(self):
        """(rho, rho', R, w) propagated from the filter time to the current time."""
        n = len(self.acc)
        dt = self.dt
        rho, v = self.x[0:3].copy(), self.x[3:6].copy()
        if n:
            a = np.asarray(self.acc)
            k = np.arange(n)
            rho = rho + v * (n * dt) + dt**2 * ((n - k - 0.5)[:, None] * a).sum(axis=0)
            v = v + dt * a.sum(axis=0)
        return rho, v, _exp(self.w * n * dt) @ self.R, self.w.copy()

    def sigma(self):
        """1-sigma of position, velocity, attitude, rate (largest axis)."""
        d, da = np.sqrt(np.diag(self.P)), np.sqrt(np.diag(self.Pa))
        return float(d[0:3].max()), float(d[3:6].max()), float(da[0:3].max()), float(da[3:6].max())
