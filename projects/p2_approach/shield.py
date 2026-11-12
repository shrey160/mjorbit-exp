"""Project 2B step B5: run-time assurance shield for 20 s epoch controllers (SPEC_2 section 17).

Explicit simplex. Before each epoch's Δv is flown, the shield predicts, from the
navigation filter's estimate (x = p, v, bias; covariance P), what happens if the
command is executed:

    burn the candidate Δv (same finite-burn profile as ManeuverEnv), coast to the
    next epoch, then hand over to the backup controller "stop and station-keep"
    (null the estimated relative velocity at that epoch and at every epoch after it)
    for `horizon` s (CW + estimated bias between stops).

This is a recoverability test: "if I do this now, can the backup still keep us out of
the keep-out zone from the next decision on?" backup_hold=False instead lets the
vehicle drift with thrusters off after the single stop (passive safety, the original
B5 wording). That version is infeasible here: under the u1 disturbance (up to 2e-5
m/s^2) a 300 s drift moves the vehicle up to ~0.9 m, while the hold point is only 1 m
outside the keep-out sphere (SPEC_2 17).

The test passes when, at every second of the prediction,
    |p_hat(t)| - n_sigma * sigma_r(t) >= keep_out,
where sigma_r is the 1-sigma position uncertainty along the line of sight, from the
filter covariance propagated with the 9-state CW model, plus a burn execution error of
exec_rel x |Δv| (1 sigma, per axis) added to the velocity covariance at each burn.

Candidates, in order of preference (the first that passes is flown):
    0 nominal (the controller's command)
    1 CW targeting to the hold point, bias-compensated (t_go = t_arrive - t, >= 20 s)
    2 stop now (Δv = -v_hat)
    3 retreat: stop and move away along the line of sight at retreat_speed
If none passes, the candidate with the largest predicted margin is flown.
Anything other than candidate 0 counts as an intervention.
"""

from __future__ import annotations

import numpy as np

from nav_filter import BiasKalman, _Disc, v_required_bias

CANDIDATES = ("nominal", "cw_hold", "stop", "retreat")


class Shield:
    def __init__(self, env, nav, horizon: float = 300.0, n_sigma: float = 3.0,
                 exec_rel: float = 0.07, t_arrive: float = 480.0, retreat_speed: float = 0.02,
                 keep_out: float | None = None, backup_hold: bool = True):
        """nav(): object with x (N,9) and P (N,9,9) for the current time (the filter
        the controller already runs, or a NavTracker for controllers without one)."""
        cfg = env.cfg
        self.env, self.nav = env, nav
        self.horizon, self.n_sigma, self.exec_rel = horizon, n_sigma, exec_rel
        self.t_arrive, self.retreat_speed = t_arrive, retreat_speed
        self.backup_hold = backup_hold
        self.keep_out = cfg.keep_out if keep_out is None else keep_out
        d = _Disc(env.mean_motion, 1.0)  # shield model runs at 1 s regardless of physics_dt
        self.Phi, self.G = d.Phi, d.G
        self.a1 = cfg.f_max / cfg.mass * 1.0  # max Δv per world axis per second
        self.n_epoch = int(round(cfg.control_dt))
        n = env.num_envs
        self.count = np.zeros(n, np.int64)  # interventions this episode
        self.choice_hist = np.zeros((n, len(CANDIDATES)), np.int64)
        self.min_margin = np.full(n, np.inf)  # predicted margin of the flown command
        self.unsafe_flown = np.zeros(n, np.int64)  # epochs where no candidate passed

    # ------------------------------------------------------------------ helpers
    def _to_world(self, C, dv_l):
        dvm = self.env.cfg.dv_max
        return np.clip(np.einsum("mij,mj->mi", C, dv_l), -dvm, dvm)

    def candidates(self, dv_nom_w, x, C, t):
        n_ = self.env.mean_motion
        p, v, b = x[:, :3], x[:, 3:6], x[:, 6:9]
        t_go = np.maximum(self.t_arrive - t, self.env.cfg.control_dt)
        cw = v_required_bias(n_, p, t_go, b) - v
        u = p / np.maximum(np.linalg.norm(p, axis=1, keepdims=True), 1e-9)
        return np.stack([np.asarray(dv_nom_w, float), self._to_world(C, cw),
                         self._to_world(C, -v), self._to_world(C, -v + self.retreat_speed * u)])

    def _exec_cov(self, P, dv_w):
        s2 = (self.exec_rel * np.linalg.norm(dv_w, axis=1)) ** 2
        for i in range(3, 6):
            P[:, i, i] += s2
        return P

    def margins(self, cands_w, x, P, C):
        """Predicted min over time of |p| - n_sigma sigma_r - keep_out, per candidate (K,N)."""
        K, N = cands_w.shape[:2]
        xs = np.tile(x, (K, 1))
        Ps = self._exec_cov(np.tile(P, (K, 1, 1)), cands_w.reshape(K * N, 3))
        Cs = np.tile(C, (K, 1, 1))
        rem = cands_w.reshape(K * N, 3).copy()
        Phi, G, PhiT, GT = self.Phi, self.G, self.Phi.T, self.G.T
        P0v = np.tile(P[:, 3:9, 3:9], (K, 1, 1))  # filter's v, b block now
        m = np.full(K * N, np.inf)
        steps = self.n_epoch + int(round(self.horizon))
        for k in range(steps):
            if k == self.n_epoch or (self.backup_hold and k > self.n_epoch
                                     and k % self.n_epoch == 0):
                # backup at the next decision: stop (and, with backup_hold, stop again
                # every epoch = station keeping in place). A stop on the estimated
                # velocity leaves the velocity error at the filter's accuracy.
                rem = self._to_world(Cs, -xs[:, 3:6])
                if k > self.n_epoch:
                    Ps[:, 3:9, 3:9] = P0v
                    Ps[:, 0:3, 3:6] = 0.0
                    Ps[:, 3:6, 0:3] = 0.0
                Ps = self._exec_cov(Ps, rem)
            d = np.clip(rem, -self.a1, self.a1)
            rem -= d
            u_l = np.einsum("mji,mj->mi", Cs, d)  # LVLH accel over this 1 s
            xs = xs @ PhiT + u_l @ GT
            Ps = Phi @ Ps @ PhiT
            p = xs[:, :3]
            r = np.linalg.norm(p, axis=1)
            e = p / np.maximum(r, 1e-9)[:, None]
            sr = np.sqrt(np.maximum(np.einsum("mi,mij,mj->m", e, Ps[:, :3, :3], e), 0.0))
            m = np.minimum(m, r - self.n_sigma * sr)
        return (m - self.keep_out).reshape(K, N)

    # ------------------------------------------------------------------ filter
    def __call__(self, dv_w: np.ndarray) -> np.ndarray:
        env = self.env
        step = env.episode_length_buf.cpu().numpy()
        start = step == 0
        if start.any():
            self.count[start] = 0
            self.choice_hist[start] = 0
            self.min_margin[start] = np.inf
            self.unsafe_flown[start] = 0
        nav = self.nav()
        x, P = np.asarray(nav.x, float), np.asarray(nav.P, float)
        C, _, _, _ = env.lvlh_state()
        t = step * env.cfg.control_dt
        cands = self.candidates(dv_w, x, C, t)
        m = self.margins(cands, x, P, C)  # (K, N)
        ok = m >= 0.0
        first = np.where(ok.any(axis=0), ok.argmax(axis=0), m.argmax(axis=0))
        idx = np.arange(env.num_envs)
        out = cands[first, idx]
        self.count += first != 0
        self.choice_hist[idx, first] += 1
        self.unsafe_flown += ~ok.any(axis=0)
        self.min_margin = np.minimum(self.min_margin, m[first, idx])
        self.last_margins, self.last_choice = m, first
        return out


class NavTracker:
    """The 9-state filter at 1 Hz for controllers that do not run one (raw-measurement
    PPO, raw CW): call once per decision before the controller, shield reads .kf."""

    def __init__(self, env):
        cfg = env.cfg
        self.env = env
        self.kf = BiasKalman(env.num_envs, env.mean_motion, cfg.nav_pos_sigma, cfg.nav_vel_sigma)
        env.nav_log = []

    def __call__(self):
        env = self.env
        step = env.episode_length_buf.cpu().numpy()
        z = np.concatenate([env.meas_p, env.meas_v], axis=1)
        if (step == 0).all():
            self.kf.reset(np.arange(env.num_envs), z)
        else:
            for u, zk, _ in env.nav_log:
                self.kf.predict(env.cfg.physics_dt, u=u)
                self.kf.update(zk)
        env.nav_log.clear()
