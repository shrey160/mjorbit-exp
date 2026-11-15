"""P5.4b: fly-in timing from a speed limit instead of lengthened minimum-jerk legs.

The geometry of the frozen fly-in (flyin.FlyIn: straight legs in the client body frame) is kept.
The timing is replaced: at every point of the path the speed is the largest one (up to v_max)
for which a servicer on the path, coasting from there for the monitor's horizon, keeps `margin`
of predicted clearance to the client turning at the estimated rate. An acceleration limit is
then applied forwards and backwards, and each leg starts and ends at rest (as before).

So the servicer flies at full speed while it is far away and slows only where the monitor needs
it, instead of stretching the whole leg.
"""
from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from flyin import FlyIn
from filt_eval import FiltSim
from safety import box_clearance


class FlyInLim(FlyIn):
    """A FlyIn whose legs follow a tabulated speed profile. Built by `limit(fp, ...)`."""

    def body_ref(self, t):
        t0 = 0.0
        for (a, b, T), (tt, ss, vv, aa) in zip(zip(self.pts[:-1], self.pts[1:], self.T), self.tab):
            if t <= t0 + T:
                e = (b - a) / max(np.linalg.norm(b - a), 1e-9)
                x = t - t0
                return a + np.interp(x, tt, ss) * e, np.interp(x, tt, vv) * e, np.interp(x, tt, aa) * e
            t0 += T
        return self.pts[-1].copy(), np.zeros(3), np.zeros(3)


def _coast_ok(r_b, e, v, w_b, Rts, taus, boxes, need):
    vel = np.cross(w_b, r_b) + v * e
    for Rt, tau in zip(Rts, taus):
        if box_clearance(Rt @ (r_b + vel * tau), boxes) < need:
            return False
    return True


def limit(fp, w_b, margin, boxes, r_serv, clear_margin, v_max=0.05, v_min=0.002, a_max=1.5e-3, horizon=30.0,
          ds=0.02, dt=0.5):
    """Turn fp (a FlyIn) into a FlyInLim in place. Returns per-leg (old s, new s, lowest speed limit)."""
    taus = np.arange(0.0, horizon + 1e-9, 2.0)
    Rts = [Rotation.from_rotvec(w_b * tau).as_matrix().T for tau in taus]
    need = margin + r_serv + clear_margin
    speeds = v_max * np.geomspace(1.0, v_min / v_max, 14)
    tab, info, T_new = [], [], []
    for a, b, T_old in zip(fp.pts[:-1], fp.pts[1:], fp.T):
        L = float(np.linalg.norm(b - a))
        if L < 1e-6:
            tab.append((np.array([0.0, T_old]), np.zeros(2), np.zeros(2), np.zeros(2)))
            T_new.append(T_old)
            info.append((round(T_old), round(T_old), 0.0))
            continue
        e = (b - a) / L
        n = max(int(np.ceil(L / ds)), 4)
        s = np.linspace(0.0, L, n + 1)
        h = s[1] - s[0]
        vs = np.empty(n + 1)
        for i, si in enumerate(s):
            r = a + si * e
            vs[i] = next((v for v in speeds if _coast_ok(r, e, v, w_b, Rts, taus, boxes, need)), v_min)
        v = vs.copy()
        v[0] = v[-1] = 0.0
        for i in range(1, n + 1):  # speed up no faster than a_max
            v[i] = min(v[i], np.sqrt(v[i - 1] ** 2 + 2 * a_max * h))
        for i in range(n - 1, -1, -1):  # slow down in time for what is ahead
            v[i] = min(v[i], np.sqrt(v[i + 1] ** 2 + 2 * a_max * h))
        seg = 2 * h / np.maximum(v[:-1] + v[1:], 1e-9)
        tk = np.concatenate([[0.0], np.cumsum(seg)])
        tt = np.arange(0.0, tk[-1] + dt, dt)
        ss = np.interp(tt, tk, s)
        # smooth over 4 s (takes the corners off the acceleration), then differentiate
        k = int(round(4.0 / dt))
        pad = np.concatenate([np.full(k, ss[0]), ss, np.full(k, ss[-1])])
        ss = np.convolve(pad, np.ones(k + 1) / (k + 1), mode="same")[k // 2: k // 2 + len(tt) + k]
        tt = np.arange(len(ss)) * dt
        vv = np.gradient(ss, dt)
        aa = np.gradient(vv, dt)
        tab.append((tt, ss, vv, aa))
        T_new.append(float(tt[-1]))
        info.append((round(T_old), round(float(tt[-1])), round(float(vs.min()), 4)))
    fp.__class__ = FlyInLim
    fp.tab, fp.T = tab, T_new
    fp.total, fp.leg1 = float(sum(T_new)), T_new[0]
    return info


class LimMixin:
    """Replaces FiltSim's leg lengthening by the speed limit."""
    margin_cap = 0.24
    a_path = 1.5e-3  # m/s^2; the frozen min-jerk legs peak at 0.7e-3 (long leg) to 1.9e-3 (short legs)

    def _slow_path(self, fp, w_b, margin):
        return limit(fp, w_b, margin, self.boxes, self.r_serv, self.cfg.clear_margin, v_max=self.cfg.flyin_v,
                     a_max=self.a_path, horizon=self.horizon)


class LimSim(LimMixin, FiltSim):
    pass
