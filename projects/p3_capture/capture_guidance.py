"""Classical baseline for Project 3A (SPEC_3 section 3): waypoint routing around the
client plus a glideslope down the capture corridor. One decision per 20 s epoch.

  corridor: the cone of half-angle `cone_deg` about the capture-face normal, from the
            feature out to `l_corridor` m. Inside it: glideslope (Project 2, section 19)
            straight to the pre-grasp point, closing speed k * range in [v_min, v_max].
  outside:  head for the corridor entry A = feature + l_corridor * n_face. If the
            straight segment to A passes within `seg_margin` m of the keep-out volume,
            go instead to an intermediate waypoint on a sphere of radius `r_safe`
            around the client, rotated from the current bearing towards n_face by at
            most `step_deg`. Transfers use the bias-compensated CW coast velocity with
            t_go = clip(distance / v_cruise, t_min, t_max), re-planned every epoch.

All positions come from env.measured_state() (the filter estimate when wrapped by
NavController); client pose comes from env.client_geometry() (assumed known, e.g. from
a pose estimator; SPEC_3 section 2).
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "p2_approach"))

from guidance import coast_velocity  # noqa: E402


def _unit(x):
    return x / np.maximum(np.linalg.norm(x, axis=1, keepdims=True), 1e-9)


def rotate_towards(u, t, step):
    """Rotate unit vectors u towards unit vectors t by at most `step` rad."""
    cosang = np.clip(np.sum(u * t, axis=1), -1.0, 1.0)
    ang = np.arccos(cosang)
    w = t - cosang[:, None] * u
    small = np.linalg.norm(w, axis=1) < 1e-6  # u ~ -t: pick a direction in the orbit plane
    if small.any():
        alt = np.cross(u[small], np.array([0.0, 0.0, 1.0]))
        bad = np.linalg.norm(alt, axis=1) < 1e-6
        alt[bad] = np.cross(u[small][bad], np.array([1.0, 0.0, 0.0]))
        w[small] = alt
    w = _unit(w)
    a = np.minimum(ang, step)[:, None]
    return np.cos(a) * u + np.sin(a) * w


class CaptureGuidance:
    def __init__(self, env, k=0.03, v_max=0.05, v_min=0.002, cone_deg=15.0, l_corridor=10.0,
                 r_safe=8.0, step_deg=60.0, seg_margin=1.5, v_cruise=0.1, t_min=40.0,
                 t_max=400.0, deadband=0.002):
        self.env = env
        self.k, self.v_max, self.v_min = k, v_max, v_min
        self.tan_cone, self.l_corridor = np.tan(np.radians(cone_deg)), l_corridor
        self.r_safe, self.step = r_safe, np.radians(step_deg)
        self.seg_margin, self.v_cruise = seg_margin, v_cruise
        self.t_min, self.t_max, self.deadband = t_min, t_max, deadband
        self.phase = np.zeros(env.num_envs, np.int64)  # 0 route, 1 waypoint, 2 corridor

    def _segment_clear(self, geo, p, a, ns=16):
        env = self.env
        m = np.full(p.shape[0], np.inf)
        for s in np.linspace(0.0, 1.0, ns):
            m = np.minimum(m, env.clearance_from(geo, p + s * (a - p)))
        return m >= self.seg_margin

    def __call__(self) -> np.ndarray:
        env = self.env
        cfg = env.cfg
        n_ = env.mean_motion
        dt = cfg.control_dt
        C, _, p, v = env.measured_state()
        b = getattr(env, "est_bias", None)
        geo = env.client_geometry()
        f, nf, g, c = geo["f"], geo["nf"], geo["g"], geo["c"]

        d = p - f
        along = np.sum(d * nf, axis=1)
        lat = np.linalg.norm(d - along[:, None] * nf, axis=1)
        corridor = (along > 0) & (lat <= self.tan_cone * along + 0.3) & \
            (along <= self.l_corridor + 1.0)

        # corridor: glideslope to the pre-grasp point (moving with the client)
        e = g - p
        rho = np.linalg.norm(e, axis=1)
        s = np.clip(rho * (1.0 - np.exp(-self.k * dt)), self.v_min * dt, self.v_max * dt)
        s = np.minimum(s, rho)
        wp_c = p + _unit(e) * s[:, None] + geo["gd"] * dt
        # outside: corridor entry, or a waypoint around the client
        A = f + self.l_corridor * nf
        clear = self._segment_clear(geo, p, A)
        u = _unit(p - c)
        wp_r = c + self.r_safe * rotate_towards(u, nf, self.step)
        wp_o = np.where(clear[:, None], A, wp_r)
        dist = np.linalg.norm(wp_o - p, axis=1)
        tgo_o = np.clip(dist / self.v_cruise, self.t_min, self.t_max)

        wp = np.where(corridor[:, None], wp_c, wp_o)
        tgo = np.where(corridor, dt, tgo_o)
        self.phase = np.where(corridor, 2, np.where(clear, 0, 1))
        dv = coast_velocity(n_, p, wp, tgo, b) - v
        dv = np.where(np.abs(dv).max(axis=1, keepdims=True) > self.deadband, dv, 0.0)
        return np.clip(np.einsum("nij,nj->ni", C, dv), -cfg.dv_max, cfg.dv_max)
