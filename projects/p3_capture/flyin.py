"""3E step E1b: fly-in from a hold point on a spinning client's spin axis to the
synchronised pre-grasp point (SPEC_3 section 11).

A point on the spin axis does not move as the client rotates, so the 3A approach can
deliver the servicer there. From the hold, the path is defined in the client's
rotating body frame (piecewise straight legs, minimum-jerk timing):
  1. along the spin axis to a safe height h_safe (clear of the client),
  2. radially out at that height to the pre-grasp point's radius (an inertial spiral),
  3. along the axis to the pre-grasp point.
If the pre-grasp point is on the spin axis (spin about the face normal) only the axial
leg is flown. The reference handed to the station-keeping controller includes the full
rotating-frame kinematics (r = R_c r_b):
  v = v_c + w x r + R_c r_b',   a = w x (w x r) + 2 w x (R_c r_b') + R_c r_b''
(client angular acceleration neglected: torque-free spin about a near-principal axis).
The attitude reference blends from the servicer's initial attitude, relative to the
client, to the synchronised (aligned) attitude over the first leg; rate reference = w_c.

Thrust feasibility (E3 validation, SPEC_3 section 13): near 2 deg/s, and for a client
spinning about an axis it nutates around, the rotating-frame feed-forward
m |w x (w x r) + w' x r + 2 w x R_c r_b' + R_c r_b''| can exceed the 0.75 N per-axis thrust
limit, because the measured rate vector drifts away from the planning axis. reference()
therefore takes a path-rate factor k (path clock advancing k x real time): the path terms
scale by k and k^2; the caller (GraspSim) picks k online so the demand fits.
"""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation


def _mj(s):
    """Minimum-jerk position, velocity, acceleration factors for s in [0, 1]."""
    s = min(max(s, 0.0), 1.0)
    return (10 * s**3 - 15 * s**4 + 6 * s**5, 30 * s**2 - 60 * s**3 + 30 * s**4,
            60 * s - 180 * s**2 + 120 * s**3)


class FlyIn:
    def __init__(self, p_b0, g_b, s_b, h_safe=2.4, v_max=0.05, t_min=20.0):
        """p_b0: start (client body frame), g_b: pre-grasp point, s_b: unit spin axis
        (body frame), on the same side as the start."""
        side = 1.0 if p_b0 @ s_b >= 0 else -1.0
        h_g = float(g_b @ s_b)
        rg = g_b - h_g * s_b
        rho = float(np.linalg.norm(rg))
        if rho < 0.3:  # pre-grasp point on the spin axis
            pts = [p_b0, g_b]
        elif side * h_g >= h_safe:  # already clear at the pre-grasp height
            pts = [p_b0, h_g * s_b, g_b]
        else:
            h1 = side * h_safe
            pts = [p_b0, h1 * s_b, h1 * s_b + rg, g_b]
        self.pts = [np.asarray(p, float) for p in pts]
        # min-jerk peak speed is 1.875 x mean, so T = 1.875 L / v_max
        self.T = [max(1.875 * np.linalg.norm(b - a) / v_max, t_min)
                  for a, b in zip(self.pts[:-1], self.pts[1:])]
        self.total = float(sum(self.T))
        self.leg1 = self.T[0]

    def body_ref(self, t):
        """(r_b, r_b', r_b'') at time t since the fly-in started."""
        t0 = 0.0
        for a, b, T in zip(self.pts[:-1], self.pts[1:], self.T):
            if t <= t0 + T:
                p, v, acc = _mj((t - t0) / T)
                d = b - a
                return a + p * d, v / T * d, acc / T**2 * d
            t0 += T
        return self.pts[-1].copy(), np.zeros(3), np.zeros(3)

    def reference(self, t, pc, vc, Rc, w_c, R_rel0, k=1.0):
        """World-frame station-keeping reference (p, v, a, R_ref, w_ref) at path time t,
        the path clock advancing at k x real time."""
        r_b, rd_b, rdd_b = self.body_ref(t)
        rd_b, rdd_b = k * rd_b, k * k * rdd_b
        r = Rc @ r_b
        rd = Rc @ rd_b
        p = pc + r
        v = vc + np.cross(w_c, r) + rd
        a = np.cross(w_c, np.cross(w_c, r)) + 2 * np.cross(w_c, rd) + Rc @ rdd_b
        h, _, _ = _mj(t / self.leg1)
        R_rel = Rotation.from_rotvec((1 - h) * Rotation.from_matrix(R_rel0).as_rotvec()).as_matrix()
        return p, v, a, Rc @ R_rel, w_c
