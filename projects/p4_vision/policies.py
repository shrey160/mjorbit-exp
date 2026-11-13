"""P4.4 (SPEC_4 4.4): grasp policies for GraspSim.policy.

A policy is called every control step in reach and approach as policy(sim, t, phase, t_phase)
and returns (p_des, R_des, close) in the world frame (see grasp_sim.py).

  ExpertPolicy  the classical expert through the policy path: expert_suggest() plus the expert's
                close test on the true pose. Checks the hook: it should grasp like the expert.
  DartPolicy    ExpertPolicy with DART noise on the executed target (Laskey et al. 2017): an
                Ornstein-Uhlenbeck perturbation of the position and rotation, ramped to zero
                near the bar, so the demonstrations visit off-nominal states; the recorder
                labels every frame with the clean expert_suggest() target.
"""
from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation


def close_test(sim):
    """The expert's close condition on the true pose: grip point within 1 cm of the bar along
    the approach axis and within 1 cm across it (as grasp_sim.run)."""
    p_bar, _ = sim.site_pose(sim.sid_bar)
    p_g, _ = sim.site_pose(sim.sid_grip)
    Rc = sim.kd.xmat[sim.bid_target].reshape(3, 3)
    rel = p_g - p_bar
    along = rel @ Rc[:, 0]
    perp = np.linalg.norm(rel - along * Rc[:, 0] - (rel @ Rc[:, 1]) * Rc[:, 1])
    return abs(along) < 0.01 and perp < 0.01


class ExpertPolicy:
    def reset(self, rng=None):
        self.last = None  # last clean target (for the recorder)

    def __init__(self):
        self.reset()

    def __call__(self, sim, t, phase, t_phase):
        p, R = sim.expert_suggest(t, phase, t_phase)
        self.last = (p, R)
        return p, R, phase == "approach" and close_test(sim)


class DartPolicy(ExpertPolicy):
    """sigma_p [m] and sigma_r [rad]: stationary std of the noise per axis; tau [s]: its
    correlation time; ramp_m: noise scales linearly to zero as the grip point comes within
    this distance of the bar (no noise at contact)."""

    # 13 Nov smoke (8 seeds): sigma_p 4 mm per axis peaked at 17 mm and caused 2 clear_pred
    # safe aborts; now 2 mm per axis, the norm capped at 2.5 sigma (5 mm) and 2.5 sigma_r.
    def __init__(self, sigma_p=0.002, sigma_r=np.radians(0.3), tau=0.5, ramp_m=0.03, dt=0.01, cap=2.5):
        self.sigma_p, self.sigma_r, self.tau, self.ramp_m, self.dt = sigma_p, sigma_r, tau, ramp_m, dt
        self.cap = cap
        self.reset()

    def reset(self, rng=None):
        super().reset()
        self.rng = np.random.default_rng(rng)
        self.np_, self.nr = np.zeros(3), np.zeros(3)
        self.noise_max = 0.0

    def __call__(self, sim, t, phase, t_phase):
        p, R, close = super().__call__(sim, t, phase, t_phase)
        a = self.dt / self.tau
        k = np.sqrt(2.0 * a)
        self.np_ += -a * self.np_ + k * self.sigma_p * self.rng.standard_normal(3)
        self.nr += -a * self.nr + k * self.sigma_r * self.rng.standard_normal(3)
        p_bar, _ = sim.site_pose(sim.sid_bar)
        p_g, _ = sim.site_pose(sim.sid_grip)
        g = float(np.clip((np.linalg.norm(p_g - p_bar) - self.ramp_m) / self.ramp_m, 0.0, 1.0))
        cap = lambda x, s: x * min(1.0, self.cap * s / max(np.linalg.norm(x), 1e-12))  # noqa: E731
        dp, dr = g * cap(self.np_, self.sigma_p), g * cap(self.nr, self.sigma_r)
        self.noise_max = max(self.noise_max, float(np.linalg.norm(dp)))
        return p + dp, Rotation.from_rotvec(dr).as_matrix() @ R, close


class BiasPolicy(ExpertPolicy):
    """P4.8 data collection: the expert approach with a per-episode offset of the target, so the
    palm camera sees the bar off-centre between the jaws (the expert on the true pose always
    arrives centred). Offset in the target frame (x approach, y bar axis, z closing direction),
    ramped in over the approach blend; the x and z parts and the rotation shrink near the bar
    (the open jaws clear the bar by only about 8 mm per side). Closes on time: one second after
    the nominal slide ends. `last` stays the clean expert target."""

    def __init__(self, max_m=(0.010, 0.030, 0.020), rot_deg=2.0, p_zero=0.15, close_extra_s=1.0):
        self.max_m, self.rot_deg, self.p_zero, self.close_extra_s = np.asarray(max_m), rot_deg, p_zero, close_extra_s
        self.reset()

    def reset(self, rng=None):
        super().reset()
        r = np.random.default_rng(rng)
        zero = r.uniform() < self.p_zero
        self.b = np.zeros(3) if zero else r.uniform(-1, 1, 3) * self.max_m
        self.b[0] = min(self.b[0], 0.005)  # not deeper than 5 mm past the bar centre
        self.rv = np.zeros(3) if zero else np.radians(r.uniform(-self.rot_deg, self.rot_deg, 3))

    def __call__(self, sim, t, phase, t_phase):
        p, R = sim.expert_suggest(t, phase, t_phase)
        self.last = (p, R)
        if phase != "approach":
            return p, R, False
        cfg, tb = sim.cfg, t - t_phase
        sb = min(tb / cfg.blend_s, 1.0)
        h = 10 * sb**3 - 15 * sb**4 + 6 * sb**5
        p_bar, _ = sim.site_pose(sim.sid_bar)
        p_g, _ = sim.site_pose(sim.sid_grip)
        d = float((p_bar - p_g) @ R[:, 0])  # distance to go along the approach axis
        near = float(np.clip((d - 0.02) / 0.06, 0.3, 1.0))
        off = h * self.b * np.array([near, 1.0, near])
        close = tb >= cfg.blend_s + cfg.pre_contact / cfg.v_approach + self.close_extra_s
        return p + R @ off, R @ Rotation.from_rotvec(h * near * self.rv).as_matrix(), close
