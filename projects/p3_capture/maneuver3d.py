"""Project 3D: reorient and reposition the captured stack (SPEC_3 section 8).

After 3C (capture, brakes, tumble, detumble, identification), each case flies:
  1. translate  the stack centre of mass by a commanded offset dr (5-15 m, random direction)
                in LVLH and hold it there. Guidance: CW targeting every 20 s epoch (as in
                P2) to arrive at T_tr; burns from the bus-centre thrusters (0.6 N max).
                strategy "align": first slew so the bus-to-CoM line points along dr, so the
                main burns pass through the CoM (no torque); corrections across that line
                get the counter-torque -d_hat x F from the wheels.
                strategy "fixed": no slew; every burn gets the counter-torque.
  2. reorient   to a commanded LVLH attitude (random, 30-120 deg away) while keeping
                station, then hold for hold_s.
Attitude control: rate-limited eigenaxis quaternion feedback with the inertia estimate,
  w_des = sat(kp/kd * e, w_max), tau = I_est kd (w_des - w_rel) + w x I_est w + ff,
  scaled to the 1 N m wheel limit. Wheel momentum is tallied (ideal wheels).
Estimates: "id" (3C identification), "prior" (nominal 300 kg uniform client, CoM at its
  centre: what a servicer would assume without identification) or "truth".
Navigation: star tracker (0.003 deg noise), gyro as 3C, CoM position from the bus position
  (close-range relative nav, 2 cm / 0.2 mm/s noise) and the CoM offset estimate.
"""

from __future__ import annotations

import dataclasses
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "p2_approach"))

import numpy as np  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402

from grasp_sim import _rotvec  # noqa: E402
from stack_sim import BOX, StackCfg, StackRun, stack_truth  # noqa: E402


@dataclasses.dataclass
class Man3DCfg:
    strategy: str = "align"  # or "fixed"
    estimates: str = "id"  # "id", "prior" or "truth"
    dr_range: tuple = (5.0, 15.0)
    att_range_deg: tuple = (30.0, 120.0)
    T_tr: float = 400.0  # s, CW arrival time for the translation
    epoch_s: float = 20.0
    f_max: float = 0.6  # N, total thrust magnitude
    deadband: float = 0.001  # m/s
    w_n: float = 0.03  # rad/s attitude bandwidth
    zeta: float = 0.9
    w_max_deg: float = 0.5  # deg/s slew-rate limit
    slew_tol_deg: float = 0.5
    slew_max_s: float = 500.0
    hold_s: float = 60.0
    nav_pos: float = 0.02
    nav_vel: float = 2e-4
    st_noise_deg: float = 0.003
    h_budget: float = 30.0  # N m s, reported as a budget check
    # wheels and momentum unloading (SPEC_3 section 9)
    wheel_h_max: float = 30.0  # N m s per body axis, wheels saturate here
    unload: bool = True  # RCS couple unloading of the wheels (default since SPEC_3 section 9)
    unload_k: float = 0.05  # 1/s
    unload_deadband: float = 0.5  # N m s per axis
    rcs_tau_max: float = 0.5  # N m per axis (0.5 N thruster pair on a 1 m lever)
    rcs_lever: float = 1.0  # m, for propellant accounting
    isp: float = 220.0  # s, hydrazine monoprop
    carry_h: bool = True  # start 3D with the wheel momentum left from 3C


def _rot_between(a, b):
    """Smallest rotation (matrix) taking unit vector a to unit vector b."""
    v = np.cross(a, b)
    s, c = np.linalg.norm(v), float(np.dot(a, b))
    if s < 1e-9:
        if c > 0:
            return np.eye(3)
        p = np.cross(a, [1.0, 0, 0]) if abs(a[0]) < 0.9 else np.cross(a, [0, 1.0, 0])
        return Rotation.from_rotvec(np.pi * p / np.linalg.norm(p)).as_matrix()
    return Rotation.from_rotvec(v / s * np.arctan2(s, c)).as_matrix()


class Stack3D(StackRun):
    def __init__(self, seed, cfg: StackCfg, mcfg: Man3DCfg, grasp_kw=None):
        for k in ("wheel_h_max", "unload", "unload_k", "unload_deadband", "rcs_tau_max",
                  "rcs_lever", "isp"):
            setattr(cfg, k, getattr(mcfg, k))  # one wheel model for 3C and 3D
        super().__init__(seed, cfg, grasp_kw)
        self.m = mcfg
        self.rng3 = np.random.default_rng(seed + 777)  # 3D commands, same across variants

    # ------------------------------------------------------------------ estimates
    def prior_estimates(self):
        """Stack properties assuming a nominal client: 300 kg uniform box, CoM at centre."""
        sim = self.sim
        km, kd = sim.km, sim.kd
        parts = []
        for i in range(1, km.nbody):
            R = kd.ximat[i].reshape(3, 3)
            I_i = R @ np.diag(km.body_inertia[i]) @ R.T
            x_i = kd.xipos[i].copy()
            m_i = km.body_mass[i]
            if i == sim.bid_target:
                m_i = 300.0
                x_i = kd.xpos[i].copy()  # body origin = geometric centre
                Rb = kd.xmat[i].reshape(3, 3)
                I_i = Rb @ np.diag(300.0 / 12 * np.array([BOX[1]**2 + BOX[2]**2, BOX[0]**2 + BOX[2]**2,
                                                          BOX[0]**2 + BOX[1]**2])) @ Rb.T
            parts.append((m_i, x_i, I_i))
        M = sum(p[0] for p in parts)
        cm = sum(p[0] * p[1] for p in parts) / M
        I = sum(p[2] + p[0] * ((p[1] - cm) @ (p[1] - cm) * np.eye(3) - np.outer(p[1] - cm, p[1] - cm))
                for p in parts)
        Rs = kd.xmat[sim.bid_chaser].reshape(3, 3)
        return dict(m_hat=float(M), I_hat=Rs.T @ I @ Rs, d_hat=Rs.T @ (kd.xpos[sim.bid_chaser] - cm))

    # ------------------------------------------------------------------ sensing
    def C_lvlh(self):
        d = self.sim.data
        o = d.world_position_from_lvlh(np.zeros(3))
        return np.stack([d.world_position_from_lvlh(e) - o for e in np.eye(3)], axis=1)

    def R_meas(self):
        R = self.sim.kd.xmat[self.sim.bid_chaser].reshape(3, 3)
        dn = Rotation.from_rotvec(np.radians(self.m.st_noise_deg) * self.rng3.normal(size=3)).as_matrix()
        return dn @ R

    def cm_nav(self, d_hat):
        """Estimated stack CoM position and velocity in LVLH."""
        sim = self.sim
        kd, d = sim.kd, sim.data
        R = kd.xmat[sim.bid_chaser].reshape(3, 3)
        pb, vb = kd.xpos[sim.bid_chaser].copy(), d.qvel[0:3].copy()
        w_world = R @ d.qvel[3:6]
        cm = pb - R @ d_hat
        vcm = vb - np.cross(w_world, R @ d_hat)
        p = d.lvlh_position_from_world(cm) + self.rng3.normal(0, self.m.nav_pos, 3)
        v = d.lvlh_velocity_from_world(cm, vcm) + self.rng3.normal(0, self.m.nav_vel, 3)
        return p, v

    def cm_true_lvlh(self):
        sim = self.sim
        sim.sync()
        M, _, _, cm = stack_truth(sim)
        m = sim.km.body_mass
        # CoM velocity from momentum of the two free bodies (arm momentum small, braked)
        ms = M - m[sim.bid_target]
        vcm = (ms * sim.data.qvel[0:3] + m[sim.bid_target] * sim.data.qvel[14:17]) / M
        d = sim.data
        return d.lvlh_position_from_world(cm), d.lvlh_velocity_from_world(cm, vcm)

    # ------------------------------------------------------------------ control
    def att_torque(self, R_cmd_world, est, ff=np.zeros(3)):
        m = self.m
        R = self.R_meas()
        e = _rotvec(R.T @ R_cmd_world)  # body-frame rotation from current to commanded
        w_ref = R.T @ np.array([0.0, 0.0, self.sim.n])  # commanded attitude fixed in LVLH
        w_rel = self._gyro_last - w_ref
        kp, kd = m.w_n**2, 2 * m.zeta * m.w_n
        w_des = (kp / kd) * e
        wmax = np.radians(m.w_max_deg)
        nw = np.linalg.norm(w_des)
        if nw > wmax:
            w_des *= wmax / nw
        I = est["I_hat"]
        g = self._gyro_last
        # gyroscopic compensation of the total momentum, stack plus wheels (H = I w + h)
        h_b = R.T @ self.h_w if hasattr(self, "h_w") else np.zeros(3)
        tau = I @ (kd * (w_des - w_rel)) + np.cross(g, I @ g + h_b) + ff
        nt = np.max(np.abs(tau))
        if nt > self.cfg.rw_limit:
            tau *= self.cfg.rw_limit / nt
        return tau, float(np.degrees(np.linalg.norm(e)))

    def burn_force(self, dv_world, R, est, axis_body=None):
        """Body-frame thrust for this control step from the remaining epoch dv (world)."""
        F_w = dv_world * est["m_hat"] / self.sim.cfg.ctrl_dt
        n = np.linalg.norm(F_w)
        if n > self.m.f_max:
            F_w *= self.m.f_max / n
        return R.T @ F_w

    def fly(self, seconds, R_cmd_lvlh, est, target_lvlh=None, t_arrive=None, metrics=None):
        """Attitude to R_cmd_lvlh (LVLH frame) and, if target given, CW translation epochs."""
        sim, m = self.sim, self.m
        dt = sim.cfg.ctrl_dt
        n_ep = int(round(m.epoch_s / dt))
        k = 0
        rem = np.zeros(3)
        t0 = self.t
        settled_at = None
        while self.t - t0 < seconds:
            C = self.C_lvlh()
            R_cmd = C @ R_cmd_lvlh
            R = self.R_meas()
            if target_lvlh is not None and k % n_ep == 0:
                p, v = self.cm_nav(est["d_hat"])
                tgo = max((t_arrive - self.t) if t_arrive is not None else 0.0, 3 * m.epoch_s)
                from guidance import coast_velocity
                dv = coast_velocity(sim.n, p[None], target_lvlh[None], np.array([tgo]))[0] - v
                rem = C @ dv if np.max(np.abs(dv)) > m.deadband else np.zeros(3)
            F_b = np.zeros(3)
            if np.linalg.norm(rem) > 1e-7:
                F_b = self.burn_force(rem, R, est)
                rem = rem - (R @ F_b) * dt / est["m_hat"]
            ff = -np.cross(est["d_hat"], F_b)
            tau, err = self.att_torque(R_cmd, est, ff)
            g, _, _, _ = self._step(F_body=F_b, tau_body=tau)
            self._gyro_last = g
            self.dv_used += np.linalg.norm(F_b) * dt / est["m_hat"]
            if metrics is not None:
                metrics["att_err_max"] = max(metrics.get("att_err_max", 0.0), err)
            if settled_at is None and err < m.slew_tol_deg:
                settled_at = self.t - t0
            k += 1
        return settled_at

    def slew(self, R_cmd_lvlh, est, target_lvlh=None):
        """Fly until within slew_tol (or slew_max_s), then 20 s more to settle."""
        sim, m = self.sim, self.m
        t0 = self.t
        while self.t - t0 < m.slew_max_s:
            self.fly(5.0, R_cmd_lvlh, est, target_lvlh=target_lvlh, t_arrive=None)
            R = sim.kd.xmat[sim.bid_chaser].reshape(3, 3)
            err = np.degrees(np.linalg.norm(_rotvec(R.T @ (self.C_lvlh() @ R_cmd_lvlh))))
            if err < m.slew_tol_deg:
                break
        self.fly(20.0, R_cmd_lvlh, est, target_lvlh=target_lvlh, t_arrive=None)
        return self.t - t0

    # ------------------------------------------------------------------ episode
    def run3d(self):
        res, est_id = self.capture_identify()
        if est_id is None:
            return res
        m = self.m
        if m.estimates == "id":
            est = est_id
        elif m.estimates == "prior":
            est = self.prior_estimates()
        else:
            self.sim.sync()
            M, I_t, d_t, _ = stack_truth(self.sim)
            est = dict(m_hat=float(M), I_hat=I_t, d_hat=d_t)
        # commands (same across variants for a given seed)
        u = self.rng3.normal(size=3)
        u /= np.linalg.norm(u)
        dr = u * self.rng3.uniform(*m.dr_range)
        ax = self.rng3.normal(size=3)
        ax /= np.linalg.norm(ax)
        ang = np.radians(self.rng3.uniform(*m.att_range_deg))
        # wheels carry the momentum left from 3C (same wheel model throughout since SPEC_3 10)
        if not m.carry_h:
            self.h_w = np.zeros(3)
        self.h_rw, self.h_rw_max, self.dv_used = np.zeros(3), 0.0, 0.0
        self.hw_max, self.sat_steps, self.rcs_impulse = 0.0, 0, 0.0
        out_h0 = float(np.max(np.abs(self.sim.kd.xmat[self.sim.bid_chaser].reshape(3, 3).T @ self.h_w)))
        C = self.C_lvlh()
        R0_lvlh = C.T @ self.sim.kd.xmat[self.sim.bid_chaser].reshape(3, 3)
        R_final_lvlh = Rotation.from_rotvec(ang * ax).as_matrix() @ R0_lvlh
        p0, _ = self.cm_true_lvlh()
        target = p0 + dr
        out = dict(dr_m=float(np.linalg.norm(dr)), att_cmd_deg=float(np.degrees(ang)),
                   h_start_3d=out_h0)
        t_start = self.t
        # 1. translation
        R_tr_lvlh = R0_lvlh
        if m.strategy == "align":
            a_lvlh = R0_lvlh @ (est["d_hat"] / np.linalg.norm(est["d_hat"]))
            # bring +a or -a onto dr, whichever is the smaller rotation
            b = u if np.dot(a_lvlh, u) >= 0 else -u
            R_tr_lvlh = _rot_between(a_lvlh, b) @ R0_lvlh
            out["align_slew_s"] = self.slew(R_tr_lvlh, est)
        mt = {}
        t_arr = self.t + m.T_tr
        self.fly(m.T_tr + 60.0, R_tr_lvlh, est, target_lvlh=target, t_arrive=t_arr, metrics=mt)
        p, v = self.cm_true_lvlh()
        out["tr_pos_err_m"] = float(np.linalg.norm(p - target))
        out["tr_vel_mm_s"] = float(np.linalg.norm(v) * 1000)
        out["tr_att_err_max_deg"] = mt.get("att_err_max", 0.0)
        out["h_after_tr"] = float(np.max(np.abs(self.h_w)))
        # 2. reorientation with station keeping
        out["final_slew_s"] = self.slew(R_final_lvlh, est, target_lvlh=target)
        self.fly(m.hold_s, R_final_lvlh, est, target_lvlh=target, t_arrive=None)
        # position and velocity are judged at the end of the station-keeping hold; attitude
        # and rate after a further 30 s attitude-only coast (no burns), because a rate taken
        # mid-burn says nothing about the settled attitude, while a coast lets an off-chief
        # position drift under CW dynamics (~2 mm/s in 30 s at 15 m radial) (SPEC_3 8)
        p, v = self.cm_true_lvlh()
        self.fly(30.0, R_final_lvlh, est)
        Rb = self.sim.kd.xmat[self.sim.bid_chaser].reshape(3, 3)
        att_err = float(np.degrees(np.linalg.norm(_rotvec(Rb.T @ (self.C_lvlh() @ R_final_lvlh)))))
        rate = float(np.degrees(np.linalg.norm(self.sim.data.qvel[3:6] - Rb.T @ np.array([0, 0, self.sim.n]))))
        out.update(final_pos_err_m=float(np.linalg.norm(p - target)), final_vel_mm_s=float(np.linalg.norm(v) * 1000),
                   final_att_err_deg=att_err, final_rate_deg_s=rate, h_rw_max_3d=self.hw_max,
                   h_end_3d=float(np.max(np.abs(self.h_w))), sat_s=self.sat_steps * self.sim.cfg.ctrl_dt,
                   rcs_impulse_Nms=self.rcs_impulse,
                   rcs_prop_g=1000 * self.rcs_impulse / m.rcs_lever / (m.isp * 9.80665),
                   dv_used=self.dv_used, t_3d_s=self.t - t_start, pads_lost_3d=self.pads_lost)
        out["success"] = bool(out["final_pos_err_m"] < 0.2 and out["final_vel_mm_s"] < 2.0
                              and att_err < 0.5 and rate < 0.01 and not self.pads_lost)
        out["within_h_budget"] = bool(self.sat_steps == 0)
        out["sat_s_3c"] = res.get("sat_s_3c")
        res.update(out)
        res["variant"] = f"{m.estimates}/{m.strategy}"
        return res


if __name__ == "__main__":
    seed = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    strat = sys.argv[2] if len(sys.argv) > 2 else "align"
    estm = sys.argv[3] if len(sys.argv) > 3 else "id"
    unl = not (len(sys.argv) > 4 and sys.argv[4] == "none")
    r = Stack3D(seed, StackCfg(), Man3DCfg(strategy=strat, estimates=estm, unload=unl)).run3d()
    for k in ("variant", "dr_m", "att_cmd_deg", "align_slew_s", "tr_pos_err_m", "tr_vel_mm_s",
              "tr_att_err_max_deg", "h_after_tr", "final_slew_s", "final_pos_err_m", "final_vel_mm_s",
              "final_att_err_deg", "final_rate_deg_s", "h_rw_max_3d", "dv_used", "t_3d_s",
              "pads_lost_3d", "success", "within_h_budget", "h_start_3d", "h_end_3d", "sat_s",
              "rcs_impulse_Nms", "rcs_prop_g"):
        print(f"{k:22s} {r.get(k)}")
