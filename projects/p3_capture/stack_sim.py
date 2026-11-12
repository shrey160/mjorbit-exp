"""Project 3C: stabilise and identify the captured stack (SPEC_3 section 7).

Each case: a client with randomised (unknown to the controller) mass, centre of mass and
inertia is grasped as in 3B; the stack is then given a random tumble, and the servicer
  1. detumbles   reaction-wheel rate damping to the LVLH rate (arm joints locked)
  2. rot. ID     known wheel-torque pulses; momentum-form Euler equation in the bus frame,
                     I (w(t2) - w(t1)) + int w x (I w) dt = int tau dt,
                 batch least squares for the 6 inertia parameters (integral form: no
                 differentiation of the noisy gyro)
  3. trans. ID   known body-fixed thrust pulses while the wheels hold attitude; with the
                 measured accelerometer delta-v (bus centre, bus frame)
                     dv = (1/m) int F dt + dw x d + int w x (w x d) dt,
                     d x int F dt = I dw + int w x (I w) dt - int tau dt,
                 least squares for 1/m and d (bus centre minus stack CoM)
  4. validate    40 s body-fixed thrust along bus y, wheels holding attitude with a soft PD,
                 first without and then with the feed-forward counter-torque -d_hat x F;
                 peak attitude error and drift
Sensors: gyro (white noise sigma_gyro per 100 Hz sample) and accelerometer delta-v
(sigma_dv per sample) at the bus centre. Truth (composite mass, CoM, inertia about the
CoM in the bus frame) comes from the MuJoCo model for scoring only.
Reaction wheels are ideal torques on the bus (the model has no wheel bodies); the
momentum they would store is integrated and reported.
"""

from __future__ import annotations

import dataclasses
import os
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import mjorbit  # noqa: F401,E402

# isort: split
import mujoco  # noqa: E402
import numpy as np  # noqa: E402
from mjorbit import OrbitInit, mjo_forward, mjo_step  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402

from grasp_sim import ARM, ARM_Q, GRIP_Q, U_ARM, U_RW, U_THR, GraspCfg, GraspSim, _rotvec  # noqa: E402

HERE = Path(__file__).parent
BOX = np.array([1.5, 1.0, 1.0])


@dataclasses.dataclass
class StackCfg:
    tumble_deg_s: tuple = (0.5, 2.0)  # stack tumble magnitude range, random axis
    detumble_kd: float = 300.0  # N m per rad/s
    detumble_tol_deg_s: float = 0.005
    detumble_max_s: float = 150.0
    id_torque: float = 0.5  # N m wheel pulses
    id_pulse_s: float = 12.0
    id_thrust: float = 0.4  # N body-fixed thrust pulses (the bus is ~1.8 m from the stack
    # CoM, so 0.4 N off-axis makes ~0.7 N m, inside the 1 N m wheels)
    hold_kp: float = 60.0  # wheel attitude hold during thrust (N m per rad)
    hold_kd: float = 400.0
    val_s: float = 40.0
    window_s: float = 4.0  # integration window for the least squares (1 s: 3-50x worse)
    sigma_gyro: float = 1e-5  # rad/s per sample
    sigma_dv: float = 1e-6  # m/s per sample
    arm_lock_wn: float = 10.0  # rad/s, PD joint-lock bandwidth when brakes=False
    brakes: bool = True  # lock the arm with joint-limit constraints after capture
    # client randomisation (unknown to the controller)
    mass_range: tuple = (200.0, 400.0)
    com_off: float = 0.10  # m per axis, client body frame
    inertia_scale: tuple = (0.8, 1.2)
    rw_limit: float = 1.0
    scene: str = "p3b_scene.xml"
    # wheel model (SPEC_3 sections 9-10): saturation, gyroscopic reaction, RCS unloading
    wheel_model: bool = True
    wheel_h_max: float = 30.0  # N m s per body axis
    unload: bool = True
    unload_k: float = 0.05  # 1/s
    unload_deadband: float = 0.5  # N m s per axis
    rcs_tau_max: float = 0.5  # N m per axis
    rcs_lever: float = 1.0  # m
    isp: float = 220.0  # s
    # client spin at capture (SPEC_3 section 10): deg/s about a client body axis
    spin_deg_s: float = 0.0
    spin_axis: int = -1  # 0/1/2 body axis, -1 random
    impose_tumble: bool = True  # 3C's imposed tumble (off when the client already spins)
    # E2 post-capture grip monitor (SPEC_3 section 12)
    grip_loose_mm: float = 0.0  # fault: jaw brakes engage this much looser per jaw (preload loss)
    grip_mon: bool = False
    grip_mon_act: bool = True  # safe the stack when the grip slips (False: record only)
    slip_shift_mm: float = 3.0  # bar shift in the gripper frame since the brakes engaged
    slip_vel_mm_s: float = 2.0  # 0.2 s average bar speed in the gripper frame
    pad_loss_s: float = 0.2
    pad_chatter_n: int = 1  # separate pad-contact drops (any length) that trip the monitor; a
    # barely loose grip (~0.07 mm) chatters for single 10 ms steps that never reach pad_loss_s,
    # a firm grip shows none (E3 validation: 0 drops in 850 successes; SPEC_3 section 13)
    safe_coast_s: float = 60.0  # after safing: no actuation, watch whether the client stays
    grip_response: str = "safe"  # "safe": stop all actuation; "reclamp": also re-squeeze
    # with the jaw drive (brake released)


class GripAbort(Exception):
    pass


def client_case_xml(rng, cfg: StackCfg, scene="p3b_scene.xml"):
    """Scene text with the client's mass properties replaced by a random <inertial>."""
    m = rng.uniform(*cfg.mass_range)
    com = rng.uniform(-cfg.com_off, cfg.com_off, 3)
    I0 = m / 12.0 * np.array([BOX[1]**2 + BOX[2]**2, BOX[0]**2 + BOX[2]**2, BOX[0]**2 + BOX[1]**2])
    I = I0 * rng.uniform(*cfg.inertia_scale, 3)
    Rp = Rotation.from_rotvec(np.radians(rng.uniform(-10, 10, 3))).as_matrix()
    J = Rp @ np.diag(I) @ Rp.T  # principal axes tilted up to 10 deg
    full = f"{J[0,0]:.6f} {J[1,1]:.6f} {J[2,2]:.6f} {J[0,1]:.6f} {J[0,2]:.6f} {J[1,2]:.6f}"
    xml = (HERE / scene).read_text()
    inertial = (f'      <inertial pos="{com[0]:.5f} {com[1]:.5f} {com[2]:.5f}" mass="{m:.4f}" '
                f'fullinertia="{full}"/>\n')
    xml = re.sub(r'(<body name="target" pos="0 0 0">\n)', r"\1" + inertial, xml)
    return xml, dict(mass=m, com=com, inertia=J)


def stack_truth(sim: GraspSim):
    """Composite mass, CoM and inertia about the CoM, in the bus frame; d = bus - CoM."""
    km, kd = sim.km, sim.kd
    m = km.body_mass[1:]
    x = kd.xipos[1:]
    M = m.sum()
    cm = (m[:, None] * x).sum(0) / M
    I = np.zeros((3, 3))
    for i in range(1, km.nbody):
        R = kd.ximat[i].reshape(3, 3)
        Ii = R @ np.diag(km.body_inertia[i]) @ R.T
        r = kd.xipos[i] - cm
        I += Ii + km.body_mass[i] * (r @ r * np.eye(3) - np.outer(r, r))
    Rs = kd.xmat[sim.bid_chaser].reshape(3, 3)
    return M, Rs.T @ I @ Rs, Rs.T @ (kd.xpos[sim.bid_chaser] - cm), cm


def _sym(p):
    return np.array([[p[0], p[3], p[4]], [p[3], p[1], p[5]], [p[4], p[5], p[2]]])


def _lin_I(w):
    """Matrix L(w) with I w = L(w) p for p = (Ixx, Iyy, Izz, Ixy, Ixz, Iyz)."""
    return np.array([[w[0], 0, 0, w[1], w[2], 0], [0, w[1], 0, w[0], 0, w[2]],
                     [0, 0, w[2], 0, w[0], w[1]]])


def _skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


class StackRun:
    def __init__(self, seed, cfg: StackCfg, grasp_kw=None):
        self.cfg = cfg
        self.rng = np.random.default_rng(seed)
        xml, self.client = client_case_xml(self.rng, cfg, scene=cfg.scene)
        path = HERE / f".case_{os.getpid()}.xml"
        path.write_text(xml)
        gk = dict(scene=path.name, hold_s=2.0)
        gk.update(grasp_kw or {})
        self.sim = GraspSim(GraspCfg(**gk))
        self.log = {}
        self.h_w, self.hw_max, self.sat_steps, self.rcs_impulse = np.zeros(3), 0.0, 0, 0.0

    # ------------------------------------------------------------------ helpers
    def _step(self, F_body=None, tau_body=None, clip=True):
        """One 100 Hz control step: arm locked, grip squeezing, given bus force/torque.
        Returns (gyro w, accel delta-v over the step, both bus frame) and true w."""
        sim, cfg = self.sim, self.cfg
        d, kd = sim.data, sim.kd
        sim.sync()
        Rs = kd.xmat[sim.bid_chaser].reshape(3, 3)
        F_body = np.zeros(3) if F_body is None else F_body
        if tau_body is None:
            tau_body = np.zeros(3)
        if cfg.wheel_model:
            tau_body = self.actuate(tau_body)  # wheels + RCS + gyroscopic reaction
        elif clip:
            tau_body = np.clip(tau_body, -cfg.rw_limit, cfg.rw_limit)
        d.ctrl[U_THR] = Rs @ F_body
        d.ctrl[U_RW] = tau_body
        # joint lock ("brakes"): PD scaled by each joint's own inertia (diagonal of the mass
        # matrix) so it is stable at the 100 Hz control rate; fixed stiff gains made the
        # light wrist joints oscillate and shook the grip loose (SPEC_3 section 7)
        if getattr(self, "locked", False):
            tau_arm = np.zeros(6)  # brakes on: the joint-limit constraints hold the arm
        else:
            Mfull = np.zeros((sim.km.nv, sim.km.nv))
            mujoco.mj_fullM(sim.km, Mfull, kd.qM)
            mii = np.diag(Mfull)[ARM]
            wn = cfg.arm_lock_wn
            tau_arm = mii * (wn**2 * (self.q_lock - d.qpos[ARM_Q]) - 2.0 * 0.9 * wn * d.qvel[ARM])
        d.ctrl[U_ARM] = np.clip(tau_arm, sim.km.actuator_ctrlrange[6:12, 0],
                                sim.km.actuator_ctrlrange[6:12, 1])
        self.arm_tau_max = max(self.arm_tau_max, float(np.max(np.abs(tau_arm) / sim.km.actuator_ctrlrange[6:12, 1])))
        v0 = d.qvel[0:3].copy()
        nsub = int(round(sim.cfg.ctrl_dt / sim.cfg.dt))
        for _ in range(nsub):
            mjo_step(sim.model, d)
        self.h_rw += -(Rs @ tau_body) * sim.cfg.ctrl_dt  # momentum the wheels would hold
        self.h_rw_max = max(self.h_rw_max, float(np.linalg.norm(self.h_rw)))
        w_true = d.qvel[3:6].copy()
        dv_true = Rs.T @ (d.qvel[0:3] - v0)
        gyro = w_true + self.rng.normal(0, cfg.sigma_gyro, 3)
        dv = dv_true + self.rng.normal(0, cfg.sigma_dv, 3)
        pad_bar, _, _ = sim.contacts()
        # grace 0.05 s after the brakes engage: the model rebuild and state copy make the
        # pad contacts resettle for 0.01-0.03 s (seen at 2 deg/s spin; SPEC_3 section 10)
        lost = len(pad_bar) < 2 and self.t >= 0.05
        self.pads_lost |= lost
        self.pad_lost_steps = getattr(self, "pad_lost_steps", 0) + int(lost)
        if lost and getattr(self, "pad_lost_first_t", None) is None:
            self.pad_lost_first_t = (self.t, getattr(self, "phase_tag", ""))
        self.clamp_min = min(getattr(self, "clamp_min", np.inf), float(sum(pad_bar.values())))
        self.t += sim.cfg.ctrl_dt
        self._grip_watch(len(pad_bar))
        return gyro, dv, tau_body, F_body

    def _grip_watch(self, n_pads):
        """E2 grip monitor: bar motion in the gripper frame since the brakes engaged (shift,
        0.2 s average speed) and pad contact; raises GripAbort when it trips (if acting)."""
        sim, cfg = self.sim, self.cfg
        if self.t < 0.05:  # contacts resettle right after the brake rebuild
            return
        sim.sync()
        p_g, R_g = sim.site_pose(sim.sid_grip)
        p_b, _ = sim.site_pose(sim.sid_bar)
        rel = R_g.T @ (p_b - p_g)
        dt = sim.cfg.ctrl_dt
        if getattr(self, "_g_base", None) is None:
            self._g_base, self._g_prev, self._g_v, self._g_off = rel, rel, 0.0, 0.0
            self.gm = dict(shift_mm=0.0, vel_mm_s=0.0, pad_off_s=0.0, pad_drops=0)
        shift = float(np.linalg.norm(rel - self._g_base) * 1000)
        a = dt / 0.2
        self._g_v = (1 - a) * self._g_v + a * float(np.linalg.norm(rel - self._g_prev) * 1000 / dt)
        self._g_prev = rel
        if n_pads < 2 and self._g_off == 0.0:
            self._g_drops = getattr(self, "_g_drops", 0) + 1  # a new pad-contact drop starts
        self._g_off = self._g_off + dt if n_pads < 2 else 0.0
        self.gm["shift_mm"] = max(self.gm["shift_mm"], shift)
        self.gm["vel_mm_s"] = max(self.gm["vel_mm_s"], self._g_v)
        self.gm["pad_off_s"] = max(self.gm["pad_off_s"], self._g_off)
        self.gm["pad_drops"] = getattr(self, "_g_drops", 0)
        self._g_shift_now = shift
        if not cfg.grip_mon or getattr(self, "safed", False):
            return
        why = ("pad_loss" if self._g_off >= cfg.pad_loss_s else
               "pad_chatter" if cfg.pad_chatter_n and getattr(self, "_g_drops", 0) >= cfg.pad_chatter_n else
               "slip_shift" if shift > cfg.slip_shift_mm else
               "slip_vel" if self._g_v > cfg.slip_vel_mm_s else "")
        if why and getattr(self, "grip_detect", None) is None:
            self.grip_detect = (why, self.t, getattr(self, "phase_tag", ""))
        if why and cfg.grip_mon_act:
            raise GripAbort(why)

    def safe_coast(self):
        """Safe the stack after a grip alarm: no wheel torque, no RCS, no thrust (only the
        stored wheel momentum's gyroscopic reaction acts), so the grip carries no internal
        load; watch whether the client stays attached."""
        self.safed = True
        if self.cfg.grip_response == "reclamp":
            # release the slipped jaw brake; the jaw drive squeezes the bar again
            self.lock_arm(pin_jaws=False)
            self.sim.sync()
        sim = self.sim
        t0 = self.t
        while self.t - t0 < self.cfg.safe_coast_s:
            sim.sync()
            Rs = sim.kd.xmat[sim.bid_chaser].reshape(3, 3)
            h_b = Rs.T @ self.h_w
            w = sim.data.qvel[3:6]
            sim.data.ctrl[U_THR] = 0.0
            sim.data.ctrl[U_RW] = -np.cross(w, h_b)
            sim.data.ctrl[U_ARM] = 0.0
            for _ in range(int(round(sim.cfg.ctrl_dt / sim.cfg.dt))):
                mjo_step(sim.model, sim.data)
            self.t += sim.cfg.ctrl_dt
            sim.sync()
            pad_bar, _, _ = sim.contacts()
            self._grip_watch(len(pad_bar))

    def grip_outcome(self):
        """Client retained: both pads on the bar and the bar within 15 mm of where it was
        when the brakes engaged."""
        sim = self.sim
        sim.sync()
        pad_bar, _, _ = sim.contacts()
        return dict(retained=bool(len(pad_bar) == 2 and getattr(self, "_g_shift_now", 0.0) < 15.0),
                    pads_end=len(pad_bar), shift_end_mm=float(getattr(self, "_g_shift_now", np.nan)),
                    **{f"gm_{k}": v for k, v in getattr(self, "gm", {}).items()},
                    grip_detect=str(getattr(self, "grip_detect", None)),
                    safed=bool(getattr(self, "safed", False)))

    def actuate(self, tau_cmd):
        """Split the commanded body torque between reaction wheels and the RCS.

        Wheels: torque limit rw_limit, momentum per axis limited to wheel_h_max (a saturated
        wheel cannot add momentum in that direction); stored momentum h (world frame,
        changed only by the wheel torque) acts on the bus as the gyroscopic reaction
        -w x h_body. RCS (if unload): a pure couple -k_u h_body, |.| <= rcs_tau_max per
        axis, so the wheels are driven back toward zero while attitude is held (the wheels
        supply tau_cmd - tau_rcs). Returns the total body torque to apply."""
        cfg = self.cfg
        R = self.sim.kd.xmat[self.sim.bid_chaser].reshape(3, 3)
        dt = self.sim.cfg.ctrl_dt
        h_b = R.T @ self.h_w
        tau_rcs = np.zeros(3)
        if cfg.unload:
            tau_rcs = np.clip(-cfg.unload_k * h_b, -cfg.rcs_tau_max, cfg.rcs_tau_max)
            tau_rcs[np.abs(h_b) < cfg.unload_deadband] = 0.0
        tau_w = np.clip(np.asarray(tau_cmd, float) - tau_rcs, -cfg.rw_limit, cfg.rw_limit)
        grow = (np.abs(h_b) >= cfg.wheel_h_max) & (np.sign(-tau_w) == np.sign(h_b))
        self.sat_steps += int(grow.any())
        tau_w[grow] = 0.0
        w = getattr(self, "_gyro_last", None)
        w = self.sim.data.qvel[3:6] if w is None else w
        self.h_w = self.h_w - (R @ tau_w) * dt
        self.hw_max = max(self.hw_max, float(np.max(np.abs(R.T @ self.h_w))))
        self.rcs_impulse += float(np.sum(np.abs(tau_rcs))) * dt
        return tau_w + tau_rcs - np.cross(w, h_b)

    def _w_lvlh_body(self):
        Rs = self.sim.kd.xmat[self.sim.bid_chaser].reshape(3, 3)
        return Rs.T @ np.array([0.0, 0.0, self.sim.n])

    # ------------------------------------------------------------------ phases
    def lock_arm(self, pin_jaws=True):
        """Arm brakes: rebuild the model with every arm joint's range pinned to its
        captured angle (a stiff joint-limit constraint, solref 0.004) and copy the full
        state across. The grip itself stays contact-only. A PD 'lock' at the 100 Hz
        control rate was either unstable (light wrist) or too soft for a 300 kg client
        (SPEC_3 section 7)."""
        old = self.sim
        # always rebuild from the unlocked case scene (a re-lock must not stack attributes)
        self._base_scene = getattr(self, "_base_scene", old.cfg.scene)
        xml = (HERE / self._base_scene).read_text()
        names = ["j1_sh_yaw", "j2_sh_pitch", "j3_elbow", "j4_wr_pitch", "j5_wr_yaw", "j6_wr_roll"]
        # non-backdrivable jaws: pin the jaw slides where they closed (the squeeze stays as
        # contact preload); servo-driven jaws act as 400 N/m springs the client bobs on
        qs = self.q_lock
        if pin_jaws:
            names = names + ["grip_u", "grip_l"]
            qg = old.data.qpos[GRIP_Q].copy()
            # E2 fault: the jaw brakes engage looser (grip_u opens +, grip_l opens -)
            qg += np.array([1.0, -1.0]) * self.cfg.grip_loose_mm / 1000.0
            qs = np.concatenate([self.q_lock, qg])
        for nm, q in zip(names, qs):
            w = 1e-4 if nm.startswith("j") else 1e-5
            xml = re.sub(rf'(<joint name="{nm}"[^>]*?)range="[^"]*"',
                         rf'\1range="{q - w:.6f} {q + w:.6f}" solreflimit="0.004 1"', xml)
        path = HERE / f".case_{os.getpid()}_locked{'' if pin_jaws else '_rc'}.xml"
        path.write_text(xml)
        new = GraspSim(dataclasses.replace(old.cfg, scene=path.name))
        o = old.data.orbit
        new.data.reset(OrbitInit(R_eci=np.array(o.R_eci, float), V_eci=np.array(o.V_eci, float),
                                 t=float(o.t)))
        new.data.qpos[:] = old.data.qpos
        new.data.qvel[:] = old.data.qvel
        new.data.ctrl[:] = old.data.ctrl
        if not pin_jaws:  # re-clamp: the jaw drive squeezes again (position servos)
            sq = old.cfg.grip_squeeze
            new.data.ctrl[12:14] = np.array([-sq, sq])
        mjo_forward(new.model, new.data)
        new.rel_pos_ref, new.rel_att_ref = old.rel_pos_ref, old.rel_att_ref
        self.sim = new
        self.locked = True

    def impose_tumble(self):
        """Give the grasped stack a rigid-body tumble (random axis, magnitude in range)."""
        sim, cfg = self.sim, self.cfg
        sim.sync()
        kd, d = sim.kd, sim.data
        M, _, _, cm = stack_truth(sim)
        m = sim.km.body_mass
        ms = m[sim.bid_target]
        vcm = ((M - ms) * d.qvel[0:3] + ms * d.qvel[14:17]) / M  # arm momentum neglected
        ax = self.rng.normal(size=3)
        ax /= np.linalg.norm(ax)
        mag = np.radians(self.rng.uniform(*cfg.tumble_deg_s))
        w = np.array([0.0, 0.0, sim.n]) + mag * ax
        qv = np.zeros(sim.km.nv)
        Rs = kd.xmat[sim.bid_chaser].reshape(3, 3)
        Rc = kd.xmat[sim.bid_target].reshape(3, 3)
        qv[0:3] = vcm + np.cross(w, kd.xpos[sim.bid_chaser] - cm)
        qv[3:6] = Rs.T @ w
        qv[14:17] = vcm + np.cross(w, kd.xpos[sim.bid_target] - cm)
        qv[17:20] = Rc.T @ w
        d.qvel[:] = qv
        mjo_forward(sim.model, d)
        self.log["tumble_deg_s"] = float(np.degrees(mag))

    def detumble(self):
        cfg = self.cfg
        t0 = self.t
        while self.t - t0 < cfg.detumble_max_s:
            g, _, _, _ = self._step(
                tau_body=-cfg.detumble_kd * (self._gyro_last - self._w_lvlh_body()))
            self._gyro_last = g
            rel = np.degrees(np.linalg.norm(g - self._w_lvlh_body()))
            if rel < cfg.detumble_tol_deg_s and self.t - t0 > 2.0:
                break
        self.log["detumble_s"] = self.t - t0
        self.log["detumble_rate_end_deg_s"] = float(np.degrees(np.linalg.norm(
            self.sim.data.qvel[3:6] - self._w_lvlh_body())))

    def _collect(self, profile, hold_att):
        """Run a list of (F_body, tau_body, seconds) segments; return per-window sums."""
        cfg = self.cfg
        n_win = int(round(cfg.window_s / self.sim.cfg.ctrl_dt))
        rows = []
        R_hold = self.sim.kd.xmat[self.sim.bid_chaser].reshape(3, 3).copy()
        att_int = np.zeros(3)
        for F, tau_ff, secs in profile:
            n = int(round(secs / self.sim.cfg.ctrl_dt))
            k = 0
            while k < n:
                w0 = self._gyro_last.copy()
                S_F, S_tau, S_dv = np.zeros(3), np.zeros(3), np.zeros(3)
                S_w, S_wwT = np.zeros(3), np.zeros((3, 3))
                ws = []
                for _ in range(min(n_win, n - k)):
                    tau = np.array(tau_ff, float)
                    if hold_att:
                        Rs = self.sim.kd.xmat[self.sim.bid_chaser].reshape(3, 3)
                        e = Rs.T @ _rotvec(R_hold @ Rs.T)
                        tau = tau + cfg.hold_kp * e - cfg.hold_kd * (self._gyro_last - self._w_lvlh_body())
                        R_hold = Rotation.from_rotvec(np.array([0, 0, self.sim.n]) * self.sim.cfg.ctrl_dt).as_matrix() @ R_hold
                    g, dv, tau_used, F_used = self._step(F_body=np.array(F, float), tau_body=tau)
                    dt = self.sim.cfg.ctrl_dt
                    wm = 0.5 * (self._gyro_last + g)
                    S_F += F_used * dt
                    S_tau += tau_used * dt
                    S_dv += dv
                    S_w += wm * dt
                    S_wwT += np.outer(wm, wm) * dt
                    ws.append(wm)
                    self._gyro_last = g
                    k += 1
                rows.append(dict(w0=w0, w1=self._gyro_last.copy(), S_F=S_F, S_tau=S_tau, S_dv=S_dv,
                                 ws=np.array(ws), dt=self.sim.cfg.ctrl_dt))
        return rows

    def identify_rot(self, rows):
        A, b = [], []
        for r in rows:
            dw = r["w1"] - r["w0"]
            # integral of w x (I w): sum_k skew(w_k) L(w_k) p dt
            G = sum(_skew(w) @ _lin_I(w) for w in r["ws"]) * r["dt"]
            A.append(_lin_I(dw) + G)
            b.append(r["S_tau"])
        p, *_ = np.linalg.lstsq(np.vstack(A), np.concatenate(b), rcond=None)
        return _sym(p)

    def identify_trans(self, rows, I_hat):
        """Unknowns x = (1/m, d). Accelerometer: dv = S_F/m + dw x d + sum w x (w x d) dt.
        Torque balance: d x S_F = I dw + sum w x I w dt - S_tau  -> -skew(S_F) d = rhs."""
        A, b = [], []
        for r in rows:
            dw = r["w1"] - r["w0"]
            Wsum = sum(_skew(w) @ _skew(w) for w in r["ws"]) * r["dt"]
            A.append(np.hstack([r["S_F"][:, None], _skew(dw) + Wsum]))
            b.append(r["S_dv"])
            gyro_term = I_hat @ dw + sum(np.cross(w, I_hat @ w) for w in r["ws"]) * r["dt"]
            A.append(np.hstack([np.zeros((3, 1)), -_skew(r["S_F"])]))
            b.append(gyro_term - r["S_tau"])
        x, *_ = np.linalg.lstsq(np.vstack(A), np.concatenate(b), rcond=None)
        return 1.0 / x[0], x[1:]

    def validate(self, d_hat):
        """Thrust along bus y (across the bus-to-CoM line, so it makes ~0.7 N m) for val_s
        with a soft wheel attitude hold, without then with the counter-torque feed-forward
        -d_hat x F. Returns peak attitude error (deg) each."""
        cfg = self.cfg
        out = {}
        F = np.array([0.0, cfg.id_thrust, 0.0])
        for lab, ff in (("no_ff", np.zeros(3)), ("ff", -np.cross(d_hat, F))):
            R_hold = self.sim.kd.xmat[self.sim.bid_chaser].reshape(3, 3).copy()
            peak = 0.0
            n = int(round(cfg.val_s / self.sim.cfg.ctrl_dt))
            for _ in range(n):
                Rs = self.sim.kd.xmat[self.sim.bid_chaser].reshape(3, 3)
                e = Rs.T @ _rotvec(R_hold @ Rs.T)
                peak = max(peak, float(np.degrees(np.linalg.norm(e))))
                tau = ff + cfg.hold_kp * 0.1 * e - cfg.hold_kd * 0.1 * (self._gyro_last - self._w_lvlh_body())
                g, _, _, _ = self._step(F_body=F, tau_body=tau)
                self._gyro_last = g
                R_hold = Rotation.from_rotvec(np.array([0, 0, self.sim.n]) * self.sim.cfg.ctrl_dt).as_matrix() @ R_hold
            out[lab] = peak
            # coast and re-hold between the two runs
            for _ in range(int(10 / self.sim.cfg.ctrl_dt)):
                g, _, _, _ = self._step(tau_body=-cfg.detumble_kd * (self._gyro_last - self._w_lvlh_body()))
                self._gyro_last = g
        return out

    # ------------------------------------------------------------------ episode
    def run(self):
        res, est = self.capture_identify()
        if est is None:
            return res
        val = self.validate(est["d_hat"])
        res.update(val_peak_no_ff_deg=val["no_ff"], val_peak_ff_deg=val["ff"])
        return res

    def capture_identify(self):
        """Grasp, brake, tumble, detumble and identify. Returns (results, estimates) with
        estimates = dict(m_hat, I_hat, d_hat) or None if the grasp failed."""
        cfg = self.cfg
        rkw = dict(pos_err=0.05, vel_err=0.002, att_err_deg=2.0,
                   spin_deg_s=cfg.spin_deg_s, spin_axis=cfg.spin_axis)
        rkw.update(getattr(self, "grasp_reset_kw", {}))  # E3: hand-off from 3A
        g = self.sim.run(self.rng, **rkw)
        self.grasp_out = g
        res = dict(grasped=bool(g["grasped"]), **{k: (v.tolist() if isinstance(v, np.ndarray) else v)
                                                   for k, v in self.client.items()})
        # E2/E3: what happened up to and including the grasp
        res.update({f"g_{k}": g.get(k) for k in (
            "outcome", "aborted", "abort_reason", "abort_phase", "safe_reached", "t_to_safe",
            "collision", "hit_force_max", "min_clear", "detect_reason", "t_detect", "t_fault",
            "fault_phase", "fault_mag", "flyin_s", "flyin_k_min", "flyin_min_clear", "flyin_end_err_m",
            "bar_shift_mm", "t_grasp")})
        if not g["grasped"] or g.get("aborted"):
            return res, None
        self.q_lock = self.sim.data.qpos[ARM_Q].copy()
        self.t, self.h_rw, self.h_rw_max = 0.0, np.zeros(3), 0.0
        self.pads_lost, self.arm_tau_max = False, 0.0
        if cfg.brakes:
            self.lock_arm()
        if cfg.impose_tumble:
            self.impose_tumble()
        else:
            self.log["tumble_deg_s"] = float(cfg.spin_deg_s)
        self.sim.sync()
        self._gyro_last = self.sim.data.qvel[3:6].copy()
        try:
            self.phase_tag = "detumble"
            self.detumble()
            self.phase_tag = "id"
            M, I_true, d_true, _ = stack_truth(self.sim)
            # rotational ID: +-torque pulses about each bus axis
            T = cfg.id_torque
            prof = []
            for ax in range(3):
                e = np.eye(3)[ax]
                prof += [((0, 0, 0), T * e, cfg.id_pulse_s), ((0, 0, 0), -T * e, cfg.id_pulse_s)]
            rows_r = self._collect(prof, hold_att=False)
            I_hat = self.identify_rot(rows_r)
            self.detumble()
            # translational ID: +-thrust along each bus axis, wheels holding attitude
            Fm = cfg.id_thrust
            prof = []
            for ax in range(3):
                e = np.eye(3)[ax]
                prof += [(Fm * e, (0, 0, 0), cfg.id_pulse_s), (-Fm * e, (0, 0, 0), cfg.id_pulse_s)]
            rows_t = self._collect(prof, hold_att=True)
            m_hat, d_hat = self.identify_trans(rows_t, I_hat)
            self.detumble()
        except GripAbort as e:
            t_ab = self.t
            self.safe_coast()
            res.update(grip_abort=str(e), grip_abort_t=t_ab, grip_abort_phase=self.phase_tag,
                       **self.grip_outcome())
            return res, None
        eI = np.linalg.norm(I_hat - I_true) / np.linalg.norm(I_true)
        evals_t, evals_h = np.linalg.eigvalsh(I_true), np.linalg.eigvalsh(I_hat)
        res.update(
            tumble_deg_s=self.log["tumble_deg_s"], detumble_s=self.log["detumble_s"],
            detumble_rate_end_deg_s=self.log["detumble_rate_end_deg_s"],
            h_rw_max=self.h_rw_max, pads_lost=self.pads_lost, arm_tau_frac_max=self.arm_tau_max,
            hw_max_3c=self.hw_max, sat_s_3c=self.sat_steps * self.sim.cfg.ctrl_dt,
            pad_lost_s=getattr(self, "pad_lost_steps", 0) * self.sim.cfg.ctrl_dt,
            pad_lost_first=str(getattr(self, "pad_lost_first_t", None)),
            clamp_min_N=float(getattr(self, "clamp_min", np.nan)),
            bar_shift_3b_mm=float(self.grasp_out.get("bar_shift_mm", np.nan)),
            rcs_prop_g_3c=1000 * self.rcs_impulse / cfg.rcs_lever / (cfg.isp * 9.80665),
            mass_true=float(M), mass_hat=float(m_hat), mass_err_pct=float(100 * (m_hat - M) / M),
            I_err_rel_pct=float(100 * eI),
            I_eig_err_pct_max=float(100 * np.max(np.abs(evals_h - evals_t) / evals_t)),
            d_true=d_true.tolist(), d_hat=d_hat.tolist(),
            I_true=I_true.tolist(), I_hat=I_hat.tolist(),
            d_err_mm=float(1000 * np.linalg.norm(d_hat - d_true)))
        res.update(grip_abort="", **self.grip_outcome())
        return res, dict(m_hat=float(m_hat), I_hat=I_hat, d_hat=d_hat)


if __name__ == "__main__":
    seed = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    r = StackRun(seed, StackCfg()).run()
    for k, v in r.items():
        print(f"{k:26s} {v}")
