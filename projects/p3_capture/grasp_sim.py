"""Project 3B: classical reach-and-grasp of the client's grapple bar (SPEC_3 section 5).

Physics runs in mjorbit (CPU backend, orbit coupling on). A plain-MuJoCo copy of the same
model is kept in sync each control step for kinematics, Jacobians and contact forces
(the mjorbit CPU data exposes qpos/qvel/xpos/xmat/ctrl only).

Episode (servicer starts near the 3A pre-grasp point, arm stowed):
  settle   0-2 s      bus station-keeping, arm holds the stowed pose
  reach    20 s       joint-space minimum-jerk move to the pre-contact pose (IK by damped
                      least squares): grip point 0.12 m in front of the bar, gripper
                      aligned with the bar frame, jaws open
  approach           Cartesian impedance; the target slides along the approach axis at
                      v_approach to the bar centre (tracks the client's motion)
  close              jaws closed (position servos, force-limited) when the grip point is
                      within 1 cm of the bar axis or after a timeout
  hold     30 s       impedance holds the current grip pose; bus references reset to the
                      captured relative pose
Bus modes: "free-flying" (thrusters and wheels hold the servicer's pose relative to the
client throughout) or "free-floating" (bus actuators off from the start of the approach).
"""

from __future__ import annotations

import dataclasses
import sys
from pathlib import Path

import mjorbit  # noqa: F401

# isort: split
import mujoco
import numpy as np
from mjorbit import MjoModel, OrbitInit, mjo_forward, mjo_step
from mjorbit.constants import GM_EARTH, R_EARTH
from scipy.spatial.transform import Rotation

XML = Path(__file__).with_name("p3b_scene.xml")
ARM = slice(6, 12)  # arm dofs in qvel (chaser free joint = 0:6)
ARM_Q = slice(7, 13)  # arm joints in qpos
GRIP_Q = slice(13, 15)
CLIENT_Q = slice(15, 22)
CLIENT_V = slice(14, 20)
U_THR, U_RW, U_ARM, U_GRIP = slice(0, 3), slice(3, 6), slice(6, 12), slice(12, 14)
U_TEST_CF, U_TEST_CT, U_TEST_SF, U_TEST_ST = slice(14, 17), slice(17, 20), slice(20, 23), slice(23, 26)
# p3b defaults; the robot-specific values (stow pose, jaws, pre-grasp distance, bounding
# radius, keep-out boxes, thrust limit, gains) come from the scene via robot_spec.py and are
# instance attributes of GraspSim. The index layout above is checked against the scene.
STOW = np.array([0.0, -1.3, 2.5, -1.2, 0.0, 0.0])
GRIP_OPEN, GRIP_CLOSED = np.array([0.06, -0.06]), np.array([-0.012, 0.012])


@dataclasses.dataclass
class GraspCfg:
    dt: float = 0.002  # physics
    ctrl_dt: float = 0.01  # controller (100 Hz)
    settle_s: float = 2.0
    reach_s: float = 20.0
    pre_contact: float = 0.12  # m in front of the bar along the approach axis
    v_approach: float = 0.01  # m/s
    approach_timeout_s: float = 30.0
    blend_s: float = 5.0  # approach starts with a min-jerk blend from the current grip pose
    close_s: float = 3.0
    hold_s: float = 30.0
    # fly-in from a spin-axis hold point (SPEC_3 section 11)
    flyin: bool = False
    flyin_D: float = 8.0  # m, hold point distance from the client centre along the spin axis
    flyin_v: float = 0.05  # m/s peak path speed in the client frame
    flyin_h_safe: float = 2.4  # m, safe height along the axis for the radial leg
    flyin_settle_s: float = 20.0
    flyin_ff_limit: bool = True  # slow the fly-in path online so the feed-forward fits the thrust
    flyin_f_ff_max: float = 0.65  # N, norm (so <= on every world axis) of the 0.75 N per-axis limit
    flyin_k_grid: tuple = (1.0, 0.85, 0.7, 0.55, 0.4, 0.25)  # path-rate factors tried
    flyin_k_lookahead_s: tuple = (0.0, 5.0, 10.0, 15.0, 20.0)  # path time ahead checked
    flyin_k_tau: float = 3.0  # s, low-pass on the path-rate factor (smooth reference)
    flyin_wdot_ff: bool = True  # feed forward the client's angular acceleration (nutation)
    flyin_wdot_tau: float = 2.0  # s, low-pass on the differentiated client rate
    servicer_radius: float = -1.0  # m, bounding sphere (-1: from the scene, robot_spec)
    clear_margin: float = 0.3
    mode: str = "free-flying"  # or "free-floating"
    # Cartesian impedance at the grip point
    kp_lin: float = 300.0
    kd_lin: float = 60.0
    kp_rot: float = 30.0
    kd_rot: float = 6.0
    kq_null: float = 2.0  # joint damping in the impedance phase
    # joint-space reach
    kq: float = 60.0
    dq: float = 15.0
    # bus station keeping (relative to the client)
    kp_pos: float = 0.6
    kd_pos: float = 18.0
    kp_att: float = 4.0
    kd_att: float = 25.0
    # gains above are for the reference servicer (p3b: 148.4 kg, bus inertia 22.67 kg m^2 per
    # axis); with gain_scale the bus position gains scale with the servicer mass and the
    # attitude gains, per body axis, with the bus inertia (same bandwidth and damping)
    gain_scale: bool = True
    gain_ref_m: float = 148.4
    gain_ref_I: tuple = (22.6667, 22.6667, 22.6667)
    tau_bus_max: float = 1.0  # N m per body axis (wheel torque used; actuator range is larger)
    altitude_km: float = 400.0
    scene: str = "p3b_scene.xml"  # scene file next to this module
    grip_squeeze: float = 0.03  # jaw slide target when closed (pads touch the bar at ~0.008);
    # 0.03 gives ~17.6 N clamp (0.012: 3.2 N, too weak for 3C loads; SPEC_3 section 6)
    # load test (SPEC_3 section 6): after capture wait load_wait_s, then ramp an internal
    # wrench (equal and opposite on client and servicer) along +-client axis load_axis
    load_kind: str = ""  # "" (off), "force" or "torque"
    load_axis: int = 0
    load_sign: float = 1.0
    load_rate: float = 1.0  # N/s or N m/s
    load_max: float = 30.0
    load_wait_s: float = 5.0
    # slip = sustained relative motion (soft contacts deform a few mm elastically under load,
    # which is not slip): bar speed in the gripper frame > slip_vel_mm_s or rotation rate
    # > slip_rate_deg_s (0.2 s average), or gross motion > slip_pos_mm / slip_rot_deg
    slip_vel_mm_s: float = 2.0
    slip_rate_deg_s: float = 1.0
    slip_pos_mm: float = 15.0
    slip_rot_deg: float = 10.0
    req_force: float = 2.0  # 3C requirement (SPEC_3 section 6): record deflection here
    req_torque: float = 1.5
    # E2 safety (SPEC_3 section 12, safety.py): monitors, abort, injected faults
    mon: bool = False  # run the monitors
    mon_act: bool = True  # abort when a monitor trips (False: record only)
    fault_kind: str = ""  # client_kick | thruster_weak | thruster_stuck | squeeze_loss | spin_axis_err
    fault_phase: str = "random"  # phase the fault starts in ("random": flyin/reach/approach)
    fault_t: float = -1.0  # s into that phase (-1: random within it)
    fault_mag: float = -1.0  # (-1: random in the kind's range, see _draw_fault)
    fault_axis: int = -1


def _rotvec(R):
    return Rotation.from_matrix(R).as_rotvec()


def _quat_from_mat(R):
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, R.reshape(-1))
    return q


class GraspSim:
    def __init__(self, cfg: GraspCfg):
        self.cfg = cfg
        xml_path = XML.with_name(cfg.scene)
        self.model = MjoModel.from_xml_path(str(xml_path), mj_timestep=cfg.dt)
        r = R_EARTH + cfg.altitude_km
        v = float(np.sqrt(GM_EARTH / r))
        self.orbit = OrbitInit(R_eci=[r, 0.0, 0.0], V_eci=[0.0, v, 0.0])
        self.n = v / r  # rad/s
        self.data = self.model.make_data(orbit=self.orbit)
        xml = xml_path.read_text()
        xml = "\n".join(line for line in xml.splitlines() if "<mjorbit " not in line
                        and "use_gravity_gradient" not in line)
        self.km = mujoco.MjModel.from_xml_string(xml)
        self.kd = mujoco.MjData(self.km)
        km = self.km
        self.sid_grip = mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_SITE, "grip_site")
        self.sid_bar = mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_SITE, "bar_site")
        self.bid_chaser = mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_BODY, "chaser")
        self.bid_target = mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_BODY, "target")
        g = lambda nm: mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_GEOM, nm)  # noqa: E731
        self.gid_pads = {g("pad_u"), g("pad_l")}
        self.gid_bar = g("handle_bar")
        self.gid_grip = {g("pad_u"), g("pad_l"), g("palm")}
        self.gid_client = {g("client_bus"), g("handle_post_l"), g("handle_post_r"), g("handle_bar")}
        root = km.body_rootid
        self.m_serv = float(sum(km.body_mass[i] for i in range(km.nbody) if root[i] == self.bid_chaser))
        self._apply_spec()
        self.step_hook = None  # P4: called each control step with the run loop's locals
        # P4.3: estimated client pose for the arm controller (None = truth). Object with
        # update(sim, t, phase), bar(sim) -> (p_bar, Rc) and client(sim) -> (p_c, Rc, v_c, w_c),
        # all world frame. Bus station keeping and the E2 monitors keep using the truth.
        self.pose_est = None
        self.jacp = np.zeros((3, km.nv))
        self.jacr = np.zeros((3, km.nv))

    def _apply_spec(self):
        """Robot description (robot_spec.py): robot-specific values from the scene."""
        from robot_spec import build_spec
        cfg, km = self.cfg, self.km
        sp = self.spec = build_spec(km)
        idx = lambda sl: list(range(sl.start, sl.stop))  # noqa: E731
        layout_ok = (sp.serv_q == slice(0, 7) and sp.serv_v == slice(0, 6) and sp.client_q == CLIENT_Q
                     and sp.client_v == CLIENT_V and list(sp.arm_q) == idx(ARM_Q)
                     and list(sp.arm_v) == idx(ARM) and list(sp.grip_q) == idx(GRIP_Q)
                     and list(sp.u_thr) == idx(U_THR) and list(sp.u_rw) == idx(U_RW)
                     and list(sp.u_arm) == idx(U_ARM) and list(sp.u_grip) == idx(U_GRIP))
        if not layout_ok:
            raise NotImplementedError(f"{cfg.scene}: joint/actuator layout differs from p3b "
                                      "(index generalisation not wired yet; see robot_spec.py)")
        self.stow = sp.stow.copy()
        self.grip_open = sp.grip_open.copy()
        self.ik_seed = sp.ready.copy() if sp.ready_given else None
        self.pregrasp_d = sp.pregrasp_d
        self.r_serv = sp.servicer_radius if cfg.servicer_radius < 0 else cfg.servicer_radius
        self.f_max = sp.f_max
        self.boxes = [(np.asarray(c, float), np.asarray(h, float)) for c, h in sp.client_boxes]
        snap = lambda x: 1.0 if abs(x - 1.0) < 1e-3 else float(x)  # noqa: E731
        if cfg.gain_scale:
            self.s_m = snap(self.m_serv / cfg.gain_ref_m)
            self.s_I = np.array([snap(a / b) for a, b in zip(sp.I_serv, cfg.gain_ref_I)])
        else:
            self.s_m, self.s_I = 1.0, np.ones(3)
        self.kp_pos, self.kd_pos = cfg.kp_pos * self.s_m, cfg.kd_pos * self.s_m

    def att_torque(self, Rs, T_world):
        """World-frame PD torque (reference-servicer gains) to the body-frame wheel command,
        scaled per body axis by the bus inertia ratio, clipped to the wheel torque."""
        return np.clip(self.s_I * (Rs.T @ T_world), -self.cfg.tau_bus_max, self.cfg.tau_bus_max)

    # ---------------------------------------------------------------- state sync
    def sync(self):
        self.kd.qpos[:] = self.data.qpos
        self.kd.qvel[:] = self.data.qvel
        self.kd.ctrl[:] = self.data.ctrl  # actuator forces enter the contact solve
        mujoco.mj_forward(self.km, self.kd)

    def site_pose(self, sid):
        return self.kd.site_xpos[sid].copy(), self.kd.site_xmat[sid].reshape(3, 3).copy()

    def site_vel(self, sid):
        mujoco.mj_jacSite(self.km, self.kd, self.jacp, self.jacr, sid)
        qv = self.kd.qvel
        return self.jacp @ qv, self.jacr @ qv

    def contacts(self):
        """(pad-bar normal force per pad dict, total client-contact force, any non-bar hit)."""
        f6 = np.zeros(6)
        pad_bar = {}
        total = 0.0
        other = False
        for i in range(self.kd.ncon):
            c = self.kd.contact[i]
            pair = {c.geom1, c.geom2}
            if not (pair & self.gid_grip and pair & self.gid_client):
                continue
            mujoco.mj_contactForce(self.km, self.kd, i, f6)
            fn = abs(f6[0])
            total += fn
            if self.gid_bar in pair and pair & self.gid_pads:
                pad = (pair & self.gid_pads).pop()
                pad_bar[pad] = pad_bar.get(pad, 0.0) + fn
            elif self.gid_bar not in pair:
                other = True
        return pad_bar, total, other

    # ---------------------------------------------------------------- setup
    def reset(self, rng=None, pos_err=0.08, vel_err=0.003, att_err_deg=3.0,
              client_tilt_deg=3.0, client_spin=True, spin_deg_s=0.0, spin_axis=-1,
              hold_dp_b=None, hold_dv_b=None):
        """Client at the origin, LVLH-fixed (spin at the orbit rate about the orbit normal,
        world z here) or inertially fixed; its body x along world +y (3A yaw 0), tilted by
        up to client_tilt_deg. Servicer at the pre-grasp point (bar centre + 1.8 m along
        the face normal) with uniform errors, attitude aligned with the client's plus
        att_err_deg, arm stowed, jaws open."""
        rng = np.random.default_rng() if rng is None else rng
        d, km = self.data, self.km
        R0 = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])  # body x -> +y
        tilt = Rotation.from_rotvec(np.radians(rng.uniform(-client_tilt_deg, client_tilt_deg, 3)))
        Rc = tilt.as_matrix() @ R0
        Rs = Rotation.from_rotvec(np.radians(rng.uniform(-att_err_deg, att_err_deg, 3))).as_matrix() @ Rc
        bar = Rc @ np.array([-0.9, 0.0, 0.0])
        nf = Rc @ np.array([-1.0, 0.0, 0.0])
        ps = bar + self.pregrasp_d * nf + rng.uniform(-pos_err, pos_err, 3)
        qpos = np.array(km.qpos0)
        qpos[0:3] = ps
        qpos[3:7] = _quat_from_mat(Rs)
        qpos[ARM_Q] = self.stow
        qpos[GRIP_Q] = self.grip_open
        qpos[15:18] = 0.0
        qpos[18:22] = _quat_from_mat(Rc)
        qvel = np.zeros(km.nv)
        w_lvlh = np.array([0.0, 0.0, self.n])  # LVLH angular velocity (world z here)
        # velocity relative to LVLH: v_world = v_rel + w x r
        # client spin (SPEC_3 section 10): spin_deg_s about a client body axis, on top of
        # the LVLH rate; the servicer starts synchronised (co-rotating at the pre-grasp
        # point), i.e. at the end of a synchronisation manoeuvre, with the usual errors
        ax = int(rng.integers(3)) if spin_axis < 0 else int(spin_axis)
        self.spin_axis_used = ax
        w_spin = Rc @ (np.radians(spin_deg_s) * np.eye(3)[ax])
        w_c = (w_lvlh if client_spin else np.zeros(3)) + w_spin
        qvel[0:3] = rng.uniform(-vel_err, vel_err, 3) + np.cross(w_lvlh, ps) + np.cross(w_spin, ps)
        qvel[3:6] = Rs.T @ (w_lvlh + w_spin)  # servicer co-rotating, body frame
        qvel[17:20] = Rc.T @ w_c
        d.reset(self.orbit)
        d.qpos[:] = qpos
        d.qvel[:] = qvel
        d.ctrl[:] = 0.0
        assert np.allclose(d.qpos, qpos), "mjorbit qpos is not a writable view"
        mjo_forward(self.model, d)
        self.sync()
        Rs_now = self.kd.xmat[self.bid_chaser].reshape(3, 3)
        Rc_now = self.kd.xmat[self.bid_target].reshape(3, 3)
        ps_now, pc_now = self.kd.xpos[self.bid_chaser], self.kd.xpos[self.bid_target]
        # bus references: nominal pre-grasp pose relative to the client (zero errors)
        self.rel_pos_ref = Rc_now.T @ ((bar + self.pregrasp_d * nf) - pc_now)
        self.rel_att_ref = Rc_now.T @ Rc_now  # identity: servicer aligned with client
        del Rs_now, ps_now
        self.flyin_path = None
        if self.cfg.flyin:
            self._setup_flyin(rng, Rc_now, pc_now, pos_err, vel_err, att_err_deg, w_spin, spin_deg_s,
                              hold_dp_b, hold_dv_b)
        # client angular velocity model (inertial) for the relative-attitude anomaly monitor
        self.w_model = Rc_now @ self.data.qvel[17:20].copy()

    def _setup_flyin(self, rng, Rc, pc, pos_err, vel_err, att_err_deg, w_spin, spin_deg_s,
                     hold_dp_b=None, hold_dv_b=None):
        """Move the servicer to a hold point on the spin axis (inertially still, not
        synchronised) and plan the fly-in to the pre-grasp point. E3: hold_dp_b / hold_dv_b
        (client body frame) are the 3A arrival errors handed over instead of random ones."""
        from flyin import FlyIn
        cfg, d = self.cfg, self.data
        g_b = self.rel_pos_ref.copy()  # pre-grasp point, client body frame
        if spin_deg_s > 0:
            s_w = w_spin / np.linalg.norm(w_spin)
        else:
            s_w = Rc @ np.array([-1.0, 0.0, 0.0])  # no spin: approach along the face normal
        f = getattr(self, "_fault", None)
        if f and f["kind"] == "spin_axis_err" and spin_deg_s > 0:
            # E2 fault: the spin axis used for planning is wrong by mag degrees
            perp = np.cross(s_w, f["dir"])
            perp /= np.linalg.norm(perp)
            s_w = Rotation.from_rotvec(np.radians(f["mag"]) * perp).as_matrix() @ s_w
            f["t_fault"] = 0.0
        s_b = Rc.T @ s_w
        h_g = float(g_b @ s_b)
        side = np.sign(h_g) if abs(h_g) > 0.5 else 1.0
        dp = rng.uniform(-pos_err, pos_err, 3)
        dvv = rng.uniform(-vel_err, vel_err, 3)
        if hold_dp_b is not None:
            dp = Rc @ np.asarray(hold_dp_b, float)
        if hold_dv_b is not None:
            dvv = Rc @ np.asarray(hold_dv_b, float)
        H0 = pc + cfg.flyin_D * side * s_w
        H = H0 + dp
        w_lvlh = np.array([0.0, 0.0, self.n])
        Rs = Rotation.from_rotvec(np.radians(rng.uniform(-att_err_deg, att_err_deg, 3))).as_matrix() @ Rc
        d.qpos[0:3] = H
        d.qpos[3:7] = _quat_from_mat(Rs)
        d.qvel[0:3] = np.cross(w_lvlh, H) + dvv
        d.qvel[3:6] = Rs.T @ w_lvlh  # inertially near-still (LVLH-fixed), not spinning
        mjo_forward(self.model, d)
        self.sync()
        self.flyin_path = FlyIn(Rc.T @ (H - pc), g_b, s_b, h_safe=cfg.flyin_h_safe,
                                v_max=cfg.flyin_v)
        self.R_rel0 = Rc.T @ Rs
        self._flyin_t = 0.0
        self._wc_prev, self._wdot = None, np.zeros(3)
        self._flyin_k, self._flyin_k_tgt, self._flyin_k_min, self._flyin_k_t = 1.0, 1.0, 1.0, 1.0

    def clearance_now(self):
        """Servicer-centre clearance to the client (bus box and bar envelope)."""
        kd = self.kd
        Rc = kd.xmat[self.bid_target].reshape(3, 3)
        x = Rc.T @ (kd.xpos[self.bid_chaser] - kd.xpos[self.bid_target])
        best = np.inf
        for c, h in self.boxes:
            q = np.abs(x - c) - h
            best = min(best, float(np.linalg.norm(np.maximum(q, 0.0)) + min(q.max(), 0.0)))
        return best - self.r_serv - self.cfg.clear_margin

    def _freeze_refs(self):
        """At the close command: hold the current grip pose (relative to the client) and
        make the bus references the current relative pose, so nothing pushes on contact."""
        kd = self.kd
        p, R = self.site_pose(self.sid_grip)
        Rc = kd.xmat[self.bid_target].reshape(3, 3)
        Rs = kd.xmat[self.bid_chaser].reshape(3, 3)
        if self.pose_est is None:
            self.hold_rel_p = Rc.T @ (p - kd.xpos[self.bid_target])
            self.hold_rel_R = Rc.T @ R
        else:  # the arm holds relative to the estimated client frame it will use in hold
            pe, Re, _, _ = self.pose_est.client(self)
            self.hold_rel_p = Re.T @ (p - pe)
            self.hold_rel_R = Re.T @ R
        self.rel_pos_ref = Rc.T @ (kd.xpos[self.bid_chaser] - kd.xpos[self.bid_target])
        self.rel_att_ref = Rc.T @ Rs

    # ---------------------------------------------------------------- controllers
    def bus_ctrl(self):
        cfg = self.cfg
        kd = self.kd
        Rs = kd.xmat[self.bid_chaser].reshape(3, 3)
        Rc = kd.xmat[self.bid_target].reshape(3, 3)
        ps, pc = kd.xpos[self.bid_chaser], kd.xpos[self.bid_target]
        vs, vc = self.data.qvel[0:3], self.data.qvel[14:17]
        if getattr(self, "flyin_path", None) is not None and self._flyin_t is not None:
            w_c = Rc @ self.data.qvel[17:20]
            self._flyin_k_t += cfg.ctrl_dt
            if cfg.flyin_ff_limit and self._flyin_k_t >= 1.0:
                # once a second: the largest path-rate factor whose feed-forward, at the
                # measured client rate and over the next 20 s, fits the budget
                self._flyin_k_t = 0.0
                for k in cfg.flyin_k_grid:
                    dem = 0.0
                    for la in cfg.flyin_k_lookahead_s:
                        pr, _, ar, _, _ = self.flyin_path.reference(
                            self._flyin_t + la * k, pc, vc, Rc, w_c, self.R_rel0, k=k)
                        dem = max(dem, float(np.linalg.norm(ar + np.cross(self._wdot, pr - pc))))
                    if self.m_serv * dem <= cfg.flyin_f_ff_max:
                        break
                self._flyin_k_tgt = k
            a_k = cfg.ctrl_dt / cfg.flyin_k_tau
            self._flyin_k += a_k * (self._flyin_k_tgt - self._flyin_k)
            self._flyin_k_min = min(self._flyin_k_min, self._flyin_k)
            p_ref, v_ref, a_ff, R_ref, w_ref = self.flyin_path.reference(
                self._flyin_t, pc, vc, Rc, w_c, self.R_rel0, k=self._flyin_k)
            if cfg.flyin_wdot_ff:
                # client angular acceleration (Euler) term w' x r: a client spinning about a
                # non-principal axis nutates, its rate drifting ~10% over a fly-in at 2 deg/s;
                # without this term the bus lagged 0.25 m and tripped the tracking monitor
                # (E3 validation, SPEC_3 section 13). w' from a low-passed difference of the
                # measured client rate (no client inertia needed).
                if self._wc_prev is not None:
                    a = cfg.ctrl_dt / cfg.flyin_wdot_tau
                    self._wdot = (1 - a) * self._wdot + a * (w_c - self._wc_prev) / cfg.ctrl_dt
                self._wc_prev = w_c.copy()
                a_ff = a_ff + np.cross(self._wdot, p_ref - pc)
            F = self.m_serv * a_ff - self.kp_pos * (ps - p_ref) - self.kd_pos * (vs - v_ref)
            e = _rotvec(R_ref @ Rs.T)
            w_s = Rs @ self.data.qvel[3:6]
            T_world = cfg.kp_att * e - cfg.kd_att * (w_s - w_ref)
            self._flyin_last = (p_ref, v_ref)
            self._bus_ref, self._F_raw = (p_ref, v_ref, R_ref), F
            return np.clip(F, -self.f_max, self.f_max), self.att_torque(Rs, T_world)
        p_ref = pc + Rc @ self.rel_pos_ref
        # synchronised station keeping: the reference point moves with the client's rotation
        # (velocity w x r, centripetal feed-forward w x (w x r)); exact for a spinning client
        w_c = Rc @ self.data.qvel[17:20]
        r = p_ref - pc
        v_ref = vc + np.cross(w_c, r)
        a_ff = np.cross(w_c, np.cross(w_c, r))
        F = self.m_serv * a_ff - self.kp_pos * (ps - p_ref) - self.kd_pos * (vs - v_ref)
        R_ref = Rc @ self.rel_att_ref
        e = _rotvec(R_ref @ Rs.T)  # world-frame rotation error
        w_s = Rs @ self.data.qvel[3:6]
        w_c = Rc @ self.data.qvel[17:20]
        T_world = cfg.kp_att * e - cfg.kd_att * (w_s - w_c)
        self._bus_ref, self._F_raw = (p_ref, v_ref, R_ref), F
        return np.clip(F, -self.f_max, self.f_max), self.att_torque(Rs, T_world)

    # ---------------------------------------------------------------- E2 safety
    def _draw_fault(self, rng):
        """Fault parameters from a copy of the case rng (the case itself is unchanged, so
        faulted and nominal runs of a seed share the same initial errors)."""
        import copy
        cfg = self.cfg
        if not cfg.fault_kind:
            self._fault = None
            return
        fr = np.random.default_rng(int(copy.deepcopy(rng).integers(2**62)) + 977)
        rng_mag = dict(client_kick=(0.5, 2.0), thruster_weak=(0.0, 0.3), thruster_stuck=(0.1, 0.4),
                       squeeze_loss=(0.008, 0.011), spin_axis_err=(5.0, 15.0))[cfg.fault_kind]
        mag = cfg.fault_mag if cfg.fault_mag >= 0 else float(fr.uniform(*rng_mag))
        if cfg.fault_kind == "squeeze_loss":
            ph = "hold"
        elif cfg.fault_phase == "random":
            ph = ["flyin", "reach", "approach"][int(fr.integers(3))] if cfg.flyin else \
                ["reach", "approach"][int(fr.integers(2))]
        else:
            ph = cfg.fault_phase
        v = fr.normal(size=3)
        self._fault = dict(kind=cfg.fault_kind, phase=ph, u=float(fr.uniform()), t_in=cfg.fault_t,
                           mag=mag, axis=int(fr.integers(3)) if cfg.fault_axis < 0 else cfg.fault_axis,
                           dir=v / np.linalg.norm(v), t_fault=None, active=False)

    def _phase_len(self, phase):
        cfg = self.cfg
        return {"flyin": self.flyin_path.total if self.flyin_path is not None else 0.0,
                "reach": cfg.reach_s, "approach": cfg.blend_s + cfg.pre_contact / cfg.v_approach,
                "hold": 10.0}.get(phase, 0.0)

    def _fault_step(self, t, phase, t_phase):
        """Start the fault when its time comes; one-off effects are applied here."""
        f = self._fault
        if f is None or f["active"] or f["kind"] == "spin_axis_err" or phase != f["phase"]:
            return
        t_in = f["t_in"] if f["t_in"] >= 0 else (1.0 + 9.0 * f["u"] if phase == "hold"
                                                 else f["u"] * self._phase_len(phase))
        if t - t_phase < t_in:
            return
        f["active"], f["t_fault"] = True, t
        if f["kind"] == "client_kick":  # impulsive client rate change (e.g. a leak, a hit)
            Rc = self.kd.xmat[self.bid_target].reshape(3, 3)
            self.data.qvel[17:20] += Rc.T @ (np.radians(f["mag"]) * f["dir"])
            mjo_forward(self.model, self.data)
            self.sync()

    def _thrust_fault(self, F):
        f = self._fault
        if f is None or not f["active"] or f["kind"] not in ("thruster_weak", "thruster_stuck"):
            return F
        Rs = self.kd.xmat[self.bid_chaser].reshape(3, 3)
        Fb = Rs.T @ F
        if f["kind"] == "thruster_weak":
            Fb[f["axis"]] *= f["mag"]
        else:
            # one thruster valve stuck open: constant extra thrust along +axis; the opposing
            # thruster still works (a fault that removed the axis entirely leaves nothing
            # to steer with, abort or not)
            Fb[f["axis"]] += f["mag"]
        return Rs @ Fb

    def unintended_hits(self, allow_grip_bar):
        """(any unintended servicer-client contact, its max normal force). Gripper-on-bar
        contact is allowed while grasping (and briefly while releasing)."""
        km, kd = self.km, self.kd
        f6 = np.zeros(6)
        hit, fmax = False, 0.0
        root = km.body_rootid
        for i in range(kd.ncon):
            c = kd.contact[i]
            b1, b2 = root[km.geom_bodyid[c.geom1]], root[km.geom_bodyid[c.geom2]]
            if {b1, b2} != {self.bid_chaser, self.bid_target}:
                continue
            pair = {c.geom1, c.geom2}
            if allow_grip_bar and self.gid_bar in pair and pair & self.gid_grip:
                continue
            mujoco.mj_contactForce(km, kd, i, f6)
            hit, fmax = True, max(fmax, abs(f6[0]))
            nm = lambda g: mujoco.mj_id2name(km, mujoco.mjtObj.mjOBJ_GEOM, g) or f"g{g}"  # noqa: E731
            self._hit_names = f"{nm(c.geom1)}/{nm(c.geom2)}"
        return hit, fmax

    def grasp_target(self, offset):
        """Grip-site target pose: on the bar, `offset` m back along the approach axis."""
        if self.pose_est is not None:
            p_bar, Rc = self.pose_est.bar(self)
            return p_bar - offset * Rc[:, 0], Rc.copy()
        p_bar, R_bar = self.site_pose(self.sid_bar)
        Rc = self.kd.xmat[self.bid_target].reshape(3, 3)
        R_des = Rc.copy()  # gripper x = client +x (into the face), y = bar axis, z = client z
        return p_bar - offset * Rc[:, 0], R_des

    def ik(self, p_des, R_des, q0, iters=200, lam=0.05):
        km, kd = self.km, self.kd
        q_save, v_save = kd.qpos.copy(), kd.qvel.copy()
        q = q0.copy()
        for _ in range(iters):
            kd.qpos[ARM_Q] = q
            mujoco.mj_kinematics(km, kd)
            mujoco.mj_comPos(km, kd)
            p, R = kd.site_xpos[self.sid_grip], kd.site_xmat[self.sid_grip].reshape(3, 3)
            err = np.concatenate([p_des - p, _rotvec(R_des @ R.T)])
            if np.linalg.norm(err) < 1e-5:
                break
            mujoco.mj_jacSite(km, kd, self.jacp, self.jacr, self.sid_grip)
            J = np.vstack([self.jacp[:, ARM], self.jacr[:, ARM]])
            dq = J.T @ np.linalg.solve(J @ J.T + lam**2 * np.eye(6), err)
            q = np.clip(q + dq, self.spec.arm_range[:, 0], self.spec.arm_range[:, 1])
        kd.qpos[:], kd.qvel[:] = q_save, v_save
        mujoco.mj_forward(km, kd)
        return q, float(np.linalg.norm(err[:3])), float(np.linalg.norm(err[3:]))

    def impedance_tau(self, p_des, R_des, v_ref, w_ref):
        cfg = self.cfg
        p, R = self.site_pose(self.sid_grip)
        mujoco.mj_jacSite(self.km, self.kd, self.jacp, self.jacr, self.sid_grip)
        qv = self.kd.qvel
        v, w = self.jacp @ qv, self.jacr @ qv
        F = cfg.kp_lin * (p_des - p) + cfg.kd_lin * (v_ref - v)
        M = cfg.kp_rot * _rotvec(R_des @ R.T) + cfg.kd_rot * (w_ref - w)
        tau = self.jacp[:, ARM].T @ F + self.jacr[:, ARM].T @ M - cfg.kq_null * qv[ARM]
        return tau

    # ---------------------------------------------------------------- episode
    def run(self, rng=None, record=False, **reset_kw):
        cfg = self.cfg
        rng = np.random.default_rng() if rng is None else rng
        self._draw_fault(rng)
        self.reset(rng, **reset_kw)
        d = self.data
        from safety import AbortRef, Monitor, MonitorCfg
        mon = Monitor(MonitorCfg(enabled=cfg.mon, act=cfg.mon_act,
                                 **getattr(self, "mon_overrides", {})), cfg.ctrl_dt) if cfg.mon else None
        self._bus_ref, self._F_raw, self._F_cmd = None, None, None
        abort = None  # dict once an abort starts
        safe_t = 0.0
        nsub = int(round(cfg.ctrl_dt / cfg.dt))
        t = 0.0
        phase = "flyin" if self.flyin_path is not None else "settle"
        t_phase = 0.0
        q_start = q_goal = None
        approach_s = 0.0
        out = dict(success=False, grasped=False, ik_err_pos=np.nan, ik_err_rot=np.nan,
                   push_dv=0.0, peak_force=0.0, other_contact=False, bus_att_err_max=0.0,
                   t_grasp=np.nan, approach_err_max=0.0, final_rel_speed=np.nan,
                   final_bar_dist=np.nan, both_pads_end=False, phase_end="")
        vc0 = None
        Rs_ref0 = None
        self._rel_at_grasp = None
        out["clamp_force"] = np.nan
        trace = [] if record else None
        hold_s = cfg.hold_s
        if cfg.load_kind:
            hold_s = cfg.load_wait_s + cfg.load_max / cfg.load_rate + 3.0
        T_total = cfg.settle_s + cfg.reach_s + cfg.approach_timeout_s + cfg.close_s + hold_s
        if self.flyin_path is not None:
            T_total += self.flyin_path.total + cfg.flyin_settle_s
            out.update(flyin_s=self.flyin_path.total, flyin_legs=len(self.flyin_path.T),
                       flyin_k_min=1.0,
                       flyin_min_clear=np.inf, flyin_sat_frac=0.0, flyin_end_err_m=np.nan,
                       flyin_end_vrel_mm_s=np.nan, flyin_end_att_deg=np.nan)
            n_fly = 0
        out.update(load_at_slip=np.nan, slip_kind="", load_at_joint_sat=np.nan,
                   sat_joint=-1, load_max_reached=0.0, slip_pos_mm_max=0.0,
                   slip_rot_deg_max=0.0)
        load_base = None
        self._slip_prev, self._slip_v = None, np.zeros(2)
        out.update(defl_mm_at_req=np.nan, defl_deg_at_req=np.nan)
        grip_cmd = self.grip_open.copy()
        out.update(aborted=False, abort_reason="", abort_phase="", t_abort=np.nan, t_fault=np.nan,
                   fault_phase=self._fault["phase"] if self._fault else "",
                   fault_mag=self._fault["mag"] if self._fault else np.nan,
                   safe_reached=False, t_to_safe=np.nan, collision=False, hit_force_max=0.0,
                   min_clear=np.inf, detect_reason="", t_detect=np.nan, outcome="")
        while t < T_total and phase != "done":
            self.sync()
            kd = self.kd
            if self.pose_est is not None:
                self.pose_est.update(self, t, phase)
            self._fault_step(t, phase, t_phase)
            out["min_clear"] = min(out["min_clear"], self.clearance_now())
            # ---- phase logic
            if phase == "flyin":
                # path clock: advances at the online path-rate factor k (1 = as planned)
                if t == t_phase:
                    self._flyin_t, self._flyin_end_t = 0.0, None
                else:
                    self._flyin_t = min(self._flyin_t + self._flyin_k * cfg.ctrl_dt, self.flyin_path.total)
                    T_total += (1.0 - self._flyin_k) * cfg.ctrl_dt  # keep the later phases' time
                if self._flyin_t >= self.flyin_path.total and self._flyin_end_t is None:
                    self._flyin_end_t = t
                out["flyin_min_clear"] = min(out["flyin_min_clear"], self.clearance_now())
                if self._flyin_end_t is not None and t - self._flyin_end_t >= cfg.flyin_settle_s:
                    p_ref, v_ref = self._flyin_last
                    out["flyin_end_err_m"] = float(np.linalg.norm(kd.xpos[self.bid_chaser] - p_ref))
                    out["flyin_end_vrel_mm_s"] = float(np.linalg.norm(d.qvel[0:3] - v_ref) * 1000)
                    Rc_ = kd.xmat[self.bid_target].reshape(3, 3)
                    Rs_ = kd.xmat[self.bid_chaser].reshape(3, 3)
                    out["flyin_end_att_deg"] = float(np.degrees(np.linalg.norm(_rotvec(Rc_.T @ Rs_))))
                    out["flyin_sat_frac"] = out["flyin_sat_frac"] / max(n_fly, 1)
                    out["flyin_s"] = t - t_phase - cfg.flyin_settle_s  # actual path duration (slowed online)
                    out["flyin_k_min"] = float(self._flyin_k_min)
                    self._flyin_t = None
                    phase, t_phase = "settle", t
            if phase == "settle" and t - t_phase >= cfg.settle_s:
                p_pre, R_pre = self.grasp_target(cfg.pre_contact)
                seed = d.qpos[ARM_Q].copy() if self.ik_seed is None else self.ik_seed.copy()
                q_goal, ep, er = self.ik(p_pre, R_pre, seed)
                out["ik_err_pos"], out["ik_err_rot"] = ep, er
                q_start = d.qpos[ARM_Q].copy()
                phase, t_phase = "reach", t
            elif phase == "reach" and t - t_phase >= cfg.reach_s:
                phase, t_phase = "approach", t
                p0, R0 = self.site_pose(self.sid_grip)
                Rc0 = kd.xmat[self.bid_target].reshape(3, 3)
                pc0 = kd.xpos[self.bid_target]
                if self.pose_est is not None:
                    pc0, Rc0, _, _ = self.pose_est.client(self)
                self.blend_rel_p = Rc0.T @ (p0 - pc0)
                self.blend_rel_R = Rc0.T @ R0
                vc0 = d.qvel[14:17].copy()
                Rs_ref0 = kd.xmat[self.bid_chaser].reshape(3, 3).copy()
            elif phase == "approach":
                p_bar, _ = self.grasp_target(0.0)
                p_g, _ = self.site_pose(self.sid_grip)
                Rc = kd.xmat[self.bid_target].reshape(3, 3)
                if self.pose_est is not None:
                    _, Rc, _, _ = self.pose_est.client(self)
                rel = p_g - p_bar
                along = rel @ Rc[:, 0]
                perp = np.linalg.norm(rel - along * Rc[:, 0] - (rel @ Rc[:, 1]) * Rc[:, 1])
                if (abs(along) < 0.01 and perp < 0.01) or t - t_phase > cfg.approach_timeout_s:
                    phase, t_phase = "close", t
                    grip_cmd = np.array([-cfg.grip_squeeze, cfg.grip_squeeze])
                    self._freeze_refs()
            elif phase == "close" and t - t_phase >= cfg.close_s:
                pad_bar, _, _ = self.contacts()
                out["grasped"] = len(pad_bar) == 2
                out["t_grasp"] = t
                out["clamp_force"] = float(sum(pad_bar.values()))
                pg_, _ = self.site_pose(self.sid_grip)
                pb_, _ = self.site_pose(self.sid_bar)
                self._rel_at_grasp = pg_ - pb_
                phase, t_phase = "hold", t
            elif phase == "hold" and (t - t_phase >= hold_s or
                                      (cfg.load_kind and np.isfinite(out["load_at_slip"]))):
                phase = "done"
                break
            elif phase == "abort":
                r = kd.xpos[self.bid_chaser] - kd.xpos[self.bid_target]
                vr = float((d.qvel[0:3] - d.qvel[14:17]) @ r / np.linalg.norm(r))
                stowed = np.max(np.abs(d.qpos[ARM_Q] - self.stow)) < 0.05
                is_safe = (np.linalg.norm(r) >= abort["cfg"].r_safe_reached and stowed
                           and d.qpos[13] > 0.05 and vr > -0.001)
                safe_t = safe_t + cfg.ctrl_dt if is_safe else 0.0
                if is_safe and not out["safe_reached"]:
                    out["safe_reached"], out["t_to_safe"] = True, t - abort["t0"]
                if safe_t >= abort["cfg"].safe_dwell_s or t - abort["t0"] > abort["cfg"].abort_max_s:
                    phase = "done"
                    break
            # ---- E2: unintended contact, monitors, abort trigger
            # releasing: gripper-bar contact is expected briefly only if the jaws were on the bar
            in_grasp = phase in ("approach", "close", "hold") or (
                phase == "abort" and out["abort_phase"] in ("close", "hold")
                and t - abort["t0"] < mon.cfg.contact_grace_s)
            hit, fh = self.unintended_hits(allow_grip_bar=in_grasp)
            if hit:
                if not out["collision"]:
                    out["hit_first"] = f"{phase} t={t:.2f} {self._hit_names}" + (
                        f" ({t - abort['t0']:.2f} s after abort)" if abort else "")
                out["collision"] = True
                out["hit_force_max"] = max(out["hit_force_max"], fh)
            if mon is not None and phase != "abort":
                why = mon.check(self, phase, t, self._F_raw, out["grasped"] and phase == "hold", hit)
                if why and not out["detect_reason"]:
                    out["detect_reason"], out["t_detect"] = why, t
                if why and cfg.mon_act:
                    pc_, ps_ = kd.xpos[self.bid_target], kd.xpos[self.bid_chaser]
                    ar = AbortRef(ps_ - pc_, d.qvel[0:3] - d.qvel[14:17], mon.cfg.r_safe, mon.cfg.a_max)
                    pg0, Rg0 = self.site_pose(self.sid_grip)
                    pb0, _ = self.site_pose(self.sid_bar)
                    # retract direction: away from the bar along the approach axis (client
                    # -x, out of the grasp face) blended with away-from-the-bar
                    Rc_ = kd.xmat[self.bid_target].reshape(3, 3)
                    u = -Rc_[:, 0] + (pg0 - pb0) / max(np.linalg.norm(pg0 - pb0), 1e-6)
                    Rs_ = kd.xmat[self.bid_chaser].reshape(3, 3).copy()
                    R_hold = Rs_
                    if why == "thr_resid" and mon.cfg.reorient:
                        # isolate the faulty body axis (largest residual component) and turn
                        # the bus so that axis is normal to the back-off plane (span of the
                        # relative position and velocity): the healthy axes then carry it
                        k = int(np.argmax(np.abs(Rs_.T @ mon.resid_vec)))
                        r_, v_ = ps_ - pc_, d.qvel[0:3] - d.qvel[14:17]
                        nrm = np.cross(r_, v_)
                        if np.linalg.norm(nrm) < 1e-3 * np.linalg.norm(r_) * max(np.linalg.norm(v_), 1e-9):
                            nrm = np.cross(r_, Rs_[:, k])  # no velocity: any normal to r
                        nrm /= np.linalg.norm(nrm)
                        a_ = Rs_[:, k]
                        nrm = nrm if a_ @ nrm >= 0 else -nrm
                        ax_ = np.cross(a_, nrm)
                        ang = float(np.arctan2(np.linalg.norm(ax_), a_ @ nrm))
                        if np.linalg.norm(ax_) > 1e-9:
                            R_hold = Rotation.from_rotvec(ang * ax_ / np.linalg.norm(ax_)).as_matrix() @ Rs_
                        out["fault_axis_isolated"] = k
                    abort = dict(t0=t, ref=ar, q0=d.qpos[ARM_Q].copy(), cfg=mon.cfg,
                                 R0=R_hold,
                                 pg0=pg0, Rg0=Rg0, u_out=u / np.linalg.norm(u),
                                 retract=phase in ("reach", "approach", "close", "hold"))
                    out.update(aborted=True, abort_reason=why, abort_phase=phase, t_abort=t)
                    self._flyin_t = None
                    phase, t_phase = "abort", t
                    T_total = max(T_total, t + mon.cfg.abort_max_s + 1.0)
                    grip_cmd = self.grip_open.copy()
            # ---- arm torques
            if phase == "abort" and abort["retract"] and t - abort["t0"] < abort["cfg"].arm_hold_s:
                # first pull the gripper straight back from the bar, inertially (not
                # co-rotating: a kicked client's bar must not drag the jaws), while the bus
                # backs off; then stow from wherever the arm is
                ac = abort["cfg"]
                s = min((t - abort["t0"]) / ac.retract_s, 1.0)
                sh = 10 * s**3 - 15 * s**4 + 6 * s**5
                sd = (30 * s**2 - 60 * s**3 + 30 * s**4) / ac.retract_s
                p_des = abort["pg0"] + sh * ac.retract_m * abort["u_out"]
                v_ref = sd * ac.retract_m * abort["u_out"]
                tau = self.impedance_tau(p_des, abort["Rg0"], v_ref, np.zeros(3))
                abort["q_stow0"] = d.qpos[ARM_Q].copy()
            elif phase == "abort":  # minimum-jerk stow
                ac = abort["cfg"]
                abort["q0"] = abort.get("q_stow0", abort["q0"])
                s = min(max((t - abort["t0"] - ac.arm_hold_s) / ac.arm_stow_s, 0.0), 1.0)
                sh = 10 * s**3 - 15 * s**4 + 6 * s**5
                sd = (30 * s**2 - 60 * s**3 + 30 * s**4) / ac.arm_stow_s
                q_d = abort["q0"] + sh * (self.stow - abort["q0"])
                qd_d = sd * (self.stow - abort["q0"])
                tau = cfg.kq * (q_d - d.qpos[ARM_Q]) + cfg.dq * (qd_d - d.qvel[ARM])
            elif phase in ("flyin", "settle", "reach"):
                if phase in ("flyin", "settle"):
                    q_d, qd_d = self.stow, np.zeros(6)
                else:
                    s = min((t - t_phase) / cfg.reach_s, 1.0)
                    sh = 10 * s**3 - 15 * s**4 + 6 * s**5
                    sd = (30 * s**2 - 60 * s**3 + 30 * s**4) / cfg.reach_s
                    q_d = q_start + sh * (q_goal - q_start)
                    qd_d = sd * (q_goal - q_start)
                tau = cfg.kq * (q_d - d.qpos[ARM_Q]) + cfg.dq * (qd_d - d.qvel[ARM])
            else:
                Rc = kd.xmat[self.bid_target].reshape(3, 3)
                wc = Rc @ d.qvel[17:20]
                p_c = kd.xpos[self.bid_target]
                vc = d.qvel[14:17]
                if self.pose_est is not None:
                    p_c, Rc, vc, wc = self.pose_est.client(self)
                if phase == "approach":
                    tb = t - t_phase
                    approach_s = min(cfg.pre_contact,
                                     cfg.v_approach * max(tb - cfg.blend_s, 0.0))
                    p_des, R_des = self.grasp_target(cfg.pre_contact - approach_s)
                    v_ref = vc + np.cross(wc, p_des - p_c) + \
                        (cfg.v_approach if 0.0 < approach_s < cfg.pre_contact else 0.0) * Rc[:, 0]
                    if tb < cfg.blend_s:  # blend from where the grip point is (client frame)
                        sb = tb / cfg.blend_s
                        h = 10 * sb**3 - 15 * sb**4 + 6 * sb**5
                        p_b = p_c + Rc @ self.blend_rel_p
                        R_b = Rc @ self.blend_rel_R
                        p_des = p_b + h * (p_des - p_b)
                        R_des = Rotation.from_rotvec(h * _rotvec(R_des @ R_b.T)).as_matrix() @ R_b
                else:  # close and hold: keep the grip pose frozen at the close command
                    p_des = p_c + Rc @ self.hold_rel_p
                    R_des = Rc @ self.hold_rel_R
                    v_ref = vc + np.cross(wc, p_des - p_c)
                tau = self.impedance_tau(p_des, R_des, v_ref, wc)
                if phase == "approach":
                    p_g, _ = self.site_pose(self.sid_grip)
                    out["approach_err_max"] = max(out["approach_err_max"],
                                                  float(np.linalg.norm(p_g - p_des)))
            if self.step_hook is not None:  # P4 demonstration recorder (SPEC_4); None = no-op
                self.step_hook(locals())
            # ---- bus
            if phase == "abort":  # inertial back-off from the client centre, stop co-rotating
                r_ref, v_ref, a_ref = abort["ref"](t - abort["t0"])
                pc_, vc_ = kd.xpos[self.bid_target], d.qvel[14:17]
                ps_, vs_ = kd.xpos[self.bid_chaser], d.qvel[0:3]
                F = self.m_serv * a_ref - self.kp_pos * (ps_ - pc_ - r_ref) - self.kd_pos * (vs_ - vc_ - v_ref)
                self._F_raw = F
                F = np.clip(F, -self.f_max, self.f_max)
                Rs = kd.xmat[self.bid_chaser].reshape(3, 3)
                w_s = Rs @ d.qvel[3:6]
                T = self.att_torque(Rs, cfg.kp_att * _rotvec(abort["R0"] @ Rs.T) - cfg.kd_att * w_s)
            elif phase == "hold" and out["grasped"]:
                # captured: thrusters off (stack control is 3C); wheels damp the stack's
                # rotation relative to LVLH (world z at the orbit rate here)
                Rs = kd.xmat[self.bid_chaser].reshape(3, 3)
                w_s = Rs @ d.qvel[3:6]
                F = np.zeros(3)
                T = self.att_torque(Rs, -cfg.kd_att * (w_s - np.array([0.0, 0.0, self.n])))
                if cfg.mode == "free-floating":
                    T = np.zeros(3)
            elif cfg.mode == "free-floating" and phase not in ("flyin", "settle", "reach"):
                F, T = np.zeros(3), np.zeros(3)
            else:
                F, T = self.bus_ctrl()
                if phase == "flyin":
                    n_fly += 1
                    out["flyin_sat_frac"] += float(np.any(np.abs(F) >= self.f_max - 1e-4))
            self._F_cmd = np.asarray(F, float)  # commanded (what the FDI expects to see)
            d.ctrl[U_THR] = self._thrust_fault(F)
            d.ctrl[U_RW] = T
            f = self._fault
            if f is not None and f["active"] and f["kind"] == "squeeze_loss" and phase != "abort":
                grip_cmd = np.array([-f["mag"], f["mag"]])  # jaw drive loses most of its squeeze
            d.ctrl[U_ARM] = np.clip(tau, self.km.actuator_ctrlrange[6:12, 0],
                                    self.km.actuator_ctrlrange[6:12, 1])
            d.ctrl[U_GRIP] = grip_cmd
            # ---- load test: internal wrench, equal and opposite on client and servicer
            if cfg.load_kind and phase == "hold" and out["grasped"] \
                    and t - t_phase >= cfg.load_wait_s:
                Rc = kd.xmat[self.bid_target].reshape(3, 3)
                Rs = kd.xmat[self.bid_chaser].reshape(3, 3)
                p_g, R_g = self.site_pose(self.sid_grip)
                p_b, _ = self.site_pose(self.sid_bar)
                rel_g = R_g.T @ (p_b - p_g)
                R_rel = R_g.T @ Rc
                if load_base is None:
                    load_base = (rel_g, R_rel, t)
                L = min(cfg.load_rate * (t - load_base[2]), cfg.load_max)
                u = cfg.load_sign * Rc[:, cfg.load_axis]
                if cfg.load_kind == "force":
                    d.ctrl[U_TEST_CF], d.ctrl[U_TEST_SF] = L * u, -L * u
                else:
                    d.ctrl[U_TEST_CT], d.ctrl[U_TEST_ST] = Rc.T @ (L * u), Rs.T @ (-L * u)
                dp = float(np.linalg.norm(rel_g - load_base[0]) * 1000)
                dr = float(np.degrees(np.linalg.norm(_rotvec(R_rel @ load_base[1].T))))
                if self._slip_prev is not None:
                    a = cfg.ctrl_dt / 0.2
                    vp = np.linalg.norm(rel_g - self._slip_prev[0]) * 1000 / cfg.ctrl_dt
                    vr = np.degrees(np.linalg.norm(_rotvec(R_rel @ self._slip_prev[1].T))) / cfg.ctrl_dt
                    self._slip_v = (1 - a) * self._slip_v + a * np.array([vp, vr])
                self._slip_prev = (rel_g, R_rel)
                out["slip_pos_mm_max"] = max(out["slip_pos_mm_max"], dp)
                out["slip_rot_deg_max"] = max(out["slip_rot_deg_max"], dr)
                out["load_max_reached"] = L
                req = cfg.req_force if cfg.load_kind == "force" else cfg.req_torque
                if L <= req:
                    out["defl_mm_at_req"], out["defl_deg_at_req"] = dp, dr
                pad_bar, _, _ = self.contacts()
                if not np.isfinite(out["load_at_slip"]) and t - load_base[2] > 0.5:
                    kind = ("pad_lost" if len(pad_bar) < 2 else
                            "translate" if (self._slip_v[0] > cfg.slip_vel_mm_s
                                            or dp > cfg.slip_pos_mm) else
                            "rotate" if (self._slip_v[1] > cfg.slip_rate_deg_s
                                         or dr > cfg.slip_rot_deg) else "")
                    if kind:
                        out["load_at_slip"], out["slip_kind"] = L, kind
                lim = self.km.actuator_ctrlrange[6:12, 1]
                sat = np.abs(tau) >= 0.999 * lim
                if sat.any() and not np.isfinite(out["load_at_joint_sat"]):
                    out["load_at_joint_sat"] = L
                    out["sat_joint"] = int(np.argmax(sat))
            # ---- metrics
            if phase in ("approach", "close"):
                _, ftot, other = self.contacts()
                out["peak_force"] = max(out["peak_force"], ftot)
                out["other_contact"] |= other
                out["push_dv"] = max(out["push_dv"],
                                     float(np.linalg.norm(d.qvel[14:17] - vc0)))
            if Rs_ref0 is not None and phase != "hold":
                Rs = kd.xmat[self.bid_chaser].reshape(3, 3)
                out["bus_att_err_max"] = max(out["bus_att_err_max"],
                                             float(np.degrees(np.linalg.norm(_rotvec(Rs @ Rs_ref0.T)))))
            if record:
                p_g, _ = self.site_pose(self.sid_grip)
                p_b, _ = self.site_pose(self.sid_bar)
                trace.append((t, phase, p_g.copy(), p_b.copy(), d.qpos.copy(), d.qvel.copy()))
            for _ in range(nsub):
                mjo_step(self.model, d)
            t += cfg.ctrl_dt
        self.sync()
        out["phase_end"] = phase
        pad_bar, _, _ = self.contacts()
        out["both_pads_end"] = len(pad_bar) == 2
        p_g, _ = self.site_pose(self.sid_grip)
        p_b, R_b = self.site_pose(self.sid_bar)
        rel = p_g - p_b
        out["final_bar_dist"] = float(np.linalg.norm(rel - (rel @ R_b[:, 1]) * R_b[:, 1]))
        # slip: bar velocity relative to the grip point (a rigid stack's rotation cancels)
        v_g, _ = self.site_vel(self.sid_grip)
        v_b, _ = self.site_vel(self.sid_bar)
        out["final_rel_speed"] = float(np.linalg.norm(v_b - v_g))
        out["bar_shift_mm"] = (float(np.linalg.norm(rel - self._rel_at_grasp) * 1000)
                               if self._rel_at_grasp is not None else np.nan)
        out["success"] = bool(out["grasped"] and out["both_pads_end"]
                              and out["final_bar_dist"] < 0.02 and out["final_rel_speed"] < 0.002)
        if out["aborted"]:
            out["success"] = False
        f = self._fault
        if f is not None and f["t_fault"] is not None:
            out["t_fault"] = f["t_fault"]
        if mon is not None:
            out["mon_max"] = {k: float(v) for k, v in mon.max.items()}
            out["mon_max_by_phase"] = {p: {k: float(v) for k, v in dd.items()}
                                       for p, dd in mon.max_by_phase.items()}
            out["mon_first"] = {k: [float(v[0]), v[1]] for k, v in mon.first.items()}
        out["outcome"] = ("collision" if out["collision"] else
                          "captured" if out["success"] else
                          "safe_abort" if out["safe_reached"] else
                          "abort_unsafe" if out["aborted"] else "failed")
        if record:
            out["trace"] = trace
        return out


if __name__ == "__main__":
    sim = GraspSim(GraspCfg(mode=sys.argv[1] if len(sys.argv) > 1 else "free-flying"))
    res = sim.run(np.random.default_rng(0))
    print({k: v for k, v in res.items() if k != "trace"})
