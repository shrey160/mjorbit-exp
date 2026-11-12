"""3E step E2: safety monitors, abort actions and fault injection (SPEC_3 section 12).

Monitors run every control step alongside GraspSim (fly-in, settle, reach, approach, close,
hold) and return the first violated check:
  clear_pred   predicted free-drift clearance of the bus sphere to the client (box + bar
               envelope rotating at the measured client rate) over a horizon < 0
  track        bus position / attitude tracking error against its synchronised reference
  rate_anom    relative-attitude anomaly: the client's measured inertial angular velocity
               jumps by more than rate_anom_deg_s within rate_window_s (the ADRAS-J abort
               trigger); not during close, when our own jaws push the client
  thrust_sat   fraction of saturated thrust commands over a sliding window too high
  contact      unexpected contact: any servicer-client contact other than gripper-on-bar
  grip_slip    after capture: bar motion in the gripper frame (shift or slip speed) or a
               pad losing contact

Abort ("safe") action, the same in every phase, chosen so that it does not depend on the
client spinning as modelled:
  - jaws open at once (releases a failing grip);
  - the arm holds its joint pose for arm_hold_s (the bus backing off pulls the gripper
    clear of the bar), then a minimum-jerk move to the stowed pose;
  - the bus stops co-rotating and backs off along the line from the client centre to the
    servicer, to a point r_safe from the client centre, inertially still relative to the
    client's centre of mass. The path is a cubic Hermite from the current relative state
    (position and velocity) to that point, with duration chosen so the feed-forward
    acceleration stays inside the thrust limit. Outside the client's swept sphere
    (client radius + servicer radius) no client rotation can reach the servicer.
Safe state: bus beyond r_safe_reached from the client centre, arm stowed, jaws open, not
closing on the client.
"""

from __future__ import annotations

import dataclasses

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

# client geometry in its body frame (p3b_scene.xml): bus box and bar envelope (default only;
# the monitors use the GraspSim robot description, sim.boxes and sim.r_serv)
_BOXES = (((0.0, 0.0, 0.0), (0.75, 0.5, 0.5)), ((-0.8375, 0.0, 0.0), (0.0875, 0.175, 0.025)))
CLIENT_RADIUS = 1.10  # m, swept sphere of the client about its centre of mass (incl. bar)


def box_clearance(x_b, boxes=_BOXES):
    best = np.inf
    for c, h in boxes:
        q = np.abs(x_b - np.asarray(c)) - np.asarray(h)
        best = min(best, float(np.linalg.norm(np.maximum(q, 0.0)) + min(q.max(), 0.0)))
    return best


@dataclasses.dataclass
class MonitorCfg:
    enabled: bool = False  # compute the monitors
    act: bool = True  # abort when one trips (False: record would-be detections only)
    horizon_s: float = 30.0
    clear_pred_min: float = 0.0  # m, predicted clearance (servicer radius and margin included)
    track_pos_m: float = 0.25
    track_att_deg: float = 8.0
    rate_anom_deg_s: float = 0.1  # jump in the client rate within rate_window_s
    rate_window_s: float = 2.0
    sat_window_s: float = 5.0
    sat_frac: float = 0.6
    sat_abort: bool = False  # thrust saturation is recorded, not an abort trigger (E3)
    near_clear: float = 1.5  # m: track and thrust_sat only count this close to the client
    # thruster FDI: servicer centre-of-mass acceleration (accelerometer) vs commanded thrust
    resid_window_s: float = 1.0
    resid_N: float = 0.04
    slip_shift_mm: float = 6.0  # bar shift in the gripper frame beyond the value at grasp
    slip_vel_mm_s: float = 3.0
    pad_loss_s: float = 0.2  # a pad off the bar this long counts as grip loss
    contact_grace_s: float = 3.0  # after an abort, gripper-bar contacts while releasing
    # abort action
    reorient: bool = True  # thruster fault: turn the faulty axis out of the back-off plane
    r_safe: float = 6.0
    r_safe_reached: float = 3.2
    a_max: float = 0.0025  # m/s^2 feed-forward budget (0.75 N on 150 kg is 0.005)
    arm_hold_s: float = 8.0  # gripper retract phase, then stow
    retract_m: float = 0.25  # gripper pulled back this far from where it was, inertially
    retract_s: float = 2.0
    arm_stow_s: float = 15.0
    abort_max_s: float = 300.0
    safe_dwell_s: float = 10.0


@dataclasses.dataclass
class FaultCfg:
    kind: str = ""  # "", client_kick, thruster_weak, thruster_stuck, squeeze_loss, spin_axis_err
    phase: str = "approach"  # phase in which the fault starts (squeeze_loss: hold)
    t_in_phase: float = 5.0  # s after the phase starts
    mag: float = 1.0  # kick: deg/s; weak: remaining thrust fraction; stuck: N;
    # squeeze_loss: jaw target (m); spin_axis_err: deg
    axis: int = -1  # body axis for thruster faults (-1: random)


class Monitor:
    def __init__(self, cfg: MonitorCfg, ctrl_dt: float):
        self.cfg = cfg
        self.dt = ctrl_dt
        self.sat_hist = []
        self.max = {k: 0.0 for k in ("clear_pred_neg", "track_pos", "track_att", "rate_anom",
                                     "thrust_sat", "thr_resid", "slip_shift", "slip_vel")}
        self.resid_hist = []
        self.w_hist = []
        self.max_by_phase = {}
        self.first = {}  # check -> (t, phase) of first violation
        self.pad_off_t = 0.0
        self.slip_prev = None
        self.slip_v = 0.0
        self.base_rel = None

    def _note(self, key, val, phase):
        self.max[key] = max(self.max[key], val)
        d = self.max_by_phase.setdefault(phase, {})
        d[key] = max(d.get(key, 0.0), val)

    def predicted_clearance(self, sim):
        """Minimum over the horizon of the bus-sphere clearance if the bus coasts from now,
        with the client rotating at its measured rate (straight-line relative drift; CW
        terms are negligible over 30 s)."""
        kd, d = sim.kd, sim.data
        pc, ps = kd.xpos[sim.bid_target], kd.xpos[sim.bid_chaser]
        Rc = kd.xmat[sim.bid_target].reshape(3, 3)
        w = Rc @ d.qvel[17:20]
        r0 = ps - pc
        v = d.qvel[0:3] - d.qvel[14:17]
        wn = np.linalg.norm(w)
        best = np.inf
        for tau in np.arange(0.0, self.cfg.horizon_s + 1e-9, 1.0):
            Rt = Rotation.from_rotvec(w * tau).as_matrix() if wn > 0 else np.eye(3)
            x_b = (Rt @ Rc).T @ (r0 + v * tau)
            best = min(best, box_clearance(x_b, sim.boxes))
        return best - sim.r_serv - sim.cfg.clear_margin

    def check(self, sim, phase, t, F_raw, grasped, hits):
        """Return the name of the first violated check, or ''. Also records signal maxima."""
        c = self.cfg
        trips = []
        kd, d = sim.kd, sim.data
        Rc = kd.xmat[sim.bid_target].reshape(3, 3)
        if phase in ("flyin", "settle", "reach", "approach", "close") and not grasped:
            cp = self.predicted_clearance(sim)
            self._note("clear_pred_neg", max(-cp, 0.0), phase)
            if cp < c.clear_pred_min:
                trips.append("clear_pred")
            near = sim.clearance_now() < c.near_clear
            # thruster FDI (no contact: then the external force on the servicer is the thrust)
            F_cmd = getattr(sim, "_F_cmd", None)
            _, ftot, _ = sim.contacts()
            if F_cmd is not None and ftot == 0.0 and phase != "close":
                mujoco.mj_subtreeVel(sim.km, kd)
                self.resid_hist.append((kd.subtree_linvel[sim.bid_chaser].copy(), F_cmd.copy()))
                n = int(round(c.resid_window_s / self.dt))
                self.resid_hist = self.resid_hist[-(n + 1):]
                if len(self.resid_hist) == n + 1:
                    dv = self.resid_hist[-1][0] - self.resid_hist[0][0]
                    Fm = np.mean([h[1] for h in self.resid_hist[1:]], axis=0)
                    rv = sim.m_serv * dv / (n * self.dt) - Fm
                    res = float(np.linalg.norm(rv))
                    self.resid_vec = rv  # world frame; isolates the faulty thruster axis
                    self._note("thr_resid", res, phase)
                    if res > c.resid_N:
                        trips.append("thr_resid")
            else:
                self.resid_hist = []
            ref = getattr(sim, "_bus_ref", None)
            self._ep = 0.0
            if ref is not None and near:
                p_ref, v_ref, R_ref = ref
                ep = float(np.linalg.norm(kd.xpos[sim.bid_chaser] - p_ref))
                self._ep = ep
                Rs = kd.xmat[sim.bid_chaser].reshape(3, 3)
                ea = float(np.degrees(np.linalg.norm(Rotation.from_matrix(R_ref @ Rs.T).as_rotvec())))
                self._note("track_pos", ep, phase)
                self._note("track_att", ea, phase)
                if ep > c.track_pos_m or ea > c.track_att_deg:
                    trips.append("track")
            if F_raw is not None:
                self.sat_hist.append(float(np.any(np.abs(F_raw) >= 0.7499)))
                n = int(c.sat_window_s / self.dt)
                self.sat_hist = self.sat_hist[-n:]
                frac = sum(self.sat_hist) / n
                self._note("thrust_sat", frac, phase)
                # saturation alone is a nuisance (transients that recover); it counts when the
                # bus is near the client and its tracking is also degrading
                # E3: record-only by default. Near 2 deg/s the servicer runs at its thrust limit
                # by design; a nutating client tripped this with clearance still >= 0.58 m
                # and the mission then succeeded. Clearance, tracking and thruster FDI cover
                # the hazards.
                if c.sat_abort and frac > c.sat_frac and near and self._ep > 0.5 * c.track_pos_m:
                    trips.append("thrust_sat")
        w = Rc @ d.qvel[17:20]
        # rate anomaly = a jump in the client's inertial angular velocity over rate_window_s.
        # A client spinning about a non-principal axis nutates (its angular velocity drifts
        # slowly even torque-free; only the momentum is constant), so comparing against the
        # rate at the start false-alarms; a kick is a step, ~1000x faster than that drift.
        self.w_hist.append(w.copy())
        nw = int(round(c.rate_window_s / self.dt))
        self.w_hist = self.w_hist[-(nw + 1):]
        ra = float(np.degrees(np.linalg.norm(w - self.w_hist[0])))
        if not grasped and phase != "close":  # closing the jaws pushes the client on purpose
            self._note("rate_anom", ra, phase)
            if ra > c.rate_anom_deg_s:
                trips.append("rate_anom")
        if hits:
            trips.append("contact")
        if grasped and phase == "hold":
            p_g, R_g = sim.site_pose(sim.sid_grip)
            p_b, _ = sim.site_pose(sim.sid_bar)
            rel = R_g.T @ (p_b - p_g)
            if self.base_rel is None:
                self.base_rel = rel
            shift = float(np.linalg.norm(rel - self.base_rel) * 1000)
            if self.slip_prev is not None:
                a = self.dt / 0.2
                vp = float(np.linalg.norm(rel - self.slip_prev) * 1000 / self.dt)
                self.slip_v = (1 - a) * self.slip_v + a * vp
            self.slip_prev = rel
            self._note("slip_shift", shift, phase)
            self._note("slip_vel", self.slip_v, phase)
            pad_bar, _, _ = sim.contacts()
            self.pad_off_t = self.pad_off_t + self.dt if len(pad_bar) < 2 else 0.0
            if shift > c.slip_shift_mm or self.slip_v > c.slip_vel_mm_s or self.pad_off_t >= c.pad_loss_s:
                trips.append("grip_slip")
        for k in trips:
            self.first.setdefault(k, (t, phase))
        return trips[0] if trips else ""


class AbortRef:
    """Inertial back-off reference relative to the client centre of mass (cubic Hermite)."""

    def __init__(self, r0, v0, r_safe, a_max):
        u = r0 / max(np.linalg.norm(r0), 1e-6)
        self.r0, self.v0 = r0.copy(), v0.copy()
        self.rf = r_safe * u if np.linalg.norm(r0) < r_safe else r0.copy()
        D = self.rf - self.r0
        for T in np.arange(20.0, 1000.0, 5.0):
            a0 = (6 * D - 4 * self.v0 * T) / T**2
            a1 = (-6 * D + 2 * self.v0 * T) / T**2
            if max(np.linalg.norm(a0), np.linalg.norm(a1)) <= a_max:
                break
        self.T = float(T)

    def __call__(self, t):
        T = self.T
        s = min(max(t / T, 0.0), 1.0)
        h00, h10, h01 = 2 * s**3 - 3 * s**2 + 1, s**3 - 2 * s**2 + s, -2 * s**3 + 3 * s**2
        d00, d10, d01 = (6 * s**2 - 6 * s) / T, (3 * s**2 - 4 * s + 1), (-6 * s**2 + 6 * s) / T
        a00, a10, a01 = (12 * s - 6) / T**2, (6 * s - 4) / T, (-12 * s + 6) / T**2
        if t >= T:
            return self.rf.copy(), np.zeros(3), np.zeros(3)
        r = h00 * self.r0 + h10 * T * self.v0 + h01 * self.rf
        v = d00 * self.r0 + d10 * self.v0 + d01 * self.rf
        a = a00 * self.r0 + a10 * self.v0 + a01 * self.rf
        return r, v, a
