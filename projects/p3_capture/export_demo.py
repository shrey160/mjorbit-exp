"""Full demo mission for the replay page (13 Nov), from export_mission.py, one case:

  far    the far approach from about 30 m to the hold point 8 m out (demo_stage_a.py record)
  flyin .. hold   fly-in and the grasp, the bar pose from the servicer's cameras: the P4.3
         estimate for the reach and the palm-camera correction for the last 12 cm
         (pose_est.PalmPoseEst), camera mounts off by up to 10 mm and 1 deg
  brakes, detumble, id, move   3C and 3D as in E3 (stack_sim, maneuver3d), unchanged
  release, retreat   NEW, demo only: brakes off, jaws open, the gripper pulls back, the bus
         backs off while the arm stows. Reported: the client's rate and speed change, contact.

Still read from the simulator's truth (stated on the page): the bus's station keeping and
fly-in use the client's true relative state and spin; so do the clearance and rate monitors.
The frozen p3_capture modules are not edited (hooks are patched in here, as export_mission.py).

    MUJOCO_GL=egl python export_demo.py <stage_a.json> <out.json> [mount_mm=10] [backoff_m=3]
    ... --release-only [--print]   re-run only the release from replay/demo_ckpt.pkl (saved by a full run)
"""
from __future__ import annotations

import dataclasses
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "p2_approach"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "p4_vision"))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402

import grasp_sim  # noqa: E402
import stack_sim  # noqa: E402
from grasp_sim import ARM, ARM_Q, U_ARM, U_GRIP, U_RW, U_THR, _rotvec  # noqa: E402
from maneuver3d import Man3DCfg, Stack3D  # noqa: E402
from stack_sim import StackCfg  # noqa: E402

A = json.loads(Path(sys.argv[1]).read_text())
out = sys.argv[2]
_pos = [a for a in sys.argv[1:] if not a.startswith("--")]
mount_mm = float(_pos[2]) if len(_pos) > 2 else 10.0
backoff_m = float(_pos[3]) if len(_pos) > 3 else 3.0
seed, c = A["seed"], A["case"]
scene = "p4_espa_arm2_cam_scene.xml"
D = "/home/shrey31/p4_data"
gfps, sfps = 4.0, 1.0

REC = dict(grasp=None, stack=[], n=0, md={}, rad=[])


def radial(sim):
    """Radial-out unit vector in the simulation world frame (LVLH x)."""
    d = sim.data
    v = d.world_position_from_lvlh([1.0, 0.0, 0.0]) - d.world_position_from_lvlh([0.0, 0.0, 0.0])
    return v / np.linalg.norm(v)


def _ghook(loc):
    if int(round(loc["t"] / 0.01)) % 25 == 0:
        REC["rad"].append((float(loc["t"]), radial(loc["self"])))


_orig_run = grasp_sim.GraspSim.run


def _run(self, rng=None, record=False, **kw):  # record the grasp trace (first call only)
    first = REC["grasp"] is None
    if first:
        self.step_hook = _ghook
    r = _orig_run(self, rng, record=first, **kw)
    if first:
        REC["grasp"] = (self, r)
    return r


grasp_sim.GraspSim.run = _run
_orig_step = stack_sim.StackRun._step
SDIV = max(1, int(round(1.0 / (sfps * 0.01))))


def _step(self, *a, **k):
    res = _orig_step(self, *a, **k)
    REC["n"] += 1
    if REC["n"] % SDIV == 0:
        REC["stack"].append((self.t, getattr(self, "phase_tag", "brakes"), self.sim, self.sim.data.qpos.copy(), radial(self.sim)))
    return res


stack_sim.StackRun._step = _step
_orig_ci = Stack3D.capture_identify


def _ci(self):
    r = _orig_ci(self)
    self.phase_tag = "move"
    return r


Stack3D.capture_identify = _ci

# ---------------------------------------------------------------- release and retreat (demo only)
def release(ck, backoff_m, t_open=6.0, t_retract=6.0, retract_m=0.20, t_back=None, t_stow=20.0, t_end=15.0):
    """After the move: rebuild the unlocked model (arm and jaw brakes off), open the jaws,
    pull the gripper straight back, then back the bus off along the line from the client
    while the arm stows. The bus holds its attitude throughout and keeps its place relative
    to the client until the back-off starts. Returns (frames, metrics)."""
    path = Path(__file__).with_name(".demo_release.xml")
    path.write_text(ck["xml"])
    new = grasp_sim.GraspSim(dataclasses.replace(ck["cfg"], scene=path.name))
    new.data.reset(stack_sim.OrbitInit(R_eci=ck["R_eci"], V_eci=ck["V_eci"], t=ck["t_orbit"]))
    new.data.qpos[:] = ck["qpos"]
    new.data.qvel[:] = ck["qvel"]
    new.data.ctrl[:] = ck["ctrl"]
    stack_sim.mjo_forward(new.model, new.data)
    sim = new
    sim.sync()
    d, kd, cfg = sim.data, sim.kd, sim.cfg
    dt, nsub = cfg.ctrl_dt, int(round(cfg.ctrl_dt / cfg.dt))
    bc, bs = sim.bid_target, sim.bid_chaser
    # thrust-limited back-off time: minimum-jerk peak acceleration 5.77 D / T^2 at 60% of the limit
    a_max = 0.6 * sim.f_max / sim.m_serv
    t_back = float(np.sqrt(5.7735 * backoff_m / a_max)) if t_back is None else t_back
    q_hold = d.qpos[ARM_Q].copy()
    Rs0 = kd.xmat[bs].reshape(3, 3).copy()
    r0 = kd.xpos[bs] - kd.xpos[bc]
    u_back = r0 / np.linalg.norm(r0)
    pg0, Rg0 = sim.site_pose(sim.sid_grip)
    u_ret = -kd.xmat[bc].reshape(3, 3)[:, 0]  # out of the grasp face, along the approach axis
    vc0 = d.qvel[14:17].copy()
    g_closed = d.ctrl[U_GRIP].copy()  # the squeeze command carried over from the grasp
    Rc_ = kd.xmat[bc].reshape(3, 3)
    wc0 = Rc_ @ d.qvel[17:20]
    T1, T2, T3 = t_open, t_open + t_retract, t_open + t_retract + t_back
    T4 = T3 + t_end
    frames, t, q_stow0 = [], 0.0, None
    dv_nat = np.zeros(3)
    m = dict(hit=False, hit_force=0.0, hit_first="", dv_client_max=0.0, dw_client_max=0.0, sat_frac=0.0, n=0,
             min_grip_bar_after_retract=np.inf, t_back=t_back)
    mj = lambda s: (10 * s**3 - 15 * s**4 + 6 * s**5, 30 * s**2 - 60 * s**3 + 30 * s**4, 60 * s - 180 * s**2 + 120 * s**3)  # noqa: E731
    k = 0
    while t < T4:
        sim.sync()
        ph = "release" if t < T2 else "retreat"
        pc, ps = kd.xpos[bc], kd.xpos[bs]
        vc, vs = d.qvel[14:17], d.qvel[0:3]
        # ---- arm
        if t < T1:  # hold the joints while the jaws open
            tau = cfg.kq * (q_hold - d.qpos[ARM_Q]) - cfg.dq * d.qvel[ARM]
        elif t < T2:  # pull the gripper straight back, inertially
            s, sd, _ = mj((t - T1) / t_retract)
            tau = sim.impedance_tau(pg0 + s * retract_m * u_ret, Rg0, sd / t_retract * retract_m * u_ret, np.zeros(3))
            q_stow0 = d.qpos[ARM_Q].copy()
        else:  # minimum-jerk stow
            s, sd, _ = mj(min((t - T2) / t_stow, 1.0))
            q_d = q_stow0 + s * (sim.stow - q_stow0)
            qd_d = (sd / t_stow) * (sim.stow - q_stow0) if t - T2 < t_stow else np.zeros(6)
            tau = cfg.kq * (q_d - d.qpos[ARM_Q]) + cfg.dq * (qd_d - d.qvel[ARM])
        # ---- bus: keep the place beside the client, then back off along the line from it
        if t < T1:
            # jaws opening: no thrust (a position loop would push the client through the grip);
            # the place beside the client is re-taken as the reference once the jaws are open
            r0 = ps - pc
            u_back = r0 / np.linalg.norm(r0)
            r_ref, v_ref, a_ref = r0, vs - vc, np.zeros(3)
        elif t < T2:
            r_ref, v_ref, a_ref = r0, np.zeros(3), np.zeros(3)
        else:
            s, sd, sdd = mj(min((t - T2) / t_back, 1.0))
            live = t - T2 < t_back
            r_ref = r0 + s * backoff_m * u_back
            v_ref = (sd / t_back) * backoff_m * u_back if live else np.zeros(3)
            a_ref = (sdd / t_back**2) * backoff_m * u_back if live else np.zeros(3)
        F = sim.m_serv * a_ref - sim.kp_pos * (ps - pc - r_ref) - sim.kd_pos * (vs - vc - v_ref)
        m["sat_frac"] += float(np.any(np.abs(F) >= sim.f_max - 1e-4))
        m["n"] += 1
        F = np.clip(F, -sim.f_max, sim.f_max)
        Rs = kd.xmat[bs].reshape(3, 3)
        w_s = Rs @ d.qvel[3:6]
        # attitude fixed in LVLH, as the stack was left by 3D (LVLH turns about world z at the
        # orbit rate): an inertial hold would brake that rate through the grip and turn the client
        w_ref = np.array([0.0, 0.0, sim.n])
        R_ref = Rotation.from_rotvec(w_ref * t).as_matrix() @ Rs0
        T = sim.att_torque(Rs, cfg.kp_att * _rotvec(R_ref @ Rs.T) - cfg.kd_att * (w_s - w_ref))
        d.ctrl[U_THR] = F
        d.ctrl[U_RW] = T
        d.ctrl[U_ARM] = np.clip(tau, sim.km.actuator_ctrlrange[6:12, 0], sim.km.actuator_ctrlrange[6:12, 1])
        # jaws: ease the squeeze off and open over t_open (a step release kicked the client by 2 mm/s)
        d.ctrl[U_GRIP] = g_closed + mj(min(t / t_open, 1.0))[0] * (sim.grip_open - g_closed)
        # ---- checks
        hit, fh = sim.unintended_hits(allow_grip_bar=t < T1 + 3.0)
        if hit and not m["hit"]:
            m["hit_first"] = f"t={t:.2f} {getattr(sim, '_hit_names', '')}"
        m["hit"] |= bool(hit)
        m["hit_force"] = max(m["hit_force"], float(fh))
        Rc = kd.xmat[bc].reshape(3, 3)
        # the client's speed change beyond free orbital motion: in this inertially oriented frame a
        # body r from the orbit's reference point accelerates by -n^2 (r - 3 (r.e) e), e = radial
        # (about 2 mm/s per 100 s at 14 m, which is not a push)
        e_r = radial(sim)
        dv_nat += -sim.n**2 * (pc - 3.0 * (pc @ e_r) * e_r) * dt
        dvx = float(np.linalg.norm(vc - vc0 - dv_nat))
        m["dv_client_max"] = max(m["dv_client_max"], dvx)
        if abs(t - T1) < 0.5 * dt:
            m["dv_client_open_mm_s"] = dvx * 1000
        m["dw_client_max"] = max(m["dw_client_max"], float(np.linalg.norm(Rc @ d.qvel[17:20] - wc0)))
        if t > T2:
            pg, _ = sim.site_pose(sim.sid_grip)
            pb, _ = sim.site_pose(sim.sid_bar)
            m["min_grip_bar_after_retract"] = min(m["min_grip_bar_after_retract"], float(np.linalg.norm(pg - pb)))
        if k % (25 if t < T2 else 100) == 0:
            frames.append((t, ph, d.qpos.copy(), radial(sim)))
        for _ in range(nsub):
            stack_sim.mjo_step(sim.model, d)
        t += dt
        k += 1
    sim.sync()
    Rc = kd.xmat[bc].reshape(3, 3)
    frames.append((t, "retreat", d.qpos.copy(), radial(sim)))
    m.update(sep_start_m=float(np.linalg.norm(r0)), sep_end_m=float(np.linalg.norm(kd.xpos[bs] - kd.xpos[bc])),
             dv_client_end_mm_s=float(np.linalg.norm(d.qvel[14:17] - vc0 - dv_nat) * 1000),
             dv_client_raw_mm_s=float(np.linalg.norm(d.qvel[14:17] - vc0) * 1000),
             dw_client_end_deg_s=float(np.degrees(np.linalg.norm(Rc @ d.qvel[17:20] - wc0))),
             stow_err_deg=float(np.degrees(np.max(np.abs(d.qpos[ARM_Q] - sim.stow)))), sat_frac=m["sat_frac"] / max(m["n"], 1),
             dv_client_max_mm_s=m.pop("dv_client_max") * 1000, dw_client_max_deg_s=float(np.degrees(m.pop("dw_client_max"))),
             duration_s=t, t_open=T1, t_retract_end=T2)
    m.pop("n")
    return sim, frames, {k_: (round(v, 4) if isinstance(v, float) else v) for k_, v in m.items()}


def turn_far(frames, meta):
    """The far approach was recorded in LVLH, which turns with the orbit, and first exported as if
    that frame stood still (constant direction to Earth): the pair looked parked in orbit until the
    fly-in. Here every far frame is rotated about the client centre by the orbit's rotation back
    from the hold time: positions, attitudes and the radial vector together. The rotation axis and
    rate come from the radial vectors recorded during the fly-in."""
    if meta["demo"].get("far_turned"):
        return
    far = [f for f in frames if f["ph"] == "far"]
    rest = [f for f in frames if f["ph"] != "far"]
    if not far or len(rest) < 2:
        return
    a = rest[0]
    b = next((f for f in rest if f["t"] - a["t"] >= 100.0), rest[-1])
    ra, rb = np.array(a["r"], float), np.array(b["r"], float)
    ax = np.cross(ra, rb)
    ang = float(np.arctan2(np.linalg.norm(ax), ra @ rb))
    ax /= np.linalg.norm(ax)
    rate = ang / (b["t"] - a["t"])
    TA = far[-1]["t"]
    for f in far:
        Q = Rotation.from_rotvec(ax * rate * (f["t"] - TA))
        x = np.array(f["x"], float).reshape(-1, 7)
        pos = Q.apply(x[:, :3])
        qw = x[:, 3:7]
        quat = (Q * Rotation.from_quat(qw[:, [1, 2, 3, 0]])).as_quat()[:, [3, 0, 1, 2]]
        out = []
        for p_, q_ in zip(pos, quat):
            out.extend([round(float(v), 4) for v in p_] + [round(float(v), 5) for v in q_])
        f["x"] = out
        f["r"] = [round(float(v), 4) for v in Q.apply(ra)]
    meta["demo"]["far_turned"] = True
    meta["demo"]["orbit_rate_deg_s"] = round(float(np.degrees(rate)), 4)
    print(f"far frames turned with the orbit: {np.degrees(rate):.4f} deg/s about {np.round(ax, 3)}, "
          f"{np.degrees(rate * (TA - far[0]['t'])):.1f} deg over the far approach")


def finish(ck, out):
    """Run the release from the saved end-of-mission state and append it to the replay data."""
    part = ck["part"]
    frames, ev, meta = part["frames"], part["events"], part["meta"]
    turn_far(frames, meta)
    rsim, rframes, rel = release(ck, ck["backoff_m"])
    print("release:", json.dumps(rel))
    md = mujoco.MjData(rsim.km)
    T_rel = frames[-1]["t"]
    ev.append(dict(t=round(T_rel, 1), what="at the commanded place and attitude: brakes off, jaws ease open"))
    ev.append(dict(t=round(T_rel + rel["t_open"], 1), what="gripper pulls back from the bar"))
    ev.append(dict(t=round(T_rel + rel["t_retract_end"], 1), what=f"servicer backs away {ck['backoff_m']:.0f} m and stows its arm"))
    for (ts, ph, q, rt) in rframes:
        if ts <= 0.0:
            continue
        md.qpos[:] = q
        mujoco.mj_kinematics(rsim.km, md)
        x = []
        for b in ck["bodies"]:
            x.extend([round(float(v), 4) for v in md.xpos[b] - ck["origin"]] + [round(float(v), 5) for v in md.xquat[b]])
        frames.append(dict(t=round(T_rel + float(ts), 2), ph=ph, x=x, r=[round(float(a), 4) for a in rt]))
    ev.append(dict(t=frames[-1]["t"], what="done"))
    meta["demo"]["release"] = rel
    Path(out).write_text(json.dumps(part, separators=(",", ":"), default=float))
    print("frames", len(frames), "T", frames[-1]["t"], [e["what"][:40] for e in ev])


CKPT = Path(__file__).with_name("replay") / "demo_ckpt.pkl"
if "--release-only" in sys.argv:  # iterate on the release without re-flying the mission
    import pickle
    ck = pickle.loads(CKPT.read_bytes())
    if "--print" in sys.argv:
        _, _, rel_ = release(ck, ck["backoff_m"])
        print("release:", json.dumps(rel_))
    else:
        finish(ck, out)
    sys.exit(0)

# ---------------------------------------------------------------- mission with the vision grasp
scfg = StackCfg(spin_deg_s=c["spin"], spin_axis=c["axis"], impose_tumble=False, grip_mon=True,
                grip_response="reclamp", scene=scene)
sr = Stack3D(seed, scfg, Man3DCfg(), grasp_kw=dict(flyin=True, mon=True))
sr.grasp_reset_kw = dict(hold_dp_b=A["dp_b"], hold_dv_b=A["dv_b"])
from cams import CamRig  # noqa: E402
from pose_est import PalmPoseEst  # noqa: E402

rig = CamRig(sr.sim.km, mount_m=mount_mm / 1000, mount_deg=mount_mm / 10)
dr = rig.randomize(np.random.default_rng(seed + 12345))
est = PalmPoseEst(f"{D}/pose_v3/best.pt", f"{D}/palm_net_v2/best.pt", rig, use_rot=False)
est.reset()
sr.sim.pose_est = est
res = sr.run3d()
gsim, g = REC["grasp"]
L = est.log
vis = dict(mount_mm=mount_mm, palm_used=est.n_palm, palm_rejected=est.n_rej,
           est_at_close_mm=round(L[-1][2] * 1000, 1) if L else None,
           est_approach_mm_med=round(float(np.median([x[2] for x in L if x[1] == "approach"])) * 1000, 1) if L else None,
           push_mm_s=round(float(g["push_dv"]) * 1000, 2), bar_shift_mm=round(float(g.get("bar_shift_mm", np.nan)), 2),
           sun_I=round(dr["sun_I"], 2))
print("grasp:", g.get("outcome"), "aborted", g.get("aborted"), vis)
print("mission:", {k: res.get(k) for k in ("success", "final_pos_err_m", "final_att_err_deg", "mass_err_pct", "dr_m", "att_cmd_deg")})


ok_mission = bool(res.get("success"))

# ---------------------------------------------------------------- replay JSON
km = gsim.km
G = mujoco.mjtGeom
tmap = {G.mjGEOM_BOX: "box", G.mjGEOM_SPHERE: "sphere", G.mjGEOM_CAPSULE: "capsule",
        G.mjGEOM_CYLINDER: "cylinder", G.mjGEOM_ELLIPSOID: "ellipsoid"}
geoms = []
for gi in range(km.ngeom):
    t_ = tmap.get(int(km.geom_type[gi]))
    rgba = km.geom_rgba[gi]
    if t_ is None or rgba[3] <= 0.0:
        continue
    geoms.append(dict(b=int(km.geom_bodyid[gi]), t=t_, s=[round(float(x), 4) for x in km.geom_size[gi]],
                      p=[round(float(x), 4) for x in km.geom_pos[gi]], q=[round(float(x), 5) for x in km.geom_quat[gi]],
                      c=[round(float(x), 3) for x in rgba], n=mujoco.mj_id2name(km, mujoco.mjtObj.mjOBJ_GEOM, gi) or ""))
bodies = sorted({x["b"] for x in geoms})
bt, bsv = gsim.bid_target, gsim.bid_chaser


def poses(model, qpos, origin=None):
    md = REC["md"].setdefault(id(model), mujoco.MjData(model))
    md.qpos[:] = qpos
    mujoco.mj_kinematics(model, md)
    o = md.xpos[bt].copy() if origin is None else origin
    x = []
    for b in bodies:
        x.extend([round(float(v), 4) for v in md.xpos[b] - o] + [round(float(v), 5) for v in md.xquat[b]])
    return x, md.xpos[bt].copy()


r4 = lambda v: [round(float(a), 4) for a in v]  # noqa: E731
frames, ev = [], []
trace = g["trace"]
# ---- far approach, mapped into the grasp simulation's world so the join is continuous: the
# client's attitude at the last far frame equals its attitude at the first grasp frame
q0 = np.array(trace[0][4], float)
ac = int(km.jnt_qposadr[km.body_jntadr[bt]])
asv = int(km.jnt_qposadr[km.body_jntadr[bsv]])
quat_R = lambda q: Rotation.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()  # noqa: E731
Rc_g0, pc0 = quat_R(q0[ac + 3:ac + 7]), q0[ac:ac + 3].copy()
rows = A["rows"]
M = Rc_g0 @ np.array(rows[-1]["R"]).T
# the two simulations place the hold point slightly differently (0.4 m in the first export): the
# difference is blended in linearly over the far approach, so the path stays continuous
gap = q0[asv:asv + 3] - (pc0 + M @ np.array(rows[-1]["p"]))
TA = float(rows[-1]["t"])
rad0 = REC["rad"][0][1] if REC["rad"] else np.array([1.0, 0, 0])
for r in rows:
    q = q0.copy()
    q[asv:asv + 3] = pc0 + M @ np.array(r["p"]) + (r["t"] / float(rows[-1]["t"])) * gap
    x_, y_, z_, w_ = Rotation.from_matrix(M @ np.array(r["R"])).as_quat()
    q[ac + 3:ac + 7] = [w_, x_, y_, z_]
    x, _ = poses(km, q)
    frames.append(dict(t=round(float(r["t"]), 2), ph="far", x=x, r=r4(rad0)))
join_gap = float(np.linalg.norm(gap))
ev.append(dict(t=0.0, what=f"far approach from {np.linalg.norm(rows[0]['p']):.0f} m, on filtered relative-navigation measurements"))
ev.append(dict(t=round(TA, 1), what="at the hold point 8 m out on the client's spin axis; fly-in starts"))
# ---- fly-in and grasp
step = max(1, int(round(1.0 / (gfps * gsim.cfg.ctrl_dt))))
for i in list(range(0, len(trace), step)) + [len(trace) - 1]:
    t, ph = trace[i][0], trace[i][1]
    x, cpos = poses(km, trace[i][4])
    rt = REC["rad"][int(np.argmin([abs(a - t) for a, _ in REC["rad"]]))][1] if REC["rad"] else rad0
    frames.append(dict(t=round(TA + float(t), 2), ph=ph, x=x, r=r4(rt)))
T_cap = TA + float(trace[-1][0])
origin = cpos  # client centre at the end of the grasp run: later frames are relative to this fixed point
first = {}
for tr in trace:
    first.setdefault(tr[1], float(tr[0]))
if "reach" in first:
    ev.append(dict(t=round(TA + first["reach"], 1), what="arm reaches out; bar pose from the wrist and bus cameras and the lidar"))
if "approach" in first:
    ev.append(dict(t=round(TA + first["approach"], 1), what="last 12 cm: the palm camera centres the bar between the jaws"))
if np.isfinite(g.get("t_grasp", np.nan)):
    ev.append(dict(t=round(TA + float(g["t_grasp"]), 1), what="jaws closed on the bar"))
# ---- 3C and 3D
last_ph = None
for (ts, ph, s, q, rt) in REC["stack"]:
    x, _ = poses(s.km, q, origin)
    tt = round(T_cap + float(ts), 2)
    if tt <= frames[-1]["t"]:
        continue
    if ph != last_ph:
        ev.append(dict(t=tt, what={"brakes": "arm brakes on", "detumble": "brakes on, detumbling the stack",
                                   "id": "identifying mass, CoM and inertia with thruster pulses",
                                   "move": "moving and turning the stack"}.get(ph, ph)))
        last_ph = ph
    frames.append(dict(t=tt, ph=ph, x=x, r=r4(rt)))
nm = lambda b: mujoco.mj_id2name(km, mujoco.mjtObj.mjOBJ_BODY, b)  # noqa: E731
num = lambda v: (round(float(v), 4) if isinstance(v, (int, float, np.floating)) else v)  # noqa: E731
meta = dict(altitude_km=float(gsim.cfg.altitude_km), seed=seed, spin_deg_s=round(float(c["spin"]), 2), fault="", scene=scene,
            outcome="captured" if g.get("grasped") else str(g.get("outcome")),
            flyin_s=round(float(g.get("flyin_s", 0.0)), 1), min_clear=round(float(g["min_clear"]), 3), t_to_safe=None,
            bodies=[nm(b) for b in bodies], client_body=bodies.index(bt), servicer_body=bodies.index(bsv),
            mission={k: num(res.get(k)) for k in ("dr_m", "att_cmd_deg", "final_pos_err_m", "final_att_err_deg", "mass_err_pct",
                                                 "I_err_rel_pct", "rcs_prop_g", "success")},
            demo=dict(start_m=round(float(np.linalg.norm(rows[0]["p"])), 1), far_s=TA, far_arrival_err_cm=round(rows[-1]["err"] * 100, 1),
                      t_capture=round(T_cap, 1), vision=vis, release=None, join_gap_m=round(join_gap, 3)))
part = dict(meta=meta, geoms=[{**x, "b": bodies.index(x["b"])} for x in geoms], events=ev, frames=frames)
print("mission frames", len(frames), "geoms", len(geoms), "T", frames[-1]["t"], "join gap m", round(join_gap, 3))
print(json.dumps(meta["mission"]))
if not ok_mission:
    ev.append(dict(t=frames[-1]["t"], what="stopped"))
    Path(out).write_text(json.dumps(part, separators=(",", ":"), default=float))
    print("mission did not finish with success: no release phase")
    sys.exit(0)
import pickle  # noqa: E402

old = sr.sim
o = old.data.orbit
ck = dict(xml=(Path(__file__).parent / sr._base_scene).read_text(), cfg=dataclasses.replace(old.cfg, scene=sr._base_scene),
          R_eci=np.array(o.R_eci, float), V_eci=np.array(o.V_eci, float), t_orbit=float(o.t),
          qpos=np.array(old.data.qpos, float), qvel=np.array(old.data.qvel, float), ctrl=np.array(old.data.ctrl, float),
          bodies=bodies, origin=np.array(origin, float), backoff_m=backoff_m, part=part)
CKPT.write_bytes(pickle.dumps(ck))
finish(ck, out)
