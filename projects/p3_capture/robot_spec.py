"""Robot description layer (SPEC_3 section 14): everything the controllers need to know
about a servicer / client pair, read from the MuJoCo model by name and structure instead
of hard-coded indices, so the same code runs on another servicer (e.g. the ESPA-class one,
p3e_espa_scene.xml) or a final model.

From the model:
  - servicer and client free joints (qpos / qvel addresses), by body name
  - arm joints (hinges in the servicer subtree, tree order) and gripper joints (slides)
  - actuators by transmission: bus thrusters (translational gear on the servicer free
    joint), bus torques / wheels (rotational gear), arm motors, gripper servos; load-test
    motors (names starting "test_") are kept apart; limits from ctrlrange
  - masses, the end-effector and capture-feature sites
  - client keep-out boxes from the client's colliding box geoms (body frame)
From <custom><numeric> entries (all optional, with defaults matching p3b_scene.xml):
  servicer_stow, servicer_ready (IK seed; default = stow), grip_open, grip_squeeze,
  pregrasp_d (bar centre to bus centre along the face normal), servicer_radius
  (bounding sphere for clearance), keepout_pad (added to the merged feature keep-out box's
  half sizes, client body frame: the hand-written p3b envelope ran 2.3 cm past the bar ends).
Also the servicer bus inertia (principal, body frame), used to scale the bus gains.
"""

from __future__ import annotations

import dataclasses

import mujoco
import numpy as np

DEFAULTS = dict(servicer_stow=[0.0, -1.3, 2.5, -1.2, 0.0, 0.0], grip_open=[0.06, -0.06],
                grip_squeeze=[0.03], pregrasp_d=[1.8], servicer_radius=[0.87],
                keepout_pad=[0.0, 0.023, 0.0])


@dataclasses.dataclass
class RobotSpec:
    nq: int
    nv: int
    nu: int
    serv_q: slice  # free joint qpos (7)
    serv_v: slice  # free joint qvel (6)
    client_q: slice
    client_v: slice
    arm_q: np.ndarray  # qpos indices of the arm joints
    arm_v: np.ndarray  # dof indices
    grip_q: np.ndarray
    u_thr: np.ndarray  # actuator indices
    u_rw: np.ndarray
    u_arm: np.ndarray
    u_grip: np.ndarray
    u_test: dict  # name -> index
    f_max: float
    tau_bus_max: float
    arm_tau_max: np.ndarray
    arm_range: np.ndarray  # (n_arm, 2)
    m_serv: float
    m_client: float
    I_serv: np.ndarray  # bus body principal inertia (3,), kg m^2
    stow: np.ndarray
    ready: np.ndarray
    ready_given: bool  # the scene names a ready pose (IK seed); else IK starts where the arm is
    grip_open: np.ndarray
    grip_squeeze: float
    pregrasp_d: float
    servicer_radius: float
    client_boxes: list  # [(centre (3,), half sizes (3,))] in the client body frame
    bid_serv: int
    bid_client: int
    sid_grip: int
    sid_bar: int

    @property
    def n_arm(self):
        return len(self.arm_q)


def _numeric(m, name):
    i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_NUMERIC, name)
    if i < 0:
        return np.asarray(DEFAULTS.get(name, []), float) if name in DEFAULTS else None
    a = m.numeric_adr[i]
    return np.array(m.numeric_data[a:a + m.numeric_size[i]], float)


def _free_joint(m, bid):
    for j in range(m.njnt):
        if m.jnt_bodyid[j] == bid and m.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE:
            return j
    raise ValueError(f"body {bid} has no free joint")


def build_spec(m: mujoco.MjModel, servicer="chaser", client="target", ee_site="grip_site",
               feature_site="bar_site") -> RobotSpec:
    nm = lambda t, i: mujoco.mj_id2name(m, t, i) or ""  # noqa: E731
    bid_s = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, servicer)
    bid_c = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, client)
    js, jc = _free_joint(m, bid_s), _free_joint(m, bid_c)
    qs, vs = m.jnt_qposadr[js], m.jnt_dofadr[js]
    qc, vc = m.jnt_qposadr[jc], m.jnt_dofadr[jc]
    arm, grip = [], []
    for j in range(m.njnt):
        if m.body_rootid[m.jnt_bodyid[j]] != bid_s or j == js:
            continue
        (arm if m.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE else grip).append(j)
    u_thr, u_rw, u_arm, u_grip, u_test = [], [], [], [], {}
    for a in range(m.nu):
        name = nm(mujoco.mjtObj.mjOBJ_ACTUATOR, a)
        if name.startswith("test_"):
            u_test[name] = a
            continue
        j = m.actuator_trnid[a, 0]
        gear = m.actuator_gear[a]
        if j == js:
            (u_thr if np.any(gear[:3] != 0) else u_rw).append(a)
        elif j in arm:
            u_arm.append(a)
        elif j in grip:
            u_grip.append(a)
    # order arm actuators like the arm joints
    u_arm.sort(key=lambda a: arm.index(m.actuator_trnid[a, 0]))
    u_grip.sort(key=lambda a: grip.index(m.actuator_trnid[a, 0]))
    m_s = float(sum(m.body_mass[i] for i in range(m.nbody) if m.body_rootid[i] == bid_s))
    m_c = float(sum(m.body_mass[i] for i in range(m.nbody) if m.body_rootid[i] == bid_c))
    boxes = []
    for g in range(m.ngeom):
        if m.geom_bodyid[g] == bid_c and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX and m.geom_contype[g]:
            # boxes are taken axis-aligned in the body frame (true for the scenes here)
            boxes.append((m.geom_pos[g].copy(), m.geom_size[g].copy()))
    boxes = _merge_feature_boxes(boxes)
    if len(boxes) == 2:
        pad = _numeric(m, "keepout_pad")
        boxes[1] = (np.round(boxes[1][0], 6), np.round(boxes[1][1] + pad, 6))
    stow = _numeric(m, "servicer_stow")
    ready = _numeric(m, "servicer_ready")
    return RobotSpec(
        nq=m.nq, nv=m.nv, nu=m.nu,
        serv_q=slice(qs, qs + 7), serv_v=slice(vs, vs + 6),
        client_q=slice(qc, qc + 7), client_v=slice(vc, vc + 6),
        arm_q=np.array([m.jnt_qposadr[j] for j in arm]), arm_v=np.array([m.jnt_dofadr[j] for j in arm]),
        grip_q=np.array([m.jnt_qposadr[j] for j in grip]),
        u_thr=np.array(u_thr), u_rw=np.array(u_rw), u_arm=np.array(u_arm), u_grip=np.array(u_grip),
        u_test=u_test,
        f_max=float(np.min(m.actuator_ctrlrange[u_thr, 1])) if u_thr else 0.0,
        tau_bus_max=float(np.min(m.actuator_ctrlrange[u_rw, 1])) if u_rw else 0.0,
        arm_tau_max=m.actuator_ctrlrange[u_arm, 1].copy(),
        arm_range=np.array([m.jnt_range[j] for j in arm]),
        m_serv=m_s, m_client=m_c, I_serv=m.body_inertia[bid_s].copy(),
        stow=stow, ready=stow.copy() if ready is None else ready, ready_given=ready is not None,
        grip_open=_numeric(m, "grip_open"), grip_squeeze=float(_numeric(m, "grip_squeeze")[0]),
        pregrasp_d=float(_numeric(m, "pregrasp_d")[0]),
        servicer_radius=float(_numeric(m, "servicer_radius")[0]),
        client_boxes=boxes, bid_serv=bid_s, bid_client=bid_c,
        sid_grip=mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, ee_site),
        sid_bar=mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, feature_site))


def _merge_feature_boxes(boxes):
    """Keep the largest box (the bus) as is and merge the small feature boxes (posts, bar)
    into one envelope box, which is how the hand-written keep-out was built."""
    if len(boxes) <= 2:
        return boxes
    vol = [np.prod(h) for _, h in boxes]
    k = int(np.argmax(vol))
    rest = [b for i, b in enumerate(boxes) if i != k]
    lo = np.min([c - h for c, h in rest], axis=0)
    hi = np.max([c + h for c, h in rest], axis=0)
    return [boxes[k], ((lo + hi) / 2, (hi - lo) / 2)]


def load_spec(xml_path, **kw):
    from pathlib import Path
    x = "\n".join(l for l in Path(xml_path).read_text().splitlines()
                  if "<mjorbit " not in l and "use_gravity_gradient" not in l)
    return build_spec(mujoco.MjModel.from_xml_string(x), **kw)


if __name__ == "__main__":
    import sys
    from pathlib import Path
    for f in sys.argv[1:] or ["p3b_scene.xml", "p3e_espa_scene.xml"]:
        s = load_spec(Path(__file__).with_name(f))
        print(f"== {f}")
        for k, v in dataclasses.asdict(s).items():
            print(f"  {k:16s} {v}")
