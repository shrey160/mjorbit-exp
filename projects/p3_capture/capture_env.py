"""Project 3A: fly a servicer to the pre-grasp point of a rectangular client satellite with
a grapple bar (SPEC_3; the client was a Starlink-like bus until 8 Nov).

Reuses Project 2's 20 s maneuver-epoch env (finite burns, uncertainty presets, 1 Hz nav
log, filter and shield hooks) and replaces its geometry through the hooks in
approach_env.ApproachEnv:

  goal      = pre-grasp point g = feature + d_pregrasp * n_face, in LVLH, moving with the
              client's attitude; success uses the speed relative to g's velocity
  clearance = signed distance from the servicer centre to the client (bus box and the
              grapple-bar envelope), minus the
              servicer's bounding radius and a safety margin; < 0 ends the episode
  min_dist  = minimum clearance over the episode (m)

Client attitude at reset: body x (long axis) along LVLH +y rotated by `yaw` about the
orbit normal, so yaw = 0 puts the capture face (-x) towards -V-bar, the side the
servicer starts on. client_rot = "lvlh" keeps that attitude fixed in LVLH (spin at the
orbit rate); "inertial" leaves the client inertially fixed, so the face turns at the
orbit rate (0.064 deg/s) relative to LVLH.
"""

from __future__ import annotations

import dataclasses
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "p2_approach"))

import mjorbit  # noqa: F401,E402

# isort: split
import numpy as np  # noqa: E402
import torch  # noqa: E402
from tensordict import TensorDict  # noqa: E402

from approach_env import lvlh_frames, world_to_lvlh  # noqa: E402
from maneuver_env import ManeuverEnv, ManeuverEnvCfg  # noqa: E402
from mjorbit_warp import mjo_forward, mjo_pull, mjo_upload  # noqa: E402

XML_P3A = Path(__file__).with_name("p3a_scene.xml")
FEATURE_B = np.array([-0.9, 0.0, 0.0])  # capture feature = grapple-bar centre, body frame
NORMAL_B = np.array([-1.0, 0.0, 0.0])  # outward normal of the capture face
# body axes -> LVLH at yaw 0: body x -> +y, body y -> -x, body z -> +z
R0 = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])


@dataclasses.dataclass
class CaptureEnvCfg(ManeuverEnvCfg):
    mass: float = 150.0
    f_max: float = 0.75  # N per world axis = 5 mm/s^2
    episode_length_s: float = 1200.0
    pos_tol: float = 0.10
    vel_tol: float = 0.005
    hold_s: float = 60.0
    # keep-out boxes in the client body frame: (centre, half sizes); bus + handle envelope
    client_boxes: tuple = (((0.0, 0.0, 0.0), (0.75, 0.5, 0.5)),
                           ((-0.8375, 0.0, 0.0), (0.0875, 0.175, 0.025)))
    servicer_radius: float = 0.87  # bounding sphere of the 1 m cube
    margin: float = 0.3  # safety margin on top of the bounding sphere
    d_pregrasp: float = 1.8  # servicer centre to the capture feature along the normal
    yaw_mode: str = "random"  # "aligned" (yaw 0) or "random" (uniform +-180 deg)
    client_rot: str = "lvlh"  # "lvlh" or "inertial"
    # E3 (SPEC_3 section 13): client spin about a body axis on top of client_rot, and the
    # goal "hold": a point hold_D m from the client centre on the spin axis (the face normal
    # when not spinning), on the capture-face side, where the fly-in (section 11) starts
    goal_mode: str = "pregrasp"  # or "hold"
    hold_D: float = 8.0
    pos_scale: float = 50.0
    fine_scale: float = 0.5


def quat_to_mat(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)], -1),
        np.stack([2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)], -1),
        np.stack([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)], -1)], -2)


def mat_to_quat(R: np.ndarray) -> np.ndarray:
    from scipy.spatial.transform import Rotation
    xyzw = Rotation.from_matrix(R).as_quat()
    return np.concatenate([xyzw[:, 3:4], xyzw[:, :3]], axis=1)


def rot_z(a: np.ndarray) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    z, o = np.zeros_like(a), np.ones_like(a)
    return np.stack([np.stack([c, -s, z], -1), np.stack([s, c, z], -1),
                     np.stack([z, z, o], -1)], -2)


def box_sdf(x: np.ndarray, half: np.ndarray) -> np.ndarray:
    """Signed distance from points x (n,3), box frame, to a box of half sizes `half`."""
    q = np.abs(x) - half
    outside = np.linalg.norm(np.maximum(q, 0.0), axis=1)
    inside = np.minimum(q.max(axis=1), 0.0)
    return outside + inside


class CaptureEnv(ManeuverEnv):
    xml_path = XML_P3A

    def __init__(self, cfg: CaptureEnvCfg) -> None:
        self.client_yaw = np.zeros(cfg.num_envs)
        self.spin_b = np.zeros((cfg.num_envs, 3))
        self.spin_axis_b = np.tile(NORMAL_B, (cfg.num_envs, 1))
        # client principal inertia (body frame) for the hold axis = angular momentum (E3)
        import mujoco
        _x = "\n".join(l for l in XML_P3A.read_text().splitlines()
                       if "<mjorbit " not in l and "use_gravity_gradient" not in l)
        _m = mujoco.MjModel.from_xml_string(_x)
        _t = mujoco.mj_name2id(_m, mujoco.mjtObj.mjOBJ_BODY, "target")
        _q = _m.body_iquat[_t]
        _Rp = np.zeros(9)
        mujoco.mju_quat2Mat(_Rp, _q)
        _Rp = _Rp.reshape(3, 3)
        self.client_I_b = np.diag(_Rp @ np.diag(_m.body_inertia[_t]) @ _Rp.T)  # ~diagonal
        self._yaw_rng = np.random.default_rng(cfg.seed + 31337)
        self._geo = None
        super().__init__(cfg)

    # ------------------------------------------------------------ client geometry
    def client_geometry(self):
        """LVLH: centre c, centre velocity cd, rotation R (body->LVLH), relative angular
        velocity w (LVLH coords), feature f, face normal nf, pre-grasp g, and g's velocity."""
        cfg = self.cfg
        C, om = lvlh_frames(np.asarray(self.data.orbit.R_eci, float),
                            np.asarray(self.data.orbit.V_eci, float))
        q = np.asarray(self.data.qpos[:, 7:14], float)
        qd = np.asarray(self.data.qvel[:, 6:12], float)
        c, cd = world_to_lvlh(C, om, q[:, 0:3], qd[:, 0:3])
        Rbw = quat_to_mat(q[:, 3:7])
        Ct = np.transpose(C, (0, 2, 1))
        R = Ct @ Rbw
        w = np.einsum("nij,nj->ni", Ct, np.einsum("nij,nj->ni", Rbw, qd[:, 3:6]) - om)
        f = c + R @ FEATURE_B
        nf = R @ NORMAL_B
        g = f + cfg.d_pregrasp * nf
        gd = cd + np.cross(w, g - c)
        if cfg.goal_mode == "hold":
            # The corridor runs from the client centre out along the hold axis. For a
            # spinning client that is its angular-momentum direction: inertially fixed,
            # whereas the body spin axis nutates (the spin plus the LVLH rate is not about
            # a principal axis) and would swing the hold point ~0.3 m at 8 m. A servicer
            # would estimate it as the mean measured rotation axis; here it is computed
            # from the client's true rate and inertia. Not spinning: the face normal.
            s_b = self.spin_axis_b
            g_b = FEATURE_B + cfg.d_pregrasp * NORMAL_B
            h = s_b @ g_b
            side = np.where(np.abs(h) > 0.5, np.sign(h), 1.0)
            ax_body = np.einsum("nij,nj->ni", R, side[:, None] * s_b)  # LVLH
            spinning = np.linalg.norm(self.spin_b, axis=1) > 0
            L_w = np.einsum("nij,nj->ni", Rbw, self.client_I_b * qd[:, 3:6])  # world
            L = np.einsum("nji,nj->ni", C, L_w)  # LVLH coordinates
            L = L / np.maximum(np.linalg.norm(L, axis=1, keepdims=True), 1e-12)
            L = L * np.sign(np.sum(L * ax_body, axis=1, keepdims=True) + 1e-12)
            nf = np.where(spinning[:, None], L, ax_body)
            f = c
            g = c + cfg.hold_D * nf
            # an inertially fixed direction turns at -omega_lvlh as seen in LVLH; the face
            # normal (not spinning) turns with the client
            om_l = np.einsum("nji,nj->ni", C, om)
            gd = np.where(spinning[:, None], cd - np.cross(om_l, g - c), cd + np.cross(w, g - c))
        return dict(c=c, cd=cd, R=R, w=w, f=f, nf=nf, g=g, gd=gd)

    def goal_lvlh(self) -> np.ndarray:
        return self.client_geometry()["g"]

    def goal_vel_lvlh(self) -> np.ndarray:
        return self.client_geometry()["gd"]

    def clearance_from(self, geo, p: np.ndarray) -> np.ndarray:
        cfg = self.cfg
        xb = np.einsum("nji,nj->ni", geo["R"], p - geo["c"])
        d = np.min([box_sdf(xb - np.asarray(c), np.asarray(h)) for c, h in cfg.client_boxes],
                   axis=0)
        return d - cfg.servicer_radius - cfg.margin

    def clearance(self, p: np.ndarray) -> np.ndarray:
        return self.clearance_from(self.client_geometry(), p)

    def proximity(self, p: np.ndarray) -> np.ndarray:
        return self.clearance(p)

    # ------------------------------------------------------------------ reset
    def reset_worlds(self, idx, p0=None, v0=None) -> None:
        super().reset_worlds(idx, p0, v0)
        if not hasattr(self, "client_yaw"):
            return
        cfg = self.cfg
        k = idx.size
        if cfg.yaw_mode == "random" and p0 is None:  # training resets: fresh draws
            self.client_yaw[idx] = self._yaw_rng.uniform(-np.pi, np.pi, k)
        elif cfg.yaw_mode == "aligned":
            self.client_yaw[idx] = 0.0
        C, om = lvlh_frames(np.asarray(self.data.orbit.R_eci, float)[idx],
                            np.asarray(self.data.orbit.V_eci, float)[idx])
        Rbw = C @ (rot_z(self.client_yaw[idx]) @ R0)
        self.data.qpos[idx, 10:14] = mat_to_quat(Rbw)
        wb = np.zeros((k, 3))
        if cfg.client_rot == "lvlh":  # body rate = LVLH rate, in body coordinates
            wb = np.einsum("nji,nj->ni", Rbw, om)
        wb = wb + self.spin_b[idx]  # E3: client spin about a body axis
        self.data.qvel[idx, 9:12] = wb
        mjo_upload(self.model, self.data, fields=("qpos", "qvel"))
        mjo_forward(self.model, self.data)
        mjo_pull(self.model, self.data, fields=("qpos", "qvel", "orbit"))
        self._refresh_phi(idx)

    def set_spin(self, spin_deg_s, axis):
        """E3: client spin rate (deg/s) about body axis index (0/1/2) per world. The spin
        axis also defines the hold-point goal (face normal when the rate is 0)."""
        n = self.num_envs
        rate = np.radians(np.broadcast_to(np.asarray(spin_deg_s, float), (n,)))
        ax = np.broadcast_to(np.asarray(axis, int), (n,))
        e = np.eye(3)[ax]
        self.spin_b = rate[:, None] * e
        self.spin_axis_b = np.where(rate[:, None] > 0, e, -np.eye(3)[np.zeros(n, int)])

    def set_initial_conditions(self, p0, v0, unc_seed=None, yaw=None):
        """Matched cases: start states, uncertainty seed and client yaw per world."""
        if yaw is not None:
            self.client_yaw[:] = yaw
        elif self.cfg.yaw_mode == "aligned":
            self.client_yaw[:] = 0.0
        return super().set_initial_conditions(p0, v0, unc_seed=unc_seed)

    # ------------------------------------------------------------ observation
    def _compute_obs(self) -> TensorDict:
        if not hasattr(self, "client_yaw"):  # base-class construction
            return super()._compute_obs()
        cfg = self.cfg
        self._measure()
        geo = self.client_geometry()
        p, v = self.meas_p, self.meas_v
        e = p - geo["g"]
        obs = np.concatenate([
            e / cfg.pos_scale, (v - geo["gd"]) / cfg.vel_scale, (p - geo["c"]) / cfg.pos_scale,
            geo["nf"], np.clip(self.clearance_from(geo, p)[:, None] / 10.0, -1.0, 1.0),
            self.time_remaining()[:, None], self._last_actions,
            np.clip(e / cfg.fine_scale, -1.0, 1.0)], axis=1)
        obs = np.nan_to_num(obs, nan=0.0, posinf=0.0, neginf=0.0)
        t = torch.as_tensor(obs, dtype=torch.float32, device=self.device)
        return TensorDict({"policy": t}, batch_size=[self.num_envs])
