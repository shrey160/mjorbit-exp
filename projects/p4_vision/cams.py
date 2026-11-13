"""Project 4 (SPEC_4 P4.0-P4.1): render the wrist and bus cameras from GraspSim's synced
plain-MuJoCo copy, with per-episode domain randomisation.

Randomisation only touches rendering fields of the plain-MuJoCo model (light, material,
geom colour, camera mount, headlight ambient). That model is also used for kinematics,
Jacobians and contact reads, none of which depend on these fields, and the physics model
(mjorbit) is separate, so the dynamics are unaffected.

Per episode (DR dict recorded with the episode):
  sun_dir       unit vector, uniform within LIT_DEG of the grasp-face normal, except the
                held-out cone (SPEC_4 section 5: a lighting condition never seen in training)
  sun_I         diffuse intensity 0.5-1.3; specular 0.1-0.6
  ambient       0.03-0.15 (earthshine, scattered light)
  client/bar/servicer colour jitter and material specular / shininess
  cam mount     residual calibration error per sensor, default +-1 mm, +-0.1 deg (v2, 12 Nov).
                v1 used +-1 cm, +-1 deg: an error no image reveals, which put a 16 mm floor
                under any absolute pose estimate (mount_floor.py). Real servicers calibrate
                their camera and lidar extrinsics before proximity operations.
Per frame: exposure gain 0.75-1.25 per episode, Gaussian pixel noise sigma 0-3 DN per episode.
Resolution per sensor (v2): wrist 128, bus 192, lidar 128 (v1: 128 / 128 / 64).

Flash lidar (bus_lidar, CamRig.lidar): square range image in mm (0 = no return), 0.3-20 m,
noise floor 0.5-2 cm per episode plus 0.5% of range, random dropout 1-5% per episode, 25%
dropout on depth-edge pixels, more at grazing incidence. Independent of the lighting.
"""
from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

import mujoco

# Sun: missions plan the final approach with the target face lit, so the direction towards
# the sun is kept within LIT_DEG of the grasp-face normal at the start (world -y: the
# servicer approaches along it in GraspSim.reset; a spinning client turns the face during
# the episode, so lighting still changes). Uniform sampling left half the episodes with the
# face in full shadow (smoke test, 12 Nov).
# Held-out lighting (SPEC_4 section 5, never in training): the sun within HOLDOUT_DEG of a
# direction 50 deg from the face normal towards the orbit normal (+z): steep light from above.
FACE_N = np.array([0.0, -1.0, 0.0])
LIT_DEG = 70.0
HOLDOUT_AXIS = np.array([0.0, -np.cos(np.radians(50)), np.sin(np.radians(50))])
HOLDOUT_DEG = 15.0
CAMS = ("wrist_cam", "bus_cam")
LIDAR = "bus_lidar"
PALM_RES = 128  # palm_cam (P4.8), present only in scenes made after 13 Nov
LIDAR_RES = 128  # pixels per side (Raven's flash lidar: 256 x 256 focal plane array); v1: 64
LIDAR_MIN, LIDAR_MAX = 0.3, 20.0  # m
RES = {"wrist_cam": 128, "bus_cam": 192}  # v1: 128 both
MOUNT_M, MOUNT_DEG = 0.001, 0.1  # residual calibration error (v1: 0.01 m, 1 deg)


def _ang(a, b):
    return float(np.degrees(np.arccos(np.clip(a @ b, -1.0, 1.0))))


def sample_sun(rng, holdout=False):
    """Light travel direction v (towards the sun = -v), uniform over the lit cone, outside
    (training) or inside (held-out evaluation) the held-out cone."""
    while True:
        v = rng.normal(size=3)
        v /= np.linalg.norm(v)
        if _ang(-v, FACE_N) > LIT_DEG:
            continue
        if (_ang(-v, HOLDOUT_AXIS) < HOLDOUT_DEG) == holdout:
            return v


class CamRig:
    def __init__(self, km: mujoco.MjModel, res=None, lidar_res=LIDAR_RES, mount_m=MOUNT_M, mount_deg=MOUNT_DEG):
        self.km = km
        self.res = dict(RES if res is None else res)
        self.mount_m, self.mount_deg = mount_m, mount_deg
        self.rs = {n: mujoco.Renderer(km, s, s) for n, s in self.res.items()}
        # no sites in the images: the grip_site / bar_site markers would hand the policy the
        # target (seen in the first smoke test)
        self.opt = mujoco.MjvOption()
        self.opt.sitegroup[:] = 0
        cid = lambda n: mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_CAMERA, n)  # noqa: E731
        self.cam_ids = [cid(n) for n in CAMS]
        assert min(self.cam_ids) >= 0, "scene has no wrist_cam / bus_cam (use p4_espa_cam_scene.xml)"
        self.lid = mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_LIGHT, "sun")
        g = lambda n: mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_GEOM, n)  # noqa: E731
        mt = lambda n: mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_MATERIAL, n)  # noqa: E731
        self.geoms = {k: g(k) for k in ("client_bus", "handle_bar", "servicer_bus")}
        self.mats = {k: mt(k) for k in ("mat_client", "mat_bar", "mat_serv")}
        self._base = dict(cam_pos=km.cam_pos.copy(), cam_quat=km.cam_quat.copy(),
                          rgba={k: km.geom_rgba[i].copy() for k, i in self.geoms.items()})
        self.dr = {}
        self.palm_id = cid("palm_cam")  # -1 in scenes without it
        self.rp = mujoco.Renderer(km, PALM_RES, PALM_RES) if self.palm_id >= 0 else None
        self.palm_light = mujoco.mj_name2id(km, mujoco.mjtObj.mjOBJ_LIGHT, "palm_light")  # -1 if absent
        # flash lidar: depth render of bus_lidar, converted to range along each pixel's ray
        self.lidar_id = cid(LIDAR)
        assert self.lidar_id >= 0, "scene has no bus_lidar (re-run make_scene.py)"
        self.lres = lidar_res
        self.rl = mujoco.Renderer(km, lidar_res, lidar_res)
        self.rl.enable_depth_rendering()
        f = (lidar_res / 2) / np.tan(np.radians(km.cam_fovy[self.lidar_id]) / 2)
        u = (np.arange(lidar_res) + 0.5 - lidar_res / 2) / f
        self._ray = np.sqrt(1.0 + u[None, :] ** 2 + u[:, None] ** 2)  # range / planar depth
        self._pix = 1.0 / f  # rad per pixel (small-angle)

    def randomize(self, rng, holdout_sun=False):
        km = self.km
        sun = sample_sun(rng, holdout_sun)
        I = rng.uniform(0.5, 1.3)
        spec = rng.uniform(0.1, 0.6)
        amb = rng.uniform(0.03, 0.15)
        km.light_dir[self.lid] = sun
        km.light_pos[self.lid] = -10.0 * sun
        km.light_diffuse[self.lid] = I
        km.light_specular[self.lid] = spec
        km.vis.headlight.ambient[:] = amb
        # client bus: grey/silver to gold MLI; bar: orange-ish; servicer: gold-ish
        tint = {"client_bus": rng.uniform([0.55, 0.55, 0.5], [0.95, 0.9, 0.85]) if rng.uniform() < 0.6
                else rng.uniform([0.7, 0.55, 0.15], [0.95, 0.8, 0.4]),
                "handle_bar": np.clip(self._base["rgba"]["handle_bar"][:3] + rng.uniform(-0.15, 0.15, 3), 0, 1),
                "servicer_bus": np.clip(self._base["rgba"]["servicer_bus"][:3] + rng.uniform(-0.15, 0.15, 3), 0, 1)}
        for k, gid in self.geoms.items():
            km.geom_rgba[gid, :3] = tint[k]
        for k, mid in self.mats.items():
            km.mat_specular[mid] = rng.uniform(0.05, 0.9)
            km.mat_shininess[mid] = rng.uniform(0.05, 0.9)
        for c in self.cam_ids + [self.lidar_id]:
            km.cam_pos[c] = self._base["cam_pos"][c] + rng.uniform(-self.mount_m, self.mount_m, 3)
            dq = Rotation.from_rotvec(np.radians(rng.uniform(-self.mount_deg, self.mount_deg, 3)))
            q = self._base["cam_quat"][c]  # MuJoCo wxyz
            r = dq * Rotation.from_quat([q[1], q[2], q[3], q[0]])
            x, y, z, w = r.as_quat()
            km.cam_quat[c] = [w, x, y, z]
        self.gain = rng.uniform(0.75, 1.25)
        self.noise = rng.uniform(0.0, 3.0)
        self._nrng = np.random.default_rng(int(rng.integers(2**31)))
        self.l_sig0 = rng.uniform(0.005, 0.02)  # m, range noise floor
        self.l_drop = rng.uniform(0.01, 0.05)  # random pixel dropout
        self.dr = dict(sun_dir=sun.tolist(), sun_I=I, sun_spec=spec, ambient=amb, holdout_sun=holdout_sun,
                       gain=self.gain, noise=self.noise, lidar_sig0=self.l_sig0, lidar_drop=self.l_drop,
                       mount_m=self.mount_m, mount_deg=self.mount_deg, res=self.res, lidar_res=self.lres,
                       tint={k: v.tolist() for k, v in tint.items()})
        if self.palm_id >= 0:
            # P4.8: the palm camera's mount error and noise stream are drawn last, so every
            # earlier draw (and so every image of the other sensors) is as before
            c = self.palm_id
            km.cam_pos[c] = self._base["cam_pos"][c] + rng.uniform(-self.mount_m, self.mount_m, 3)
            dq = Rotation.from_rotvec(np.radians(rng.uniform(-self.mount_deg, self.mount_deg, 3)))
            q = self._base["cam_quat"][c]
            x, y, z, w = (dq * Rotation.from_quat([q[1], q[2], q[3], q[0]])).as_quat()
            km.cam_quat[c] = [w, x, y, z]
            self._prng = np.random.default_rng(int(rng.integers(2**31)))
        return self.dr

    def render_palm(self, kd: mujoco.MjData):
        """(128, 128, 3) uint8 from palm_cam, with the episode's exposure gain and noise level."""
        if self.palm_light >= 0:  # the palm spot is lit only for this image
            self.km.light_active[self.palm_light] = 1
        self.rp.update_scene(kd, camera=self.palm_id, scene_option=self.opt)
        img = self.rp.render().astype(np.float32) * self.gain
        if self.palm_light >= 0:
            self.km.light_active[self.palm_light] = 0
        if self.noise > 0:
            img += self._prng.normal(0.0, self.noise, img.shape)
        return np.clip(img, 0, 255).astype(np.uint8)

    def lidar(self, kd: mujoco.MjData):
        """(LIDAR_RES, LIDAR_RES) uint16 range in mm along each pixel's ray, 0 = no return.
        Model: no return beyond LIDAR_MAX or inside LIDAR_MIN (space background, the arm
        right in front of the sensor); Gaussian range noise sigma0 + 0.5% of range; random
        dropout; 25% dropout on depth-edge (mixed) pixels; dropout rising from 70 deg incidence
        on continuous surfaces. Lighting has no effect, unlike the cameras."""
        self.rl.update_scene(kd, camera=self.lidar_id, scene_option=self.opt)
        z = self.rl.render().astype(np.float64)  # planar depth, m
        rng = z * self._ray
        # depth edges (a jump > 5% of range to a 4-neighbour): mixed pixels, dropped 25% of
        # the time (dropping every edge pixel erased the 3-pixel-tall bar, smoke test 12 Nov);
        # continuous surfaces: dropout rising from 70 deg incidence
        zp = np.pad(z, 1, mode="edge")
        jump = np.max(np.abs(np.stack([zp[:-2, 1:-1], zp[2:, 1:-1], zp[1:-1, :-2], zp[1:-1, 2:]]) - z), 0)
        edge = jump > 0.05 * z
        gy, gx = np.gradient(z)
        tan_inc = np.hypot(gx, gy) / np.maximum(z * self._pix, 1e-9)
        inc = np.degrees(np.arctan(tan_inc))
        p_drop = self.l_drop + np.where(edge, 0.25, 0.8 * np.clip((inc - 70.0) / 15.0, 0.0, 1.0))
        n = self._nrng
        rng = rng + n.normal(0.0, 1.0, rng.shape) * (self.l_sig0 + 0.005 * rng)
        bad = (z > LIDAR_MAX) | (z < LIDAR_MIN) | (n.uniform(size=rng.shape) < p_drop)
        out = np.round(rng * 1000.0)
        out[bad] = 0
        return np.clip(out, 0, 65535).astype(np.uint16)

    def render(self, kd: mujoco.MjData):
        """{camera name: (S, S, 3) uint8} from the synced plain-MuJoCo data."""
        out = {}
        for n, c in zip(CAMS, self.cam_ids):
            r = self.rs[n]
            r.update_scene(kd, camera=c, scene_option=self.opt)
            img = r.render().astype(np.float32) * self.gain
            if self.noise > 0:
                img += self._nrng.normal(0.0, self.noise, img.shape)
            out[n] = np.clip(img, 0, 255).astype(np.uint8)
        return out

    def close(self):
        for r in (*self.rs.values(), self.rl):
            try:
                r.close()
            except Exception:  # noqa: BLE001 (EGL teardown errors are harmless)
                pass
