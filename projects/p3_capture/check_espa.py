"""Check the ESPA servicer scene: mass, stowed envelope, bounding radius, and that the arm
reaches the pre-contact pose from the pre-grasp point (SPEC_3 section 14)."""
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

P = Path(__file__).with_name("p3e_espa_scene.xml")
xml = "\n".join(l for l in P.read_text().splitlines() if "<mjorbit " not in l and "use_gravity_gradient" not in l)
m = mujoco.MjModel.from_xml_string(xml)
d = mujoco.MjData(m)
num = lambda n: m.numeric_data[m.numeric_adr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_NUMERIC, n)]:][:m.numeric_size[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_NUMERIC, n)]]  # noqa: E731
bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "chaser")
tid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "target")
mass = sum(m.body_mass[i] for i in range(m.nbody) if m.body_rootid[i] == bid)
print(f"servicer mass {mass:.1f} kg (ESPA limit 181)")
stow = num("servicer_stow")
d.qpos[7:13] = stow
d.qpos[13:15] = num("grip_open")
mujoco.mj_forward(m, d)
# stowed extent in the bus frame: sample points on every servicer geom
R = d.xmat[bid].reshape(3, 3)
pc = d.xpos[bid]
pts = []
for g in range(m.ngeom):
    if m.body_rootid[m.geom_bodyid[g]] != bid:
        continue
    Rg = d.geom_xmat[g].reshape(3, 3)
    s = m.geom_size[g]
    if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX:
        corners = np.array([[a, b, c] for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)]) * s
    elif m.geom_type[g] in (mujoco.mjtGeom.mjGEOM_CAPSULE, mujoco.mjtGeom.mjGEOM_CYLINDER):
        corners = np.array([[x, y, z] for x in (-s[0], s[0]) for y in (-s[0], s[0]) for z in (-s[1] - s[0], s[1] + s[0])])
    else:
        corners = np.array([[x, y, z] for x in (-s[0], s[0]) for y in (-s[0], s[0]) for z in (-s[0], s[0])])
    for cpt in corners:
        pts.append(R.T @ (d.geom_xpos[g] + Rg @ cpt - pc))
pts = np.array(pts)
lo, hi = pts.min(0), pts.max(0)
ext = hi - lo
print("stowed extent (bus frame) x/y/z [m]:", np.round(ext, 3), " lo", np.round(lo, 3), " hi", np.round(hi, 3))
env = np.array([0.61, 0.71, 0.97])
print("fits ESPA 0.61 x 0.71 x 0.97 (sorted):", bool(np.all(np.sort(ext) <= np.sort(env))))
print(f"bounding radius from bus centre: {np.linalg.norm(pts, axis=1).max():.3f} m (scene says {num('servicer_radius')[0]})")
# reach: servicer at the pre-grasp point, aligned with the client; pre-contact 0.12 m before the bar
D = num("pregrasp_d")[0]
R0 = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
bar = R0 @ np.array([-0.9, 0, 0])
nf = R0 @ np.array([-1.0, 0, 0])
d.qpos[0:3] = bar + D * nf
q = np.zeros(4)
mujoco.mju_mat2Quat(q, R0.reshape(-1))
d.qpos[3:7] = q
d.qpos[15:18] = 0
d.qpos[18:22] = q
sg = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "grip_site")
jp, jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
for off in (0.12, 0.0):
    p_des, R_des = bar - off * R0[:, 0], R0
    qa = num("servicer_ready").copy()
    for _ in range(400):
        d.qpos[7:13] = qa
        mujoco.mj_kinematics(m, d)
        mujoco.mj_comPos(m, d)
        p, Rg = d.site_xpos[sg], d.site_xmat[sg].reshape(3, 3)
        err = np.concatenate([p_des - p, Rotation.from_matrix(R_des @ Rg.T).as_rotvec()])
        mujoco.mj_jacSite(m, d, jp, jr, sg)
        J = np.vstack([jp[:, 6:12], jr[:, 6:12]])
        qa = np.clip(qa + J.T @ np.linalg.solve(J @ J.T + 0.05**2 * np.eye(6), err),
                     m.jnt_range[1:7, 0], m.jnt_range[1:7, 1])
    print(f"IK to bar - {off:.2f} m: pos err {np.linalg.norm(err[:3])*1000:.2f} mm, rot err {np.degrees(np.linalg.norm(err[3:])):.3f} deg, q {np.round(qa, 2)}")
