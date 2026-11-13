"""Static checks of the arm2 scene against p3e_espa (13 Nov): masses, joint inertia seen at
each joint, stowed envelope (ESPA 61 x 71 x 97 cm), bounding radius, reach."""
import os
import sys

import mujoco
import numpy as np

os.chdir(os.path.dirname(os.path.abspath(__file__)))


def corners(m, d, g):
    """World-frame AABB corners of a geom (exact for box/cylinder/capsule/sphere bounds)."""
    s, R, p = m.geom_size[g], d.geom_xmat[g].reshape(3, 3), d.geom_xpos[g]
    t = m.geom_type[g]
    if t == mujoco.mjtGeom.mjGEOM_BOX:
        h = s
    elif t == mujoco.mjtGeom.mjGEOM_SPHERE:
        return p - s[0], p + s[0]
    elif t == mujoco.mjtGeom.mjGEOM_CYLINDER:
        h = np.array([s[0], s[0], s[1]])
    elif t == mujoco.mjtGeom.mjGEOM_CAPSULE:
        h = np.array([s[0], s[0], s[1] + s[0]])
    else:
        h = m.geom_rbound[g] * np.ones(3)
    e = np.abs(R) @ h
    return p - e, p + e


for fn in ("p3e_espa_scene.xml", "p3e_espa_arm2_scene.xml"):
    xml = open(fn).read()
    xml = "\n".join(line for line in xml.splitlines() if "<mjorbit " not in line
                     and "use_gravity_gradient" not in line)
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    ch = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "chaser")
    sub = [b for b in range(m.nbody) if m.body_rootid[b] == ch]
    m_serv = sum(m.body_mass[b] for b in sub)
    m_arm = m_serv - m.body_mass[ch]
    stow = m.numeric_data[m.numeric_adr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_NUMERIC, "servicer_stow")]:][:6]
    jq = [m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in
          ("j1_sh_yaw", "j2_sh_pitch", "j3_elbow", "j4_wr_pitch", "j5_wr_yaw", "j6_wr_roll")]
    jv = [m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in
          ("j1_sh_yaw", "j2_sh_pitch", "j3_elbow", "j4_wr_pitch", "j5_wr_yaw", "j6_wr_roll")]
    print(f"\n== {fn}: servicer {m_serv:.2f} kg, bus {m.body_mass[ch]:.2f}, arm {m_arm:.2f}")
    for name, q in (("stow", stow), ("ready", [0, -0.8, 1.2, -0.4, 0, 0]), ("stretched", [0, 0, 0, 0, 0, 0])):
        d.qpos[:] = m.qpos0
        d.qpos[jq] = q
        mujoco.mj_forward(m, d)
        M = np.zeros((m.nv, m.nv))
        mujoco.mj_fullM(m, M, d.qM)
        diag = np.diag(M)[jv]
        lo, hi = np.full(3, 1e9), np.full(3, -1e9)
        Rb, pb = d.xmat[ch].reshape(3, 3), d.xpos[ch]
        rad = 0.0
        for g in range(m.ngeom):
            if m.body_rootid[m.geom_bodyid[g]] != ch:
                continue
            a, b = corners(m, d, g)
            for c in np.array(np.meshgrid(*zip(a, b))).T.reshape(-1, 3):
                cb = Rb.T @ (c - pb)
                lo, hi = np.minimum(lo, cb), np.maximum(hi, cb)
                rad = max(rad, np.linalg.norm(cb))
        print(f"  {name:9s} box x {lo[0]:+.3f}..{hi[0]:+.3f} y {lo[1]:+.3f}..{hi[1]:+.3f} z {lo[2]:+.3f}..{hi[2]:+.3f}"
              f"  size {hi[0]-lo[0]:.3f} x {hi[1]-lo[1]:.3f} x {hi[2]-lo[2]:.3f} m  radius {rad:.3f}")
        print(f"            joint-space inertia diag (kg m^2): {np.round(diag, 3)}")
    sid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "grip_site")
    print("  grip site at stretch (bus frame):", np.round(d.xmat[ch].reshape(3, 3).T @ (d.site_xpos[sid] - d.xpos[ch]), 3))
    print("  actuators:", [f"{m.actuator(i).name} {m.actuator_ctrlrange[i,1]:g}" for i in range(6, 12)])
    print("  frictionloss:", m.dof_frictionloss[jv], " armature:", m.dof_armature[jv])
