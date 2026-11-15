"""Check the stowed arm for geometry passing through other servicer geometry (15 Nov).

Kinematics only (no simulation run): the servicer is put in its stowed pose and every pair of
servicer geoms on different, non-adjacent bodies is tested with mj_geomDistance (meshes by their
convex hull, as MuJoCo collides them). Visual-only geoms are included, since the user sees them.

python dbg_stow.py [scene.xml] [q1 q2 q3 q4 q5 q6]   (optional stow pose to try instead)
"""
from __future__ import annotations

import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "p3_capture"))
from grasp_sim import ARM_Q, GraspCfg, GraspSim  # noqa: E402

scene = next((a for a in sys.argv[1:] if a.endswith(".xml")), "p4_espa_arm2_cam_scene.xml")
q_try = [float(a) for a in sys.argv[1:] if not a.endswith(".xml") and not a.startswith("--")]
sim = GraspSim(GraspCfg(scene=scene, hold_s=5.0))
sim.reset(np.random.default_rng(0))
m, d = sim.km, sim.kd
if q_try:
    d.qpos[ARM_Q] = q_try
    mujoco.mj_forward(m, d)
print("scene", scene, "stow", np.round(sim.stow, 4).tolist(), "tested", np.round(d.qpos[ARM_Q], 4).tolist())

name = lambda t, i: mujoco.mj_id2name(m, t, i) or f"#{i}"  # noqa: E731
root = sim.bid_chaser
serv = [b for b in range(m.nbody) if b == root or m.body_rootid[b] == m.body_rootid[root]]
print("\nservicer bodies (id, name, parent):")
for b in serv:
    print(f"  {b:3d} {name(mujoco.mjtObj.mjOBJ_BODY, b):24s} parent {name(mujoco.mjtObj.mjOBJ_BODY, m.body_parentid[b])}")
geoms = [g for g in range(m.ngeom) if m.geom_bodyid[g] in serv]
tn = {int(getattr(mujoco.mjtGeom, k)): k[7:] for k in dir(mujoco.mjtGeom) if k.startswith("mjGEOM_")}
rows = []
fromto = np.zeros(6)
for i, g1 in enumerate(geoms):
    for g2 in geoms[i + 1:]:
        b1, b2 = m.geom_bodyid[g1], m.geom_bodyid[g2]
        if b1 == b2 or m.body_parentid[b1] == b2 or m.body_parentid[b2] == b1:
            continue  # same link or neighbours joined by a joint
        dist = mujoco.mj_geomDistance(m, d, g1, g2, 0.05, fromto)
        if dist < 0.005:
            col = lambda g: bool(m.geom_contype[g] or m.geom_conaffinity[g])  # noqa: E731
            rows.append((dist, g1, g2, b1, b2, col(g1), col(g2), fromto[:3].copy()))
rows.sort(key=lambda r: r[0])
print(f"\npairs closer than 5 mm (negative = passing through), {len(rows)} found:")
for dist, g1, g2, b1, b2, c1, c2, pt in rows:
    G, B = mujoco.mjtObj.mjOBJ_GEOM, mujoco.mjtObj.mjOBJ_BODY
    print(f"  {1e3 * dist:7.1f} mm  {name(B, b1)}/{name(G, g1)} [{tn[int(m.geom_type[g1])]}{'' if c1 else ', visual'}]"
          f"  x  {name(B, b2)}/{name(G, g2)} [{tn[int(m.geom_type[g2])]}{'' if c2 else ', visual'}]"
          f"  at {np.round(pt, 3).tolist()}")

B = mujoco.mjtObj.mjOBJ_BODY
bn = {b: name(B, b) for b in serv}
grp = lambda *names: [g for g in geoms if bn[m.geom_bodyid[g]] in names]  # noqa: E731
tip, fore, upper = grp("wrist1", "wrist2", "wrist3", "jaw_u", "jaw_l"), grp("fore"), grp("upper")
# pairs that a stow pose can change (wrist-internal overlaps and the shoulder housing on its mount are built in)
PAIRS = ([(a, b) for a in tip for b in grp("chaser", "arm_base", "upper")] + [(a, b) for a in fore for b in grp("chaser", "arm_base")]
         + [(a, b) for a in upper[1:] for b in grp("chaser")] + [(a, b) for a in grp("wrist3", "jaw_u", "jaw_l") for b in fore])
PSET = {frozenset(p) for p in PAIRS}
# distances from MuJoCo's own collision code (mj_geomDistance returned 0 for some overlapping pairs):
# a copy of the model in which every servicer geom collides, with a 60 mm margin, so each pair
# closer than 120 mm appears as a contact carrying its signed distance
import copy  # noqa: E402
m2 = copy.copy(m)
m2.geom_contype[geoms], m2.geom_conaffinity[geoms], m2.geom_margin[geoms] = 1, 1, 0.06
for arr, val in (("body_contype", 1), ("body_conaffinity", 1), ("body_margin", 0.06)):  # broadphase, per body
    if hasattr(m2, arr):
        getattr(m2, arr)[serv] = val
d2 = mujoco.MjData(m2)
FAR = 0.12
ENV = np.sort([0.61, 0.71, 0.97])  # ESPA envelope (check_espa.py)


def corners(g):
    sz, t = m.geom_size[g], m.geom_type[g]
    if t == mujoco.mjtGeom.mjGEOM_BOX:
        c = np.array([[a, b, c] for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)]) * sz
    elif t in (mujoco.mjtGeom.mjGEOM_CAPSULE, mujoco.mjtGeom.mjGEOM_CYLINDER):
        h = sz[1] + (sz[0] if t == mujoco.mjtGeom.mjGEOM_CAPSULE else 0.0)
        c = np.array([[x, y, z] for x in (-sz[0], sz[0]) for y in (-sz[0], sz[0]) for z in (-h, h)])
    else:
        r = sz[0] if t == mujoco.mjtGeom.mjGEOM_SPHERE else m.geom_rbound[g]
        c = np.array([[x, y, z] for x in (-r, r) for y in (-r, r) for z in (-r, r)])
    return d.geom_xpos[g] + c @ d.geom_xmat[g].reshape(3, 3).T


def score(q):
    """(clearance over PAIRS, bounding radius about the bus origin, fits the ESPA envelope, extent)"""
    d.qpos[ARM_Q] = q
    mujoco.mj_forward(m, d)
    d2.qpos[:] = d.qpos
    mujoco.mj_forward(m2, d2)
    dmin = FAR
    for c in d2.contact[:d2.ncon]:
        if frozenset((int(c.geom1), int(c.geom2))) in PSET:
            dmin = min(dmin, float(c.dist))
    Rb = d.xmat[root].reshape(3, 3)
    pts = np.vstack([corners(g) for g in geoms]) - d.xpos[root]
    ext = np.ptp(pts @ Rb, axis=0)
    return dmin, float(np.linalg.norm(pts, axis=1).max()), bool(np.all(np.sort(ext) <= ENV + 1e-6)), ext


c0 = score(sim.stow)
print("collision-code check: contacts at the current stow", d2.ncon, "(0 would mean the check is blind)")
print(f"\ncurrent stow: clearance {1e3 * c0[0]:.1f} mm, radius {c0[1]:.3f} m, fits ESPA {c0[2]}, extent {np.round(c0[3], 3).tolist()}")
if q_try:
    c1 = score(q_try)
    print(f"tested pose : clearance {1e3 * c1[0]:.1f} mm, radius {c1[1]:.3f} m, fits ESPA {c1[2]}, extent {np.round(c1[3], 3).tolist()}")
    # unfold path: joint-space line to the ready pose (what the reach and the abort stow fly)
    ready = sim.ik_seed if sim.ik_seed is not None else sim.stow
    cl = [score(np.asarray(q_try) + a * (ready - np.asarray(q_try)))[0] for a in np.linspace(0, 1, 41)]
    print(f"line to the ready pose {np.round(ready, 2).tolist()}: smallest clearance {1e3 * min(cl):.1f} mm at {np.argmin(cl) / 40:.2f} of the way")

if "--search" in sys.argv:
    rng = np.random.default_rng(1)
    lo, hi = m.jnt_range[1:7, 0], m.jnt_range[1:7, 1]
    need = 0.025
    best = []
    for it in range(6000):
        q = rng.uniform(lo, hi)
        q[0] = 0.0
        q[5] = rng.choice([0.0, 1.5708, -1.5708, 3.1])
        sc = score(q)
        if sc[0] >= need:
            best.append((sc[1], sc[2], sc[0], q))
    print(f"\nrandom search: {len(best)} of 6000 poses clear by {1e3 * need:.0f} mm; {sum(b[1] for b in best)} of them fit ESPA")
    best.sort(key=lambda b: (not b[1], b[0]))
    out = []
    for rad, fits, dm, q in best[:8]:  # local refinement of the best few
        for it in range(400):
            q2 = np.clip(q + rng.normal(0, 0.05, 6) * [0, 1, 1, 1, 1, 0], lo, hi)
            sc = score(q2)
            if sc[0] >= need and (sc[2], -sc[1]) > (fits, -rad):
                rad, fits, dm, q = sc[1], sc[2], sc[0], q2
        out.append((rad, fits, dm, q))
    for rad, fits, dm, q in sorted(out, key=lambda b: (not b[1], b[0])):
        print(f"  radius {rad:.3f} m  fits ESPA {fits}  clearance {1e3 * dm:5.1f} mm  q = {np.round(q, 3).tolist()}")
