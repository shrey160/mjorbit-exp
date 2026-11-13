"""Check that p3e_espa_arm2_scene.next.xml (gripper visuals) has the same physics as
p3e_espa_arm2_scene.xml: masses, inertias, CoMs, and a short free simulation."""
import os
import mujoco
import numpy as np
os.chdir(os.path.dirname(os.path.abspath(__file__)))
def load(fn):
    xml = open(fn).read()
    xml = "\n".join(l for l in xml.splitlines() if "<mjorbit " not in l and "use_gravity_gradient" not in l)
    return mujoco.MjModel.from_xml_string(xml)
a, b = load("p3e_espa_arm2_scene.xml"), load("p3e_espa_arm2_scene.next.xml")
for k in ("body_mass", "body_inertia", "body_ipos", "body_iquat", "dof_armature", "dof_frictionloss", "actuator_ctrlrange"):
    print(k, np.array_equal(getattr(a, k), getattr(b, k)))
ga = [g for g in range(a.ngeom) if a.geom_contype[g] or a.geom_conaffinity[g]]
gb = [g for g in range(b.ngeom) if b.geom_contype[g] or b.geom_conaffinity[g]]
print("colliding geoms", len(ga), len(gb), np.array_equal(a.geom_size[ga], b.geom_size[gb]))
out = []
for m in (a, b):
    d = mujoco.MjData(m)
    d.qpos[7:13] = [0, -0.8, 1.2, -0.4, 0, 0]
    rng = np.random.default_rng(0)
    for i in range(2000):
        d.ctrl[6:12] = rng.normal(0, 3, 6)
        d.ctrl[12:14] = [0.03, -0.03] if i > 1000 else [0.06, -0.06]
        mujoco.mj_step(m, d)
    out.append(d.qpos.copy())
print("qpos after 2000 steps identical:", np.array_equal(out[0], out[1]), np.abs(out[0] - out[1]).max())
