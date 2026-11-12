"""Probe the mjorbit CPU API for the 3B controller (which MuJoCo fields are exposed)."""
import mjorbit  # noqa: F401
import mujoco
import numpy as np
from mjorbit import MjoModel, OrbitInit, mjo_step
X = "examples/ppo/astrobee_grasp.xml"
m = MjoModel.from_xml_path(X, mj_timestep=0.01)
d = m.make_data(orbit=OrbitInit(R_eci=[6778.0, 0, 0], V_eci=[0, 7.67, 0]))
print([a for a in dir(m) if not a.startswith("_")][:80])
print([a for a in dir(d) if not a.startswith("_")][:120])
mjo_step(m, d)
for f in ("qpos", "qvel", "site_xpos", "xpos", "xmat", "ncon", "contact", "ctrl", "nv"):
    try:
        v = getattr(d, f) if hasattr(d, f) else getattr(m, f)
        print(f, type(v), getattr(v, "shape", v))
    except Exception as e:
        print(f, "ERR", e)
try:
    mm = mujoco.MjModel.from_xml_path(X)
    print("plain mujoco loads it:", mm.nv)
except Exception as e:
    print("plain mujoco ERR", str(e)[:200])
