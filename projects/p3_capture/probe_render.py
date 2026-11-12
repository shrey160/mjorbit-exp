"""Project 4 feasibility: offscreen rendering of the ESPA scene (EGL under WSL2) and whether
the installed mujoco-warp has a batch renderer. MUJOCO_GL=egl python probe_render.py"""
import os
import time
from pathlib import Path

import mujoco
import numpy as np

print("MUJOCO_GL =", os.environ.get("MUJOCO_GL"))
try:
    import mujoco_warp as mjw
    print("mujoco_warp", getattr(mjw, "__version__", "?"), "render-related:",
          [n for n in dir(mjw) if "render" in n.lower() or "ray" in n.lower()])
except Exception as e:  # noqa: BLE001
    print("mujoco_warp import failed:", e)
P = Path(__file__).with_name("p3e_espa_scene.xml")
xml = "\n".join(l for l in P.read_text().splitlines() if "<mjorbit " not in l and "use_gravity_gradient" not in l)
m = mujoco.MjModel.from_xml_string(xml)
d = mujoco.MjData(m)
mujoco.mj_forward(m, d)
r = mujoco.Renderer(m, 224, 224)
cam = mujoco.MjvCamera()
cam.type = mujoco.mjtCamera.mjCAMERA_FREE
cam.lookat[:] = d.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "target")]
cam.distance, cam.azimuth, cam.elevation = 4.0, 200.0, -20.0
r.update_scene(d, camera=cam)
img = r.render()
t0 = time.time()
for _ in range(100):
    r.update_scene(d, camera=cam)
    r.render()
dt = (time.time() - t0) / 100
print(f"rendered {img.shape}, mean {img.mean():.1f}, {1 / dt:.0f} frames/s at 224x224")
print("GL vendor check: non-black image =", bool(img.max() > 0))
try:
    from PIL import Image
    Image.fromarray(img).save(Path(__file__).with_name("probe_render.png"))
    print("saved probe_render.png")
except Exception as e:  # noqa: BLE001
    np.save(Path(__file__).with_name("probe_render.npy"), img)
    print("PIL missing, saved .npy:", e)
