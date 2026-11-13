"""Render the old ESPA arm next to arm2 (13 Nov) in three poses: stowed, ready, reaching
the bar. Writes output/arm2/arm_compare.png."""
import os
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw

HERE = Path(__file__).parent
OUT = HERE.parents[1] / "output" / "arm2"
W, H = 640, 520


def load(fn):
    xml = (HERE / fn).read_text()
    xml = "\n".join(l for l in xml.splitlines() if "<mjorbit " not in l and "use_gravity_gradient" not in l)
    m = mujoco.MjModel.from_xml_string(xml)
    m.vis.global_.offwidth, m.vis.global_.offheight = W, H
    m.vis.headlight.ambient[:] = 0.35
    m.vis.headlight.diffuse[:] = 0.6
    return m


POSES = {"stowed": [0, -1.5708, 3.05, 0.09, 0, 0], "ready": [0, -0.8, 1.2, -0.4, 0, 0],
         "reaching": [0.3, 0.1, 0.9, -0.6, 0.3, 0.5]}
rows = []
for fn, label in (("p3e_espa_scene.xml", "old ESPA arm (10.1 kg)"), ("p3e_espa_arm2_scene.xml", "arm2 (16.55 kg)")):
    m = load(fn)
    d = mujoco.MjData(m)
    r = mujoco.Renderer(m, H, W)
    ch = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "chaser")
    tiles = []
    for name, q in POSES.items():
        d.qpos[:] = m.qpos0
        d.qpos[7:13] = q
        d.qpos[15:18] = [0.0, 50.0, 0.0]  # client out of view
        mujoco.mj_forward(m, d)
        cam = mujoco.MjvCamera()
        cam.lookat[:] = d.xpos[ch] + (np.array([0.35, 0.0, -0.05]) if name == "stowed" else np.array([0.6, 0.0, 0.0]))
        cam.distance = 1.6 if name == "stowed" else 2.4
        cam.azimuth, cam.elevation = (160.0 if name == "stowed" else 130.0), -18.0
        r.update_scene(d, cam)
        img = Image.fromarray(r.render())
        ImageDraw.Draw(img).text((10, 8), f"{label}: {name}", fill=(255, 255, 255))
        tiles.append(img)
    rows.append(tiles)
    r.close()
grid = Image.new("RGB", (W * 3, H * 2))
for i, row in enumerate(rows):
    for j, t in enumerate(row):
        grid.paste(t, (j * W, i * H))
OUT.mkdir(parents=True, exist_ok=True)
grid.save(OUT / "arm_compare.png")
print("wrote", OUT / "arm_compare.png")
