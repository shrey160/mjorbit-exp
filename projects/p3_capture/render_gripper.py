"""Close-up of the gripper: old visuals vs the arm2 visuals draft (13 Nov), jaws open and
closed. Writes output/arm2/gripper_compare.png."""
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


rows = []
for fn, label in (("p3e_espa_arm2_scene.xml", "now"), ("p3e_espa_arm2_scene.next.xml", "new visuals")):
    m = load(fn)
    d = mujoco.MjData(m)
    r = mujoco.Renderer(m, H, W)
    sid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "grip_site")
    tiles = []
    for jaws, az, el in (("open", 135.0, -20.0), ("closed", 135.0, -20.0), ("open", 200.0, 25.0)):
        d.qpos[:] = m.qpos0
        d.qpos[7:13] = [0, -0.8, 1.2, -0.4, 0, 0]
        d.qpos[13:15] = [0.06, -0.06] if jaws == "open" else [-0.008, 0.008]
        d.qpos[15:18] = [0.0, 50.0, 0.0]
        mujoco.mj_forward(m, d)
        cam = mujoco.MjvCamera()
        cam.lookat[:] = d.site_xpos[sid] - 0.06 * d.site_xmat[sid].reshape(3, 3)[:, 0]
        cam.distance, cam.azimuth, cam.elevation = 0.55, az, el
        r.update_scene(d, cam)
        img = Image.fromarray(r.render())
        ImageDraw.Draw(img).text((10, 8), f"gripper {label}: jaws {jaws}", fill=(255, 255, 255))
        tiles.append(img)
    rows.append(tiles)
    r.close()
grid = Image.new("RGB", (W * 3, H * 2))
for i, row in enumerate(rows):
    for j, t in enumerate(row):
        grid.paste(t, (j * W, i * H))
OUT.mkdir(parents=True, exist_ok=True)
grid.save(OUT / "gripper_compare.png")
print("wrote", OUT / "gripper_compare.png")
