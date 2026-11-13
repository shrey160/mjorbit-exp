"""P4.8: is the wrist camera's view of the jaws and the bar obstructed, and where would a
better camera sit? One oracle episode (arm2 camera scene); at a few moments of the approach and
the close it renders the current wrist_cam and candidate mounts given in the wrist3 (gripper)
frame (x = approach, y = bar axis, z = closing direction) with a free camera, and writes a sheet.
bash run_p4.sh probe_wristcam.py <seed> <out.png>"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "p3_capture"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import mujoco  # noqa: E402
from cams import CamRig  # noqa: E402
from gen_demos import SCENE  # noqa: E402
from grasp_sim import GraspCfg, GraspSim  # noqa: E402

seed, out = int(sys.argv[1]), sys.argv[2]
S = 320
# name: (camera position, look-at point, fovy) in the wrist3 frame; the grip point is (0.13, 0, 0)
CANDS = {"side (y+10cm)": ((0.00, 0.10, 0.00), (0.14, 0.0, 0.0), 70),
         "side high": ((-0.02, 0.09, 0.06), (0.14, 0.0, 0.0), 70),
         "between jaws (palm)": ((0.03, 0.0, 0.0), (0.30, 0.0, 0.0), 90)}
sim = GraspSim(GraspCfg(scene=SCENE, hold_s=5.0, mon=True, mon_act=True))
rig = CamRig(sim.km)
km = sim.km
wid = rig.cam_ids[0]
body = km.cam_bodyid[wid]
print("wrist_cam body:", mujoco.mj_id2name(km, mujoco.mjtObj.mjOBJ_BODY, body), "pos", km.cam_pos[wid], "fovy", km.cam_fovy[wid])
R = mujoco.Renderer(km, S, S)
R.enable_segmentation_rendering()  # colour by what each pixel shows: lighting does not matter
gname = [mujoco.mj_id2name(km, mujoco.mjtObj.mjOBJ_GEOM, i) or "" for i in range(km.ngeom)]
bar_g = gname.index("handle_bar")
pad_g = [i for i, n in enumerate(gname) if "pad" in n.lower()]
cl_b = sim.bid_target
print("pad geoms:", [gname[i] for i in pad_g])
COL = dict(bg=(0, 0, 0), bar=(255, 140, 0), pad=(0, 220, 80), client=(110, 80, 50), serv=(150, 150, 160))
stats = {}


def seg(name, col):
    s_ = R.render()
    gid, typ = s_[..., 0], s_[..., 1]
    isg = typ == mujoco.mjtObj.mjOBJ_GEOM
    g = np.where(isg, gid, -1)
    im = np.zeros((S, S, 3), np.uint8)
    body = np.where(g >= 0, km.geom_bodyid[np.clip(g, 0, None)], -1)
    im[g >= 0] = COL["serv"]
    im[(g >= 0) & (body == cl_b)] = COL["client"]
    bar = g == bar_g
    pad = np.isin(g, pad_g)
    im[bar], im[pad] = COL["bar"], COL["pad"]
    stats.setdefault(col, {})[name] = (round(100 * bar.mean(), 1), round(100 * pad.mean(), 1))
    return im
shots, want = {}, {"approach 0 s": ("approach", 0.0), "approach 8 s": ("approach", 8.0), "approach 14 s": ("approach", 14.0),
                   "close 0.1 s": ("close", 0.1), "close 2.5 s": ("close", 2.5)}
tph = {}


def hook(L):
    ph, t = L["phase"], L["t"]
    tph.setdefault(ph, t)
    for name, (p, dt) in want.items():
        if name in shots or ph != p or t - tph[ph] < dt:
            continue
        kd = sim.kd
        Rw, pw = kd.xmat[body].reshape(3, 3), kd.xpos[body]
        row = []
        R.update_scene(kd, camera=wid, scene_option=rig.opt)
        row.append(seg(name, "current"))
        for cn, (pos, look, fovy) in CANDS.items():
            P, Lk = pw + Rw @ np.array(pos), pw + Rw @ np.array(look)
            f = Lk - P
            d = np.linalg.norm(f)
            f /= d
            cam = mujoco.MjvCamera()
            cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            cam.lookat[:] = Lk
            cam.distance = d
            cam.azimuth = np.degrees(np.arctan2(f[1], f[0]))
            cam.elevation = np.degrees(np.arcsin(f[2]))
            old = km.vis.global_.fovy
            km.vis.global_.fovy = fovy
            R.update_scene(kd, camera=cam, scene_option=rig.opt)
            row.append(seg(name, cn))
            km.vis.global_.fovy = old
        shots[name] = row


sim.step_hook = hook
draw = np.random.default_rng(seed + 12345)
spin, axis = float(draw.uniform(0.0, 2.0)), int(draw.integers(3))
rig.randomize(draw)
r = sim.run(np.random.default_rng(seed), spin_deg_s=spin, spin_axis=axis)
print(r["outcome"], "spin", round(spin, 2), "axis", axis, "shots", list(shots))
g = 1.0
for c, d in stats.items():
    print(f'{c:22s} % of image showing (bar, pads):', d)
rows = [np.concatenate(shots[k], 1) for k in want if k in shots]
img = np.clip(np.concatenate(rows, 0).astype(np.float32) * g, 0, 255).astype(np.uint8)
try:
    from PIL import Image
    Image.fromarray(img).save(out)
except ImportError:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.imsave(out, img)
print("wrote", out, img.shape, "columns: current wrist_cam |", " | ".join(CANDS), "; rows:", list(want))
