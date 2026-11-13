"""P4.3 replay export: one episode of the classical grasp steered by the learned bar-pose
estimate (pose_est.NetPoseEst), recorded for the browser replay page (replay/index.html):
model geometry once, then per frame (every --every control steps, default 25 = 0.25 s) the
body poses (client-centred, inertial axes), the wrist / bus camera images (JPEG) and the lidar
range image (false colour PNG), the true and estimated bar position and the estimate error.

Same episode draws as gen_demos.py / eval_est.py for a given seed (case, rendering, sim).
python export_p4_replay.py --ckpt <best.pt> --seed 6000017 --out replay/run_a.json [--holdout-sun]
"""
from __future__ import annotations

import argparse
import base64
import io
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "p3_capture"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import mujoco  # noqa: E402
from PIL import Image  # noqa: E402

from cams import CamRig  # noqa: E402
from gen_demos import SCENE  # noqa: E402
from grasp_sim import GraspCfg, GraspSim  # noqa: E402
from pose_est import NetPoseEst  # noqa: E402

PHASES = ("settle", "reach", "approach", "close", "hold")


def b64(img: Image.Image, fmt: str, **kw) -> str:
    buf = io.BytesIO()
    img.save(buf, fmt, **kw)
    return base64.b64encode(buf.getvalue()).decode()


def lidar_rgb(r):
    """Near = bright, far = dark, no return = dark red (as contact_sheet.py)."""
    m = r.astype(np.float32) / 1000.0
    g = np.clip(1.0 - (m - 0.3) / 3.0, 0.08, 1.0) * 255
    rgb = np.stack([g, g, g], -1).astype(np.uint8)
    rgb[r == 0] = (90, 0, 0)
    return Image.fromarray(rgb)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ckpt", required=True)
    ap.add_argument("--seed", type=int, required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--every", type=int, default=25)
    ap.add_argument("--holdout-sun", action="store_true")
    a = ap.parse_args()

    sim = GraspSim(GraspCfg(scene=SCENE, hold_s=5.0, mon=True, mon_act=True))
    rig = CamRig(sim.km)
    est = NetPoseEst(a.ckpt, rig)
    km, kd = sim.km, sim.kd
    draw = np.random.default_rng(a.seed + 12345)  # as gen_demos.py / eval_est.py
    spin = float(draw.uniform(0.0, 2.0))
    axis = int(draw.integers(3))
    dr = rig.randomize(draw, holdout_sun=a.holdout_sun)
    est.reset()
    sim.pose_est = est

    G = mujoco.mjtGeom
    tmap = {G.mjGEOM_BOX: "box", G.mjGEOM_SPHERE: "sphere", G.mjGEOM_CAPSULE: "capsule",
            G.mjGEOM_CYLINDER: "cylinder", G.mjGEOM_ELLIPSOID: "ellipsoid"}
    geoms = []
    for g in range(km.ngeom):
        t = tmap.get(int(km.geom_type[g]))
        rgba = km.geom_rgba[g]
        if t is None or rgba[3] <= 0.0 or km.geom_group[g] > 2:
            continue
        geoms.append(dict(b=int(km.geom_bodyid[g]), t=t, s=[round(float(x), 4) for x in km.geom_size[g]],
                          p=[round(float(x), 4) for x in km.geom_pos[g]],
                          q=[round(float(x), 5) for x in km.geom_quat[g]],
                          c=[round(float(x), 3) for x in rgba[:3]],
                          n=mujoco.mj_id2name(km, mujoco.mjtObj.mjOBJ_GEOM, g) or ""))
    bodies = sorted({g["b"] for g in geoms})
    frames = []
    n = [0]

    def hook(L):
        n[0] += 1
        if (n[0] - 1) % a.every or L["phase"] not in PHASES:
            return
        c = kd.xpos[sim.bid_target].copy()
        poses = []
        for b in bodies:
            poses.extend([round(float(x), 4) for x in kd.xpos[b] - c] + [round(float(x), 5) for x in kd.xquat[b]])
        pb, _ = sim.site_pose(sim.sid_bar)
        pg, _ = sim.site_pose(sim.sid_grip)
        f = dict(t=round(float(L["t"]), 2), ph=L["phase"], x=poses,
                 bar=[round(float(x), 4) for x in pb - c], grip=[round(float(x), 4) for x in pg - c])
        if est.p_b is not None:
            pe, _ = est.bar(sim)
            f["est"] = [round(float(x), 4) for x in pe - c]
            f["err_mm"] = round(float(np.linalg.norm(pe - pb) * 1000), 1)
        im = rig.render(kd)
        f["wrist"] = b64(Image.fromarray(im["wrist_cam"]), "JPEG", quality=82)
        f["bus"] = b64(Image.fromarray(im["bus_cam"]), "JPEG", quality=82)
        f["lidar"] = b64(lidar_rgb(rig.lidar(kd)), "PNG", optimize=True)
        frames.append(f)

    sim.step_hook = hook
    r = sim.run(np.random.default_rng(a.seed), spin_deg_s=spin, spin_axis=axis)
    push_ok = r["push_dv"] <= 0.002
    meta = dict(seed=a.seed, spin_deg_s=round(spin, 2), spin_axis=axis, holdout_sun=a.holdout_sun,
                sun_I=round(dr["sun_I"], 2), ambient=round(dr["ambient"], 3),
                success=bool(r["success"]) and not r["aborted"] and push_ok, outcome=r.get("outcome", ""),
                push_mm_s=round(float(r["push_dv"]) * 1000, 2), bar_shift_mm=round(float(r["bar_shift_mm"]), 2),
                t_grasp=round(float(r["t_grasp"]), 1), ckpt=Path(a.ckpt).name,
                est_at_close_mm=round(float(est.log[-1][2] * 1000), 1) if est.log else None,
                bodies=[mujoco.mj_id2name(km, mujoco.mjtObj.mjOBJ_BODY, b) for b in bodies],
                client_body=bodies.index(sim.bid_target), servicer_body=bodies.index(sim.bid_chaser))
    ev = [dict(t=meta["t_grasp"], what="jaws closed on the bar")] if np.isfinite(r["t_grasp"]) else []
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    Path(a.out).write_text(json.dumps(dict(meta=meta, geoms=[{**g, "b": bodies.index(g["b"])} for g in geoms],
                                           events=ev, frames=frames), separators=(",", ":")))
    errs = [f["err_mm"] for f in frames if "err_mm" in f]
    print({k: meta[k] for k in ("seed", "spin_deg_s", "success", "push_mm_s", "bar_shift_mm", "t_grasp", "est_at_close_mm")},
          f"frames {len(frames)}, est error median {np.median(errs):.1f} mm, size {Path(a.out).stat().st_size / 1e6:.1f} MB")
    rig.close()


if __name__ == "__main__":
    main()
