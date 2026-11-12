"""Export a GraspSim run (fly-in, grasp, or a fault and abort) as replay JSON for a 3D
browser viewer: model geometry once, then body poses per frame in a client-centred,
inertially oriented frame (positions relative to the client's centre).

    python ../projects/p3_capture/export_replay.py <out.json> <seed> <spin_deg_s> [fault] [fps]
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from grasp_sim import GraspCfg, GraspSim  # noqa: E402

out, seed, spin = sys.argv[1], int(sys.argv[2]), float(sys.argv[3])
fault = sys.argv[4] if len(sys.argv) > 4 and sys.argv[4] != "-" else ""
fps = float(sys.argv[5]) if len(sys.argv) > 5 else 4.0
cfg = GraspCfg(flyin=True, mon=True, fault_kind=fault,
               fault_phase="approach" if fault == "client_kick" else "random")
sim = GraspSim(cfg)
r = sim.run(np.random.default_rng(seed), record=True, spin_deg_s=spin)
km, kd = sim.km, sim.kd

# geometry: visible geoms of every body
G = mujoco.mjtGeom
tmap = {G.mjGEOM_BOX: "box", G.mjGEOM_SPHERE: "sphere", G.mjGEOM_CAPSULE: "capsule",
        G.mjGEOM_CYLINDER: "cylinder", G.mjGEOM_ELLIPSOID: "ellipsoid"}
geoms = []
for g in range(km.ngeom):
    t = tmap.get(int(km.geom_type[g]))
    rgba = km.geom_rgba[g]
    if t is None or rgba[3] <= 0.0:
        continue
    geoms.append(dict(b=int(km.geom_bodyid[g]), t=t, s=[round(float(x), 4) for x in km.geom_size[g]],
                      p=[round(float(x), 4) for x in km.geom_pos[g]],
                      q=[round(float(x), 5) for x in km.geom_quat[g]],
                      c=[round(float(x), 3) for x in rgba],
                      n=mujoco.mj_id2name(km, mujoco.mjtObj.mjOBJ_GEOM, g) or ""))
bodies = sorted({g["b"] for g in geoms})

# frames
trace = r["trace"]
step = max(1, int(round(1.0 / (fps * cfg.ctrl_dt))))
frames = []
for i in range(0, len(trace), step):
    t, phase, p_g, p_b, qpos, qvel = trace[i]
    kd.qpos[:] = qpos
    mujoco.mj_kinematics(km, kd)
    c = kd.xpos[sim.bid_target].copy()
    poses = []
    for b in bodies:
        p = kd.xpos[b] - c
        q = kd.xquat[b]
        poses.extend([round(float(x), 4) for x in p] + [round(float(x), 5) for x in q])
    frames.append(dict(t=round(float(t), 2), ph=phase, x=poses))

ev = []
if np.isfinite(r.get("t_fault", np.nan)):
    ev.append(dict(t=round(float(r["t_fault"]), 2), what=f"fault injected: {fault.replace('_', ' ')}"))
if np.isfinite(r.get("t_detect", np.nan)):
    ev.append(dict(t=round(float(r["t_detect"]), 2), what=f"monitor tripped: {r['detect_reason']}"))
if r.get("aborted"):
    ev.append(dict(t=round(float(r["t_abort"]), 2), what="abort: open jaws, retract, back off"))
if np.isfinite(r.get("t_grasp", np.nan)) and not r.get("aborted"):
    ev.append(dict(t=round(float(r["t_grasp"]), 2), what="jaws closed on the bar"))
meta = dict(seed=seed, spin_deg_s=spin, fault=fault, outcome=r["outcome"],
            flyin_s=round(float(r.get("flyin_s", 0.0)), 1), min_clear=round(float(r["min_clear"]), 3),
            t_to_safe=None if not np.isfinite(r["t_to_safe"]) else round(float(r["t_to_safe"]), 1),
            bodies=[mujoco.mj_id2name(km, mujoco.mjtObj.mjOBJ_BODY, b) for b in bodies],
            client_body=bodies.index(sim.bid_target), servicer_body=bodies.index(sim.bid_chaser))
Path(out).write_text(json.dumps(dict(meta=meta, geoms=[{**g, "b": bodies.index(g["b"])} for g in geoms],
                                     events=ev, frames=frames), separators=(",", ":")))
print(meta["outcome"], len(frames), "frames", len(geoms), "geoms", ev)
