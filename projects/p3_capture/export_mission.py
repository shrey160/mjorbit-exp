"""Export one whole E3 mission (fly-in from the hold point, grasp, brakes, detumble,
identification, 3D move and turn) as replay JSON for replay/index.html (13 Nov).
Same format as export_replay.py: geometry once, then body poses per frame.
Positions: relative to the client centre during fly-in and grasp; after capture,
relative to a fixed point (the client centre at capture), so the 3D move is visible.

    python export_mission.py <out.json> <seed> [scene] [grasp_fps] [stack_fps]
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "p2_approach"))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

import grasp_sim  # noqa: E402
import stack_sim  # noqa: E402
from maneuver3d import Man3DCfg, Stack3D  # noqa: E402
from stack_sim import StackCfg  # noqa: E402

out = sys.argv[1]
seed = int(sys.argv[2])
scene = sys.argv[3] if len(sys.argv) > 3 else "p3e_espa_arm2_scene.xml"
gfps = float(sys.argv[4]) if len(sys.argv) > 4 else 4.0
sfps = float(sys.argv[5]) if len(sys.argv) > 5 else 1.0
H = json.loads(Path(__file__).with_name("e3_handoff_espa_validation.json").read_text())
c = {x["seed"]: x for x in H["cases"]}[seed]

REC = dict(grasp=None, stack=[], n=0, t0=None, md={}, rad=[])


def radial(sim):
    """Radial-out (away from Earth) unit vector in the simulation world frame: LVLH x
    (mjorbit: +x radial, -x nadir)."""
    d = sim.data
    v = d.world_position_from_lvlh([1.0, 0.0, 0.0]) - d.world_position_from_lvlh([0.0, 0.0, 0.0])
    return v / np.linalg.norm(v)


def _ghook(loc):
    if int(round(loc["t"] / 0.01)) % 25 == 0:
        REC["rad"].append((float(loc["t"]), radial(loc["self"])))
_orig_run = grasp_sim.GraspSim.run


def _run(self, rng=None, record=False, **kw):  # record the grasp trace (first call only)
    if REC["grasp"] is None:
        self.step_hook = _ghook
        o = self.data.orbit
        print("check: R_eci dir", np.round(np.asarray(o.R_eci, float) / np.linalg.norm(o.R_eci), 4), "LVLH x in world", np.round(radial(self), 4))
    r = _orig_run(self, rng, record=REC["grasp"] is None, **kw)
    if REC["grasp"] is None:
        REC["grasp"] = (self, r)
    return r


grasp_sim.GraspSim.run = _run
_orig_step = stack_sim.StackRun._step
SDIV = max(1, int(round(1.0 / (sfps * 0.01))))


def _step(self, *a, **k):
    res = _orig_step(self, *a, **k)
    REC["n"] += 1
    if REC["n"] % SDIV == 0:
        REC["stack"].append((self.t, getattr(self, "phase_tag", "brakes"), self.sim, self.sim.data.qpos.copy(), radial(self.sim)))
    return res


stack_sim.StackRun._step = _step
_orig_ci = Stack3D.capture_identify


def _ci(self):
    r = _orig_ci(self)
    self.phase_tag = "move"
    return r


Stack3D.capture_identify = _ci

gfault = c["fault"] if c["fault"] in ("client_kick", "thruster_weak", "thruster_stuck", "spin_axis_err") else ""
scfg = StackCfg(spin_deg_s=c["spin"], spin_axis=c["axis"], impose_tumble=False, grip_mon=True,
                grip_response="reclamp", grip_loose_mm=c["loose"] if c["fault"] == "grip_loose" else 0.0,
                scene=scene)
sr = Stack3D(seed, scfg, Man3DCfg(), grasp_kw=dict(flyin=True, mon=True, fault_kind=gfault))
sr.grasp_reset_kw = dict(hold_dp_b=c["dp_b"], hold_dv_b=c["dv_b"])
res = sr.run3d()

gsim, g = REC["grasp"]
km = gsim.km
G = mujoco.mjtGeom
tmap = {G.mjGEOM_BOX: "box", G.mjGEOM_SPHERE: "sphere", G.mjGEOM_CAPSULE: "capsule",
        G.mjGEOM_CYLINDER: "cylinder", G.mjGEOM_ELLIPSOID: "ellipsoid"}
geoms = []
for gi in range(km.ngeom):
    t = tmap.get(int(km.geom_type[gi]))
    rgba = km.geom_rgba[gi]
    if t is None or rgba[3] <= 0.0:
        continue
    geoms.append(dict(b=int(km.geom_bodyid[gi]), t=t, s=[round(float(x), 4) for x in km.geom_size[gi]],
                      p=[round(float(x), 4) for x in km.geom_pos[gi]], q=[round(float(x), 5) for x in km.geom_quat[gi]],
                      c=[round(float(x), 3) for x in rgba], n=mujoco.mj_id2name(km, mujoco.mjtObj.mjOBJ_GEOM, gi) or ""))
bodies = sorted({x["b"] for x in geoms})
bt = gsim.bid_target


def poses(model, qpos, origin=None):
    md = REC["md"].setdefault(id(model), mujoco.MjData(model))
    md.qpos[:] = qpos
    mujoco.mj_kinematics(model, md)
    o = md.xpos[bt].copy() if origin is None else origin
    x = []
    for b in bodies:
        x.extend([round(float(v), 4) for v in md.xpos[b] - o] + [round(float(v), 5) for v in md.xquat[b]])
    return x, md.xpos[bt].copy()


frames = []
trace = g["trace"]
step = max(1, int(round(1.0 / (gfps * gsim.cfg.ctrl_dt))))
for i in list(range(0, len(trace), step)) + [len(trace) - 1]:
    t, ph = trace[i][0], trace[i][1]
    x, cpos = poses(km, trace[i][4])
    rt = REC["rad"][int(np.argmin([abs(a - t) for a, _ in REC["rad"]]))][1] if REC["rad"] else np.array([1.0, 0, 0])
    frames.append(dict(t=round(float(t), 2), ph=ph, x=x, r=[round(float(v), 4) for v in rt]))
T_cap = float(trace[-1][0])
origin = cpos  # client centre at the end of the grasp run
ev = [dict(t=round(float(g["t_grasp"]), 1), what="jaws closed on the bar")] if np.isfinite(g.get("t_grasp", np.nan)) else []
last_ph = None
for (ts, ph, s, q, rt) in REC["stack"]:
    x, _ = poses(s.km, q, origin)
    tt = round(T_cap + float(ts), 2)
    if tt <= frames[-1]["t"]:
        continue
    if ph != last_ph:
        ev.append(dict(t=tt, what={"brakes": "arm brakes on", "detumble": "brakes on, detumbling the stack",
                                   "id": "identifying mass, CoM and inertia with thruster pulses",
                                   "move": "moving and turning the stack"}.get(ph, ph)))
        last_ph = ph
    frames.append(dict(t=tt, ph=ph, x=x, r=[round(float(v), 4) for v in rt]))
ok = bool(res.get("final_pos_err_m") is not None and res.get("success", True))
ev.append(dict(t=frames[-1]["t"], what="done" if "final_pos_err_m" in res else "stopped"))
nm = lambda b: mujoco.mj_id2name(km, mujoco.mjtObj.mjOBJ_BODY, b)  # noqa: E731
meta = dict(altitude_km=float(gsim.cfg.altitude_km), seed=seed, spin_deg_s=round(float(c["spin"]), 2), fault=c["fault"], scene=scene,
            outcome="captured" if g.get("grasped") else str(g.get("outcome")),
            flyin_s=round(float(g.get("flyin_s", 0.0)), 1), min_clear=round(float(g["min_clear"]), 3), t_to_safe=None,
            bodies=[nm(b) for b in bodies], client_body=bodies.index(bt), servicer_body=bodies.index(gsim.bid_chaser),
            mission=dict(dr_m=res.get("dr_m"), att_cmd_deg=res.get("att_cmd_deg"), final_pos_err_m=res.get("final_pos_err_m"),
                         final_att_err_deg=res.get("final_att_err_deg"), mass_err_pct=res.get("mass_err_pct"),
                         I_err_rel_pct=res.get("I_err_rel_pct"), rcs_prop_g=res.get("rcs_prop_g"), t_capture=round(T_cap, 1)))
meta["mission"] = {k: (round(float(v), 4) if isinstance(v, (int, float, np.floating)) and v is not None else v)
                   for k, v in meta["mission"].items()}
Path(out).write_text(json.dumps(dict(meta=meta, geoms=[{**x, "b": bodies.index(x["b"])} for x in geoms],
                                     events=ev, frames=frames), separators=(",", ":")))
print("frames", len(frames), "geoms", len(geoms), "T", frames[-1]["t"], json.dumps(meta["mission"]), [e["what"] for e in ev])
