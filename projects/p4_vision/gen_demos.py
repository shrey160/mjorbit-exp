"""Project 4 (SPEC_4 P4.2): classical-expert grasp demonstrations with camera images.

Episode: GraspSim on p4_espa_cam_scene.xml (ESPA servicer, free-flying), servicer
synchronised at the pre-grasp point with the E1 start errors, client spin 0-2 deg/s about a
random body axis; settle, reach, approach, close, hold (hold_s 5); E2 monitors on (aborted or
failed episodes are counted and not saved). Recorded at 10 Hz (every 10th control step):

  v2 (12 Nov): images and lidar every --img-steps control steps (default 25 = 0.25 s;
  consecutive 10 Hz frames are near-duplicates), with their own state / labels at the image
  time; the rows below stay at 10 Hz. Calibrated mounts; bus camera 192, lidar 128.
  wrist     (Ti, 128, 128, 3) uint8 wrist_cam (cams.py, randomised per episode)
  bus       (Ti, 192, 192, 3) uint8 bus_cam
  lidar     (Ti, 128, 128) uint16   bus flash lidar range in mm, 0 = no return (cams.py)
  img_state (Ti, 29), img_priv (Ti, 39), img_t, img_phase (Ti,): as below, at the image times
  state   (T, 29)  arm q (6), arm qd (6), jaws (2), grip-point position (3) and rotation
                   (6, first two matrix columns) in the bus frame, grip-point linear and
                   angular velocity relative to the bus, in the bus frame (3 + 3)
  action  (T, 10)  expert grip-point target in the bus frame: position (3), rotation (6);
                   close flag (1). Target = forward kinematics of the joint reference while
                   settling/reaching, the impedance target while approaching/closing/holding.
  priv    (T, 39)  privileged labels, never policy inputs: bar position (3) and rotation (6)
                   in the bus frame; client angular velocity in the bus frame (3); bar pose
                   in the wrist camera, bus camera and lidar frames (9 + 9 + 9) for the P4.3
                   pose estimator
  t, phase (T,)    time [s], phase index (PHASES)

Splits (episode seed = base + i): demo 8,000,000+, validation 6,000,000+, test 7,000,000+.
Resumable: existing episode files are skipped.

  MUJOCO_GL=egl python gen_demos.py --split demo --n 500 --workers 20 --out /home/.../demos_v1
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from multiprocessing import Pool
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "p3_capture"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import mujoco  # noqa: E402
from cams import CamRig  # noqa: E402
from grasp_sim import ARM_Q, GRIP_Q, GraspCfg, GraspSim  # noqa: E402

SCENE = "p4_espa_cam_scene.xml"
SPLITS = {"demo": 8000, "validation": 6000, "test": 7000}
PHASES = ("settle", "reach", "approach", "close", "hold")


def rot6(R):
    return np.concatenate([R[:, 0], R[:, 1]])


class Recorder:
    """GraspSim.step_hook: rows (state, action, labels) every `every`-th control step (10 Hz);
    images, lidar and their own state / labels every `img_steps`-th control step (25 = 0.25 s)."""

    def __init__(self, sim: GraspSim, rig: CamRig, every=10, img_steps=25):
        self.sim, self.rig, self.every, self.img_steps = sim, rig, every, img_steps
        self.fk = mujoco.MjData(sim.km)  # separate data for the reference's forward kinematics
        self.reset()

    def reset(self):
        self.n = 0
        self.rows = dict(state=[], action=[], priv=[], t=[], phase=[])
        self.imgs = dict(wrist=[], bus=[], lidar=[], state=[], priv=[], t=[], phase=[])

    def target(self, L):
        ph = L["phase"]
        if ph in ("approach", "close", "hold"):
            return L["p_des"], L["R_des"]
        if ph in ("settle", "reach"):
            sim, fk = self.sim, self.fk
            fk.qpos[:] = sim.kd.qpos
            fk.qpos[ARM_Q] = L["q_d"]
            mujoco.mj_kinematics(sim.km, fk)
            return fk.site_xpos[sim.sid_grip].copy(), fk.site_xmat[sim.sid_grip].reshape(3, 3).copy()
        return None

    def __call__(self, L):
        self.n += 1
        row, img = (self.n - 1) % self.every == 0, (self.n - 1) % self.img_steps == 0
        if not (row or img) or L["phase"] not in PHASES:
            return
        tgt = self.target(L)
        if tgt is None:
            return
        sim, kd = self.sim, self.sim.kd
        Rs, ps = kd.xmat[sim.bid_chaser].reshape(3, 3), kd.xpos[sim.bid_chaser]
        ws = Rs @ kd.qvel[3:6]
        vs = kd.qvel[0:3]
        pg, Rg = sim.site_pose(sim.sid_grip)
        vg, wg = sim.site_vel(sim.sid_grip)
        v_rel = Rs.T @ (vg - vs - np.cross(ws, pg - ps))
        w_rel = Rs.T @ (wg - ws)
        state = np.concatenate([kd.qpos[ARM_Q], kd.qvel[6:12], kd.qpos[GRIP_Q],
                                Rs.T @ (pg - ps), rot6(Rs.T @ Rg), v_rel, w_rel])
        pt, Rt = tgt
        close = float(L["phase"] in ("close", "hold"))
        action = np.concatenate([Rs.T @ (pt - ps), rot6(Rs.T @ Rt), [close]])
        pb, Rb = sim.site_pose(sim.sid_bar)
        Rc = kd.xmat[sim.bid_target].reshape(3, 3)
        wc = Rc @ kd.qvel[17:20]
        cam = []
        for c in self.rig.cam_ids + [self.rig.lidar_id]:
            Rk, pk = kd.cam_xmat[c].reshape(3, 3), kd.cam_xpos[c]
            cam += [Rk.T @ (pb - pk), rot6(Rk.T @ Rb)]
        priv = np.concatenate([Rs.T @ (pb - ps), rot6(Rs.T @ Rb), Rs.T @ wc, *cam])
        ph = PHASES.index(L["phase"])
        if img:
            m = self.imgs
            im = self.rig.render(kd)
            m["wrist"].append(im["wrist_cam"])
            m["bus"].append(im["bus_cam"])
            m["lidar"].append(self.rig.lidar(kd))
            m["state"].append(state)
            m["priv"].append(priv)
            m["t"].append(L["t"])
            m["phase"].append(ph)
        if row:
            r = self.rows
            r["state"].append(state)
            r["action"].append(action)
            r["priv"].append(priv)
            r["t"].append(L["t"])
            r["phase"].append(ph)


_W = {}


def _init(img_steps, out):
    import os
    for k in range(20):  # mjorbit compiles via a temp XML next to the scene: parallel workers race
        try:
            sim = GraspSim(GraspCfg(scene=SCENE, hold_s=5.0, mon=True, mon_act=True))
            break
        except ValueError as e:
            if "parse" not in str(e).lower() and "empty" not in str(e).lower():
                raise
            time.sleep(0.3 + 0.3 * k + 0.05 * (os.getpid() % 10))
    rig = CamRig(sim.km)
    rec = Recorder(sim, rig, img_steps=img_steps)
    sim.step_hook = rec
    _W.update(sim=sim, rig=rig, rec=rec, out=Path(out))


def _episode(job):
    seed, holdout = job
    f = _W["out"] / f"ep_{seed}.npz"
    if f.exists():
        return dict(seed=seed, skipped=True)
    sim, rig, rec = _W["sim"], _W["rig"], _W["rec"]
    draw = np.random.default_rng(seed + 12345)  # case and rendering draws, apart from the sim's rng
    spin = float(draw.uniform(0.0, 2.0))
    axis = int(draw.integers(3))
    dr = rig.randomize(draw, holdout_sun=holdout)
    rec.reset()
    t0 = time.time()
    r = sim.run(np.random.default_rng(seed), spin_deg_s=spin, spin_axis=axis)
    meta = dict(seed=seed, spin=spin, axis=axis, dr=dr, wall_s=round(time.time() - t0, 1),
                **{k: r.get(k) for k in ("success", "grasped", "outcome", "aborted", "abort_reason",
                                         "push_dv", "bar_shift_mm", "t_grasp", "collision")})
    if r.get("success") and not r.get("aborted"):
        rows, im = rec.rows, rec.imgs
        meta["img_steps"] = rec.img_steps
        np.savez_compressed(f.with_suffix(".tmp.npz"), wrist=np.stack(im["wrist"]), bus=np.stack(im["bus"]),
                            lidar=np.stack(im["lidar"]), img_state=np.array(im["state"], np.float32),
                            img_priv=np.array(im["priv"], np.float32), img_t=np.array(im["t"], np.float32),
                            img_phase=np.array(im["phase"], np.int8),
                            state=np.array(rows["state"], np.float32), action=np.array(rows["action"], np.float32),
                            priv=np.array(rows["priv"], np.float32), t=np.array(rows["t"], np.float32),
                            phase=np.array(rows["phase"], np.int8), meta=json.dumps(meta, default=float))
        f.with_suffix(".tmp.npz").rename(f)
        meta["frames"] = len(rows["t"])
        meta["img_frames"] = len(im["t"])
    return meta


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--split", choices=list(SPLITS), default="demo")
    ap.add_argument("--start", type=int, default=0)
    ap.add_argument("--n", type=int, default=4)
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--img-steps", type=int, default=25, help="images every N control steps (25 = 0.25 s)")
    ap.add_argument("--holdout-sun", action="store_true", help="sun inside the held-out cone (eval only)")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    base = SPLITS[a.split] * 1000
    jobs = [(base + a.start + i, a.holdout_sun) for i in range(a.n)]
    t0 = time.time()
    log = out / "episodes.jsonl"
    with Pool(a.workers, initializer=_init, initargs=(a.img_steps, str(out))) as pool, log.open("a") as fh:
        for i, m in enumerate(pool.imap_unordered(_episode, jobs, chunksize=1), 1):
            if not m.get("skipped"):
                fh.write(json.dumps(m, default=float) + "\n")
                fh.flush()
            if i % 10 == 0 or i == len(jobs):
                print(f"  {i}/{len(jobs)} episodes, {(time.time() - t0) / 60:.1f} min", flush=True)
    ms = [json.loads(ln) for ln in log.read_text().splitlines() if ln.strip()]
    ok = [m for m in ms if m.get("frames")]
    print(f"{len(ok)}/{len(ms)} saved (successful), frames {sum(m['frames'] for m in ok)}, "
          f"failed/aborted: {[(m['seed'], m['outcome'], m['abort_reason']) for m in ms if not m.get('frames')][:10]}")


if __name__ == "__main__":
    main()
