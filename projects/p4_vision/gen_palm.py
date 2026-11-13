"""P4.8 (SPEC_4 4.5): data for the palm-camera offset network. Episodes as gen_demos.py (arm2
camera scene with palm_cam, same case and rendering draws per seed), but the approach runs
policies.BiasPolicy, so the bar is seen off-centre, and the camera mounts carry the test-size
calibration error (default 10 mm, 1 deg per axis, per sensor). Every 0.1 s in approach and close
and the first second of hold:

  palm   (T, 128, 128, 3) uint8  palm_cam image
  rel    (T, 9)  the bar site in the grip-point frame: position (3) [m], rot6 (6)  <- the label
  jaw    (T, 2)  jaw positions;  t, phase (T,)

All episodes are saved, whatever the outcome (aborted ones up to the abort): the labels are
geometry, not demonstrations. Resumable.

  bash run_p4.sh gen_palm.py --start 1000 --n 150 --workers 8 --out /home/shrey31/p4_data/palm_v1
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from multiprocessing import Pool
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "p3_capture"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from cams import CamRig  # noqa: E402
from gen_demos import PHASES, SCENE, SPLITS, rot6  # noqa: E402
from grasp_sim import GRIP_Q, GraspCfg, GraspSim  # noqa: E402
from policies import BiasPolicy  # noqa: E402


def bar_in_grip(sim):
    """The bar site in the grip-point frame: position [m] and rotation matrix."""
    pg, Rg = sim.site_pose(sim.sid_grip)
    pb, Rb = sim.site_pose(sim.sid_bar)
    return Rg.T @ (pb - pg), Rg.T @ Rb


class PalmRecorder:
    def __init__(self, sim, rig, every=10):
        self.sim, self.rig, self.every = sim, rig, every
        self.reset()

    def reset(self):
        self.n, self.t_hold = 0, None
        self.d = dict(palm=[], rel=[], jaw=[], t=[], phase=[])

    def __call__(self, L):
        ph, t = L["phase"], L["t"]
        if ph not in ("approach", "close", "hold"):
            return
        if ph == "hold":
            self.t_hold = t if self.t_hold is None else self.t_hold
            if t - self.t_hold > 1.0:
                return
        self.n += 1
        if (self.n - 1) % self.every:
            return
        sim, d = self.sim, self.d
        p, R = bar_in_grip(sim)
        d["palm"].append(self.rig.render_palm(sim.kd))
        d["rel"].append(np.concatenate([p, rot6(R)]))
        d["jaw"].append(sim.kd.qpos[GRIP_Q].copy())
        d["t"].append(t)
        d["phase"].append(PHASES.index(ph))


_W = {}


def _init(out, mount_m, mount_deg):
    for k in range(20):
        try:
            sim = GraspSim(GraspCfg(scene=SCENE, hold_s=5.0, mon=True, mon_act=True))
            break
        except ValueError as e:
            if "parse" not in str(e).lower() and "empty" not in str(e).lower():
                raise
            time.sleep(0.3 + 0.3 * k + 0.05 * (os.getpid() % 10))
    rig = CamRig(sim.km, mount_m=mount_m, mount_deg=mount_deg)
    assert rig.palm_id >= 0, "scene has no palm_cam: re-run make_scene.py --arm arm2"
    rec = PalmRecorder(sim, rig)
    sim.step_hook = rec
    sim.policy = BiasPolicy()
    _W.update(sim=sim, rig=rig, rec=rec, out=Path(out))


def _episode(seed):
    f = _W["out"] / f"ep_{seed}.npz"
    if f.exists():
        return dict(seed=seed, skipped=True)
    sim, rig, rec = _W["sim"], _W["rig"], _W["rec"]
    draw = np.random.default_rng(seed + 12345)  # as gen_demos.py
    spin = float(draw.uniform(0.0, 2.0))
    axis = int(draw.integers(3))
    dr = rig.randomize(draw)
    rec.reset()
    sim.policy.reset(seed + 4242)
    t0 = time.time()
    r = sim.run(np.random.default_rng(seed), spin_deg_s=spin, spin_axis=axis)
    d = rec.d
    meta = dict(seed=seed, spin=spin, axis=axis, mount_m=dr["mount_m"], mount_deg=dr["mount_deg"],
                bias_mm=(sim.policy.b * 1000).round(2).tolist(), bias_rot_deg=np.degrees(sim.policy.rv).round(2).tolist(),
                frames=len(d["t"]), wall_s=round(time.time() - t0, 1), scene=SCENE,
                **{k: r.get(k) for k in ("success", "outcome", "aborted", "abort_reason", "push_dv", "collision")})
    if len(d["t"]) >= 20:
        np.savez_compressed(f.with_suffix(".tmp.npz"), palm=np.stack(d["palm"]), rel=np.array(d["rel"], np.float32),
                            jaw=np.array(d["jaw"], np.float32), t=np.array(d["t"], np.float32),
                            phase=np.array(d["phase"], np.int8), meta=json.dumps(meta, default=float))
        f.with_suffix(".tmp.npz").rename(f)
    return meta


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--split", choices=list(SPLITS), default="demo")
    ap.add_argument("--start", type=int, default=1000)  # demo seeds 8,001,000+: apart from demos_v1-v4
    ap.add_argument("--n", type=int, default=4)
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--mount-m", type=float, default=0.010)
    ap.add_argument("--mount-deg", type=float, default=1.0)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    seeds = [SPLITS[a.split] * 1000 + a.start + i for i in range(a.n)]
    t0, log = time.time(), out / "episodes.jsonl"
    with Pool(a.workers, initializer=_init, initargs=(str(out), a.mount_m, a.mount_deg)) as pool, log.open("a") as fh:
        for i, m in enumerate(pool.imap_unordered(_episode, seeds, chunksize=1), 1):
            if not m.get("skipped"):
                fh.write(json.dumps(m, default=float) + "\n")
                fh.flush()
            if i % 10 == 0 or i == len(seeds):
                print(f"  {i}/{len(seeds)} episodes, {(time.time() - t0) / 60:.1f} min", flush=True)
    ms = [json.loads(ln) for ln in log.read_text().splitlines() if ln.strip()]
    from collections import Counter
    print(f"{len(ms)} episodes, {sum(m['frames'] for m in ms)} frames, outcomes {dict(Counter(m['outcome'] for m in ms))}, "
          f"aborts {dict(Counter(m['abort_reason'] for m in ms if m['aborted']))}, collisions {sum(bool(m['collision']) for m in ms)}")


if __name__ == "__main__":
    main()
