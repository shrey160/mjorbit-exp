"""P4.3 closed loop: the classical grasp on the estimated bar pose (pose_est.NetPoseEst) vs the
classical grasp on the true pose (oracle), on the same episodes as gen_demos.py (same case,
rendering and simulator draws for a given seed).

python eval_est.py --mode est --ckpt <best.pt> --split validation --n 200 --workers 20 --out <file.json>
python eval_est.py --mode oracle --split validation --n 200 --workers 20 --out <file.json>
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
from cams import CamRig  # noqa: E402
from gen_demos import SCENE, SPLITS  # noqa: E402
from grasp_sim import GraspCfg, GraspSim  # noqa: E402

KEEP = ("success", "grasped", "outcome", "aborted", "abort_reason", "abort_phase", "push_dv", "bar_shift_mm",
        "t_grasp", "collision", "peak_force", "both_pads_end", "ik_err_pos", "approach_err_max", "phase_end")
_W = {}


def _init(mode, ckpt):
    import os
    import torch
    torch.set_num_threads(1)
    for k in range(20):
        try:
            sim = GraspSim(GraspCfg(scene=SCENE, hold_s=5.0, mon=True, mon_act=True))
            break
        except ValueError as e:
            if "parse" not in str(e).lower() and "empty" not in str(e).lower():
                raise
            time.sleep(0.3 + 0.3 * k + 0.05 * (os.getpid() % 10))
    rig = CamRig(sim.km)  # v2 rig: calibrated mounts, bus 192, lidar 128 (as the demos)
    est = None
    if mode == "est":
        from pose_est import NetPoseEst
        est = NetPoseEst(ckpt, rig)
    _W.update(sim=sim, rig=rig, est=est)


def _episode(job):
    seed, holdout = job
    sim, rig, est = _W["sim"], _W["rig"], _W["est"]
    draw = np.random.default_rng(seed + 12345)  # as gen_demos.py
    spin = float(draw.uniform(0.0, 2.0))
    axis = int(draw.integers(3))
    dr = rig.randomize(draw, holdout_sun=holdout)
    if est is not None:
        est.reset()
    sim.pose_est = est
    t0 = time.time()
    r = sim.run(np.random.default_rng(seed), spin_deg_s=spin, spin_axis=axis)
    m = dict(seed=seed, spin=spin, axis=axis, sun_I=dr["sun_I"], wall_s=round(time.time() - t0, 1),
             **{k: r.get(k) for k in KEEP})
    if est is not None and est.log:
        L = est.log
        for ph in ("settle", "reach", "approach"):
            e = [x[2] for x in L if x[1] == ph]
            a = [x[3] for x in L if x[1] == ph]
            if e:
                m[f"est_{ph}_mm"] = float(np.median(e) * 1000)
                m[f"est_{ph}_deg"] = float(np.median(a))
        m["est_last_mm"] = float(L[-1][2] * 1000)  # at the close command (latched)
    return m


def wilson(k, n, z=1.96):
    if n == 0:
        return (np.nan, np.nan)
    p = k / n
    c = (p + z * z / (2 * n)) / (1 + z * z / n)
    h = z * np.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / (1 + z * z / n)
    return (round(c - h, 4), round(c + h, 4))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=("est", "oracle"), required=True)
    ap.add_argument("--ckpt", default="")
    ap.add_argument("--split", choices=list(SPLITS), default="validation")
    ap.add_argument("--start", type=int, default=0)
    ap.add_argument("--n", type=int, default=20)
    ap.add_argument("--workers", type=int, default=20)
    ap.add_argument("--holdout-sun", action="store_true")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    base = SPLITS[a.split] * 1000
    jobs = [(base + a.start + i, a.holdout_sun) for i in range(a.n)]
    t0 = time.time()
    ms = []
    with Pool(a.workers, initializer=_init, initargs=(a.mode, a.ckpt)) as pool:
        for i, m in enumerate(pool.imap_unordered(_episode, jobs, chunksize=1), 1):
            ms.append(m)
            if i % 10 == 0 or i == len(jobs):
                print(f"  {i}/{len(jobs)}, {(time.time() - t0) / 60:.1f} min", flush=True)
    ms.sort(key=lambda m: m["seed"])
    n = len(ms)
    PUSH_MAX = 0.002  # m/s, SPEC_4 section 1 (fixed 12 Nov before any estimator result)
    for m in ms:
        m["p4_success"] = bool(m["success"]) and not m["aborted"] and not m["collision"] and m["push_dv"] <= PUSH_MAX
    ks = sum(m["p4_success"] for m in ms)
    k_sim = sum(bool(m["success"]) and not m["aborted"] for m in ms)
    unsafe = sum(bool(m["collision"]) for m in ms)
    from collections import Counter
    summ = dict(mode=a.mode, ckpt=a.ckpt, split=a.split, n=n, holdout_sun=a.holdout_sun,
                success=ks / n, success_ci95=wilson(ks, n), sim_success=k_sim / n, collisions=unsafe,
                push_over_limit=sum(m["push_dv"] > PUSH_MAX for m in ms),
                aborts=dict(Counter(m["abort_reason"] for m in ms if m["aborted"])),
                push_dv_mm_s_max=float(max(m["push_dv"] for m in ms) * 1000),
                bar_shift_mm_max=float(np.nanmax([m["bar_shift_mm"] for m in ms])),
                wall_min=round((time.time() - t0) / 60, 1))
    for ph in ("settle", "reach", "approach"):
        v = [m[f"est_{ph}_mm"] for m in ms if f"est_{ph}_mm" in m]
        if v:
            summ[f"est_{ph}_mm_median"] = float(np.median(v))
    v = [m["est_last_mm"] for m in ms if "est_last_mm" in m]
    if v:
        summ["est_at_close_mm"] = dict(median=float(np.median(v)), p95=float(np.percentile(v, 95)), max=float(max(v)))
    Path(a.out).write_text(json.dumps(dict(summary=summ, episodes=ms), indent=1, default=float))
    print(json.dumps(summ, indent=1, default=float))


if __name__ == "__main__":
    main()
