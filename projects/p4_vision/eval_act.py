"""P4.4e (SPEC_4 4.4): closed-loop grasp with a LeRobot ACT checkpoint (act_policy.ACTGraspPolicy)
on the same episodes as gen_demos.py / eval_est.py (same case, rendering and simulator draws for a
seed), so the results compare one to one with the oracle and the P4.3 estimate runs.
The policy drives reach and approach and decides when to close; GraspSim's own close and hold
follow (as the expert). E2 monitors on. Runs in the LeRobot env (imports mjorbit from source):

  P4_SCENE=p4_espa_arm2_cam_scene.xml MUJOCO_GL=egl bash lrrun.sh python $P4/eval_act.py \
      --ckpt <pretrained_model> --split validation --n 25 --workers 8 --out <file.json>
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

_R = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_R / "simulator" / "src"))  # mjorbit from source (not installed in the LeRobot env)
sys.path.insert(0, str(_R / "projects" / "p3_capture"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from cams import CamRig  # noqa: E402
from eval_est import KEEP, wilson  # noqa: E402
from gen_demos import SCENE, SPLITS  # noqa: E402
from grasp_sim import GraspCfg, GraspSim  # noqa: E402

PUSH_MAX = 0.002  # m/s, SPEC_4 section 1
_W = {}


def _init(ckpt, device, ens, m, replan, shadow=False, expert_reach=False):
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
    rig = CamRig(sim.km)
    from act_policy import ACTGraspPolicy
    pol = ACTGraspPolicy(ckpt, rig, device=device, ens=ens, m=m, replan=replan, shadow=shadow, expert_reach=expert_reach)
    sim.policy = pol
    _W.update(sim=sim, rig=rig, pol=pol)


def _episode(job):
    seed, holdout = job
    sim, rig, pol = _W["sim"], _W["rig"], _W["pol"]
    draw = np.random.default_rng(seed + 12345)  # as gen_demos.py
    spin = float(draw.uniform(0.0, 2.0))
    axis = int(draw.integers(3))
    dr = rig.randomize(draw, holdout_sun=holdout)
    pol.reset()
    t0 = time.time()
    r = sim.run(np.random.default_rng(seed), spin_deg_s=spin, spin_axis=axis)
    m = dict(seed=seed, spin=spin, axis=axis, sun_I=dr["sun_I"], wall_s=round(time.time() - t0, 1),
             n_pred=pol.n_pred, **{k: r.get(k) for k in KEEP})
    L = pol.log
    for ph in ("reach", "approach"):
        e = [x[2] for x in L if x[1] == ph]
        a = [x[3] for x in L if x[1] == ph]
        if e:
            m[f"tgt_{ph}_mm_med"] = float(np.median(e) * 1000)
            m[f"tgt_{ph}_mm_max"] = float(np.max(e) * 1000)
            m[f"tgt_{ph}_deg_med"] = float(np.median(a))
    app = [x for x in L if x[1] == "approach"]
    if app:
        m["t_approach_s"] = float(app[-1][0] - app[0][0])
        m["closed_by_policy"] = bool(app[-1][4])
        m["expert_test_at_close"] = bool(app[-1][5])  # was the expert's 1 cm close condition met?
        m["tgt_err_at_close_mm"] = float(app[-1][2] * 1000)
    # every 0.5 s: t, phase, |target - expert| mm, |target - grip| mm, |expert - grip| mm, close flag
    m["trace"] = [(round(x[0], 2), x[1], round(x[2] * 1000, 1), round(x[6] * 1000, 1), round(x[7] * 1000, 1), int(x[4]))
                  for x in L[::5]]
    m["p4_success"] = bool(m["success"]) and not m["aborted"] and not m["collision"] and m["push_dv"] <= PUSH_MAX
    return m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ckpt", required=True)
    ap.add_argument("--split", choices=list(SPLITS), default="validation")
    ap.add_argument("--start", type=int, default=0)
    ap.add_argument("--n", type=int, default=25)
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--device", default="cpu")
    ap.add_argument("--ens", choices=("temporal", "none"), default="temporal")
    ap.add_argument("--m", type=float, default=0.01, help="temporal-ensembling weight exp(-m i)")
    ap.add_argument("--replan", type=int, default=1, help="frames between predictions (chunk execution with --ens none)")
    ap.add_argument("--shadow", action="store_true", help="diagnostic: the expert drives, ACT only predicts (live open-loop error)")
    ap.add_argument("--expert-reach", action="store_true", help="hybrid diagnostic: classical reach (true pose), ACT from approach on")
    ap.add_argument("--holdout-sun", action="store_true")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    base = SPLITS[a.split] * 1000
    jobs = [(base + a.start + i, a.holdout_sun) for i in range(a.n)]
    t0 = time.time()
    ms = []
    out = Path(a.out)
    with Pool(a.workers, initializer=_init, initargs=(a.ckpt, a.device, a.ens, a.m, a.replan, a.shadow, a.expert_reach)) as pool, \
            out.with_suffix(".partial.jsonl").open("w") as fh:
        for i, m in enumerate(pool.imap_unordered(_episode, jobs, chunksize=1), 1):
            ms.append(m)
            fh.write(json.dumps(m, default=float) + "\n")
            fh.flush()
            print(f"  {i}/{len(jobs)} seed {m['seed']} {m['outcome']} ok={int(m['p4_success'])} "
                  f"abort={m['abort_reason'] or '-'} push {m['push_dv'] * 1000:.2f} mm/s "
                  f"app {m.get('tgt_approach_mm_med', np.nan):.1f} mm, {(time.time() - t0) / 60:.1f} min", flush=True)
    ms.sort(key=lambda m: m["seed"])
    n = len(ms)
    ks = sum(m["p4_success"] for m in ms)
    from collections import Counter
    summ = dict(mode="act", ckpt=a.ckpt, ens=a.ens, m=a.m, replan=a.replan, shadow=a.shadow, expert_reach=a.expert_reach, split=a.split, n=n, holdout_sun=a.holdout_sun,
                success=ks / n, success_ci95=wilson(ks, n),
                sim_success=sum(bool(m["success"]) and not m["aborted"] for m in ms) / n,
                collisions=sum(bool(m["collision"]) for m in ms),
                push_over_limit=sum(m["push_dv"] > PUSH_MAX for m in ms),
                aborts=dict(Counter(m["abort_reason"] for m in ms if m["aborted"])),
                outcomes=dict(Counter(m["outcome"] for m in ms)),
                push_dv_mm_s_max=float(max(m["push_dv"] for m in ms) * 1000),
                bar_shift_mm_max=float(np.nanmax([m["bar_shift_mm"] for m in ms])),
                closed_by_policy=sum(bool(m.get("closed_by_policy")) for m in ms),
                expert_test_at_close=sum(bool(m.get("expert_test_at_close")) for m in ms),
                wall_min=round((time.time() - t0) / 60, 1))
    for k in ("tgt_reach_mm_med", "tgt_approach_mm_med", "tgt_approach_mm_max", "tgt_err_at_close_mm", "t_approach_s"):
        v = [m[k] for m in ms if k in m]
        if v:
            summ[k] = dict(median=float(np.median(v)), max=float(np.max(v)))
    out.write_text(json.dumps(dict(summary=summ, episodes=ms), indent=1, default=float))
    print(json.dumps(summ, indent=1, default=float))


if __name__ == "__main__":
    main()
