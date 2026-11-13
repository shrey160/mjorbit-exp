"""P4.4a check (SPEC_4 4.4): the policy hook on the plain arm2 scene, no cameras.
Runs the same episodes (gen_demos case draws) with the classical expert (policy None), the
expert through the policy path (ExpertPolicy) and DART noise (DartPolicy); prints outcome,
push, bar shift, grasp time and approach tracking error side by side.

python policy_smoke.py [n=8] [split=validation] [workers=8] [scene=p3e_espa_arm2_scene.xml]"""
from __future__ import annotations

import sys
import time
from multiprocessing import Pool
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "p3_capture"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from grasp_sim import GraspCfg, GraspSim  # noqa: E402
from policies import DartPolicy, ExpertPolicy  # noqa: E402

SPLITS = {"demo": 8000, "validation": 6000, "test": 7000}
_W = {}


def _init(scene):
    import os
    for k in range(20):
        try:
            _W["sim"] = GraspSim(GraspCfg(scene=scene, hold_s=5.0, mon=True, mon_act=True))
            return
        except ValueError as e:
            if "parse" not in str(e).lower() and "empty" not in str(e).lower():
                raise
            time.sleep(0.3 + 0.3 * k + 0.05 * (os.getpid() % 10))


def _episode(job):
    seed, mode = job
    sim = _W["sim"]
    draw = np.random.default_rng(seed + 12345)  # as gen_demos.py
    spin, axis = float(draw.uniform(0.0, 2.0)), int(draw.integers(3))
    pol = {"expert": None, "policy": ExpertPolicy(), "dart": DartPolicy()}[mode]
    if pol is not None:
        pol.reset(seed + 777)
    sim.policy = pol
    t0 = time.time()
    r = sim.run(np.random.default_rng(seed), spin_deg_s=spin, spin_axis=axis)
    return dict(seed=seed, mode=mode, spin=spin, ok=bool(r["success"]) and not r["aborted"] and not r["collision"],
                outcome=r["outcome"], abort=r["abort_reason"], push=r["push_dv"] * 1000, shift=r["bar_shift_mm"],
                t_grasp=r["t_grasp"], app_err=r["approach_err_max"] * 1000, wall=time.time() - t0,
                noise=getattr(pol, "noise_max", 0.0) * 1000)


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 8
    split = sys.argv[2] if len(sys.argv) > 2 else "validation"
    workers = int(sys.argv[3]) if len(sys.argv) > 3 else 8
    scene = sys.argv[4] if len(sys.argv) > 4 else "p3e_espa_arm2_scene.xml"
    base = SPLITS[split] * 1000
    jobs = [(base + i, m) for i in range(n) for m in ("expert", "policy", "dart")]
    with Pool(workers, initializer=_init, initargs=(scene,)) as pool:
        R = pool.map(_episode, jobs, chunksize=1)
    by = {(r["seed"], r["mode"]): r for r in R}
    print(f"{'seed':>8} {'spin':>5} | {'mode':>6} {'ok':>3} {'outcome':>16} {'push':>5} {'shift':>5} "
          f"{'t_grasp':>7} {'apperr':>6} {'noise':>5}")
    for i in range(n):
        for m in ("expert", "policy", "dart"):
            r = by[(base + i, m)]
            print(f"{r['seed']:>8} {r['spin']:5.2f} | {m:>6} {int(r['ok']):>3} {r['outcome'][:16]:>16} "
                  f"{r['push']:5.2f} {r['shift']:5.2f} {r['t_grasp']:7.2f} {r['app_err']:6.1f} {r['noise']:5.1f}"
                  + (f" abort={r['abort']}" if r["abort"] else ""))
    for m in ("expert", "policy", "dart"):
        rs = [by[(base + i, m)] for i in range(n)]
        print(f"{m}: {sum(r['ok'] for r in rs)}/{n} ok, push max {max(r['push'] for r in rs):.2f} mm/s, "
              f"approach err max {max(r['app_err'] for r in rs):.1f} mm, wall {np.mean([r['wall'] for r in rs]):.1f} s")


if __name__ == "__main__":
    main()
