"""arm2 smoke test (13 Nov): classical 3B grasp on p3e_espa vs p3e_espa_arm2, same seeds.
Records success, client push, approach error and, through step_hook, the peak arm joint
speed (limit 10 deg/s, not enforced in physics) and the peak arm torque as a fraction of
the actuator limit.  python arm2_smoke.py --n 20 --spin 0 --workers 8 --out x.json"""
import argparse
import json
import os
import sys
import time
from multiprocessing import Pool
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "p2_approach"))
import numpy as np  # noqa: E402

_SIM = None
_REC = {}
SCENES = {"espa": "p3e_espa_scene.xml", "arm2": "p3e_espa_arm2_scene.xml"}


def _hook(loc):
    sim, d = loc["self"], loc["d"]
    from grasp_sim import ARM
    qd = np.abs(np.asarray(d.qvel)[ARM])
    _REC["qd_max"] = np.maximum(_REC["qd_max"], qd)
    tau = loc.get("tau")
    if tau is not None:
        lim = sim.km.actuator_ctrlrange[6:12, 1]
        _REC["tau_frac"] = np.maximum(_REC["tau_frac"], np.abs(np.asarray(tau)) / lim)
    ph = loc.get("phase", "")
    if ph in ("reach", "approach"):
        _REC.setdefault("qd_phase", {}).setdefault(ph, 0.0)
        _REC["qd_phase"][ph] = max(_REC["qd_phase"][ph], float(np.degrees(qd.max())))


def _init(scene, flyin):
    global _SIM
    from grasp_sim import GraspCfg, GraspSim
    time.sleep(0.3 * (os.getpid() % 16))
    for k in range(10):
        try:
            _SIM = GraspSim(GraspCfg(scene=scene, flyin=flyin))
            _SIM.step_hook = _hook
            return
        except ValueError:
            time.sleep(0.5 + 0.5 * k)
    raise RuntimeError("could not build GraspSim")


def _run(args):
    seed, kw = args
    _REC.clear()
    _REC["qd_max"], _REC["tau_frac"] = np.zeros(6), np.zeros(6)
    t0 = time.time()
    r = _SIM.run(np.random.default_rng(seed), **kw)
    keep = ("success", "grasped", "push_dv", "approach_err_max", "other_contact", "peak_force",
            "both_pads_end", "phase_end", "t_grasp", "ik_err_pos")
    out = {k: (float(r[k]) if isinstance(r[k], (np.floating, float)) else r[k]) for k in keep if k in r}
    out.update(seed=seed, wall_s=round(time.time() - t0, 1),
               qd_max_deg=np.degrees(_REC["qd_max"]).round(2).tolist(),
               tau_frac=_REC["tau_frac"].round(3).tolist(), qd_phase_deg=_REC.get("qd_phase", {}))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", choices=list(SCENES), required=True)
    ap.add_argument("--n", type=int, default=20)
    ap.add_argument("--spin", type=float, default=0.0)
    ap.add_argument("--flyin", action="store_true")
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    kw = dict(spin_deg_s=a.spin)
    jobs = [(100000 + i, kw) for i in range(a.n)]  # evaluate_p3b validation seeds
    with Pool(a.workers, initializer=_init, initargs=(SCENES[a.arm], a.flyin)) as pool:
        res = pool.map(_run, jobs)
    push = np.array([r["push_dv"] for r in res]) * 1000
    qd = np.array([r["qd_max_deg"] for r in res])
    tf = np.array([r["tau_frac"] for r in res])
    s = dict(arm=a.arm, n=a.n, spin=a.spin, flyin=a.flyin,
             success=int(sum(r["success"] for r in res)),
             p4_push_ok=int(sum(p <= 2.0 for p in push)),
             other_contact=int(sum(bool(r.get("other_contact")) for r in res)),
             push_mm_s_median=round(float(np.median(push)), 2), push_mm_s_max=round(float(push.max()), 2),
             approach_err_mm_max=round(1000 * max(r["approach_err_max"] for r in res), 1),
             joint_speed_deg_s_max=qd.max(0).round(1).tolist(),
             over_10deg_s=int((qd.max(1) > 10.0).sum()),
             tau_frac_max=tf.max(0).round(2).tolist(),
             fails=[(r["seed"], r["phase_end"]) for r in res if not r["success"]])
    print(json.dumps(s, indent=1))
    json.dump(dict(summary=s, episodes=res), open(a.out, "w"), indent=1)


if __name__ == "__main__":
    main()
