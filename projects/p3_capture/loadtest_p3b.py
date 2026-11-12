"""3B grip load test (SPEC_3 section 6).

    python ../projects/p3_capture/loadtest_p3b.py [--n 10] [--squeeze 0.012 0.03] [--out f.json]

For each case (validation seeds 100000+), grasp as in 3B (free-flying), wait 5 s, then ramp
an internal wrench, equal and opposite on client and servicer, along +-x, +-y, +-z of
the client frame (x = into the capture face, y = bar axis, z = across the jaws' closing
direction... see SPEC_3 section 6). Forces ramp at 1 N/s to 30 N, torques at 0.5 N m/s
to 15 N m. Reports the load at first slip (sustained bar motion relative to the gripper
> 2 mm/s or > 1 deg/s, gross motion > 15 mm or 10 deg, or a pad losing contact), the load
at first arm-joint torque saturation, and the elastic deflection at the 3C requirement
(2 N, 1.5 N m).
"""

from __future__ import annotations

import argparse
import json
import sys
from multiprocessing import Pool
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import numpy as np  # noqa: E402

_SIM = None
AXES = "xyz"
JOINTS = ["sh_yaw", "sh_pitch", "elbow", "wr_pitch", "wr_yaw", "wr_roll"]


def _init(scene):
    import os
    import time
    global _SIM
    from grasp_sim import GraspCfg, GraspSim
    time.sleep(0.3 * (os.getpid() % 16))
    for k in range(10):
        try:
            _SIM = GraspSim(GraspCfg(scene=scene))
            return
        except ValueError:
            time.sleep(0.5 + 0.5 * k)
    raise RuntimeError("could not build GraspSim")


def _run(job):
    seed, over = job
    for k, v in over.items():
        setattr(_SIM.cfg, k, v)
    r = _SIM.run(np.random.default_rng(seed))
    keep = ("grasped", "clamp_force", "load_at_slip", "slip_kind", "load_at_joint_sat",
            "sat_joint", "load_max_reached", "slip_pos_mm_max", "slip_rot_deg_max",
            "defl_mm_at_req", "defl_deg_at_req")
    return {**over, "seed": seed, **{k: r[k] for k in keep}}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=10)
    ap.add_argument("--squeeze", type=float, nargs="*", default=[0.012, 0.03])
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--scene", default="p3b_scene.xml")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    jobs = []
    for sq in args.squeeze:
        for kind, rate, mx in (("force", 1.0, 30.0), ("torque", 0.5, 15.0)):
            for ax in range(3):
                for sg in (1.0, -1.0):
                    over = dict(grip_squeeze=sq, load_kind=kind, load_axis=ax, load_sign=sg,
                                load_rate=rate, load_max=mx)
                    jobs += [(100000 + i, over) for i in range(args.n)]
    print(f"scene {args.scene}")
    with Pool(args.workers, initializer=_init, initargs=(args.scene,)) as pool:
        res = pool.map(_run, jobs)
    rows = []
    print(f"{'squeeze':>8s} {'load':>10s} {'clamp N':>8s} {'slip load med/min':>18s} "
          f"{'slip kind':>12s} {'joint sat load med/min':>23s} {'sat joint':>10s} "
          f"{'held to max':>11s} {'defl@req mm/deg':>16s}")
    for sq in args.squeeze:
        for kind in ("force", "torque"):
            for ax in range(3):
                for sg in (1.0, -1.0):
                    rs = [r for r in res if r["grip_squeeze"] == sq and r["load_kind"] == kind
                          and r["load_axis"] == ax and r["load_sign"] == sg and r["grasped"]]
                    if not rs:
                        continue
                    sl = np.array([r["load_at_slip"] for r in rs], float)
                    js = np.array([r["load_at_joint_sat"] for r in rs], float)
                    kinds = sorted({r["slip_kind"] for r in rs if r["slip_kind"]})
                    sj = sorted({JOINTS[r["sat_joint"]] for r in rs if r["sat_joint"] >= 0})
                    held = float(np.mean(~np.isfinite(sl)))
                    lab = f"{kind[0].upper()} {'+' if sg > 0 else '-'}{AXES[ax]}"
                    fmt = lambda a: (f"{np.nanmedian(a):.2f}/{np.nanmin(a):.2f}"  # noqa: E731
                                     if np.isfinite(a).any() else "none")
                    print(f"{sq:8.3f} {lab:>10s} {np.median([r['clamp_force'] for r in rs]):8.1f} "
                          f"{fmt(sl):>18s} {','.join(kinds) or '-':>12s} {fmt(js):>23s} "
                          f"{','.join(sj) or '-':>10s} {held:11.0%} "
                          f"{np.nanmax([r['defl_mm_at_req'] for r in rs]):7.2f}/"
                          f"{np.nanmax([r['defl_deg_at_req'] for r in rs]):.2f}")
                    rows.append(dict(squeeze=sq, load=lab, n=len(rs),
                                     clamp_N=float(np.median([r["clamp_force"] for r in rs])),
                                     slip_load_median=float(np.nanmedian(sl)) if np.isfinite(sl).any() else None,
                                     slip_load_min=float(np.nanmin(sl)) if np.isfinite(sl).any() else None,
                                     slip_kinds=kinds, joint_sat_load_min=float(np.nanmin(js)) if np.isfinite(js).any() else None,
                                     sat_joints=sj, held_to_max=held,
                                     defl_mm_at_req_max=float(np.nanmax([r["defl_mm_at_req"] for r in rs])),
                                     defl_deg_at_req_max=float(np.nanmax([r["defl_deg_at_req"] for r in rs]))))
    if args.out:
        Path(args.out).write_text(json.dumps({"rows": rows, "raw": res}, indent=2, default=float))
        print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
