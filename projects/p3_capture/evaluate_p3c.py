"""Evaluate 3C (detumble, identification, CoM-compensated thrust) over randomised clients.

    python ../projects/p3_capture/evaluate_p3c.py [--n 50] [--set k=v ...] [--out f.json]
Validation seeds 200000+ (client properties, tumble, sensor noise, grasp errors).
"""

from __future__ import annotations

import argparse
import json
import sys
from multiprocessing import Pool
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import numpy as np  # noqa: E402


def _run(job):
    import os
    import time
    seed, over = job
    from stack_sim import StackCfg, StackRun
    cfg = StackCfg(**over)
    for k in range(10):
        try:
            return {"seed": seed, **StackRun(seed, cfg).run()}
        except ValueError as e:  # mjorbit temp-file race on compile
            if "parse" not in str(e).lower() and "empty" not in str(e).lower():
                raise
            time.sleep(0.3 + 0.3 * k + 0.01 * (os.getpid() % 10))
    raise RuntimeError("compile failed repeatedly")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=50)
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--set", nargs="*", default=[], metavar="K=V")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    over = {}
    for kv in args.set:
        k, v = kv.split("=")
        if v.lower() in ("true", "false"):
            over[k] = v.lower() == "true"
            continue
        try:
            over[k] = float(v)
        except ValueError:
            over[k] = v
    jobs = [(200000 + i, over) for i in range(args.n)]
    with Pool(args.workers) as pool:
        res = pool.map(_run, jobs)
    ok = [r for r in res if r.get("grasped")]
    a = lambda k: np.array([r[k] for r in ok], float)  # noqa: E731
    summ = {
        "settings": over, "n": args.n, "grasped": len(ok),
        "pads_lost_episodes": int(sum(r["pads_lost"] for r in ok)),
        "pad_lost_s_max": float(a("pad_lost_s").max()),
        "pad_lost_first_examples": [r["pad_lost_first"] for r in ok if r["pads_lost"]][:6],
        "clamp_min_N_min": float(np.nanmin(a("clamp_min_N"))),
        "bar_shift_3b_mm_max": float(np.nanmax(a("bar_shift_3b_mm"))),
        "tumble_deg_s_max": float(a("tumble_deg_s").max()),
        "detumble_s_median": float(np.median(a("detumble_s"))),
        "detumble_s_max": float(a("detumble_s").max()),
        "wheel_momentum_Nms_max_ideal_tally": float(a("h_rw_max").max()),
        "wheel_h_peak_axis_Nms_max": float(a("hw_max_3c").max()),
        "wheel_saturated_episodes": int(sum(r["sat_s_3c"] > 0 for r in ok)),
        "rcs_prop_g_median": float(np.median(a("rcs_prop_g_3c"))),
        "rcs_prop_g_max": float(a("rcs_prop_g_3c").max()),
        "mass_err_pct_abs_median": float(np.median(np.abs(a("mass_err_pct")))),
        "mass_err_pct_abs_max": float(np.max(np.abs(a("mass_err_pct")))),
        "I_err_rel_pct_median": float(np.median(a("I_err_rel_pct"))),
        "I_err_rel_pct_max": float(np.max(a("I_err_rel_pct"))),
        "I_eig_err_pct_max": float(np.max(a("I_eig_err_pct_max"))),
        "com_err_mm_median": float(np.median(a("d_err_mm"))),
        "com_err_mm_max": float(np.max(a("d_err_mm"))),
        "val_att_peak_no_ff_deg_median": float(np.median(a("val_peak_no_ff_deg"))),
        "val_att_peak_ff_deg_median": float(np.median(a("val_peak_ff_deg"))),
        "val_att_peak_ff_deg_max": float(np.max(a("val_peak_ff_deg"))),
    }
    for k, v in summ.items():
        print(f"{k:32s} {v}")
    if args.out:
        Path(args.out).write_text(json.dumps({"summary": summ, "raw": res}, indent=2, default=float))
        print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
