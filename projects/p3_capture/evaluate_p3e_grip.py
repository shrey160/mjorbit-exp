"""3E step E2b: post-capture grip monitor under a loose-jaw fault (SPEC_3 section 12).

    python ../projects/p3_capture/evaluate_p3e_grip.py --n 40 --loose 1.0 [--no-act] [--no-mon]
        [--split validation|tune] [--set k=v ...] [--out f.json]

Each case: capture (3B), brakes engage with each jaw grip_loose_mm looser than where it
closed (preload loss), then 3C detumble and identification load the grip. With the
monitor acting, a slip or pad-loss alarm safes the stack (no actuation) for safe_coast_s.
Outcome: client retained (both pads on the bar, bar within 15 mm) or lost.
"""

from __future__ import annotations

import argparse
import json
import sys
from collections import Counter
from multiprocessing import Pool
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import numpy as np  # noqa: E402

SPLIT = {"validation": 210000, "tune": 310000}


def _run(job):
    import os
    import time
    seed, over = job
    from stack_sim import StackCfg, StackRun
    over = dict(over)
    if over["grip_loose_mm"] < 0:  # random fault size per case (matched across settings)
        over["grip_loose_mm"] = float(np.random.default_rng([seed, 7]).uniform(0.05, 0.3))
    cfg = StackCfg(**over)
    for k in range(10):
        try:
            sr = StackRun(seed, cfg)
            res, _ = sr.capture_identify()
            if res.get("grasped") and "retained" not in res:
                res.update(sr.grip_outcome())
            return {"seed": seed, "loose_mm": cfg.grip_loose_mm, **{k2: v for k2, v in res.items()
                                     if not isinstance(v, (list, dict))}}
        except ValueError as e:
            if "parse" not in str(e).lower() and "empty" not in str(e).lower():
                raise
            time.sleep(0.3 + 0.3 * k + 0.01 * (os.getpid() % 10))
    raise RuntimeError("compile failed repeatedly")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=40)
    ap.add_argument("--loose", type=float, default=0.0)
    ap.add_argument("--no-mon", action="store_true")
    ap.add_argument("--no-act", action="store_true")
    ap.add_argument("--split", choices=list(SPLIT), default="validation")
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--set", nargs="*", default=[], metavar="K=V")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    over = dict(grip_loose_mm=args.loose, grip_mon=not args.no_mon, grip_mon_act=not args.no_act)
    for kv in args.set:
        k, v = kv.split("=")
        if v.lower() in ("true", "false"):
            over[k] = v.lower() == "true"
            continue
        try:
            over[k] = float(v)
        except ValueError:
            over[k] = v
    jobs = [(SPLIT[args.split] + i, over) for i in range(args.n)]
    with Pool(args.workers) as pool:
        res = pool.map(_run, jobs)
    ok = [r for r in res if r.get("grasped")]
    f = lambda k: [float(r[k]) for r in ok if k in r and r[k] is not None]  # noqa: E731
    summ = {
        "settings": over, "n": args.n, "grasped": len(ok),
        "retained": sum(r["retained"] for r in ok), "lost": sum(not r["retained"] for r in ok),
        "alarms": sum(r["grip_detect"] != "None" for r in ok),
        "alarm_kinds": dict(Counter(r["grip_detect"].split(",")[0].strip("('") for r in ok
                                    if r["grip_detect"] != "None")),
        "safed": sum(r["safed"] for r in ok),
        "lost_safed": sum((not r["retained"]) and r["safed"] for r in ok),
        "lost_not_safed": sum((not r["retained"]) and not r["safed"] for r in ok),
        "shift_mm_max_median": float(np.median(f("gm_shift_mm"))) if ok else np.nan,
        "shift_mm_max_max": float(np.max(f("gm_shift_mm"))) if ok else np.nan,
        "slip_vel_mm_s_max": float(np.max(f("gm_vel_mm_s"))) if ok else np.nan,
        "pad_off_s_max": float(np.max(f("gm_pad_off_s"))) if ok else np.nan,
        "clamp_min_N_median": float(np.median(f("clamp_min_N"))) if f("clamp_min_N") else np.nan,
        "mass_err_pct_abs_max": float(np.max(np.abs(f("mass_err_pct")))) if f("mass_err_pct") else np.nan,
        "cases": ok,
    }
    for k, v in summ.items():
        if k != "cases":
            print(f"{k:22s} {v}")
    if args.out:
        Path(args.out).write_text(json.dumps(summ, indent=1, default=str))
        print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
