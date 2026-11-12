"""Evaluate 3D variants (estimates x translation strategy) on matched cases (SPEC_3 section 8).

    python ../projects/p3_capture/evaluate_p3d.py [--n 40] [--variants id/align id/fixed ...]
Variant = estimates/strategy[/Man3DCfg overrides, comma separated], e.g. id/align/unload=true.
Validation seeds 300000+ (client, tumble, sensor noise and commands matched across variants).
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
    seed, spec = job
    from maneuver3d import Man3DCfg, Stack3D
    from stack_sim import StackCfg
    parts = spec.split("/")
    kw = dict(estimates=parts[0], strategy=parts[1])
    for kv in (parts[2].split(",") if len(parts) > 2 else []):
        k_, v_ = kv.split("=")
        kw[k_] = {"true": True, "false": False}.get(v_.lower(), None)
        if kw[k_] is None:
            kw[k_] = float(v_)
    for k in range(10):
        try:
            r = Stack3D(seed, StackCfg(), Man3DCfg(**kw)).run3d()
            r["seed"], r["variant"] = seed, spec
            return r
        except ValueError as e:
            if "parse" not in str(e).lower() and "empty" not in str(e).lower():
                raise
            time.sleep(0.3 + 0.3 * k + 0.01 * (os.getpid() % 10))
    raise RuntimeError("compile failed repeatedly")


def wilson(k, n, z=1.96):
    p = k / n
    den = 1 + z * z / n
    c = (p + z * z / (2 * n)) / den
    h = z * np.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return (round(c - h, 3), round(c + h, 3))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=40)
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--variants", nargs="*", default=["id/align", "id/fixed", "prior/align", "truth/align"])
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    jobs = [(300000 + i, v) for v in args.variants for i in range(args.n)]
    with Pool(args.workers) as pool:
        res = pool.map(_run, jobs)
    rows = []
    keys = [("success", "rate"), ("within_h_budget", "rate"), ("sat_s", "med/max"),
            ("h_start_3d", "med/max"), ("h_end_3d", "med/max"), ("rcs_prop_g", "med/max"),
            ("final_pos_err_m", "med/max"),
            ("final_vel_mm_s", "med/max"), ("final_att_err_deg", "med/max"),
            ("tr_pos_err_m", "med/max"), ("tr_att_err_max_deg", "med/max"),
            ("h_rw_max_3d", "med/max"), ("dv_used", "med/max"), ("align_slew_s", "med/max"),
            ("final_slew_s", "med/max"), ("t_3d_s", "med/max")]
    print(f"{'metric':22s}" + "".join(f"{v[:24]:>26s}" for v in args.variants))
    table = {v: {} for v in args.variants}
    for v in args.variants:
        rs = [r for r in res if r.get("variant") == v]
        table[v]["n"] = len(rs)
        for k, kind in keys:
            a = np.array([r.get(k, np.nan) for r in rs], float)
            if kind == "rate":
                table[v][k] = (float(np.nanmean(a)), wilson(int(np.nansum(a)), len(a)))
            else:
                table[v][k] = (float(np.nanmedian(a)), float(np.nanmax(a))) if np.isfinite(a).any() else None
    for k, kind in keys:
        cells = []
        for v in args.variants:
            x = table[v][k]
            cells.append(f"{x[0]:>26.1%}" if kind == "rate" else
                         (f"{x[0]:>13.3g}/{x[1]:<12.3g}" if x else f"{'-':>26s}"))
        print(f"{k:22s}" + "".join(cells))
    if args.out:
        Path(args.out).write_text(json.dumps({"table": table, "raw": res}, indent=2, default=float))
        print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
