"""Evaluate the 3B classical grasp over randomized cases (SPEC_3 section 5).

    python ../projects/p3_capture/evaluate_p3b.py [--n 100] [--mode free-flying|free-floating]
        [--dt 0.002] [--split validation|test] [--workers 8] [--out f.json] [--set k=v ...]

Per case (seeded, matched across settings): servicer position error up to +-pos_err per
axis at the pre-grasp point, velocity error up to +-vel_err, attitude error up to
+-att_err_deg, client tilt up to +-client_tilt_deg, client LVLH-fixed (spinning at the
orbit rate) unless --no-spin.
"""

from __future__ import annotations

import argparse
import json
import sys
from multiprocessing import Pool
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "p2_approach"))

import numpy as np  # noqa: E402

SPLIT_SEEDS = {"validation": 100, "test": 1000}
_SIM = None


def _init(cfg_kw):
    """mjorbit compiles through a temp XML beside the scene file; parallel workers can race
    on it, so stagger the start and retry."""
    import os
    import time
    global _SIM
    from grasp_sim import GraspCfg, GraspSim
    time.sleep(0.3 * (os.getpid() % 16))
    for k in range(10):
        try:
            _SIM = GraspSim(GraspCfg(**cfg_kw))
            return
        except ValueError:
            time.sleep(0.5 + 0.5 * k)
    raise RuntimeError("could not build GraspSim")


def _run(args):
    seed, reset_kw = args
    r = _SIM.run(np.random.default_rng(seed), **reset_kw)
    return {k: (float(v) if isinstance(v, (np.floating, float)) else v) for k, v in r.items()}


def wilson(k, n, z=1.96):
    p = k / n
    den = 1 + z * z / n
    c = (p + z * z / (2 * n)) / den
    h = z * np.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return (round(c - h, 3), round(c + h, 3))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=100)
    ap.add_argument("--mode", default="free-flying")
    ap.add_argument("--dt", type=float, default=0.002)
    ap.add_argument("--split", choices=list(SPLIT_SEEDS), default="validation")
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--no-spin", action="store_true")
    ap.add_argument("--pos-err", type=float, default=0.08)
    ap.add_argument("--vel-err", type=float, default=0.003)
    ap.add_argument("--att-err", type=float, default=3.0)
    ap.add_argument("--tilt", type=float, default=3.0)
    ap.add_argument("--spin", type=float, default=0.0, help="client spin, deg/s (SPEC_3 10)")
    ap.add_argument("--spin-axis", type=int, default=-1)
    ap.add_argument("--set", nargs="*", default=[], metavar="K=V", help="GraspCfg overrides")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    cfg_kw = dict(mode=args.mode, dt=args.dt)
    for kv in args.set:
        k, v = kv.split("=")
        if v.lower() in ("true", "false"):
            cfg_kw[k] = v.lower() == "true"
            continue
        try:
            cfg_kw[k] = float(v)
        except ValueError:
            cfg_kw[k] = v
    reset_kw = dict(pos_err=args.pos_err, vel_err=args.vel_err, att_err_deg=args.att_err,
                    client_tilt_deg=args.tilt, client_spin=not args.no_spin,
                    spin_deg_s=args.spin, spin_axis=args.spin_axis)
    base = SPLIT_SEEDS[args.split] * 1000
    jobs = [(base + i, reset_kw) for i in range(args.n)]
    with Pool(args.workers, initializer=_init, initargs=(cfg_kw,)) as pool:
        res = pool.map(_run, jobs)
    succ = np.array([r["success"] for r in res])
    arr = lambda k: np.array([r[k] for r in res], float)  # noqa: E731
    summary = {
        "settings": {**cfg_kw, **reset_kw, "n": args.n, "split": args.split},
        "success": float(succ.mean()), "success_ci95": wilson(int(succ.sum()), succ.size),
        "grasped_at_close": float(np.mean([r["grasped"] for r in res])),
        "other_contact_episodes": float(np.mean([r["other_contact"] for r in res])),
        "push_dv_mm_s_median": float(np.median(arr("push_dv")) * 1000),
        "push_dv_mm_s_max": float(arr("push_dv").max() * 1000),
        "peak_contact_force_N_median": float(np.median(arr("peak_force"))),
        "peak_contact_force_N_max": float(arr("peak_force").max()),
        "note_contact_force": "approach + close phases, includes the clamp",
        "bus_att_err_deg_max": float(arr("bus_att_err_max").max()),
        "approach_track_err_m_max": float(arr("approach_err_max").max()),
        "final_bar_dist_mm_max": float(np.nanmax(arr("final_bar_dist")) * 1000),
        "final_slip_speed_mm_s_max": float(np.nanmax(arr("final_rel_speed")) * 1000),
        "bar_shift_in_grip_mm_max": float(np.nanmax(arr("bar_shift_mm"))),
        "clamp_force_N_median": float(np.nanmedian(arr("clamp_force"))),
        "ik_err_pos_mm_max": float(np.nanmax(arr("ik_err_pos")) * 1000),
        "t_grasp_s_median": float(np.nanmedian(arr("t_grasp"))),
        **({"flyin_s_median": float(np.nanmedian(arr("flyin_s"))),
            "flyin_s_max": float(np.nanmax(arr("flyin_s"))),
            "flyin_min_clear_m_min": float(np.nanmin(arr("flyin_min_clear"))),
            "flyin_thrust_sat_frac_max": float(np.nanmax(arr("flyin_sat_frac"))),
            "flyin_end_err_m_max": float(np.nanmax(arr("flyin_end_err_m"))),
            "flyin_end_vrel_mm_s_max": float(np.nanmax(arr("flyin_end_vrel_mm_s"))),
            "flyin_end_att_deg_max": float(np.nanmax(arr("flyin_end_att_deg")))}
           if "flyin_s" in res[0] else {}),
        "failures": [{"seed": base + i, **{k: r[k] for k in ("grasped", "both_pads_end",
                      "final_bar_dist", "final_rel_speed", "other_contact", "phase_end")}}
                     for i, r in enumerate(res) if not r["success"]][:20],
    }
    for k, v in summary.items():
        if k != "failures":
            print(f"{k:32s} {v}")
    if summary["failures"]:
        print("failures (first 20):")
        for f in summary["failures"]:
            print("  ", f)
    if args.out:
        Path(args.out).write_text(json.dumps(summary, indent=2))
        print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
