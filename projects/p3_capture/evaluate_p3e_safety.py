"""3E step E2: evaluate safety monitors and aborts with injected faults (SPEC_3 section 12).

    python ../projects/p3_capture/evaluate_p3e_safety.py --n 50 [--fault KIND] [--no-act]
        [--no-mon] [--spin-max 2] [--split validation|tune] [--set k=v ...] [--mon k=v ...]
        [--out f.json]

Per case: fly-in from the spin-axis hold, client spin uniform in [0, spin_max] deg/s about a
random body axis, one fault of the given kind (random phase, time and size; see
GraspSim._draw_fault). The case seed is matched across monitor on / off / record-only.
Seeds: tune = 3000 x 1000 (threshold setting), validation = 100 x 1000.
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

SPLIT_SEEDS = {"validation": 100, "tune": 3000, "test": 1000, "val2": 4000}
_SIM = None


def _parse(kvs):
    out = {}
    for kv in kvs:
        k, v = kv.split("=")
        if v.lower() in ("true", "false"):
            out[k] = v.lower() == "true"
            continue
        try:
            out[k] = float(v) if any(c in v for c in ".e") else int(v)
        except ValueError:
            out[k] = v
    return out


def _init(cfg_kw, mon_kw):
    import os
    import time
    global _SIM
    from grasp_sim import GraspCfg, GraspSim
    time.sleep(0.3 * (os.getpid() % 16))
    for k in range(10):
        try:
            _SIM = GraspSim(GraspCfg(**cfg_kw))
            _SIM.mon_overrides = mon_kw
            return
        except ValueError:
            time.sleep(0.5 + 0.5 * k)
    raise RuntimeError("could not build GraspSim")


def _clean(v):
    if isinstance(v, dict):
        return {k: _clean(x) for k, x in v.items()}
    if isinstance(v, (np.floating, float)):
        return float(v)
    if isinstance(v, (np.bool_, bool)):
        return bool(v)
    if isinstance(v, (np.integer,)):
        return int(v)
    return v


def _run(args):
    seed, reset_kw = args
    r = _SIM.run(np.random.default_rng(seed), **reset_kw)
    r.pop("trace", None)
    return _clean({**r, "seed": seed, "spin": reset_kw["spin_deg_s"]})


def wilson(k, n, z=1.96):
    if n == 0:
        return (np.nan, np.nan)
    p = k / n
    den = 1 + z * z / n
    c = (p + z * z / (2 * n)) / den
    h = z * np.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return (round(c - h, 3), round(c + h, 3))


def summarize(res, fault):
    n = len(res)
    oc = Counter(r["outcome"] for r in res)
    fired = [r for r in res if np.isfinite(r["t_fault"])]
    # a detection counts if it comes at or after the fault; earlier ones are false alarms
    det = [r for r in fired if np.isfinite(r["t_detect"]) and r["t_detect"] >= r["t_fault"] - 1e-9]
    false_alarm = [r for r in res if np.isfinite(r["t_detect"]) and
                   (not np.isfinite(r["t_fault"]) or r["t_detect"] < r["t_fault"] - 1e-9)]
    delay = np.array([r["t_detect"] - r["t_fault"] for r in det])
    tsafe = np.array([r["t_to_safe"] for r in res if np.isfinite(r["t_to_safe"])])
    s = {
        "n": n, "fault": fault or "none",
        "outcomes": dict(oc),
        "captured": oc.get("captured", 0) / n,
        "collision": oc.get("collision", 0) / n, "collision_ci95": wilson(oc.get("collision", 0), n),
        "safe_or_captured": (oc.get("captured", 0) + oc.get("safe_abort", 0)) / n,
        "safe_or_captured_ci95": wilson(oc.get("captured", 0) + oc.get("safe_abort", 0), n),
        "fault_fired": len(fired),
        "detected": len(det), "detect_rate": len(det) / max(len(fired), 1),
        "detect_reasons": dict(Counter(r["detect_reason"] for r in det)),
        "false_alarms": len(false_alarm),
        "false_alarm_reasons": dict(Counter(r["detect_reason"] for r in false_alarm)),
        "detect_delay_s_median": float(np.median(delay)) if delay.size else np.nan,
        "detect_delay_s_max": float(delay.max()) if delay.size else np.nan,
        "t_to_safe_s_median": float(np.median(tsafe)) if tsafe.size else np.nan,
        "t_to_safe_s_max": float(tsafe.max()) if tsafe.size else np.nan,
        "min_clear_m_min": float(min(r["min_clear"] for r in res)),
        "hit_force_N_max": float(max(r["hit_force_max"] for r in res)),
        "fault_phases": dict(Counter(r["fault_phase"] for r in fired)),
    }
    if "mon_max" in res[0]:
        keys = res[0]["mon_max"].keys()
        s["mon_signal_max"] = {k: float(max(r["mon_max"][k] for r in res)) for k in keys}
        by = {}
        for r in res:
            for ph, dd in r["mon_max_by_phase"].items():
                for k, v in dd.items():
                    by.setdefault(ph, {})[k] = max(by.get(ph, {}).get(k, 0.0), v)
        s["mon_signal_max_by_phase"] = by
    s["cases"] = [{k: r[k] for k in ("seed", "spin", "outcome", "fault_phase", "fault_mag", "t_fault",
                                     "detect_reason", "t_detect", "t_to_safe", "min_clear",
                                     "hit_force_max", "abort_phase")} for r in res]
    return s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=50)
    ap.add_argument("--fault", default="")
    ap.add_argument("--no-mon", action="store_true")
    ap.add_argument("--no-act", action="store_true", help="monitors record but do not abort")
    ap.add_argument("--spin-max", type=float, default=2.0)
    ap.add_argument("--no-flyin", action="store_true")
    ap.add_argument("--split", choices=list(SPLIT_SEEDS), default="validation")
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--set", nargs="*", default=[], metavar="K=V", help="GraspCfg overrides")
    ap.add_argument("--mon", nargs="*", default=[], metavar="K=V", help="MonitorCfg overrides")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    cfg_kw = dict(flyin=not args.no_flyin, mon=not args.no_mon, mon_act=not args.no_act,
                  fault_kind=args.fault, **_parse(args.set))
    mon_kw = _parse(args.mon)
    base = SPLIT_SEEDS[args.split] * 1000
    jobs = []
    for i in range(args.n):
        spin = float(np.random.default_rng([base + i, 31]).uniform(0.0, args.spin_max))
        jobs.append((base + i, dict(spin_deg_s=spin)))
    with Pool(args.workers, initializer=_init, initargs=(cfg_kw, mon_kw)) as pool:
        res = pool.map(_run, jobs)
    s = {"settings": {**cfg_kw, "mon_overrides": mon_kw, "split": args.split,
                      "spin_max": args.spin_max}, **summarize(res, args.fault)}
    for k, v in s.items():
        if k not in ("cases", "mon_signal_max_by_phase"):
            print(f"{k:24s} {v}")
    if args.out:
        Path(args.out).write_text(json.dumps(s, indent=1))
        print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
