"""P4.3 spin-rate breakdown of the closed-loop results (13 Nov): per spin bin, success, push,
bar shift and estimate error, for the oracle, the estimate and the held-out sun runs."""
import json, sys
import numpy as np
import os
os.chdir(os.path.dirname(os.path.abspath(__file__)))

FILES = [("oracle", "p43_val_oracle.json"), ("estimate", "p43_val_est_final.json"),
         ("est, held-out sun", "p43_val_est_final_holdout.json")]
BINS = [0.0, 0.5, 1.0, 1.5, 2.01]
KEYS = ["est_settle_mm", "est_reach_mm", "est_approach_mm", "est_last_mm"]

for name, fn in FILES:
    ep = json.load(open(fn))["episodes"]
    print(f"\n== {name} ({fn}), n={len(ep)}")
    print(f"{'spin deg/s':>11} {'n':>3} {'succ':>5} {'push med/max mm/s':>18} {'shift max mm':>12}"
          + "".join(f" {k[4:-3]+' med/p95':>15}" for k in KEYS if k in ep[0]))
    for lo, hi in zip(BINS[:-1], BINS[1:]):
        b = [e for e in ep if lo <= e["spin"] < hi]
        if not b:
            continue
        push = np.array([e["push_dv"] for e in b]) * 1000
        line = (f"{lo:4.1f}-{min(hi,2.0):3.1f}    {len(b):3d} {sum(e['p4_success'] for e in b):5d}"
                f" {np.median(push):8.2f} / {push.max():5.2f}  {max(e['bar_shift_mm'] for e in b):12.2f}")
        for k in KEYS:
            if k in b[0]:
                v = np.array([e[k] for e in b])
                line += f" {np.median(v):7.1f} / {np.percentile(v, 95):5.1f}"
        print(line)
    if "est_last_mm" in ep[0]:
        s = np.array([e["spin"] for e in ep]); v = np.array([e["est_last_mm"] for e in ep])
        print(f"   corr(spin, error at close) = {np.corrcoef(s, v)[0, 1]:+.2f}")
