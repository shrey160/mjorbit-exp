"""Replay chosen E3 validation missions and print the post-capture grip/pad history."""
import json, sys, time
from multiprocessing import Pool
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
import evaluate_e3 as E

D = Path(__file__).parent

def run(c):
    import stack_sim
    ev = []
    orig = stack_sim.StackRun._grip_watch
    def watch(self, n_pads):
        if n_pads < 2 and self.t >= 0.05:
            ev.append((round(self.t, 2), getattr(self, "phase_tag", ""), n_pads))
        return orig(self, n_pads)
    stack_sim.StackRun._grip_watch = watch
    r = E._mission(c)
    # group consecutive steps into gaps
    gaps, cur = [], None
    for t, ph, n in ev:
        if cur and t - cur[1] < 0.015 and ph == cur[2]:
            cur[1] = t
        else:
            if cur: gaps.append(cur)
            cur = [t, t, ph]
    if cur: gaps.append(cur)
    return dict(seed=c["seed"], fault=c["fault"], loose=round(c["loose"], 3), outcome=E.classify(c, r),
                grip_detect=r.get("grip_detect"), n_gaps=len(gaps),
                gaps=[(g[0], round(g[1] - g[0] + 0.01, 2), g[2]) for g in gaps[:12]])

if __name__ == "__main__":
    seeds = [int(s) for s in sys.argv[1:]]
    cases = [c for c in json.loads((D / "e3_handoff_validation.json").read_text())["cases"] if c["seed"] in seeds]
    with Pool(len(cases)) as p:
        for o in p.imap_unordered(run, cases):
            print(json.dumps(o), flush=True)
