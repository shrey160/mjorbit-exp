"""Compare two eval_est.py result files episode by episode (same seeds): are the outcome
metrics identical?   python cmp_runs.py <a.json> <b.json>"""
import json
import sys

KEYS = ("success", "outcome", "push_dv", "bar_shift_mm", "t_grasp", "collision", "peak_force")
a, b = (json.load(open(f))["episodes"] for f in sys.argv[1:3])
bs = {m["seed"]: m for m in b}
same = 0
for m in a:
    o = bs.get(m["seed"])
    diff = None if o is None else {k: (m.get(k), o.get(k)) for k in KEYS if str(m.get(k)) != str(o.get(k))}
    same += diff == {}
    if diff:
        print(m["seed"], "DIFF", diff)
print(f"{same}/{len(a)} identical")
