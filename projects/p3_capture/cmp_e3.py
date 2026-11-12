"""Compare E3 stage-B results per seed against a reference results file (wiring regression).
python cmp_e3.py pick <ref.json> <n>      print a seed list covering every outcome / fault
python cmp_e3.py cmp <ref.json> <new.json>  per-seed outcome and key metrics, exact match"""
import json
import sys
from collections import defaultdict

KEYS = ("outcome", "g_outcome", "g_abort_reason", "final_pos_err_m", "final_att_err_deg", "mass_err_pct",
        "rcs_prop_g", "g_flyin_min_clear", "g_flyin_k_min", "gm_pad_drops")


def load(f):
    return {m["seed"]: m for m in json.load(open(f))["missions"]}


if sys.argv[1] == "pick":
    ref, n = load(sys.argv[2]), int(sys.argv[3])
    groups = defaultdict(list)
    for s, m in sorted(ref.items()):
        groups[(m["outcome"], m["fault"])].append(s)
    pick = []
    while len(pick) < n and any(groups.values()):
        for k in sorted(groups):
            if groups[k] and len(pick) < n:
                pick.append(groups[k].pop(0))
    print(" ".join(map(str, sorted(pick))))
else:
    ref, new = load(sys.argv[2]), load(sys.argv[3])
    bad = 0
    for s in sorted(new):
        a, b = ref[s], new[s]
        diff = [k for k in KEYS if str(a.get(k)) != str(b.get(k))]
        bad += bool(diff)
        print(s, b["fault"] or "-", b["outcome"], "IDENTICAL" if not diff else
              "DIFF " + "; ".join(f"{k}: {a.get(k)} -> {b.get(k)}" for k in diff))
    print(f"{len(new) - bad}/{len(new)} identical")
