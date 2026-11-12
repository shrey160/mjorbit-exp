"""Print a few training curves from a run's tensorboard log.

    pixi run -e rl --locked python ../projects/p2_approach/tb_summary.py ../projects/p2_approach/logs/s1
"""

import sys

from tensorboard.backend.event_processing.event_accumulator import EventAccumulator

TAGS = ("Train/mean_reward", "x/success_ema", "x/collide_ema", "x/dv_success_ema", "x/pos_err_mean", "x/ok_frac",
        "x/thrust_sat_frac", "Policy/mean_std",
        "Perf/total_fps", "Perf/collection_time")

ea = EventAccumulator(sys.argv[1])
ea.Reload()
tags = ea.Tags()["scalars"]
want = [t for t in tags if any(t.endswith(x.split("/")[-1]) for x in TAGS)]
for t in want:
    ev = ea.Scalars(t)
    n = len(ev)
    picks = sorted({0, n // 4, n // 2, 3 * n // 4, n - 1})
    print(f"{t:40s} " + "  ".join(f"it{ev[i].step}={ev[i].value:.4g}" for i in picks))
