"""P4.8 diagnosis: where does the client push come from at 3 deg/s? One oracle episode per
(seed, spin); prints, around the close, the grip point relative to the bar in the client frame
(x = approach axis), the client's speed change since approach start, the pad forces, the bus
station-keeping error and the thrust demand.   bash run_p4.sh dbg_push.py <spin> <seeds...>"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "p3_capture"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from gen_demos import SCENE  # noqa: E402
from grasp_sim import GraspCfg, GraspSim  # noqa: E402

spin, seeds = float(sys.argv[1]), [int(s) for s in sys.argv[2:]]
sim = GraspSim(GraspCfg(scene=SCENE, hold_s=5.0, mon=True, mon_act=True))
rows = []


def hook(L):
    ph = L["phase"]
    if ph not in ("reach", "approach", "close", "hold"):
        return
    kd, d = sim.kd, L["d"]
    Rc = kd.xmat[sim.bid_target].reshape(3, 3)
    pg, _ = sim.site_pose(sim.sid_grip)
    pb, _ = sim.site_pose(sim.sid_bar)
    vg, _ = sim.site_vel(sim.sid_grip)
    vb, _ = sim.site_vel(sim.sid_bar)
    pads, ftot, _ = sim.contacts()
    vc0 = L.get("vc0")
    dv = np.linalg.norm(d.qvel[14:17] - vc0) if vc0 is not None else 0.0
    ref = getattr(sim, "_bus_ref", None)
    be = np.linalg.norm(kd.xpos[sim.bid_chaser] - ref[0]) if ref is not None else np.nan
    F = getattr(sim, "_F_raw", np.zeros(3))
    rows.append((L["t"], ph, *(Rc.T @ (pg - pb) * 1000), *(Rc.T @ (vg - vb) * 1000), dv * 1000, ftot,
                 len(pads), be * 1000, float(np.max(np.abs(F)))))


sim.step_hook = hook
for seed in seeds:
    rows.clear()
    draw = np.random.default_rng(seed + 12345)
    draw.uniform(0.0, 2.0)
    axis = int(draw.integers(3))
    r = sim.run(np.random.default_rng(seed), spin_deg_s=spin, spin_axis=axis)
    A = rows
    print(f"\n== seed {seed} spin {spin} axis {axis}: {r['outcome']} push {r['push_dv']*1000:.2f} mm/s shift {r['bar_shift_mm']:.2f} "
          f"peakF {r['peak_force']:.1f} f_max {sim.f_max}")
    for ph in ("reach", "approach", "close", "hold"):
        B = [a for a in A if a[1] == ph]
        if B:
            print(f"  {ph:8s} bus err max {max(b[11] for b in B):6.1f} mm, thrust demand max {max(b[12] for b in B):.3f} N, "
                  f"client dv end {B[-1][8]:.2f} max {max(b[8] for b in B):.2f} mm/s")
    tc = next(a[0] for a in A if a[1] == "close")
    print("    t-tc  phase    rel x/y/z mm        relvel x/y/z mm/s    dv mm/s  F N  pads  bus mm  thrust N")
    for a in A:
        dt = a[0] - tc
        if -6.01 <= dt <= 3.5 and abs((dt * 100) % 50) < 0.5 or (-0.01 <= dt <= 1.0 and abs((dt * 100) % 10) < 0.5):
            print(f"   {dt:5.2f} {a[1]:8s} {a[2]:6.1f} {a[3]:6.1f} {a[4]:6.1f}   {a[5]:6.2f} {a[6]:6.2f} {a[7]:6.2f}   {a[8]:5.2f} {a[9]:5.1f}  {a[10]}  {a[11]:5.1f}  {a[12]:.3f}")
