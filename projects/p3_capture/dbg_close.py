"""Trace bus / client velocity and arm joint speed around the jaw close (ESPA push debug).
python dbg_close.py <scene> [seed] [t0] [t1]"""
import sys

import numpy as np
from grasp_sim import GraspCfg, GraspSim

scene = sys.argv[1]
seed = int(sys.argv[2]) if len(sys.argv) > 2 else 100000
t0, t1 = (float(sys.argv[3]), float(sys.argv[4])) if len(sys.argv) > 4 else (37.0, 44.0)
s = GraspSim(GraspCfg(scene=scene))
r = s.run(np.random.default_rng(seed), record=True)
print("push_dv mm/s", r["push_dv"] * 1000, "t_grasp", r["t_grasp"])
for t, ph, pg, pb, q, v in r["trace"]:
    if t0 <= t <= t1 and abs(t * 5 - round(t * 5)) < 1e-6:
        print(f"{t:5.1f} {ph:8s} bus v mm/s {np.round(v[0:3] * 1000, 2)} w deg/s {np.round(np.degrees(v[3:6]), 3)}"
              f" client v {np.round(v[14:17] * 1000, 2)} w {np.round(np.degrees(v[17:20]), 3)}"
              f" |qd_arm| {np.abs(v[6:12]).max():.4f} jaws {np.round(q[13:15], 4)}")


def torque_log(scene, seed, t0, t1):
    """Wheel command (raw and clipped) and arm torques per control step around the close."""
    s = GraspSim(GraspCfg(scene=scene))
    log = []
    att0, imp0 = s.att_torque, s.impedance_tau
    s.att_torque = lambda Rs, Tw: (log.append(["T", s.s_I * (Rs.T @ Tw)]), att0(Rs, Tw))[1]
    s.impedance_tau = lambda *a: (log.append(["tau", imp0(*a)]), log[-1][1])[1]
    s.run(np.random.default_rng(seed))
    Ts = [v for k, v in log if k == "T"]
    dt = s.cfg.ctrl_dt
    for i in range(int(t0 / dt), int(t1 / dt), 10):
        print(f"{i * dt:5.1f} T_raw {np.round(Ts[i], 3)}")


if len(sys.argv) > 5 and sys.argv[5] == "torque":
    torque_log(scene, seed, t0, t1)
