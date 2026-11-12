"""Check 3C sensing against truth: known thrust -> bus accel vs F/m (+ rotation terms);
known wheel torque -> angular accel vs I^-1 tau (debugging aid)."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import numpy as np  # noqa: E402
from grasp_sim import ARM_Q  # noqa: E402
from stack_sim import StackCfg, StackRun, stack_truth  # noqa: E402

r = StackRun(0, StackCfg())
g = r.sim.run(r.rng, pos_err=0.05, vel_err=0.002, att_err_deg=2.0)
r.q_lock = r.sim.data.qpos[ARM_Q].copy()
r.t, r.h_rw, r.h_rw_max, r.pads_lost, r.arm_tau_max = 0.0, np.zeros(3), 0.0, False, 0.0
r.lock_arm()
sim = r.sim
sim.sync()
r._gyro_last = sim.data.qvel[3:6].copy()
for _ in range(500):
    r._gyro_last = r._step(tau_body=-300 * (r._gyro_last - r._w_lvlh_body()))[0]
M, I, d, cm = stack_truth(sim)
print("truth M", M, "d", np.round(d, 3))
print("I truth\n", np.round(I, 2))
# torque pulse about each bus axis, no thrust: measure dw over 2 s
for ax in range(3):
    tau = 0.5 * np.eye(3)[ax]
    w0 = sim.data.qvel[3:6].copy()
    for _ in range(200):
        r._step(tau_body=tau)
    w1 = sim.data.qvel[3:6].copy()
    alpha = (w1 - w0) / 2.0
    print(f"torque {ax}: alpha meas {np.round(alpha, 7)}  pred {np.round(np.linalg.solve(I, tau), 7)}")
    for _ in range(200):
        r._step(tau_body=-tau)
# thrust along each bus axis, no torque, 2 s: bus accel vs F/m + alpha x d
for ax in range(3):
    F = 0.4 * np.eye(3)[ax]
    w0 = sim.data.qvel[3:6].copy()
    S = np.zeros(3)
    for _ in range(200):
        _, dv, _, _ = r._step(F_body=F)
        S += dv
    w1 = sim.data.qvel[3:6].copy()
    print(f"thrust {ax}: dv meas {np.round(S, 6)}  pred F/m {np.round(F * 2 / M, 6)} "
          f"+ dw x d {np.round(np.cross(w1 - w0, d), 6)}   dw meas {np.round(w1 - w0, 6)} "
          f"pred I^-1 (d x F) t {np.round(np.linalg.solve(I, np.cross(d, F)) * 2, 6)}")
