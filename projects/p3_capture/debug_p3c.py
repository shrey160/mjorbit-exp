"""Trace grip state through 3C tumble and detumble (debugging aid)."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import numpy as np  # noqa: E402
from grasp_sim import ARM, ARM_Q  # noqa: E402
from stack_sim import StackCfg, StackRun  # noqa: E402

seed = int(sys.argv[1]) if len(sys.argv) > 1 else 0
mode = sys.argv[2] if len(sys.argv) > 2 else "tumble"
r = StackRun(seed, StackCfg())
g = r.sim.run(r.rng, pos_err=0.05, vel_err=0.002, att_err_deg=2.0)
print("grasped", g["grasped"], "clamp", g["clamp_force"])
r.q_lock = r.sim.data.qpos[ARM_Q].copy()
r.t, r.h_rw, r.h_rw_max, r.pads_lost, r.arm_tau_max = 0.0, np.zeros(3), 0.0, False, 0.0
if r.cfg.brakes:
    r.lock_arm()
sim = r.sim


def status(tag):
    sim.sync()
    pads, ftot, other = sim.contacts()
    kd = sim.kd
    dist = np.linalg.norm(kd.xpos[sim.bid_chaser] - kd.xpos[sim.bid_target])
    qerr = np.linalg.norm(sim.data.qpos[ARM_Q] - r.q_lock)
    print(f"{tag:10s} t={r.t:6.2f} pads={len(pads)} F={ftot:6.2f} dist={dist:.4f} "
          f"qerr={qerr:.5f} qd={np.linalg.norm(sim.data.qvel[ARM]):.5f} "
          f"w={np.degrees(np.linalg.norm(sim.data.qvel[3:6])):.3f}")


status("start")
for _ in range(100):
    r._step()
status("locked 1s")
if mode == "tumble":
    r.impose_tumble()
    status("tumbled")
r._gyro_last = sim.data.qvel[3:6].copy()
for k in range(30):
    for _ in range(100):
        g_ = r._step(tau_body=-r.cfg.detumble_kd * (r._gyro_last - r._w_lvlh_body()))[0]
        r._gyro_last = g_
    status(f"detumble{k}")
    if r.pads_lost:
        break
