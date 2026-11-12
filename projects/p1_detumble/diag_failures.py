"""Trace the failing cases of a pointing policy: angle and spin over time.

Run from simulator/:
    pixi run -e rl --locked python ../projects/p1_detumble/diag_failures.py <checkpoint.pt> [obs v1|v2]
"""
import sys

sys.path.insert(0, "/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p1_detumble")
import mjorbit  # noqa: F401
import numpy as np
import torch
from evaluate_pointing import make_cases
from pointing_env import error_angle, error_quat
from rsl_rl.runners import OnPolicyRunner
from train import build_train_cfg, make_env

ckpt = sys.argv[1]
obs_v = sys.argv[2] if len(sys.argv) > 2 else "v1"
n = 200
env = make_env("p3", obs=obs_v, num_envs=n, auto_reset=False, seed=100)
runner = OnPolicyRunner(env, build_train_cfg(0), log_dir=None, device="cuda")
runner.load(ckpt)
pol = runner.get_inference_policy(device="cuda")
w0, quat, q_tgt = make_cases("p3", "validation", n, env.cfg)
init = np.degrees(error_angle(error_quat(quat, q_tgt)))
obs = env.set_pointing_conditions(w0, quat, q_tgt)
T = env.max_episode_length
ang = np.zeros((n, T))
spd = np.zeros((n, T))
act = np.zeros((n, T, 3))
with torch.inference_mode():
    for k in range(T):
        a = pol(obs).clamp(-1, 1)
        obs, *_ = env.step(a)
        ang[:, k] = np.degrees(env.pointing_error()[1])
        spd[:, k] = np.linalg.norm(env.body_rates(), axis=1)
        act[:, k] = a.cpu().numpy()
hold = 50
ok = ((ang < 5) & (spd < 0.01))[:, -hold:].all(axis=1)
fail = np.flatnonzero(~ok)
print(f"failures {fail.size}/{n}; initial angle of failures: {np.round(np.sort(init[fail]), 0)}")
print(f"initial angle of successes: median {np.median(init[ok]):.0f}, max {init[ok].max():.0f}")
big_ok = ok & (init > 150)
print(f"targets >150 deg: {int((init > 150).sum())}, succeeded {int(big_ok.sum())}")
ts = [0, 50, 100, 150, 200, 250, 300, 350, 399]
for i in fail[:8]:
    print(f"case {i} init {init[i]:.0f} deg | angle@t(s) " +
          " ".join(f"{k // 10}:{ang[i, k]:.0f}" for k in ts) +
          f" | max speed {spd[i].max():.3f} | sat frac {(np.abs(act[i]) > 0.999).any(axis=1).mean():.2f}"
          f" | mean |a| {np.abs(act[i]).mean():.2f}")
# successes with large initial angle, for comparison
for i in np.flatnonzero(big_ok)[:3]:
    print(f"OK   {i} init {init[i]:.0f} deg | angle@t(s) " +
          " ".join(f"{k // 10}:{ang[i, k]:.0f}" for k in ts) +
          f" | max speed {spd[i].max():.3f} | sat frac {(np.abs(act[i]) > 0.999).any(axis=1).mean():.2f}")
