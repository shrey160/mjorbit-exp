"""Probe a pointing policy's equilibrium: its action at zero error/spin, and where episodes end.

Run from simulator/:  pixi run -e rl --locked python ../projects/p1_detumble/diag_offset.py <checkpoint.pt> [obs v1|v2]
"""
import sys

sys.path.insert(0, "/mnt/c/Shrey_Projs/robotics/mjorbit/projects/p1_detumble")
import mjorbit  # noqa: F401
import numpy as np
import torch
from evaluate_pointing import make_cases
from rsl_rl.runners import OnPolicyRunner
from tensordict import TensorDict
from train import build_train_cfg, make_env

ckpt = sys.argv[1]
obs_v = sys.argv[2] if len(sys.argv) > 2 else "v1"
env = make_env("p3", obs=obs_v, num_envs=64, auto_reset=False, seed=100)
runner = OnPolicyRunner(env, build_train_cfg(0), log_dir=None, device="cuda")
runner.load(ckpt)
pol = runner.get_inference_policy(device="cuda")

# 1. Action at "exactly on target, at rest, previous action 0".
z = torch.zeros((64, 12 if obs_v == "v2" else 9), device="cuda")
a0 = pol(TensorDict({"policy": z}, batch_size=[64]))[0].detach().cpu().numpy()
print("action at zero obs:", np.round(a0, 4), " torque N*m:", np.round(0.01 * a0, 6))

# 2. Final error vectors (body frame) after a rollout: same direction for all cases?
w0, quat, q_tgt = make_cases("p3", "validation", 64, env.cfg)
obs = env.set_pointing_conditions(w0, quat, q_tgt)
with torch.inference_mode():
    for _ in range(env.max_episode_length):
        a = pol(obs).clamp(-1, 1)
        obs, *_ = env.step(a)
qe, ang = env.pointing_error()
e_deg = np.degrees(2 * np.arcsin(np.clip(np.linalg.norm(qe[:, 1:], axis=1), 0, 1)))
dirs = qe[:, 1:] / np.linalg.norm(qe[:, 1:], axis=1, keepdims=True)
print("final error deg: min %.3f max %.3f" % (e_deg.min(), e_deg.max()))
print("final error direction (body), mean:", np.round(dirs.mean(axis=0), 3),
      " spread (std):", np.round(dirs.std(axis=0), 3))
print("final actions mean:", np.round(a.cpu().numpy().mean(axis=0), 4),
      " final obs[0]:", np.round(obs["policy"][0].cpu().numpy(), 4))
