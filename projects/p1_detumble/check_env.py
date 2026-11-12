"""Check the GPU detumble env against the CPU reference model before training.

Run from the simulator repo root:
    pixi run -e rl --locked python ../projects/p1_detumble/check_env.py

1. Same initial spin/attitude/orbit on GPU (env) and CPU (mjorbit), first with an
   open-loop torque sequence (shows how the small O(dt) backend difference grows
   while tumbling), then with the damping law closed-loop (the pass criterion).
2. Zero torque: |w| should stay roughly constant (gyroscopic exchange only).
3. Observation/action scaling and reward sign sanity.
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import mjorbit  # noqa: E402,F401

# isort: split

import numpy as np  # noqa: E402
import torch  # noqa: E402
from detumble_env import XML_PATH, DetumbleEnv, DetumbleEnvCfg  # noqa: E402

import mjorbit as cpu  # noqa: E402
from mjorbit_warp import mjo_pull  # noqa: E402


def main() -> None:
    env = DetumbleEnv(DetumbleEnvCfg(num_envs=4, auto_reset=False, seed=3))
    w0 = np.array([[0.20, -0.12, 0.25], [0.05, 0.0, 0.0], [0.0, 0.0, 0.0], [-0.3, 0.3, -0.3]])
    q = np.random.default_rng(1).normal(size=(4, 4))
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    obs = env.set_initial_conditions(w0, q)
    print("inertia from model:", np.round(env.inertia, 4))
    print("obs[0] at reset:", np.round(obs["policy"][0].cpu().numpy(), 4),
          "(expect w0/0.3 then zeros)")

    mjo_pull(env.model, env.data, fields=("orbit",))
    r0 = np.asarray(env.data.orbit.R_eci[0], dtype=float).copy()
    v0 = np.asarray(env.data.orbit.V_eci[0], dtype=float).copy()

    # Open-loop torque sequence: world 0 gets varied torques, world 2 gets zero.
    steps = 300
    t = np.arange(steps) * 0.1
    seq = np.stack([np.sin(0.3 * t), np.cos(0.2 * t), np.sign(np.sin(0.05 * t + 0.3))], axis=1)
    gpu_w = np.zeros((steps, 3))
    zero_speed = []
    rewards = []
    for k in range(steps):
        a = np.zeros((4, 3))
        a[0] = seq[k]
        a[3] = -np.sign(env.body_rates()[3])  # bang-bang brake on world 3
        _, rew, _, _ = env.step(torch.as_tensor(a, dtype=torch.float32))
        gpu_w[k] = env.body_rates()[0]
        zero_speed.append(np.linalg.norm(env.body_rates()[2]))
        rewards.append(rew.cpu().numpy())

    # CPU reference for world 0.
    model = cpu.MjoModel.from_xml_path(str(XML_PATH), mj_timestep=0.01)
    data = model.make_data(orbit=cpu.OrbitInit(R_eci=r0, V_eci=v0))
    data.qpos[3:7] = q[0]
    data.qvel[:] = 0.0
    data.qvel[3:6] = w0[0]
    cpu.mjo_forward(model, data)
    cpu_w = np.zeros((steps, 3))
    for k in range(steps):
        data.ctrl[:] = 0.01 * seq[k]
        for _ in range(10):
            cpu.mjo_step(model, data)
        cpu_w[k] = np.asarray(data.qvel[3:6])

    err = np.abs(gpu_w - cpu_w)
    print(f"\n[1] GPU vs CPU body rate, 30 s open-loop torques: max |dw| = {err.max():.2e} rad/s,"
          f" final GPU {np.round(gpu_w[-1], 5)} CPU {np.round(cpu_w[-1], 5)}")
    print(f"    relative to typical |w| {np.abs(cpu_w).mean():.3f}: {err.max() / np.abs(cpu_w).mean():.1e}")
    for ts in (1, 5, 10, 20, 30):
        k = int(ts / 0.1) - 1
        print(f"    t={ts:>2d} s: max |dw| so far {err[: k + 1].max():.2e}, |w| {np.linalg.norm(cpu_w[k]):.3f}")

    # Closed loop: the same damping law run on both backends (the case that matters).
    env2 = DetumbleEnv(DetumbleEnvCfg(num_envs=2, auto_reset=False, seed=4))
    env2.set_initial_conditions(np.tile(w0[3], (2, 1)), np.tile(q[3], (2, 1)))
    mjo_pull(env2.model, env2.data, fields=("orbit",))
    r2 = np.asarray(env2.data.orbit.R_eci[0], dtype=float).copy()
    v2 = np.asarray(env2.data.orbit.V_eci[0], dtype=float).copy()
    inertia = env2.inertia
    gw = np.zeros((steps, 3))
    for k in range(steps):
        a = np.clip(-2.0 * inertia * env2.body_rates() / 0.01, -1, 1)
        env2.step(torch.as_tensor(a, dtype=torch.float32))
        gw[k] = env2.body_rates()[0]
    d2 = model.make_data(orbit=cpu.OrbitInit(R_eci=r2, V_eci=v2))
    d2.qpos[3:7] = q[3]
    d2.qvel[:] = 0.0
    d2.qvel[3:6] = w0[3]
    cpu.mjo_forward(model, d2)
    cw = np.zeros((steps, 3))
    for k in range(steps):
        d2.ctrl[:] = np.clip(-2.0 * inertia * np.asarray(d2.qvel[3:6]), -0.01, 0.01)
        for _ in range(10):
            cpu.mjo_step(model, d2)
        cw[k] = np.asarray(d2.qvel[3:6])
    e2 = np.abs(gw - cw).max()
    print(f"\n[1b] closed-loop damping from (-0.3,0.3,-0.3), GPU vs CPU: max |dw| = {e2:.2e} rad/s;"
          f" final |w| GPU {np.linalg.norm(gw[-1]):.2e} CPU {np.linalg.norm(cw[-1]):.2e}")

    print(f"\n[2] zero-torque world starting at rest: max |w| over 30 s = {max(zero_speed):.2e} rad/s")
    r = np.asarray(rewards)
    print(f"\n[3] world 3 bang-bang brake from (-0.3,0.3,-0.3): final |w| ="
          f" {np.linalg.norm(env.body_rates()[3]):.4f};"
          f" reward first {r[0, 3]:+.3f} -> last {r[-1, 3]:+.3f} (should rise)")
    # The backends differ by an O(dt) integration term once rotation is multi-axis
    # (~1e-5 rad/s after 1 s at dt=0.01, ~5x smaller at dt=0.002, zero for single-axis
    # spin). Open-loop tumbling amplifies it, so judge on the closed-loop case:
    # agreement well under the 0.01 rad/s success threshold and both ending at rest.
    ok = e2 < 1e-3 and np.linalg.norm(gw[-1]) < 0.01 and np.linalg.norm(cw[-1]) < 0.01
    print("\nRESULT:", "PASS" if ok else "FAIL",
          "(closed-loop GPU/CPU gap < 1e-3 rad/s and both at rest at the end)")


if __name__ == "__main__":
    main()
