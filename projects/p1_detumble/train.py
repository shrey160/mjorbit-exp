"""Train Project 1 policies (1A detumble, 1B pointing) with rsl-rl PPO.

Run from the simulator repo root, one curriculum stage at a time:
    pixi run -e rl --locked python ../projects/p1_detumble/train.py --stage s1 --run-name s1
    pixi run -e rl --locked python ../projects/p1_detumble/train.py --stage s2 --run-name s2 \
        --resume ../projects/p1_detumble/logs/s1/model_<last>.pt
    pixi run -e rl --locked python ../projects/p1_detumble/train.py --stage s3 --run-name s3 \
        --resume ../projects/p1_detumble/logs/s2/model_<last>.pt

On --resume, rsl-rl continues the iteration count, so checkpoint numbers keep
rising across stages (e.g. s1 ends at model_149.pt, a 150-iteration s2 at
model_298.pt). Check the folder for the last file.

Detumble stages (SPEC section 6):
    s1  one random axis, |w| in [0.05, 0.1] rad/s
    s2  each axis uniform in +-0.1 rad/s
    s3  each axis uniform in +-0.3 rad/s (target distribution)

Pointing stages (Project 1B, SPEC_1B.md), each with residual spin +-0.02 rad/s per axis:
    p1  target error up to 30 deg
    p2  target error up to 90 deg
    p3  target error up to 180 deg (target distribution)
The stage name picks the task: s* -> detumble env, p* -> pointing env.

Logs/checkpoints: projects/p1_detumble/logs/<run>; view with
    pixi run -e rl tensorboard --logdir ../projects/p1_detumble/logs
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import mjorbit  # noqa: E402,F401

# isort: split

import torch  # noqa: E402
import numpy as np  # noqa: E402
from detumble_env import DetumbleEnv, DetumbleEnvCfg  # noqa: E402
from pointing_env import PointingEnv, PointingEnvCfg  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

STAGES = {
    "s1": dict(spin_mode="one_axis", spin_min=0.05, spin_max=0.1),
    "s2": dict(spin_mode="box", spin_min=0.0, spin_max=0.1),
    "s3": dict(spin_mode="box", spin_min=0.0, spin_max=0.3),
    "p1": dict(err_min=0.0, err_max=np.radians(30.0)),
    "p2": dict(err_min=0.0, err_max=np.radians(90.0)),
    "p3": dict(err_min=0.0, err_max=np.pi),
}


def make_env(stage: str, reward: str = "v1", obs: str = "v1", **kw):
    """Detumble env for s* stages, pointing env for p* stages (reward/obs versions)."""
    if stage.startswith("p"):
        return PointingEnv(PointingEnvCfg(reward_version=reward, obs_version=obs,
                                          **kw, **STAGES[stage]))
    return DetumbleEnv(DetumbleEnvCfg(**kw, **STAGES[stage]))


def build_train_cfg(seed: int, gamma: float = 0.99) -> dict:
    """rsl-rl 5.x config; same structure as simulator/examples/ppo/train.py,
    with a smaller network. gamma 0.99 (~10 s horizon at 10 Hz) suits detumbling;
    long pointing slews may need more (SPEC_1B section 12)."""
    return {
        "seed": seed,
        "num_steps_per_env": 32,
        "save_interval": 50,
        "obs_groups": {"actor": ["policy"], "critic": ["policy"]},
        "logger": "tensorboard",
        "algorithm": {
            "class_name": "PPO",
            "num_learning_epochs": 5,
            "num_mini_batches": 4,
            "clip_param": 0.2,
            "gamma": gamma,
            "lam": 0.95,
            "value_loss_coef": 1.0,
            "entropy_coef": 0.005,
            "learning_rate": 1.0e-3,
            "max_grad_norm": 1.0,
            "schedule": "adaptive",
            "desired_kl": 0.01,
            "rnd_cfg": None,
            "symmetry_cfg": None,
        },
        "actor": {
            "class_name": "MLPModel",
            "hidden_dims": [64, 64],
            "activation": "elu",
            "obs_normalization": False,  # observations are already scaled to ~[-1, 1]
            "distribution_cfg": {"class_name": "GaussianDistribution", "init_std": 0.5},
        },
        "critic": {
            "class_name": "MLPModel",
            "hidden_dims": [64, 64],
            "activation": "elu",
            "obs_normalization": False,
        },
    }


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--stage", choices=sorted(STAGES), default="s1")
    ap.add_argument("--num-envs", type=int, default=1024)
    ap.add_argument("--max-iterations", type=int, default=150)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--device", default="cuda")
    ap.add_argument("--run-name", default=None)
    ap.add_argument("--resume", default=None, help="checkpoint .pt to continue from")
    ap.add_argument("--tau-max", type=float, default=0.01,
                    help="per-axis torque limit, N*m (0.002 = hard variant)")
    ap.add_argument("--reward", choices=["v1", "v2", "v3"], default="v1",
                    help="pointing reward version (p* stages only; SPEC_1B sections 2, 9, 15)")
    ap.add_argument("--obs", choices=["v1", "v2"], default="v1",
                    help="pointing observation version (v2 adds zoomed error; SPEC_1B 11)")
    ap.add_argument("--gamma", type=float, default=0.99, help="PPO discount factor")
    args = ap.parse_args()

    torch.manual_seed(args.seed)
    torch.cuda.manual_seed_all(args.seed)

    env = make_env(args.stage, reward=args.reward, obs=args.obs, num_envs=args.num_envs,
                   device=args.device, seed=args.seed, tau_max=args.tau_max)
    run = args.run_name or f"{args.stage}-{time.strftime('%Y%m%d-%H%M%S')}"
    log_dir = Path(__file__).parent / "logs" / run
    runner = OnPolicyRunner(env, build_train_cfg(args.seed, args.gamma), log_dir=str(log_dir),
                            device=args.device)
    if args.resume:
        runner.load(args.resume)
    t0 = time.perf_counter()
    runner.learn(num_learning_iterations=args.max_iterations, init_at_random_ep_len=True)
    print(f"stage {args.stage} done in {time.perf_counter() - t0:.0f} s -> {log_dir}")


if __name__ == "__main__":
    main()
