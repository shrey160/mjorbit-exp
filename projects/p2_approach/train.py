"""Train Project 2A approach policies with rsl-rl PPO (SPEC_2).

Run from the simulator repo root, one curriculum stage at a time:
    pixi run -e rl --locked python ../projects/p2_approach/train.py --stage a1 --run-name a1
    pixi run -e rl --locked python ../projects/p2_approach/train.py --stage a2 --run-name a2 \
        --resume ../projects/p2_approach/logs/a1/model_<last>.pt
rsl-rl continues iteration numbering on --resume; check the folder for the last file.

Stages (start distance from the target, 60 deg cone around -V-bar):
    a1  5-15 m    a2  10-30 m    a3  20-50 m (target distribution)
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
from approach_env import UNCERTAINTY, ApproachEnv, ApproachEnvCfg  # noqa: E402
from maneuver_env import ManeuverEnv, ManeuverEnvCfg  # noqa: E402
from residual_env import ResidualEnv, ResidualEnvCfg  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

STAGES = {
    "a1": dict(r_min=5.0, r_max=15.0),
    "a2": dict(r_min=10.0, r_max=30.0),
    "a3": dict(r_min=20.0, r_max=50.0),
}


def make_env(stage: str, env: str = "thrust", **kw) -> ApproachEnv:
    if env == "maneuver":
        kw.pop("deadzone", None)
        return ManeuverEnv(ManeuverEnvCfg(**kw, **STAGES[stage]))
    if env == "residual":
        kw.pop("deadzone", None)
        return ResidualEnv(ResidualEnvCfg(**kw, **STAGES[stage]))
    return ApproachEnv(ApproachEnvCfg(**kw, **STAGES[stage]))


def build_train_cfg(seed: int, gamma: float, entropy: float = 0.005, lam: float = 0.95,
                    std_min: float = 1e-6, steps_per_env: int = 32,
                    init_std: float = 0.5) -> dict:
    return {
        "seed": seed,
        "num_steps_per_env": steps_per_env,
        "save_interval": 50,
        "obs_groups": {"actor": ["policy"], "critic": ["policy"]},
        "logger": "tensorboard",
        "algorithm": {
            "class_name": "PPO",
            "num_learning_epochs": 5,
            "num_mini_batches": 4,
            "clip_param": 0.2,
            "gamma": gamma,
            "lam": lam,
            "value_loss_coef": 1.0,
            "entropy_coef": entropy,
            "learning_rate": 1.0e-3,
            "max_grad_norm": 1.0,
            "schedule": "adaptive",
            "desired_kl": 0.01,
            "rnd_cfg": None,
            "symmetry_cfg": None,
        },
        "actor": {
            "class_name": "MLPModel",
            "hidden_dims": [128, 128],
            "activation": "elu",
            "obs_normalization": False,
            "distribution_cfg": {"class_name": "GaussianDistribution", "init_std": init_std,
                                 "std_range": (std_min, 1.0)},
        },
        "critic": {
            "class_name": "MLPModel",
            "hidden_dims": [128, 128],
            "activation": "elu",
            "obs_normalization": False,
        },
    }


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--stage", choices=sorted(STAGES), default="a1")
    ap.add_argument("--num-envs", type=int, default=4096)
    ap.add_argument("--max-iterations", type=int, default=300)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--device", default="cuda")
    ap.add_argument("--run-name", default=None)
    ap.add_argument("--resume", default=None)
    ap.add_argument("--gamma", type=float, default=0.998,
                    help="PPO discount; also used for the potential-based shaping term")
    ap.add_argument("--w-fuel", type=float, default=None, help="override fuel weight")
    ap.add_argument("--entropy", type=float, default=0.005, help="PPO entropy coefficient")
    ap.add_argument("--env", choices=["thrust", "maneuver", "residual"], default="thrust",
                    help="thrust = 1 Hz forces; maneuver = 20 s dv decisions (SPEC_2 11); "
                         "residual = CW epoch + nav9 plus a learned correction (SPEC_2 16)")
    ap.add_argument("--init-std", type=float, default=0.5, help="initial action std")
    ap.add_argument("--base", choices=["cw", "gs"], default=None,
                    help="residual env base guidance (gs = glideslope hybrid, SPEC_2 19)")
    ap.add_argument("--res-scale", type=float, default=None, help="residual m/s at |a|=1")
    ap.add_argument("--zero-init", action="store_true",
                    help="scale the actor's last layer by 0.01 so the start policy is a ~ 0")
    ap.add_argument("--steps-per-env", type=int, default=32)
    ap.add_argument("--unc", choices=sorted(UNCERTAINTY), default="none",
                    help="train under this uncertainty preset (domain randomization)")
    ap.add_argument("--std-min", type=float, default=1e-6,
                    help="floor on the policy's action std (keeps exploration alive)")
    ap.add_argument("--lam", type=float, default=0.95, help="GAE lambda")
    ap.add_argument("--w-shape", type=float, default=None, help="override distance-shaping weight")
    ap.add_argument("--deadzone", type=float, default=0.0, help="action deadzone (0.2 = v3)")
    args = ap.parse_args()

    torch.manual_seed(args.seed)
    torch.cuda.manual_seed_all(args.seed)
    kw = dict(num_envs=args.num_envs, device=args.device, seed=args.seed,
              gamma_shaping=args.gamma)
    kw["deadzone"] = args.deadzone
    kw.update(UNCERTAINTY[args.unc])
    if args.w_shape is not None:
        kw["w_shape"] = args.w_shape
    if args.w_fuel is not None:
        kw["w_fuel"] = args.w_fuel
    if args.res_scale is not None:
        kw["res_scale"] = args.res_scale
    if args.base is not None:
        kw["base"] = args.base
    env = make_env(args.stage, env=args.env, **kw)
    run = args.run_name or f"{args.stage}-{time.strftime('%Y%m%d-%H%M%S')}"
    log_dir = Path(__file__).parent / "logs" / run
    runner = OnPolicyRunner(env, build_train_cfg(args.seed, args.gamma, args.entropy, args.lam,
                                                     args.std_min, args.steps_per_env,
                                                     args.init_std), log_dir=str(log_dir),
                            device=args.device)
    if args.resume:
        runner.load(args.resume)
    elif args.zero_init:
        lin = [m for m in runner.alg.actor.mlp.modules() if isinstance(m, torch.nn.Linear)][-1]
        with torch.no_grad():
            lin.weight.mul_(0.01)
            lin.bias.zero_()
    t0 = time.perf_counter()
    runner.learn(num_learning_iterations=args.max_iterations, init_at_random_ep_len=True)
    print(f"stage {args.stage} done in {time.perf_counter() - t0:.0f} s -> {log_dir}")


if __name__ == "__main__":
    main()
