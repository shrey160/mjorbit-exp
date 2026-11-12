# Research notes and reading sequence

Reviewed 6 November 2026. Sources below support simulator facts and background. The schedule, simplified tasks, hardware experiment sizes, and success targets are original proposals for this learning programme.

## Recommended reading

| When | Primary source | What to extract |
|---|---|---|
| Start | [mjorbit paper](https://arxiv.org/pdf/2609.08010) | Introduction, architecture, and examples first; return to numerical conditioning after learning frames |
| Week 1 | [mjorbit frames and units](https://johnzhang3.github.io/mjorbit/docs/frames.html) | Label every position, velocity, force, and torque by frame and units |
| Week 2 | [Spinning Up: key RL concepts](https://spinningup.openai.com/en/latest/spinningup/rl_intro.html) | Observation, policy, return, and value function; explain them in your own words |
| Week 3 | [PPO explanation](https://spinningup.openai.com/en/latest/algorithms/ppo.html) and [original PPO paper](https://arxiv.org/abs/1707.06347) | Understand sampling, advantage, actor/critic updates, and clipping; defer full derivations |
| Week 4 | [mjorbit actuators and sensors](https://johnzhang3.github.io/mjorbit/docs/actuators-sensors.html) | Physical commands, limits, wheel momentum, thruster direction |
| Week 5 | [MIT 16.346, Lecture 26: CW relative motion](https://ocw.mit.edu/courses/16-346-astrodynamics-fall-2008/e4f0632a9f1c98f7e9b25492e1a30eb1_lec_26.pdf) | Why relative orbital motion differs from free-space straight-line motion; scope of linearization |
| Week 5 | [NASA rendezvous and proximity operations](https://www.nasa.gov/reference/jsc-rendezvous-prox-ops-docking-subsystems/) | Distinguish approach, relative navigation, and connection as engineering problems |
| Weeks 8-10 | [mjorbit PPO example guide](https://github.com/johnzhang3/mjorbit/blob/main/examples/ppo/README.md) | Inspect what is learned, what is scripted, permitted contacts, and close-to-far resets |
| During implementation | [Gymnasium: time limits](https://gymnasium.farama.org/tutorials/gymnasium_basics/handling_time_limits/) | Understand genuine endings versus artificial cutoffs, then map that distinction to rsl-rl |

Optional context: [NASA robotic servicing arm](https://www.nasa.gov/isam/robotic-servicing-arm/) illustrates why practical capture tasks can require substantial arm dexterity. Our first prepared-handle exercise deliberately restricts geometry.

## Verified project facts that affect this plan

- The paper combines a CPU implementation and a GPU implementation; its demonstrations include planning and policy learning. Its reported results are not laptop training benchmarks. [Paper](https://arxiv.org/pdf/2609.08010)
- Documented GPU/RL environments target Linux x86-64 with NVIDIA hardware. Windows is not an established release target. Keep locked dependencies for reproduction. [Installation](https://johnzhang3.github.io/mjorbit/docs/installation.html)
- The GPU guide describes explicit device/host synchronization and differences from CPU functionality. Design noise injection and model features against the actual backend rather than assuming full parity. [GPU guide](https://johnzhang3.github.io/mjorbit/docs/gpu.html)
- Existing PPO examples use numerical state, contact-supported capture, and staged training. The Astrobee example controls the gripper; the truss example contains automatic wrist/grip behaviour and starts with a hold already established. [PPO examples](https://github.com/johnzhang3/mjorbit/blob/main/examples/ppo/README.md)
- The reference PPO configuration uses a small multilayer network. This supports trying local training, but does not establish the memory required by a modified contact environment. [Training configuration](https://github.com/johnzhang3/mjorbit/blob/main/examples/ppo/train.py)
- CUDA is available through WSL2 in supported configurations. Exact mjorbit setup remains to be verified on this laptop. [NVIDIA guide](https://docs.nvidia.com/cuda/wsl-user-guide/)

## Open questions before implementation

1. How much system RAM/storage is available, and is WSL2 already configured with GPU visibility?
2. Which repository revision and dependency lock reproduce a minimal CPU and GPU run?
3. Which simple chaser geometry, torque/force bounds, and inertia make the desired reset ranges feasible?
4. What batch size provides useful complete training throughput within available VRAM?
5. Which collision pairs and arm degrees of freedom are needed for the first reachable capture pose?
6. Which observation/frame choice makes each initial task sufficiently observable?

Resolve these through short setup/physics experiments. None requires committing to a large training run first.
