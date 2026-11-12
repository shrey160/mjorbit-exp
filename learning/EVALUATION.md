# Evaluation and laptop experiment plan

These gates are educational targets proposed for this programme, not measured results or mission qualification criteria. Check actuator authority, geometry, and observability before freezing them. Adjust infeasible tasks before training, not after viewing test failures.

## Proposed project gates

| Project | Initial test | Success condition |
|---|---|---|
| 1A: detumble | Feasible randomized three-axis spins; 30 s episode | Angular-speed magnitude below 0.01 rad/s continuously for the final 5 s; at least 95% success |
| 1B: point and hold | Requested attitudes from a declared distribution | Orientation error below 5 degrees and speed below 0.01 rad/s for the final 5 s; at least 90% success |
| 2: approach and hold | Declared 5-20 m proximity cases with feasible initial velocity | Reach within 0.2 m of the holding-point vector, relative speed in the goal frame below 0.02 m/s, attitude error below 5 degrees, and hold for 10 s before deadline; at least 90% success |
| 3: capture and retain | Declared reachable, slow, prepared-handle cases | Valid opposed-jaw grasp, continuous retention for 20 s, limited target motion relative to gripper, no forbidden collision or numerical invalidity; at least 80% initial success, 90% stretch target |

For Project 2, include the required rotating-frame velocity when judging a rotating-frame goal. All holds must be consecutive and within episode duration. A task failure or forbidden collision overrides a later apparent success. In Project 3, tune a physically justified relative-motion tolerance and contact-speed ceiling after inspecting geometry and solver behaviour, then freeze both before training. Do not treat an uncalibrated peak contact-force number as a physical certification metric.

## What counts as a grasp?

Require the handle to be between the jaws, the jaws to be commanded closed, valid opposing contact evidence without implausible penetration, and retained relative pose throughout the hold. Account for short contact-buffer fluctuations through a documented tolerance; absence of contact must not allow indefinite success. A gentle, specified disturbance during a separate retention test helps reveal false grasps. Do not infer success from a closed-gripper command or one frame of contact.

Whole-body and arm-target prohibited contacts must be checked separately from intended jaw-handle contact. If collision pairs are disabled for an early exercise, list them and limit the claim accordingly. Enable appropriate collision monitoring before claiming integrated capture with collision avoidance.

## Evaluation protocol

1. Define training, validation, and test distributions and independent reset-seed lists. Training can sample continuously; do not train on the fixed test cases.
2. Use a small validation set during development. Choose checkpoints and curriculum changes with validation, never with the final test set.
3. During early debugging use one training seed and short runs. For the final chosen configuration train three independent seeds when resources allow.
4. Evaluate each final policy on 200 fresh test episodes (600 across three policies). Use matched initial conditions for baseline comparisons. Record whether actions are deterministic or stochastic.
5. Report per-seed success rate and variability, episode counts, and a binomial confidence interval where appropriate. Do not imply that 600 correlated episodes eliminate uncertainty across training runs.
6. Count collisions, escapes, invalid states, timeouts, and loss of grasp. Show the failure categories, not only average reward or selected videos.
7. After freezing the primary result, test separate harder distributions: stronger spin, inertia changes, noise, low actuator authority, and greater target motion. Label these stress tests separately.

A baseline and PPO must share observations where relevant, physical actuator limits, control/physics rate, initial states, and success conditions. Report any privileged model knowledge a baseline uses. Compare effort among successful episodes together with success rate, so quick failures cannot look efficient.

For finite task deadlines, define whether the time limit is part of the task and expose remaining time where needed. For artificial rollout cutoffs, preserve value bootstrapping. Correct termination/truncation treatment depends on the environment and training library; follow its conventions explicitly. [Time-limit explanation](https://gymnasium.farama.org/tutorials/gymnasium_basics/handling_time_limits/)

## Metrics to save

- Task success and settling/capture time, with failures retained in the denominator.
- Angular speed and attitude error; relative position/velocity and minimum clearance.
- Actuator saturation and action changes; integral of force magnitude as a translation-effort proxy. Without an explicit propellant model, do not call this measured fuel consumption.
- For physical wheels: momentum/speed and limit events.
- For the arm: joint-limit events, grip loss, target motion relative to gripper, and contact/penetration diagnostics.
- Episode return broken into reward components, policy/value losses, exploration level, and training health diagnostics.
- Policy control transitions, physics steps, PPO updates, wall-clock duration, peak VRAM, and software/repository revision.

## Laptop resource plan

Confirmed hardware is a Ryzen AI 9 HX 370 and RTX 4060 Laptop GPU with about 8 GB VRAM. No training throughput or task-memory measurement exists yet.

The first setup session should verify system RAM, free storage, WSL2/CUDA visibility, and the exact locked simulator/training versions. mjorbit documents Linux/NVIDIA for RL; WSL2 is a candidate route because NVIDIA supports CUDA there, not because this exact project has already been verified in WSL. [mjorbit installation](https://johnzhang3.github.io/mjorbit/docs/installation.html), [NVIDIA WSL guide](https://docs.nvidia.com/cuda/wsl-user-guide/)

1. Inspect one CPU world and the baseline before batching.
2. Verify a small GPU batch and sensible motion; compare CPU/GPU smooth trajectories with precision-appropriate tolerances rather than requiring identical contact trajectories.
3. Benchmark 32, 64, 128, and 256 worlds as candidate sizes, stopping before memory pressure or instability. These are probes, not promised capacities. Increase further only if measurements justify it.
4. Measure both physics throughput and complete training throughput, including observations, rewards, resets, policy execution, and updates. Report kernel-compilation startup separately.
5. Train without continuous rendering; render sampled evaluation trajectories separately. Keep initial networks modest and use the existing PPO implementation.
6. Run a bounded pilot before a longer job; save resumable checkpoints and inspect physical metrics. Check memory again after adding the arm/contact buffers.

Reducing world count reduces experience per update at fixed rollout length. Keep comparisons explicit about transition budget and PPO settings; do not assume the same iteration count is equivalent. A physics-step benchmark cannot directly predict end-to-end learning time. Estimate longer runs only from a measured complete pilot and retain overhead/learning uncertainty.

## Experiment record template

Copy this section for each meaningful run; no entries are results until an experiment has actually run.

- Question and prediction:
- One intended change and comparison run:
- Project/stage, scenario, geometry, active physics:
- Observation/action contract and controller version:
- Actuator limits, physics timestep, control interval:
- Training seed, reset distributions, validation/test split:
- World count, rollout length, total control transitions, PPO settings:
- Software versions and repository revision:
- Runtime, peak VRAM, compilation time separately:
- Success denominator, rate, uncertainty, and failure categories:
- Physical metrics, plots, representative clips:
- Interpretation, limitations, and next experiment:
