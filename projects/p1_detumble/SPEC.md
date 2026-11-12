# Project 1A spec: detumble a rigid spacecraft

Written 7 November 2026. Status: **implemented and trained (one training seed); gate 1A met on the test set.** See §11 for results. Measured facts below come from [measure.py](measure.py) on the CPU backend. Everything labelled "choice" is a design decision you can change before training, not a physical fact.

Project 1B (point and hold) is sketched at the end and gets its own spec once 1A works.

## 1. The question

Can a PPO policy, given only the spacecraft's spin and its own last command, bring a randomly spinning spacecraft to rest within 30 s as reliably as a simple damping controller, and with similar torque effort?

Detumbling only removes spin. The final orientation is whatever it happens to be; that is 1B's job.

## 2. Spacecraft and environment

Model file: [p1_spacecraft.xml](p1_spacecraft.xml).

| Item | Value | Source |
|---|---|---|
| Body | One rigid box, 0.5 × 0.3 × 0.2 m, 10 kg, no arm, no contact | choice |
| Inertia (principal, body axes) | Ixx 0.1083, Iyy 0.2417, Izz 0.2833 kg·m² | hand calculation, matches compiled model exactly |
| Actuator | 3 ideal body-frame torques, ±0.01 N·m per axis, as `motor` actuators on the free joint's rotational DOFs | choice |
| Orbit | 400 km circular, R = 6778.137 km, V = 7.6686 km/s | choice |
| Active physics | Two-body reference orbit, differential gravity, gravity-gradient torque. J2, drag, solar pressure, magnetic field **off** | choice (XML `<mjorbit>` flags) |
| Physics timestep | 0.01 s, `implicitfast` integrator | choice |
| Control interval | 0.1 s (10 Hz, command held for 10 physics steps) | choice |
| Episode | 30 s = 300 control steps | choice |

The unequal inertias are deliberate. They mean a spin about one axis leaks into the others (see §3), which is what makes this more than three independent one-axis problems.

This is an idealized actuator. A real spacecraft would use reaction wheels, which store the removed momentum and can saturate, or thrusters, which only push one way. Those come in a later stage (§9).

## 3. What the measurements showed

**Torque frame and sign.** A +0.01 N·m command on each axis for 1 s, from rest, produced exactly the predicted body angular velocity: 0.0923, 0.0414 and 0.0353 rad/s on x, y and z (that is τ·t/I). The result was identical with the spacecraft rotated 90°, so the commands act in the **body frame**, with positive commands giving positive body rotation. Note: free-joint *angular* DOFs are body-frame, but *linear* DOFs are world-frame, so a thruster added the same way in Project 2 would push in world axes, not body axes.

**Angular velocity is read from `qvel[3:6]`, in the body frame.**

**Coasting conserves momentum but the body spin wanders.** With no torque for 60 s, total angular momentum stayed at 0.027877 kg·m²/s (direction drift 0.02°) and kinetic energy changed by only 8 × 10⁻⁶. Yet the body-frame spin components swung widely, for example y went from +0.05 to as low as −0.11 rad/s. Two consequences:
- The observation must be the body rate *now*. A policy cannot treat each axis independently.
- Effort cannot be less than the momentum to remove: ∫‖τ‖ dt ≥ ‖H₀‖. That is a hard floor for any controller. (The effort metric reported later sums per-axis absolute torques, which can exceed this vector bound by up to √3 for multi-axis spins.)

**Gravity-gradient torque is negligible.** At rest with the long axis 45° off the local vertical, the measured torque was about 2.6 × 10⁻⁷ N·m, matching theory, which is 0.003% of the actuator limit. It stays on for realism, but it will not affect 1A. It does not need to be in the observation.

## 4. Observation (6 values)

| Index | Quantity | Scaling |
|---|---|---|
| 0–2 | Body angular velocity ω (rad/s), from `qvel[3:6]` | ÷ 0.3 |
| 3–5 | Previous action, normalized | none |

No attitude, orbit, or time information. Because gravity gradient is negligible and the torque model is ideal, ω plus the last command is enough to predict the next state. Remaining time is omitted because success depends only on the final 5 s and the baseline settles well before the deadline. Revisit this if the policy learns to "wait".

## 5. Action (3 values)

Policy output a ∈ [−1, 1]³ (clipped) → body torque τ = 0.01 · a N·m, held for 0.1 s.

## 6. Resets, curriculum and episode end

Random attitude (uniform quaternion) at every reset; it does not matter for 1A but avoids an identity-attitude bias that would matter for 1B.

| Stage | Initial body spin | Move on when |
|---|---|---|
| S1 | One random axis, magnitude 0.05–0.1 rad/s | ≥ 95% validation success |
| S2 | Each axis uniform in ±0.1 rad/s | ≥ 95% validation success |
| S3 (target) | Each axis uniform in ±0.3 rad/s | final evaluation |

- **Time-out at 300 steps** is a truncation: report it as `time_outs` in the step extras so rsl-rl bootstraps the value. This is the same convention as `examples/ppo/astrobee_env.py`.
- **Terminate early** only if ‖ω‖ > 1.0 rad/s (the policy has spun the craft up badly) or the state is non-finite. Both count as failures.

## 7. Reward (initial weights; tune on validation only)

Per control step:

r = −‖ω‖ / 0.3 − 0.02 ‖a‖² − 0.02 ‖a − a_prev‖² + 0.5 · [‖ω‖ < 0.01]

- The rate term dominates.
- The effort and smoothness terms are small, so they discourage chattering without making "do nothing" attractive.
- The bonus rewards actually reaching rest, not just slowing down.

The task is judged on the success gate and physical metrics, never on reward.

## 8. Success, baseline and evaluation

**Success (gate 1A):** ‖ω‖ < 0.01 rad/s continuously for the final 5 s of the 30 s episode. Target ≥ 95% on the test set.

**Baseline:** saturated rate damping, τ = clip(−K · I · ω, ±0.01 N·m) with K = 2 /s, at the same 10 Hz. It uses the true inertia, which is privileged knowledge the policy lacks; say so when comparing.

Measured baseline (50 random spins and attitudes per row, seed 0):

| Initial spin per axis | K (/s) | Success | Settle median / max | Effort ∫Σ\|τ\| | Saturated steps |
|---|---|---|---|---|---|
| ±0.1 rad/s | 0.5 | 100% | 4.5 / 5.3 s | 0.033 N·m·s | 1% |
| ±0.1 rad/s | 2.0 | 100% | 1.6 / 2.5 s | 0.029 N·m·s | 4% |
| ±0.3 rad/s | 0.5 | 100% | 8.0 / 10.7 s | 0.101 N·m·s | 12% |
| ±0.3 rad/s | 2.0 | 100% | 4.4 / 7.7 s | 0.091 N·m·s | 15% |

Settle time is when ‖ω‖ last dropped below 0.01 rad/s and stayed there.

**What this means:** with ±0.01 N·m and a 30 s episode, the baseline passes every case with 20+ s to spare. Its effort is already close to the ‖H₀‖ floor. So **PPO has little room to beat it.** Matching it is the realistic goal. The value of 1A is a verified training pipeline and a clean baseline comparison, not a better controller. If you want the deadline to bite, lower the torque limit (§10).

**Evaluation protocol** (from [EVALUATION.md](../../learning/EVALUATION.md)):
- Separate seed lists for validation and test. Never train on test seeds.
- Final: 200 test episodes per trained seed, three training seeds, deterministic policy (mean action).
- Run the baseline on exactly the same initial conditions.
- Report per metric: success rate with a binomial confidence interval, settle time, final ‖ω‖, effort ∫Σ|τ|dt alongside the ‖H₀‖ floor, saturation fraction, and action change. Count every episode, failures included.

## 9. Implementation

All files are in this folder, outside the upstream repo. Run each from `simulator/` with `/home/shrey31/.pixi/bin/pixi run -e rl --locked python ../projects/p1_detumble/<file>`.

| File | Purpose |
|---|---|
| [detumble_env.py](detumble_env.py) | Batched rsl-rl 5.x `VecEnv` on `mjorbit_warp`: §4–7 contract, 10× control decimation, per-world resets, `set_initial_conditions` for matched evaluation |
| [check_env.py](check_env.py) | GPU env vs CPU model before training. Passes (see §11). |
| [train.py](train.py) | PPO, one curriculum stage per call (`--stage s1/s2/s3`, `--resume`). Network 64-64, γ = 0.99. |
| [evaluate.py](evaluate.py) | Policy vs damping baseline on identical cases. `--split validation` (seed 100) or `test` (seed 1000); `--out` writes JSON. |
| [tb_summary.py](tb_summary.py) | Prints training curves from a run's tensorboard log |

Before each new run, write your prediction (settle time, effort, success), then compare.

Later realism stages, one at a time: inertia randomized ±20% (the policy cannot observe it); observation noise on ω; real reaction wheels via `ReactionWheelSpec` with a momentum limit, adding wheel speed to the observation and checking that wheel capacity exceeds the largest initial ‖H₀‖.

## 10. Decisions still open

- **Torque limit.** 0.01 N·m makes 1A easy, and the worst case settles in about 8 s. 0.002 N·m would make the largest spins need roughly 40 s, so the 30 s deadline and saturation would matter. Recommendation: keep 0.01 for the first pipeline run, then try 0.002 as the "hard" variant.
- **Gravity gradient** stays on. It is too small to matter here, and turning it off changes nothing measurable.

## 11. Results (7 November 2026, training seed 0)

**GPU env vs CPU model.** Under the same closed-loop damping law from (−0.3, 0.3, −0.3) rad/s, the two backends differ by at most 2.9 × 10⁻⁴ rad/s and both end at 4.5 × 10⁻⁷ rad/s. Open loop, the backends differ by a small step-size term once rotation involves more than one axis. It is about 1 × 10⁻⁵ rad/s after 1 s at dt = 0.01, about 5× smaller at dt = 0.002, and zero for single-axis spin. Tumbling amplifies it, to 7 × 10⁻³ rad/s after 30 s of open-loop torques that spun the craft above 1 rad/s. It is a discretization difference, not a bug, and is far below the 0.01 rad/s success threshold in the closed-loop case that matters.

**Training** (1,024 worlds, about 30k env steps/s, curriculum continued from each previous checkpoint):

| Stage | Iterations | Wall time | Final checkpoint | Validation success (200 cases) |
|---|---|---|---|---|
| S1 one axis, 0.05–0.1 rad/s | 150 | ~2.5 min | `logs/s1/model_149.pt` | 100% (CI 98.1–100%) |
| S2 box ±0.1 rad/s | 150 | ~2.5 min | `logs/s2/model_298.pt` | 100% |
| S3 box ±0.3 rad/s | 250 | ~4.2 min | `logs/s3/model_547.pt` | 100% |

**Test set** (S3 distribution, seed 1000, 200 cases, deterministic policy, the same cases for both). Raw data: [results_test_s3_seed0.json](results_test_s3_seed0.json).

| Metric | Damping K = 2 | PPO policy |
|---|---|---|
| Success (95% CI) | 100% (98.1–100%) | 100% (98.1–100%) |
| Failures / not at rest | 0 / 0 | 0 / 0 |
| Settle time median / p95 / max | 5.2 / 7.9 / 8.4 s | 5.15 / 7.8 / 8.3 s |
| Final ‖ω‖ worst case | 6.9 × 10⁻⁷ rad/s | 1.6 × 10⁻⁴ rad/s |
| Effort ∫Σ\|τᵢ\| dt (mean) | 0.0955 N·m·s | 0.1024 N·m·s (+7%) |
| Effort ÷ ‖H₀‖ | 1.51 | 1.63 |
| Saturated control steps | 16.1% | 16.4% |
| Mean action change per step | 0.0097 | 0.0120 |

**Reading it:**
- Gate 1A is met: ≥ 95% success on the test set.
- As predicted in §8, the policy **matches the baseline but does not beat it**. Settle times are the same to within 0.1 s. The policy uses 7% more torque effort, is slightly jerkier, and never quite reaches zero: it leaves a residual of about 10⁻⁴ rad/s by commanding small nonzero torques at rest. The baseline also uses the true inertia, which the policy does not see.
- The effort metric sums absolute per-axis torques, while the ‖H₀‖ floor bounds the vector magnitude. So for multi-axis spins the ratio can legitimately exceed 1 by up to √3. That is why the baseline shows 1.00 on single-axis S1 cases but about 1.5 here.

### Hard variant: torque limit 0.002 N·m (7 November 2026)

This is a stress test on the same ±0.3 rad/s distribution, with the episode still 30 s. The 95% gate does not apply, because many cases cannot be done at all.

**Feasibility.** The largest torque vector the per-axis limits allow has magnitude √3 · 0.002 N·m. So any controller needs at least ‖H₀‖ / (√3 · τ_max) seconds to stop the spin. That bound is optimistic. On the test set it exceeds the 25 s available before the 5 s hold in **31 of 200 cases** (29 of 200 on validation). Those cases are impossible for any controller. The median bound is 18.4 s.

**Baseline gain doesn't matter.** On validation, K = 2 and K = 20 (nearly pure maximum-torque braking) both scored 50%. With most steps saturated, the damping law already applies the box-corner torque τ = −τ_max · sign(h) on every axis. That is the step-by-step choice that shrinks ‖H‖ fastest.

**Training.** Fine-tuned from `logs/s3/model_547.pt` for 400 iterations (~7 min), run `logs/hard`. Training success plateaued at about 47% from iteration ~650. Validation: `model_750` 49.5%, `model_946` 49.0%, baseline 50%. `model_750` was chosen on validation.

**Test set** (seed 1000, 200 cases, the same cases for both). Raw data: [results_test_hard_seed0.json](results_test_hard_seed0.json).

| Metric | Damping K = 2 | PPO `hard/model_750` |
|---|---|---|
| Success (95% CI) | 48% (41.2–54.9%) | 47% (40.2–53.9%) |
| Not at rest at the end | 104 | 106 |
| Settle time median / p95 / max (successes) | 17.0 / 24.4 / 25.0 s | 16.75 / 24.1 / 24.5 s |
| Effort ∫Σ\|τᵢ\| dt (successes) | 0.0730 N·m·s | 0.0746 N·m·s |
| Saturated control steps | 79.4% | 77.4% |

Per-case overlap: both succeed 93, baseline only 3, policy only 1, neither 103.

**Reading it:**
- The policy ties the baseline, within one or two cases.
- It does not find the possible advantage of steering the momentum direction toward a body diagonal (where the box-limited torque is largest) using the gyroscopic coupling.
- That is consistent with the greedy damping law already being close to time-optimal here. It does not rule out a better non-greedy strategy, which a dedicated optimal-control study could answer.
- About 15% of cases are provably impossible, so the success ceiling is at most 84.5%, and probably lower.

**Not done yet:**
- Training seeds 1 and 2 for both variants (the protocol calls for three).
- Stress tests: inertia randomization, observation noise.

## Project 1B sketch (point and hold)

- Add attitude error to the observation as the vector part of the error quaternion, with sign chosen so the scalar part is ≥ 0. That keeps 0° and 360° equivalent and gives a continuous signal.
- Request random target attitudes.
- Baseline: proportional-derivative attitude control, τ = clip(−Kp · I · e − Kd · I · ω).
- Gate 1B: error < 5° and ‖ω‖ < 0.01 rad/s for the final 5 s; ≥ 90% success.
- A 180° slew at 0.01 N·m about the Izz axis needs roughly 2·√(θ·I/τ) ≈ 19 s at bang-bang optimum. So check the episode length against the largest requested slew before freezing 1B.
