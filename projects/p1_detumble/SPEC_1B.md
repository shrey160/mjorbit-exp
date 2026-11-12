# Project 1B spec: point to a requested attitude and hold it

Written 7 November 2026. Status: **trained (seed 0); gate 1B met. Chosen policy `logs/p3rv1ov2/model_797.pt` (reward v1 + zoomed-error observation, `--obs v2`): 100% success, settle median 12.9 s vs the baseline's 11.85 s, final error 1.17°.** Results and the full investigation are in §8–14.

The spacecraft, actuator, orbit and timing are the same as in [Project 1A](SPEC.md) §2: a 10 kg box, I = 0.108 / 0.242 / 0.283 kg·m², ideal body torques of ±0.01 N·m per axis, 0.01 s physics, 10 Hz control.

## 1. The question

Given its attitude error and spin, can a PPO policy turn the spacecraft to a requested orientation and hold it as reliably as a tuned attitude controller, and faster?

## 2. Contract

| Item | Definition |
|---|---|
| Observation (9) | Attitude error **e** (3), body ω ÷ 0.3 (3), previous action (3) |
| Attitude error | **e** = vector part of q_e = conj(q_current) ⊗ q_target, sign-flipped so q_e's scalar part ≥ 0. In the body frame. ‖**e**‖ = sin(angle/2), and **e** points along the body axis to rotate about. |
| Action (3) | a ∈ [−1, 1]³ → body torque τ = 0.01 · a N·m, held 0.1 s |
| Episode | 40 s (400 control steps). Longer than 1A because a 180° turn needs about 19 s at this torque limit. |
| Reset | Random attitude. Residual spin uniform ±0.02 rad/s per axis. Target = current attitude rotated about a random axis by an angle uniform in [0, θ_max]. |
| Early termination | ‖ω‖ > 1 rad/s or a non-finite state (failure) |
| Success (gate 1B) | Angle error < 5° **and** ‖ω‖ < 0.01 rad/s for the final 5 s. Target ≥ 90%. |

Why the sign fix: q and −q describe the same orientation. Choosing the scalar part ≥ 0 picks the shorter way round (≤ 180°) and makes **e** continuous everywhere except exactly at 180°, where the shorter direction flips.

**Curriculum** (`train.py --stage`):

| Stage | θ_max |
|---|---|
| p1 | 30° |
| p2 | 90° |
| p3 (target distribution) | 180° |

**Reward per step** (initial weights):

r = −angle/π − 0.2 ‖ω‖/0.3 − 0.02 ‖a‖² − 0.02 ‖a − a_prev‖² + 0.5·[angle < 5°] + 0.5·[angle < 5° and ‖ω‖ < 0.01]

## 3. Feasibility

An optimistic lower bound on any rest-to-rest turn uses the smallest inertia (0.108) and the largest torque magnitude the per-axis limits allow (√3 · 0.01 N·m): t ≥ 2√(θ / α). That is at most 8.8 s for the validation targets (largest 176°), against 35 s before the hold window. No case is provably impossible. The bound is loose: a real turn about the high-inertia z axis needs about 19 s.

## 4. Baseline: rate-limited attitude controller

Implemented in [evaluate_pointing.py](evaluate_pointing.py). It uses the true inertia, which is privileged knowledge the policy does not have.

- angle, axis from the error quaternion
- ω_ref = axis · min(Kp · angle, √(2 β α_min · angle)), with α_min = τ_max / I_max = 0.0353 rad/s² (the weakest axis's deceleration)
- τ = clip(Kd · I · (ω_ref − ω), ±τ_max)

The square-root term caps the turn rate at what can still be braked in the remaining angle, so it doesn't overshoot.

**Gain tuning on validation only** (p3 distribution, seed 100, 200 cases; all 100% success, overshoot about 0.03°):

| Kp | Kd | β | Settle median / max | Effort | Saturated steps |
|---|---|---|---|---|---|
| 0.3 | 2 | 0.5 | 15.1 / 22.9 s | n/a | 8% |
| **0.6** | **2** | **0.8** | **11.4 / 18.5 s** | **0.146 N·m·s** | **12%** |
| 1.0 | 4 | 0.8 | 11.35 / 18.7 s | 0.148 | 12% |
| 0.6 | 4 | 0.9 | 11.5 / 18.3 s | 0.154 | 14% |
| 1.0 | 4 | 0.95 | 10.9 / 19.5 s | 0.159 | 14% |
| 2.0 | 6 | 0.95 | 11.05 / 19.1 s | 0.160 | 13% |

Gains above about 0.6 / 2 / 0.8 stop helping, and the median sits at about 11 s. **Frozen: Kp = 0.6, Kd = 2, β = 0.8.**

**Where a policy could do better:** the rate cap assumes the weakest-axis deceleration for every turn. Turns about the light x axis could brake about 2.6× harder. Turns along a diagonal can also use torque on all three axes at once. A policy that learns axis-dependent braking could settle faster. The baseline's success rate leaves no room, so the comparison is about settle time.

## 5. Evaluation

Same protocol as 1A ([SPEC.md](SPEC.md) §8):
- Validation seed 100 for checkpoint choice; test seed 1000 used once.
- 200 cases each, deterministic policy, both controllers on identical spins, attitudes and targets.
- Metrics: success with a Wilson CI, success on targets > 120°, settle time, final angle and spin, overshoot, effort, saturation, action change, per-case overlap.

## 6. Files

| File | Purpose |
|---|---|
| [pointing_env.py](pointing_env.py) | `PointingEnv`, a subclass of `DetumbleEnv` (hooks `_task_terms`, `_on_reset`), plus quaternion helpers |
| [train.py](train.py) | `--stage p1/p2/p3` selects the pointing env |
| [run_pointing_curriculum.sh](run_pointing_curriculum.sh) | Runs p1 → p2 → p3, each resuming the previous stage's last checkpoint |
| [evaluate_pointing.py](evaluate_pointing.py) | Policy vs baseline. Without `--checkpoint`, baseline only. |

## 7. Not in scope yet

- Targets that move (e.g. tracking nadir as the orbit turns).
- Wheel momentum limits.
- Starting from a large spin, which would mean detumbling and pointing in one episode.
- Observation noise.

## 8. Results (7 November 2026, training seed 0)

**Training** (1,024 worlds, fresh network; the 1A policy can't be reused because the observation grew from 6 to 9 values):

| Stage | Iterations | Final checkpoint | Training success at end |
|---|---|---|---|
| p1 ≤ 30° | 200 | `logs/p1/model_199.pt` | 100% (from about iteration 150) |
| p2 ≤ 90° | 200 | `logs/p2/model_398.pt` | 100% |
| p3 ≤ 180° | 400 | `logs/p3/model_797.pt` | 100% |

Total about 13 minutes. Validation: `model_700` and `model_797` both 100%, with median settle 15.55 s and 15.3 s. `model_797` was chosen.

**Test set** (seed 1000, 200 cases, targets up to 180°, median 87°). Raw data: [results_test_p3_seed0.json](results_test_p3_seed0.json).

| Metric | Rate-limited PD (0.6 / 2 / 0.8) | PPO `p3/model_797` |
|---|---|---|
| Success (95% CI) | 100% (98.1–100%) | 100% (98.1–100%) |
| Success on targets > 120° | 100% | 100% |
| Settle time median / p95 / max | **11.85** / 16.8 / 18.0 s | 15.8 / 22.6 / 26.1 s |
| Final angle error, worst | 0.001° | **2.83°** |
| Final ‖ω‖, worst | 1.1 × 10⁻⁵ rad/s | 1.7 × 10⁻⁴ rad/s |
| Mean overshoot | 0.02° | 0.04° |
| Effort (successes) | 0.154 N·m·s | 0.160 N·m·s |
| Saturated steps | 13.4% | 12.1% |

Per-case overlap: both succeed in all 200.

**Reading it:**
- **Gate 1B is met** (100% ≥ 90%).
- **The policy is clearly worse than the baseline on quality.** It settles about 4 s later at the median, 8 s later in the worst case, and stops about 2.8° short of the target in every case.
- **The 2.8° offset comes from the reward, not a capability limit.** Everything under 5° earns the full 0.5 + 0.5 bonus, and the remaining −angle/π penalty at 2.8° is only about 0.016 per step. So the policy has almost no reason to close the last few degrees, and it settles just inside the success threshold. This is the "threshold satisficing" failure: it optimizes exactly what was rewarded.
- The axis-dependent braking advantage did not appear. Slower settling suggests the policy brakes cautiously, not optimally.

**Candidate fixes, to be tuned on validation only:**
- Make the attitude term steep near zero, e.g. −angle/π − w·(1 − exp(−(angle/2°)²)), or a log-shaped term.
- Replace the flat < 5° bonus with a graded precision bonus, like the truss example's "two Gaussian precision bonuses".
- Add a time/speed incentive so settling earlier pays: the success bonus is already per-step, so reaching it sooner earns more, but the 2.8° plateau also gets it.

## 9. Reward v2 (`--reward v2`)

Changes from v1, with everything else in the contract unchanged:

r = −angle/π − 0.02 ‖a‖² − 0.02 ‖a − a_prev‖²
  − 0.5 · e^{−(θ°/15)²} · ‖ω‖/0.3   (rate penalty only near the target)
  + 0.5 · e^{−(θ°/5)²}   (coarse precision)
  + 1.0 · e^{−(θ°/0.5)²}   (fine precision)
  + 0.5 · e^{−(θ°/0.5)²} · e^{−(‖ω‖/0.005)²}   (fine-pointed and still)

where θ° is the angle error in degrees.

- At 2.8° the v1 plateau now forgoes about 1.6 reward per step, against 0.016 before.
- Far from the target the rate penalty is near zero, so fast turns are not punished. v1 penalized ‖ω‖ everywhere, which is a likely cause of the slow settling.

**Prediction, written before training:** final error below 0.5° in nearly all cases, and median settle closer to the baseline's 11.85 s, possibly below it. Risk: more overshoot, since the rate penalty only acts near the target.

Runs: `logs/p1v2`, `logs/p2v2`, `logs/p3v2` via `run_pointing_curriculum.sh v2`. Results in §10.

## 10. Reward v2 result: prediction wrong, diagnosis changed

Validation (seed 100), `p3v2` checkpoints vs the frozen baseline:

| Checkpoint | Success | Settle median / max | Final error p95 / max |
|---|---|---|---|
| `p3v2/model_700` | 100% | 15.5 / 27.6 s | 2.63° / 2.64° |
| `p3v2/model_797` | 100% | 15.5 / 28.8 s | 3.03° / 3.04° |
| baseline | 100% | 11.4 / 18.5 s | 0.001° |

The reshaped reward changed almost nothing, so the satisficing explanation in §8 was wrong. Probing the policies directly ([diag_offset.py](diag_offset.py)):

| Policy | Action at "on target, at rest, previous action 0" | Final error over 64 cases | Final error direction (body frame) |
|---|---|---|---|
| v1 `p3/model_797` | (−0.21, +0.23, +0.16) of full torque | 2.78–2.83° | (0.665, −0.549, −0.507), spread ≤ 0.001 |
| v2 `p3v2/model_797` | (−0.20, −0.30, −0.23) | 2.97–3.04° | (0.234, 0.852, 0.468), spread ≤ 0.004 |

**Diagnosis: a fixed point of the network, not of the reward.**
- Each policy has one zero-torque point about 3° off target, in a fixed body direction, and every episode converges to it.
- Near the target the error input is tiny (sin(θ/2) ≈ 0.02 at 2.5°), so the network needs a steep slope (gain) there.
- Its zero crossing sits at bias ÷ gain ≈ 0.2 ÷ 10 ≈ 0.02 in input units, about 2.5°.
- Reward shaping can't move that much, because the problem is the network's resolution near zero input.

## 11. Observation v2 (`--obs v2`): zoomed attitude error

Append clip(**e** ÷ 0.02, ±1) to the observation: 12 values, where indices 0–8 are unchanged, so the baseline is unaffected. This copy saturates at about 2.3°. The same leftover bias then moves the stopping point about 50× closer, to roughly 0.05°.

**Prediction:** final error < 0.2° in all cases. Settle time is uncertain.

Runs: `logs/p1v3`, `p2v3`, `p3v3` (reward v2 + obs v2) via `run_pointing_curriculum.sh v2 v2`. Results in §12.

## 12. Obs v2 result: offset fixed, large turns regressed

`p3v3/model_797` probe: action at zero observation is (0.013, 0.016, 0.002), about 1% of full torque, down from 20–30%.

Validation (seed 100), against the frozen baseline (100%, settle 11.4 / 18.5 s):

| Checkpoint | Success (95% CI) | Success, targets > 120° | Settle median / p95 / max | Final error p95 | Final error max |
|---|---|---|---|---|---|
| `p3v3/model_700` | 91% (86.2–94.2%) | 70% | 17.45 / 32.6 / 33.9 s | 0.016° | 36.5° |
| `p3v3/model_797` | 92% (87.4–95.0%) | 74% | 17.6 / 32.2 / 34.7 s | 0.011° | 29.3° |

Training success on p3 plateaued at about 91% (the earlier versions reached 100%). Per-case overlap for `model_797`: both 184, baseline only 16, policy only 0.

**Reading it:**
- The precision prediction held: typical final error is now about 0.01°, roughly 250× better.
- The large turns got worse. Failures are still turning at the deadline (final ‖ω‖ up to 0.06 rad/s), and the policy is at full torque only 11% of the time.

**Hypothesis: discount horizon too short.** γ = 0.99 at 10 Hz weights rewards about 10 s ahead (1/(1−γ) = 100 steps). A 180° turn takes about 19 s or more, so the precision bonuses at the end are heavily discounted when the turn starts. v1's flat 5° bonus was reached sooner, which hid this. The truss example uses γ = 0.997 (about 33 s).

**Prediction for γ = 0.997:** large-turn success back toward 100%, and median settle closer to the baseline, with precision kept.

Runs: `logs/p1v3g0.997`, `p2v3g0.997`, `p3v3g0.997` via `GAMMA=0.997 run_pointing_curriculum.sh v2 v2`. Results in §13.

## 13. γ = 0.997 result: small gain; failures are slow cruising

Validation, against the baseline (100%, 11.4 / 18.5 s):

| Checkpoint | Success (95% CI) | Success, targets > 120° | Settle median / p95 / max | Final error p95 |
|---|---|---|---|---|
| `p3v3g0.997/model_700` | 92.5% (88.0–95.4%) | 75% | 15.5 / 30.4 / 34.9 s | 0.021° |
| `p3v3g0.997/model_797` | 93.5% (89.2–96.2%) | 79% | 15.4 / 31.2 / 35.0 s | 0.022° |

The prediction held only partly: an improvement, but no return to 100%.

Failure traces ([diag_failures.py](diag_failures.py), `model_797`, 13 failures, all with initial error 113–172°):
- **Failing turns:** cruise at 0.07–0.10 rad/s peak, average command 2–9% of full torque, at full torque on only 1–8% of steps. They are still turning at the deadline (e.g. case 110: 171° → 64° after 39 s).
- **Successful large turns:** reach 0.17–0.29 rad/s and are at full torque on about 20% of steps (e.g. case 11: 163° → 0° in 20 s).

So the policy sometimes settles into a slow cruise and never speeds up. It is not oscillating or confused near 180°.

**Next ablation:** reward v1 + obs v2. v1 (flat bonuses, rate penalized everywhere) succeeded on all large turns with obs v1. If it also does with the zoomed observation, the reshaped reward is what caused slow cruising, and v1 + obs v2 gives both reliability and precision.

Runs: `logs/p1rv1ov2`, `p2rv1ov2`, `p3rv1ov2` via `run_pointing_curriculum.sh v1 v2` (γ = 0.99). Results in §14.

## 14. Reward v1 + obs v2: chosen policy

Training success reached 100% in every stage.

Validation:

| Checkpoint | Success | Targets > 120° | Settle median / p95 / max | Final error (all cases) |
|---|---|---|---|---|
| `p3rv1ov2/model_700` | 100% | 100% | 12.85 / 18.6 / 20.2 s | 1.13° |
| `p3rv1ov2/model_797` | 100% | 100% | 12.6 / 18.4 / 21.2 s | 1.17° |

The ablation confirmed it: with v1's reward, large turns succeed again, so the reshaped v2 reward caused the slow cruising. **`p3rv1ov2/model_797` chosen on validation.**

**Test set** (seed 1000, used once; 200 cases, targets up to 180°). Raw data: [results_test_p3rv1ov2_seed0.json](results_test_p3rv1ov2_seed0.json).

| Metric | Rate-limited PD | PPO v1 (`p3/model_797`, §8) | **PPO chosen (`p3rv1ov2/model_797`)** |
|---|---|---|---|
| Success (95% CI) | 100% (98.1–100%) | 100% | **100% (98.1–100%)** |
| Targets > 120° | 100% | 100% | **100%** |
| Settle median / p95 / max | 11.85 / 16.8 / 18.0 s | 15.8 / 22.6 / 26.1 s | **12.9 / 18.4 / 21.1 s** |
| Final angle error (worst) | 0.001° | 2.83° | **1.17°** |
| Final ‖ω‖ (worst) | 1.1 × 10⁻⁵ rad/s | 1.7 × 10⁻⁴ | **1.1 × 10⁻⁵** |
| Overshoot (mean) | 0.02° | 0.04° | **0.02°** |
| Effort (successes) | 0.154 N·m·s | 0.160 | **0.171** |
| Mean action change | 0.015 | 0.016 | **0.028** |

**Summary of the 1B investigation:**
1. v1 met the gate but was slow and stopped 2.8° short.
2. The offset was not reward satisficing. It was the network's fixed zero-torque point, bias ÷ gain, because near-target inputs are tiny (§10). A zoomed copy of the error input (obs v2) shrinks it, about 250× with reward v2 and 2.4× with reward v1.
3. Reward v2 (graded precision bonuses, rate penalty only near the target) gave about 0.01° precision but made the policy sometimes cruise slowly through large turns: 92–94% success, about 75–79% on targets over 120°. γ = 0.997 helped only slightly (§12–13).
4. Reward v1 + obs v2 gives full reliability and near-baseline speed (median +1.05 s, worst +3.1 s) with a 1.17° residual offset.

**Where it stands:**
- The policy still does not beat the baseline: about 1 s slower at the median, about 11% more effort, jerkier actions, and 1.17° vs 0.001° final error.
- The baseline uses the true inertia. The policy learned without it.

**Next options, validation only:**
- v1 reward plus only a fine precision bonus (keep v1's flat 5° terms and rate penalty, add + w·e^{−(θ°/0.5)²}). This aims at v2's precision without its cruising.
- An odd-symmetric policy, π(o) = (g(o) − g(−o))/2, which makes zero error give exactly zero torque by construction.
- More training seeds, as the protocol requires.

## 15. Reward v3 (`--reward v3`): v1 plus a fine precision bonus

r_v3 = r_v1 + 0.5 · e^{−(θ°/0.5)²}

- Everything in v1 is kept, including the flat < 5° bonuses and the rate penalty everywhere, which kept large turns brisk.
- At the 1.17° stopping point the new term is worth about 0.002. Fully on target it is worth 0.5 per step, so closing the last degree now pays.
- Used with obs v2 (zoomed error).

**Prediction, written before training:** final error < 0.2°; success and settle time similar to `p3rv1ov2` (100%, median about 12.6–12.9 s on validation/test).

Runs: `logs/p1rv3ov2`, `p2rv3ov2`, `p3rv3ov2` via `run_pointing_curriculum.sh v3 v2`. It is compared with `p3rv1ov2/model_797` on validation first. The test set is used only if v3 is chosen over it. Results in §16.

## 16. Reward v3 result: precision fixed, settling slower (validation only)

Training success reached 100% in every stage. Validation (seed 100, 200 cases):

| Metric | Baseline | Chosen v1 + zoom (`p3rv1ov2/model_797`) | v3 + zoom `model_700` | v3 + zoom `model_797` |
|---|---|---|---|---|
| Success | 100% | 100% | 100% | 100% |
| Targets > 120° | 100% | 100% | 100% | 100% |
| Settle median / p95 / max | 11.4 / 16.6 / 18.5 s | **12.6 / 18.4 / 21.2 s** | 16.1 / 25.8 / 27.4 s | 15.8 / 25.3 / 27.8 s |
| Final error, max | 0.001° | 1.17° | 0.029° | **0.028°** |
| Effort (successes) | 0.146 N·m·s | 0.160 | 0.140 | 0.142 |
| Mean action change | 0.015 | 0.027 | 0.034 | 0.034 |

**Reading it:**
- **Precision:** the prediction held better than expected. The worst final error is 0.028°, about 40× better than v1 + zoom, with no loss of reliability.
- **Speed:** the prediction failed. The median "settled" time (inside 5° and still) is about 3 s later and the worst case about 6.6 s later. The policy now approaches more slowly, apparently trading speed for a gentler final approach.
- **Effort** is slightly lower than both the v1 + zoom policy and the baseline.
- **Neither policy dominates.** Both meet gate 1B at 100%. The choice depends on which matters more: settle time (v1 + zoom) or final accuracy (v3 + zoom). The test set has been used once, for `p3rv1ov2/model_797`. If v3 is also tested, that is a second test-set use and should be reported as such.

## 17. Final 1B choice: v3 + zoom, tested (second test-set use)

The user chose the precise policy, `logs/p3rv3ov2/model_797.pt` (`--reward v3 --obs v2`), as the final 1B policy. It was run once on the test set (seed 1000, 200 cases; median initial error 87°, max 180°; 0 provably infeasible). **This is the second use of the test set.** The first was `p3rv1ov2/model_797` (§14), and that comparison informed this choice, so these numbers are slightly less independent than a single first look would be. Results: `results_test_p3rv3ov2_seed0.json`.

| Metric | Baseline (rate-limited PD) | v1 + zoom, test (§14) | **v3 + zoom, test (final)** |
|---|---|---|---|
| Success (95% CI) | 100% [98.1, 100] | 100% | **100% [98.1, 100]** |
| Targets > 120° | 100% | 100% | 100% |
| Settle median / p95 / max | 11.85 / 16.8 / 18.0 s | 12.9 / 18.4 / 21.1 s | **16.25 / 25.0 / 26.8 s** |
| Final error, p95 / max | 0.0006° / 0.001° | 1.17° max | **0.0275° / 0.028°** |
| Effort (successes) | 0.154 N·m·s | 0.171 | **0.145** |
| Saturated-step fraction | 0.134 | — | 0.127 |
| Mean action change | 0.015 | — | 0.035 |
| Per-case overlap | — | — | both succeed on all 200 |

Test results match validation (§16) closely: settle times within 0.5 s, the same 0.028° worst error, and effort below the baseline.

**Verdict on gate 1B:** passed. The final policy is 100% reliable, uses about 6% less torque effort than the hand-written controller, and points within 0.03°. It settles about 4.4 s slower at the median (about 37%), and its commands change about 2× more between steps. It does not beat the analytic controller, which is near-optimal for this idealized problem. What it shows is that PPO plus a well-chosen observation can match that controller's reliability and come close on precision.
