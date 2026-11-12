# Project 2A: approach and hold near a co-orbiting satellite

Status: **Project 2 done. 2A: m3/model_1500 (ideal), u1/model_2099 (u1). 2B (sections 15–18): filter, residual r1/model_599, shield, 1000-case Monte Carlo and the single test-set run (section 18). Test set used once, 8 Nov.**

## 1. Task

A 10 kg chaser (the Project 1 spacecraft) starts 20–50 m from an inert 100 kg, 1 m target in the same 400 km circular orbit. It must fly to a hold point 3 m behind the target, brake, and stay there without entering a keep-out sphere.

**Scope of 2A: translation only.** The chaser's attitude is assumed held inertially fixed by its reaction wheels (solved in Project 1). Its body-fixed thrusters therefore act along fixed world axes, and the scene models them as three world-frame force actuators of ±0.05 N each. Coupling attitude and translation is a later stage (2B).

Frames: x radial (away from Earth), y along-track (direction of motion), z orbit normal. These are the **LVLH** axes (local vertical, local horizontal), which rotate once per orbit. The simulator world frame is chief-centered but does not rotate. All task quantities are in LVLH.

## 2. Contract

| Item | Value |
|---|---|
| Hold point | (0, −3, 0) m LVLH: 3 m behind the target on V-bar (the along-track axis) |
| Keep-out | distance to target centre < 2.0 m means **failure** and the episode ends (contact is at about 1.2 m) |
| Success | ‖p − hold‖ < 0.25 m **and** ‖v_LVLH‖ < 0.01 m/s, continuously for the final 60 s |
| Episode | 600 s, control at 1 Hz (600 decisions) |
| Initial state | distance 20–50 m, direction uniform within a 60° cone around −V-bar (the along-track axis); LVLH velocity uniform ±0.02 m/s per axis |
| Actuator | ±0.05 N per world axis (5 mm/s² on 10 kg), held for 1 s |
| Fuel (Δv) | Σ‖F‖₁/m · dt, the per-axis 1-norm, because each axis has its own thruster pair |
| Splits | validation seed 100 (all choices), test seed 1000 (once), 200 cases each |

The start cone keeps the straight line to the hold point clear of the target. Starts from the far side, where the chaser must go around the target, are a later harder variant.

## 3. Measurements (`measure.py`, CPU)

| Check | Result |
|---|---|
| Masses | chaser 10.00 kg, target 100.00 kg |
| Thrust pulse, 10 s at +0.05 N world x | Δv_x = 0.05000 m/s (expected 0.05); Δv_y,z ≈ 2e-4 m/s, from differential gravity at 20 m |
| Free drift vs Clohessy–Wiltshire (the analytic relative-orbit solution), 600 s | error 0.1 mm at dt 0.01 s; target at the chief stays at 0 m |
| Physics dt vs dt 0.01, 600 s drift | 0.05 s: 0.5 mm; 0.1 s: 1.1 mm; 0.2 s: 2.4 mm; 0.5 s: 6 mm; **1.0 s: 12 mm** |
| Hold point (0, −3, 0), zero LVLH velocity, 600 s | moves 0.000 m: a natural equilibrium, so holding is free |
| Hold point offset 0.5 m radially | drifts 0.37 m in 600 s (leaves tolerance in about 5 min) |
| Hold point offset 0.5 m cross-track | oscillates: 0.11 m off after 600 s |
| Relative-gravity accelerations | 3n²x at 50 m is 1.9e-4 m/s²; Coriolis at 0.1 m/s is 2.3e-4 m/s²; the thrusters are about 26× stronger |
| No control, 50 cases | 0% success, no collisions |

**Physics timestep: 1.0 s** (decimation 1). The drift error is 1.2 cm per episode against a 25 cm tolerance, and the closed-loop baseline is unchanged (Δv 0.678 vs 0.679 m/s, settle median 226 vs 225 s). This makes training about 10× cheaper than dt 0.1. The final test will be cross-checked at dt 0.1.

## 4. Baseline (`baseline.py`)

Velocity-limited PD in LVLH with feedforward that cancels the relative-gravity (CW) terms:

```
e      = hold − p
v_ref  = ê · min(Kp‖e‖, √(2β a_min ‖e‖), v_max)
u      = Kd (v_ref − v) − (3n²x + 2n v_y,  −2n v_x,  −n² z)
F      = clip(m · C_lvlh→world · u, ±0.05 N) per world axis
```

Sweep on 60 validation cases (Kp ∈ {0.02, 0.05, 0.1}, Kd ∈ {0.2, 0.5, 2}, v_max ∈ {0.1, 0.2, 0.4} m/s):
- **v_max sets the speed–fuel trade-off.** Kp and Kd barely matter, apart from Kp 0.02 with v_max 0.1, which is too slow (87% success).
- **Kd = 2 chatters** at 1 Hz and burns 5–7 m/s of Δv.
- Frozen gains: **Kp 0.05, Kd 0.5, β 0.8.**

A fixed v_max is a weak fuel baseline: near starts cruise as fast as far ones. The **planned** variant picks its cruise speed per case, v_max = d₀ / T_plan.

Validation, all 200 cases, dt 1.0 s:

| Baseline | Success | Collisions | Settle median / max | Δv mean | Max closing speed inside 10 m |
|---|---|---|---|---|---|
| fixed v_max 0.2 (fast) | 100% | 0 | 226 / 292 s | 0.678 m/s | 0.200 m/s |
| fixed v_max 0.1 | 100% | 0 | 381 / 511 s | 0.397 | 0.100 |
| planned T 250 s | 100% | 0 | 290 / 301 s | 0.502 | 0.187 |
| planned T 350 s | 100% | 0 | 381 / 393 s | 0.391 | 0.134 |
| **planned T 400 s (frozen fuel baseline)** | **100%** | **0** | **428 / 440 s** | **0.356** | 0.117 |
| planned T 450 s | 100% | 0 | 475 / 486 s | 0.330 | 0.104 |

A rough Δv floor is accelerate-then-brake covering d₀ in 540 s, ignoring the initial velocity and CW terms: 0.127 m/s mean as a 2-norm, or about 0.19 m/s once the per-axis 1-norm is counted. The baseline's continuous PD braking tail and constant CW cancellation leave a gap of roughly 0.15 m/s.

## 5. Gate for 2A

Success alone is not a useful target: the baseline already reaches 100%. The learning target is **fuel**:

- **Required:** success ≥ 98% (Wilson lower bound ≥ 95%), zero collisions, on validation then test.
- **Goal:** mean Δv at least 15% below the planned-400 s baseline (≤ 0.30 m/s) at that reliability.
- **Reported:** settle time, max closing speed inside 10 m, saturation, action smoothness, per-case overlap with the baseline, and a dt-0.1 re-run of the final policy.

## 6. Environment design (to build)

- **Backend:** GPU `mjorbit_warp`, 4096 worlds, dt 1.0 s, 1 control step per physics step. Estimated about 3 s per PPO iteration (32 steps × 4096 worlds).
- **LVLH per world:** computed from the pulled chief state (R, V): x̂ = R/‖R‖, ẑ = R×V/‖R×V‖, ŷ = ẑ×x̂, ω = ‖R×V‖/‖R‖², v_LVLH = Cᵀ(v_world − ω×r). Checked against the CPU helpers before training.
- **Observation (13):** (p − hold)/50 (3), v_LVLH/0.2 (3), time remaining / 600 (1), previous action (3), and a zoomed position error clip((p − hold)/0.5, ±1) (3), the Project 1B lesson.
- **Action:** a ∈ [−1, 1]³ is the LVLH force command × 0.05 N. It is rotated to world and clipped per world axis.
- **Reward (v1):**
  - −w_pos·‖p − hold‖/50 (progress)
  - −w_fuel·‖F‖₁/F_max
  - −w_smooth·‖Δa‖²
  - +w_near·exp(−(‖e‖/0.25)²)
  - +w_hold·[success condition]
  - −10 and terminate on keep-out entry
- **Discount:** γ = 0.99 gives a 100 s horizon at 1 Hz, which is short for a 400 s transfer. Expect to need γ 0.995–0.998. Test this early.
- **Curriculum:** a1 at 5–15 m, a2 at 10–30 m, a3 at 20–50 m (the target distribution).

## 7. Risks

- **Fuel vs time is a credit-assignment problem over hundreds of steps.** A policy that learns "go fast" is easy, and "coast slowly then brake exactly" is hard.
- **The fuel term can teach "don't move".** Holding still is safe but fails the task. The progress and hold terms must outweigh fuel early. Consider ramping w_fuel up over training.
- **The policy may exploit CW drift** (radial offset → free along-track motion). That is legitimate and is a reason RL could beat the baseline.
- **Collisions are rare events** in the start cone, so a policy may never learn to avoid the target. The harder far-side variant would test this.

## 8. Build and check (`approach_env.py`, `check_env.py`)

- GPU env as in section 6: 4096 worlds, dt 1 s, LVLH computed per world from the pulled chief state. The episode end is a true termination (time is observed), so `time_outs` is never set.
- LVLH helpers match the CPU conversions to 1.4e-14.
- The GPU baseline reproduces the CPU results: planned T400 gives 100%, settle 428/440 s, Δv 0.348 (CPU 0.356); fast gives 100%, 228/292 s, Δv 0.679 (CPU 0.678). All policy comparisons below use the GPU numbers.
- 0.7 s per PPO iteration (4096 worlds × 32 steps).

## 9. Training history (validation, stage a3, 200 cases unless noted)

| Run | Change | Outcome |
|---|---|---|
| a1v1 | reward as in section 6: shaping 0.1/m, fuel 0.2 | **collapse to "never thrust"**: action std 0.5 → 0, 0% success. Exploration noise costs fuel at once, while the payoff needs precise flying |
| a1v2 | potential shaping 1.0/m plus precision potentials (5 near, 5 near-and-still), fuel 0.02 | learns the task: 100% on a1. But it flies fast: Δv 0.48 vs planned 0.094 on a1 cases |
| a2/a3 v2f02 | from a1v2, fuel raised to 0.2 | 100% success, **Δv 0.79**, worse than the fast baseline. Std collapses to 0.01, so no further exploration |
| a1f02e01 | from scratch, fuel 0.2, entropy 0.01 | collapse (std 0.03, 0%) |
| a1dz | from scratch, fuel 0.2, 0.2 action deadzone | collapse ("don't move" became even easier) |
| a3ft | from a3v2f02, shaping 0.1/m, GAE λ 0.99, **std floor 0.1** | Δv falls to 0.54–0.67, but success swings between checkpoints (23%–100%): the policy arrives near the deadline |

Diagnostic (`diag_returns.py`): under the training reward, planned T400 scores 58.5 against 45.6 for the fast baseline (discounted). **The reward ranks frugal flying correctly.** The failure is optimisation: fuel is cheapest to save through decisions made hundreds of steps before the payoff, and 1 Hz direct thrust with fuel-costly Gaussian exploration does not find them.

Best reliable checkpoint, `a3ft/model_1050`: 100% success, 0 collisions, settle median 450 s, **Δv 0.581**. That beats the fast baseline (0.679) but not planned PD (0.348).

## 10. Stronger classical baseline: CW impulsive targeting (`baseline.py --cw`)

Burn onto the coasting CW arc that reaches the hold point at t_arrive, coast (re-targeting only when the velocity error exceeds 2 mm/s), then hand over to PD hold 10 s before arrival.

| t_arrive | Success | Settle median / max | Δv mean | Min distance |
|---|---|---|---|---|
| 300 s | 100% | 313 / 360 s | 0.370 | 2.40 m |
| 400 s | 100% | 411 / 420 s | 0.270 | 2.83 m |
| **480 s** | **100%** | **490 / 497 s** | **0.227** | 2.96 m |

**The gate in section 5 was set against a weak baseline.** CW targeting alone already meets it. Any learned controller should be compared against CW targeting at 480 s (Δv 0.227 m/s). In this idealised setting (perfect knowledge, exact CW-like dynamics, no noise), CW targeting is close to the optimum. An RL policy can only expect an advantage once the model is wrong or uncertain.

## 11. Maneuver-epoch actions (`maneuver_env.py`, `--env maneuver`)

The research (Hovell & Ulrich's "deep guidance"; impulsive-manoeuvre RL for station keeping and inspection) and the section 9 diagnosis both point the same way: shorten the decision horizon.

- **Decision every 20 s** gives 30 decisions per episode instead of 600.
- **Action:** an LVLH Δv request, ±0.1 m/s per axis. It is rotated to world, clipped per axis, and flown as a full-thrust burn from the start of the epoch (at most 20 s at 5 mm/s²), followed by a coast. Physics stays at 1 s. Keep-out and success are checked every second.
- **Reward per decision:** potential shaping (γ = 0.99 per decision), −w_fuel·‖Δv‖₁/0.1, the hold-window terms summed over the epoch's seconds, the terminal success bonus, and −10 on keep-out entry.
- **Training:** 2048 worlds × 16 decisions per iteration, about 1.8 s per iteration.

**Benchmarks (GPU, validation, 200 cases).** The 1 Hz CW version reproduces the CPU result (0.231 vs 0.227).

| Controller | Success | Settle median | Δv mean |
|---|---|---|---|
| PD planned T400 (1 Hz) | 100% | 428 s | 0.348 |
| CW targeting 480 s, 1 Hz | 100% | 491 s | **0.232** |
| CW targeting 480 s, 20 s epochs (same action space as the policy) | 100% | 522 s | **0.292** |

**Runs:**
- **m1:** from scratch on a3, w_shape 1.0/m, w_fuel 2, std floor 0.05, 400 iterations. No "never thrust" collapse. Checkpoint 200 reached 97.5% success at Δv 0.510, and checkpoint 399 reached **100% success at Δv 0.494**, settling at 443 s (early).
- **m2:** from m1, with shaping 0.1/m, GAE λ 0.99, w_fuel 4, std floor 0.02, 400 iterations. Every checkpoint was 100% with 0 collisions, and Δv fell monotonically 0.458 → 0.441 → 0.416 → 0.405 → **0.396** (model_798). It still settles at about 440 s, so it is not using the full time budget.
- **m3:** m2 continued for 800 more iterations. Δv plateaued around 0.32. Checkpoints 1200–1597 were 100% (1550 was 99.5%), with Δv 0.334 → 0.320 and settle about 500 s.

## 12. Selected policy: `logs/m3/model_1500.pt` (validation; `results_val_m3_1500.json`)

Selection rule: 100% validation success first, then lowest mean Δv, among checkpoints 1200–1597.

| Metric | PD planned T400 | CW 480 s, 1 Hz | CW 480 s, epochs | **PPO m3/1500** |
|---|---|---|---|---|
| Success (95% CI) | 100% [98.1, 100] | 100% | 100% | **100% [98.1, 100]** |
| Collisions | 0 | 0 | 0 | 0 |
| Settle median / max | 428 / 440 s | 491 / 497 s | 522 / 527 s | 502 / 541 s |
| Δv mean / p95 | 0.348 / 0.491 | **0.231 / 0.326** | 0.292 / 0.429 | **0.320 / 0.439** |
| Final error, max | 0.00004 m | 0.0006 m | 0.050 m | 0.009 m |
| Min distance to target | 3.00 m | 2.99 m | 2.21 m | 2.70 m |
| Max closing speed inside 10 m | 0.117 m/s | 0.103 | 0.103 | 0.106 |

**Reading it:**
- **Against the PD baseline:** the learned policy now beats it on fuel (−8%) at the same reliability.
- **Against CW targeting in the same 20 s action space:** it is 10% worse on average (paired +0.028 m/s), but better on 16% of cases.
- **Against CW targeting at 1 Hz:** 38% worse. This controller has finer correction authority, not a better plan.
- **Precision:** the policy holds more precisely than epoch-CW (9 mm vs 50 mm final error).

**Gate (revised):** RL does not beat the classical benchmark in this idealised setting. That is expected: with perfect state, exact CW-like dynamics and no noise, CW targeting is close to the fuel optimum, and PPO is rediscovering it from reward. The test set has not been used. Next is the uncertainty experiment (thruster gain and misalignment errors, navigation noise, J2 and drag), where open-loop CW plans degrade and a closed-loop learned policy can compete.

**Lessons:**
1. A fuel cost on per-step Gaussian exploration collapses exploration ("never thrust").
2. Strong potential shaping fixes learning but, with GAE λ 0.95, biases toward fast arrival. Weaken the shaping once the task is learned.
3. A short decision horizon (30 decisions) is what made fuel optimisation tractable.
4. Benchmark against the strongest classical method, not the first one written.

## 13. Uncertainty u1 (`--unc u1`)

Sampled per episode, unknown to every controller; success and reward use the true state. Draws are matched across controllers (seeded per split).

| Error | Model |
|---|---|
| Thrust gain | per world axis, uniform 0.9–1.1 (still capped at 0.05 N) |
| Thrust misalignment | random axis, angle uniform 0–3° |
| Unmodelled disturbance | constant LVLH acceleration, uniform ±2e-5 m/s² per axis (about 10× the chaser–target differential drag at 400 km) |
| Navigation noise | white, σ = 5 cm on position and 1 mm/s on velocity per measurement |

The "KF" variants add `cw_baseline.CWKalman`: a 6-state linear Kalman filter with CW dynamics and the controller's own commanded input. Process noise is σ_a² = (3e-5)² + (0.12·|commanded accel|)².

Validation (seed 100, 200 cases), u1:

| Controller | Success | Collisions | Settle median | Δv mean / p95 |
|---|---|---|---|---|
| PD planned, raw measurements | 100% | 0 | 431 s | 1.55 / 1.64 |
| PD planned + KF | 100% | 0 | 429 s | 0.375 / 0.513 |
| CW 1 Hz, raw, deadband 2 / 5 / 10 mm/s | 100% | 0 | 492 s | 2.44 / 1.45 / 0.90 |
| **CW 1 Hz + KF, deadband 2 mm/s** | **100%** | **0** | 490 s | **0.272** / 0.367 |
| CW epoch, raw, deadband 2 / 5 / 10 mm/s | 83% / 78% / 35% | 0 / 1 / 3 | 523 s | 0.41 / 0.38 / 0.39 |
| CW epoch + KF, deadband 2 / 5 mm/s | 77% / 79% | 1 / 0 | 514–521 s | 0.28 / 0.29 |
| PPO m3/1500 (trained ideal only, zero-shot) | 98% | 0 | 502 s | 0.361 / 0.486 |
| **PPO u1/model_2099 (fine-tuned under u1, 600 iterations)** | **100%** | **0** | 484 s | **0.344** / 0.455 |

**Reading it:**
- **1 Hz controllers on raw measurements chase noise:** 1.5–2.4 m/s, 5–10× their ideal fuel. A navigation filter is mandatory, and with it CW 1 Hz is again the best controller (0.272).
- ~~In the 20 s action space the learned policy is the only 100%-reliable controller; one correction per 20 s is too coarse for epoch CW.~~ **Corrected in section 15:** the 77–79% for "CW epoch + KF" came from a modelling error in that filter, which applied each epoch's Δv instantly at the start of the epoch, whereas the env flies it as a burn of up to 20 s. With the burn modelled second by second, epoch CW reaches 97–99.5%. PPO's remaining edge in this action space is small (section 15).
- **PPO still uses 26% more fuel than the best filtered 1 Hz classical controller.** It also receives raw measurements with no memory, so it cannot average noise or identify this episode's disturbance or thrust errors.

## 14. Plan for 2B: GNC-structured learning

Flight rendezvous systems separate **navigation** (a filter that also estimates biases and unmodelled accelerations), **guidance** (CW, multi-impulse or glideslope targeting through hold points and corridors, with passively safe trajectories), **control** (thruster selection; burns cut off on accelerometer-measured Δv) and an independent **safety monitor** (keep-out, closing speed, abort/collision-avoidance manoeuvre). Learning research keeps navigation and safety classical and puts the policy in guidance:
- velocity or Δv commands tracked by a classical controller ("deep guidance", Hovell & Ulrich);
- recurrent/meta-RL when parameters are hidden (Federici et al.);
- run-time assurance (a simplex "shield") to make learned commands certifiable (Dunlap, Hobbs et al., AFRL).

2B applies that structure. Each step is evaluated under both `none` and `u1`, on validation only, with matched draws.

| Step | What | Done when |
|---|---|---|
| B1 Navigation | 9-state Kalman filter (p, v, constant disturbance bias) with CW dynamics, shared by every controller. Check consistency: normalised estimation error ≈ 1 and bias estimate converging. | bias error < 30% of the true bias after 200 s on validation |
| B2 Burn execution (option) | accelerometer cutoff: burn until measured Δv reaches the request (accelerometer scale error 1%, not 10%). Kept as a switch so u1 results stay comparable. | flag `burn_cutoff` and a re-run of the section 13 table |
| B3 Classical benchmark | CW targeting with the bias-compensated filter (prediction includes the estimated disturbance), at 1 Hz and at 20 s epochs. Optional: Hablani glideslope. | the strongest classical number in each action space |
| B4a Policy on the filter | PPO, 20 s epochs. Observation = filter estimate (p, v, bias), diagonal of the covariance, time remaining, previous action. | compared with B3 epoch |
| B4b Residual policy | Δv = Δv_CW(filter) + 0.02 m/s × a, with a ∈ [−1, 1]³, last layer initialised near zero, so training starts at CW performance and learns corrections (arrival timing, hold-phase efficiency, error compensation). | compared with B3 epoch and 1 Hz |
| B4c Recurrent (stretch) | rsl-rl recurrent actor on raw measurements, for the meta-RL comparison with B4a. | optional |
| B5 Safety shield | explicit-simplex run-time assurance applied to *every* controller: before executing a Δv, propagate the CW free drift for 300 s from the estimated post-burn state with a 3σ position margin. If it enters 2 m + margin, substitute the backup command (CW targeting to the hold point). Count interventions. | 0 collisions, intervention rate reported |
| B6 Evaluation | Monte Carlo of 1000 validation cases under `none` and `u1`: success with Wilson CI, collisions, Δv mean/p95/p99, settle, shield interventions, dt 0.1 re-run. **Test set (seed 1000) once**, for the chosen configuration only. | final table in this spec |

**Gate for 2B:** with the shared filter and the shield, a learned policy in the 20 s action space must match the best classical controller's reliability (100% validation, 0 collisions) and use less Δv than CW targeting in the same action space, under u1. The comparison against 1 Hz CW plus the filter is reported separately, as a different actuator budget.

**Expected outcome (prediction):** B4b residual beats B3 epoch on fuel under u1, because it learns arrival timing and hold-phase economy on top of CW. B4a matches B3 epoch. Neither beats 1 Hz CW plus filter in the ideal case.

**Budget:** about 25 min per training run (2048 worlds, 1.8 s per iteration) and about 5 min per 1000-case evaluation.

Sources:
- [Hovell & Ulrich, JSR 2021](https://www.carleton.ca/spacecraft/wp-content/uploads/JSR-2021.pdf)
- [Federici et al., meta-RL rendezvous](https://core.ac.uk/works/166182712)
- [Dunlap et al., comparing run-time assurance for docking](https://arxiv.org/pdf/2110.00447)
- [Hobbs et al., RTA tutorial](https://coogan.ece.gatech.edu/papers/pdf/hobbs2022csm.pdf)
- [AFRL safe-autonomy RTA framework](https://arxiv.org/pdf/2209.01120.pdf)
- [Augmented-state relative navigation (MILTON)](https://sites.utexas.edu/near/files/2025/11/MILTON_AIAA_JGCD_R2.pdf)
- [Accelerometer bias in rendezvous filters (NASA)](https://ntrs.nasa.gov/api/citations/20120002557/downloads/20120002557.pdf)
- [Passively safe NRHO rendezvous](https://arxiv.org/pdf/2505.17251)
- [Passive-safety guidance (MERL)](https://shadow.merl.com/publications/docs/TR2020-098.pdf)
- [Glideslope guidance (LAAS)](https://hal.laas.fr/hal-01887840)
- [Learned CBF residual for rendezvous](https://www.arxiv.org/pdf/2602.00366)

## 15. 2B step B1 and B3: bias-estimating navigation and the classical benchmark (7 Nov)

**Filter (`nav_filter.py`).** `BiasKalman`: state (p, v, b) in LVLH, where b is a constant unknown acceleration. Exact CW discretisation by matrix exponential; process noise by Van Loan. White acceleration noise on v with PSD (3e-6)² + (0.12·|u|)², and a random walk on b with PSD (1e-8)². Joseph-form update. `NavController` wraps any classical controller. Modes:
- 1 Hz env: one update per step.
- Maneuver env with 1 Hz navigation: `ManeuverEnv.nav_log` records a measurement and the commanded acceleration every second, from its own noise stream (`unc_seed + 2`). The policy still sees only the end-of-epoch measurement.
- Maneuver env with one measurement per epoch: the filter predicts the flown burn profile second by second.

Guidance with `est_bias` uses `v_required_bias` (CW plus constant acceleration); PD subtracts b as feed-forward.

**Checks (`check_nav.py`, validation seed 100, 200 cases, u1, CW epoch guidance).** Discretisation: Φ9 equals the CW STM to 3e-14; the forced response matches RK4 to 2e-17.

| Filter | NIS (exp. 6) | NEES p,v (exp. 6) | NEES b (exp. 3) | Bias error / true bias at 200 s: median, p90, share < 30% | at 400 s median |
|---|---|---|---|---|---|
| 9-state, 1 Hz nav | 6.03 | 8.4 | 2.5 | 0.14, 0.33, 88% | 0.04 |
| 9-state, 1 measurement per epoch | 6.07 | 6.0 | 2.2 | 0.38, 0.85, 36% | 0.13 |
| 6-state, 1 Hz nav (σ_w 3e-6) | 7.2 | 202 (inconsistent: the bias is unmodelled) | – | – | – |

**B1 gate met with 1 Hz navigation:** median bias error 14% at 200 s. The 88% share below 30% is not 100%, so the gate holds in the median, not for every case. The p,v NEES spikes to about 35 right after the 480 s arrival burn, because the thrust-gain error is constant per episode while the filter models it as white noise. Accelerometer cutoff (B2) or gain states would remove this.

**Ablation of the old 77% (same cases).** 6-state filter with one measurement per epoch and the burn modelled as flown: 97% (σ_w 3e-6) and 99% (σ_w 3e-5, the old tuning). The cause was the instant-Δv prediction, not the measurement rate or the bias.

**B3 table** (`evaluate.py --env maneuver --kf --nav`, `results_val_nav9_{u1,none}.json`, validation, 200 cases, matched draws):

| Controller | u1 success / collisions | u1 Δv mean / p95 | u1 min dist | ideal success | ideal Δv |
|---|---|---|---|---|---|
| CW 1 Hz + 6-state KF, db 2 | 100% / 0 | **0.272** / 0.367 | 2.97 | 100% | 0.234 |
| CW 1 Hz + nav9 (bias-compensated), db 2 / 5 | 100% / 0 | 0.349 / 0.290 | 2.97 | 100% | 0.236 |
| PD planned + nav9 | 100% / 0 | 0.368 | 2.92 | 100% | 0.348 |
| CW epoch + old KF, db 2 | 77% / 1 | 0.281 | 1.92 | 90% | 0.322 |
| **CW epoch + nav9, db 2** (benchmark) | **98% / 0** | **0.334** / 0.476 | 2.11 | 100% | 0.292 |
| CW epoch + nav9, db 5 | 99.5% / 1 | 0.331 | 1.99 | 99.5% / 1 | 0.309 |
| CW epoch + nav9, 1 measurement per epoch, db 2 | 97.5% / 0 | 0.352 | 2.02 | 100% | 0.332 |
| PPO u1/model_2099 (raw end-of-epoch measurement) | **100% / 0** | 0.344 / 0.452 | 2.75 | 100% | 0.315 |

Paired against CW epoch nav9 db 2:
- **u1:** both succeed in 196 cases, PPO alone in 4, CW alone in 0. On the shared cases PPO uses +0.009 m/s Δv and is lower in 35% of them.
- **Ideal:** PPO uses +0.023 and is lower in 22.5%.

**Reading it:**
- **The classical benchmark was much weaker than it should have been.** With correct burn modelling and 1 Hz navigation, epoch CW is nearly as reliable as PPO and 3% cheaper on Δv. PPO's advantage in the 20 s action space shrinks to 4 of 200 cases.
- **Epoch CW passes close to the keep-out zone:** minimum distance about 2.0–2.1 m against the 2 m limit, with one collision at db 5. PPO keeps 2.75 m. The safety shield (B5) matters for the classical controllers too.
- **Bias compensation does not help the 1 Hz controller.** It raises Δv from 0.272 to 0.349 at db 2. Probable reason, not yet verified: every change in the bias estimate moves v_req past the 2 mm/s deadband and triggers a burn. At db 5 it costs 0.290.
- **Bias compensation barely changes epoch CW** (98% vs 99% without it, `check_nav.py`). At ±2e-5 m/s² the disturbance moves the chaser by 4 mm per 20 s, so filter quality matters more than the bias model.

**Gate for 2B, restated.** In the 20 s action space under u1:
- reliability at least 98–100% with 0 collisions;
- Δv below **0.334** (CW epoch + nav9, db 2).

PPO u1/2099 meets the reliability bar but misses on Δv by 0.01. Next is B4b: a residual policy on top of CW epoch + nav9, with the filter estimate in the observation.

## 16. 2B step B4b: residual policy on CW epoch + nav9 (7 Nov)

**Env (`residual_env.py`, `--env residual`).**
- The 9-state filter runs inside the env at 1 Hz.
- Each epoch it computes the bias-compensated CW command (t_arrive 480 s, deadband 2 mm/s). The policy adds Δv_LVLH = Δv_CW + 0.02·a, with a ∈ [−1, 1]³; the sum is clipped per world axis to 0.1 and flown as in ManeuverEnv.
- **Observation (25), all from the filter, never the true state:**
  - p − hold, v, b, and a zoomed copy of the position error;
  - the square roots of the covariance diagonal;
  - time remaining, the previous residual action, and Δv_CW/0.02.
- Reward, success and failure are ManeuverEnv's and use the true state.
- **Check:** with a = 0 it reproduces the benchmark: 98% and Δv 0.332, against 98–98.5% and 0.334 for CW epoch + nav9. The GPU physics varies slightly from run to run.

**Training (`run_r1.sh`, run `r1`).**
- From scratch, with the actor's last layer scaled by 0.01 (`--zero-init`), so training starts at a ≈ 0.
- Settings: u1, stage a3, 2048 worlds, 16 steps per env, γ 0.99, λ 0.99, w_shape 0.1, w_fuel 4, initial std 0.15, std floor 0.02, 600 iterations.
- It took **74 min, about 7 s per iteration**, 4× the maneuver env, because the numpy filter runs every second for all worlds. Moving it to torch on the GPU is the obvious speed-up.
- The std reached its floor by about iteration 250.

**Validation, u1, 200 matched cases** (`results_val_r1_{250,400,599}_u1.json`):

| Controller | Success / collisions | Δv mean / p95 | Paired Δv vs benchmark (share of cases lower) | Overlap: both / benchmark only / policy only / neither | Min dist |
|---|---|---|---|---|---|
| CW epoch + nav9, db 2 (benchmark) | 98.5% / 0 | 0.334 / 0.476 | – | – | 2.06 |
| r1 / 250 | 99.5% / 0 | 0.325 / 0.459 | −0.011 (86%) | 197 / 0 / 2 / 1 | 2.09 |
| r1 / 400 | 99.5% / 0 | 0.322 / 0.449 | −0.014 (87%) | 197 / 0 / 2 / 1 | 2.12 |
| **r1 / 599** | **99.0% / 0** | **0.314 / 0.440** | **−0.021 (94%)** | 196 / 1 / 2 / 1 | 2.08 |
| PPO u1/2099 (direct Δv, raw measurements; section 15) | 100% / 0 | 0.344 / 0.452 | +0.009 (35%) | 196 / 0 / 4 / 0 | 2.75 |

**Reading it:**
- **The prediction held.** The residual policy beats its own classical base on fuel: 6% lower Δv on average, and lower in 94% of paired cases. It is at least as reliable (99% vs 98.5%).
- **Against the 2B gate restated in section 15** (success ≥ 98–100%, 0 collisions, Δv < 0.334): r1/599 passes on validation. The original wording asked for 100%, and r1 has 2 failures in 200. The Wilson intervals of 99% and 100% overlap at n = 200, so the 1000-case Monte Carlo (B6) decides.
- **Different controllers fail different cases.** The direct policy u1/2099 succeeds on every case, including the one that every CW-based controller misses. Its Δv is higher.
- **Fuel ranking:**

  | Controller | Δv |
  |---|---|
  | CW 1 Hz + KF (separate actuator budget) | 0.272 |
  | r1 residual | 0.314 |
  | CW epoch + nav9 | 0.334 |
  | direct PPO | 0.344 |

  The 1 Hz controller is still cheapest: re-targeting every second is worth about 0.04 m/s.
- **The residual policy is no safer than its base:** its minimum distance is 2.08 m against the 2 m keep-out zone, so B5 (the shield) still applies.

**Selected for B5/B6, provisionally:** `logs/r1/model_599.pt` (lowest Δv; reliability within noise of the others). Revisit after the 1000-case Monte Carlo.

**Speed-up (after r1).** `nav_filter.BiasKalmanTorch` is the same filter in torch float64 on the GPU: Cholesky solves, Joseph form, and a whole epoch of 1 Hz measurements per transfer (`run_log`). `ResidualEnv` uses it by default (`torch_filter=True`). `cw_forced_mats` now computes the matrix exponential once per distinct t_go.
- Equivalence (`profile_residual.py`): after 300 steps with resets, the numpy and torch filters agree to 6e-16 relative in x and 2e-15 in P. Re-evaluating r1/599 gives identical results (99%, Δv 0.3139; a = 0: 98%, 0.3317).
- Timing at 2048 worlds:

  | Measure | Before | After |
  |---|---|---|
  | ResidualEnv step | 383 ms | 192 ms (ManeuverEnv: 80 ms) |
  | CW command | 23 ms | 1 ms |
  | Filter per epoch | 243 ms | 52 ms |
  | Training iteration | 7.4 s | **3.65 s** (2×) |

- The rest of the filter cost is kernel-launch overhead: about 25 small ops per simulated second. CUDA graphs or a fused kernel would remove it if needed.


## 17. 2B step B5: run-time assurance shield (8 Nov)

**Design (`shield.py`, `evaluate.py --shield`).** An explicit-simplex filter on every 20 s Δv request. It hooks into `ManeuverEnv.step_world_dv`, so it sits behind every epoch controller in the same way: raw CW, CW + nav9, the residual env, and the direct PPO policy. Controllers that run no filter get their own 9-state filter (`NavTracker`).
- **Prediction, from the filter's estimate (p, v, b) and covariance P,** using the 9-state CW model in 1 s steps:
  1. fly the candidate as a finite burn (same per-world-axis profile as the env) and coast to the next epoch;
  2. hand over to the backup controller, **stop and station-keep**, which nulls the estimated velocity at that epoch and at every later epoch for 300 s.
  - After each backup stop, the velocity covariance returns to the filter's current one.
  - Every burn adds a 1σ execution error of 7% of |Δv| per axis to the velocity covariance.
- **Safe** when, at every second of the prediction, |p̂| − 3σ_r ≥ 2 m, where σ_r is the position σ along the line of sight.
- **Candidates, in order:**
  1. nominal;
  2. bias-compensated CW to the hold point;
  3. stop now;
  4. stop and retreat at 2 cm/s.

  The first safe candidate is flown. If none is safe, the one with the largest predicted margin is flown. Anything other than the nominal counts as an intervention.

**Change from the section 14 wording (free drift).** The original test was "free drift for 300 s after the burn". It is infeasible here. Under u1 the disturbance (up to 2e-5 m/s²) moves a coasting vehicle up to about 0.9 m in 300 s, while the hold point is only 1 m outside the keep-out sphere. Any approach that arrives at the hold point while closing also fails the test.
- **20-case smoke test with free drift:** every controller fell to 0% success, with 0 collisions. The shield intervened about 10 times per episode and parked the vehicle about 4.6 m out.
- That version is still available as `--shield-drift`.

**Validation, 200 matched cases** (`results_b5_val_{u1,none}.json`, `results_b5_val_ppo_{u1,none}.json`; `run_b5.sh <unc> 200 val`):

| u1 | Success / collisions | Δv mean / p95 | Min dist | Episodes with an intervention (mean count) |
|---|---|---|---|---|
| CW epoch + nav9, db 2 | 98% / 0 | 0.334 / 0.476 | 2.11 | – |
| **CW epoch + nav9 + S** (benchmark) | **100% / 0** | 0.349 / 0.538 | 2.40 | 29% (0.29) |
| Residual r1/599 | 99.5% / 0 | 0.314 / 0.439 | 2.15 | – |
| **Residual r1/599 + S** | **100% / 0** | **0.325** / 0.503 | 2.39 | 24% (0.25) |
| Direct PPO u1/2099 + S | 100% / 0 | 0.344 / 0.455 | 2.83 | **0%** |
| CW epoch raw + S | 84% / 0 | 0.421 / 0.592 | 2.38 | 32% (0.33) |

| none | Success / collisions | Δv mean | Min dist | Intervened |
|---|---|---|---|---|
| CW epoch + nav9 / + S | 100% / 0 | 0.292 / 0.308 | 2.14 / 2.43 | – / 34% |
| Residual r1/599 / + S | 100% / 0 | 0.298 / 0.311 | 2.22 / 2.43 | – / 19% |
| Direct PPO u1/2099 + S | 100% / 0 | 0.315 | 2.91 | 0% |

- **Paired, u1, both shielded:** r1 − benchmark = −0.024 m/s, lower in 94% of cases; all 200 cases succeed for both.
- **Paired, none:** r1 is +0.003 against the benchmark. Its fuel advantage is specific to the uncertain case it was trained on.

**When the shield acts** (`diag_shield.py`, CW epoch + nav9 under u1): every flagged epoch falls at 460–480 s, the last epochs before CW's planned arrival.
- **Intervened with a safe option (29 episodes):** at 460 s, 4.7 m from the target, closing at 8.6 cm/s. The nominal margin was about −0.08 m. The shield chose stop (16) or retreat (12).
- **No certified option (40 epochs, one per episode):** at 480 s, on the hold point (3.0 m), still closing at 7.5 cm/s. At full thrust (5 mm/s²) the braking distance is v²/2a ≈ 0.56 m. That leaves about 0.44 m, less than the 3σ margin, so no candidate can be certified. The shield flies the largest-margin option, usually retreat.

**Reading it:**
- **B5 done-when is met:** 0 collisions for every controller, with intervention rates reported. The true minimum distance rises from 2.03–2.20 m to 2.38–2.40 m. The shield is conservative: its predicted worst margin is −0.43 m, yet the real distance never drops below 2.38 m.
- **The shield also raises reliability.** CW + nav9 goes from 98% to 100%, and r1 from 99.5% to 100%. The failing cases were the ones that overshot at arrival. The cost is about 0.012–0.015 m/s of Δv.
- **The root cause is guidance, not the policy.** A single CW arc with a fixed arrival time from 20–50 m arrives at the hold point at about 7–9 cm/s and relies on one hard braking burn 1 m from the keep-out zone. That is not recoverable within the margin, which explains every near-miss since section 13 (min dist 2.03–2.1 m). Flight approaches avoid this with a closing-speed limit near the target, such as glideslope guidance (Hablani) or a speed corridor.
- **The direct PPO policy never triggers the shield** (minimum distance 2.83 m). It learned a slower final approach on its own, at the price of higher Δv.
- **Gate for 2B (section 14), validation, 200 cases:**
  - **r1 + S passes:** 100%, 0 collisions, and Δv 0.325 against 0.349 for epoch CW + S in the same action space.
  - **Direct PPO + S also passes on reliability,** with Δv 0.344 < 0.349. It needs no interventions.
  - The 1000-case Monte Carlo (B6) and the single test-set run remain.
- **Not done:**
  - the shield is only for the 20 s action space; the 1 Hz controllers would need a per-second version;
  - recursive feasibility is not guaranteed, as the "no certified option" epochs show;
  - the 1 Hz-only "CW epoch nav9 1meas" variant is skipped under the shield, because the shield's filter assumes 1 Hz navigation.

## 18. 2B step B6: Monte Carlo and the test set (8 Nov)

**Selection rule, fixed before the 1000-case results were seen.** The candidates are r1/599 + S (residual) and u1/2099 + S (direct PPO). Both run behind the B5 shield.
1. 0 collisions under both `none` and `u1`. A collision disqualifies.
2. Higher u1 success. If the Wilson 95% intervals overlap, treat it as a tie.
3. On a tie, lower mean Δv under u1.

The chosen policy, the shielded CW epoch + nav9 benchmark and shielded raw CW then run **once** on the test set (seed 1000, 1000 cases, u1 and none). Nothing is changed after that run.

**Validation Monte Carlo, 1000 matched cases** (`run_b6.sh`; `results_b5_mc_{u1,none}.json`, `results_b5_mc_ppo_{u1,none}.json`):

| u1 | Success (Wilson 95%) | Collisions | Δv mean / p95 / p99 | Min dist | Intervened episodes |
|---|---|---|---|---|---|
| CW epoch raw | 79.8% | **1** | 0.403 / 0.541 / 0.573 | **1.84** | – |
| CW epoch raw + S | 79.9% | 0 | 0.410 / 0.584 / 0.657 | 2.31 | 26% |
| CW epoch + nav9 | 98.1% | **1** | 0.323 / 0.469 / 0.513 | **1.94** | – |
| CW epoch + nav9 + S (benchmark) | 99.7% (99.1–99.9) | 0 | 0.333 / 0.515 / 0.581 | 2.37 | 25% |
| Residual a = 0 + S | 99.8% | 0 | 0.332 / 0.513 / 0.569 | 2.36 | 25% |
| r1/599 | 99.5% | 0 | 0.305 / 0.436 / 0.468 | 2.04 | – |
| **r1/599 + S** | **99.6% (99.0–99.8)** | 0 | **0.313** / 0.476 / 0.555 | 2.36 | 24% |
| u1/2099 + S (direct PPO) | **100% (99.6–100)** | 0 | 0.340 / 0.463 / 0.499 | 2.72 | 0% |

- **none:** every controller, shielded or not, gets 100% with 0 collisions. Shielded Δv: benchmark 0.292, r1 0.298, direct PPO 0.307.
- **Paired, u1, both shielded:** r1 − benchmark = −0.020, lower in 91% of cases. Under none it is +0.006.
- **At 1000 cases the unshielded CW controllers collide** (1 each, minimum distance 1.84 / 1.94 m). 200 cases had not shown this. No shielded controller collides.

**Physics step 0.1 s re-run, 200 cases, u1** (`results_b5_dt01_{u1,ppo_u1}.json`):

| Controller | Success |
|---|---|
| CW + nav9 + S | 95.5% |
| a = 0 + S | 97.5% |
| r1 + S | 91.0% |
| direct PPO + S | **100%** |

All have 0 collisions.
- **Cause, checked:** CW's fixed 480 s arrival. It settles at about 522 s, and success needs settling by 540 s. That slack is small enough for the finer step to push marginal cases past the deadline.
- With `--cw-arrive 440` at dt 0.1, CW + nav9 + S reaches 100% (unshielded: 99.5%, with 1 collision).
- Direct PPO arrives earlier (median settle 484 s) and is unaffected.
- So the 1 s results flatter every CW-based controller's reliability margin. r1 inherits the late arrival from its base and does not learn to fix it within a ±2 cm/s residual.
- Fixed at the same time: at dt < 1 the settle time was reported in physics steps. It is now in seconds.

**Selection by the rule:** the Wilson intervals of r1 + S (99.0–99.8%) and direct PPO + S (99.6–100%) overlap, so it is a tie. Lower Δv decides: **r1/599 + S** (0.313 vs 0.340). Direct PPO had 0 failures against r1's 4 and was the more robust policy at dt 0.1; the rule as fixed did not weigh either.

**Test set, seed 1000, 1000 cases, run once** (`run_test.sh`; `results_test_r1_599_{u1,none}.json`):

| Test, u1 | Success (Wilson 95%) | Collisions | Δv mean / p95 / p99 | Min dist | Intervened |
|---|---|---|---|---|---|
| CW epoch raw / + S | 80.8% / 81.6% | **2** / 0 | 0.395 / 0.403 | 1.95 / 2.27 | – / 24% |
| CW epoch + nav9 / + S | 98.6% / 99.5% (98.8–99.8) | 0 / 0 | 0.315 / 0.322 | 2.01 / 2.32 | – / 23% |
| Residual a = 0 / + S | 98.8% / 100% (99.6–100) | 0 / 0 | 0.315 / 0.323 | 2.02 / 2.35 | – / 23% |
| r1/599 (no shield) | 99.2% | 0 | 0.296 / 0.438 / 0.476 | 2.04 | – |
| **r1/599 + S (chosen)** | **99.2% (98.4–99.6)** | **0** | **0.303** / 0.471 / 0.549 | 2.30 | 21% |

- **none, test:** every controller gets 100% with 0 collisions. r1 + S uses 0.289 against 0.281 for the benchmark (+0.008, paired).
- **Paired, u1:** r1 + S − benchmark + S = **−0.0185 m/s, lower in 89%.** Overlap on success: 991 both / 4 benchmark only / 1 r1 only / 4 neither.

**Gate for 2B (section 14) on the test set, as stated before the run:**
- **Collisions:** met. 0 for the chosen configuration; the shield removed the unshielded raw-CW collisions.
- **Δv below CW targeting in the same action space, under u1:** met. 0.303 vs 0.322, −5.7%.
- **Match the best classical controller's reliability:** **not met.**
  - r1 + S gets 99.2%. The same CW base behind the same shield gets 100% (a = 0 + S); the NavController version of the benchmark gets 99.5%.
  - The intervals touch at 99.6%.
  - The residual buys about 6% fuel at the cost of about 0.5–0.8 points of success, all of them timing failures, none of them collisions.

**Conclusions for Project 2:**
1. Classical structure carries most of the result: the bias-estimating filter and CW guidance, and the shield for safety (0 collisions in every shielded run, against 1–2 collisions per 1000 without it).
2. A residual policy trims fuel by about 6% under uncertainty and gives nothing in the ideal case. It slightly reduces reliability.
3. The direct PPO policy was the most robust (100% validation at dt 1 and dt 0.1, never needing the shield) and less fuel-efficient (+9% vs r1).
4. The remaining weakness is guidance: a late, fast CW arrival. The next improvement is in the base, not the policy: an earlier arrival or a glideslope with a closing-speed limit. A residual on such a base, or a residual that may also shift arrival time, is the obvious follow-up.

## 19. Follow-up after the test set: glideslope base guidance (8 Nov)

The test set is spent, so everything here uses validation (seed 100) for tuning and a **fresh split, `val2` (seed 2000)**, for confirmation.

**Design (`guidance.py`).** Closed-loop glideslope, after Hablani et al. 2002, run each 20 s epoch.
- The vehicle moves along the straight line from the estimated position to the hold point, at closing speed ρ̇ = −kρ, clipped to [v_min, v_max].
- Each epoch it targets the next waypoint on that line, using the bias-compensated CW coast velocity with t_go = 20 s.
- **Hybrid:** outside r_switch metres from the hold point it flies bias-compensated CW targeting to the hold point with arrival time T. That is the cheap coasting arc; the glideslope takes over only near the target.
- Controller: `GlideslopeEpoch`. Evaluation: `evaluate.py --glideslope K,VMAX,VMIN[,RSWITCH,TARRIVE]`. Residual base: `ResidualEnvCfg(base="gs")`, `--base gs`.

**Tuning on validation, u1, 200 cases, shielded** (`results_gs_tune{1,2,3}_u1.json`):

| Variant (k, v_max, r_switch, T) | Success | Δv | Settle median / max | Min dist | Interventions |
|---|---|---|---|---|---|
| CW epoch + nav9 + S (reference) | 99.5% | 0.345 | 522 / 541 | 2.40 | 27% |
| Pure glideslope 0.03, 0.12 | 100% | 0.473 | 353 / 459 | 2.95 | 0 |
| Pure glideslope 0.02, 0.2 | 100% | 0.709 | 316 / 384 | 2.94 | 0 |
| Hybrid 0.03, 0.1, 2 m, 440 | 100% | 0.293 | 476 / 512 | 2.47 | 18% |
| **Hybrid 0.03, 0.1, 3 m, 440 (chosen)** | **100%** | **0.292** | 477 / **492** | **2.95** | **0** |
| Hybrid 0.03, 0.1, 3 m, 460 | 100% | 0.282 | 497 / 508 | 2.95 | 0 |
| Hybrid 0.03, 0.1, 5 m, 420 | 100% | 0.347 | 461 / 480 | 2.95 | 0 |
| Hybrid 0.04, 0.1, 3 m, 440 | 100% | 0.305 | 462 / 471 | 2.95 | 0 |

- **Pure glideslope is safe and early but expensive.** Cruising a straight line at constant speed costs 0.47–0.71 m/s.
- **The hybrid keeps the CW arc and replaces only the last 3 m.** That turns out cheaper than CW alone, because CW's hard braking at arrival and its station-keeping corrections cost more than a gentle exponential approach.
- Switching at 2 m comes too late: the shield fires again.
- T 460 is 0.01 cheaper, but I kept T 440 for its 48 s margin to the settle deadline. That margin was the fragility found in section 18.

**Confirmation on val2, 1000 cases, everything shielded** (`run_gs_val2.sh`; `results_gs_val2*.json`):

| Controller | u1 success (Wilson) | u1 Δv mean / p95 / p99 | none Δv | dt 0.1, u1, 200 cases | Min dist | Interventions (u1) |
|---|---|---|---|---|---|---|
| CW epoch + nav9 | 99.8% | 0.322 / 0.502 / 0.577 | 0.281 | 95.0% | 2.36 | 24% |
| Residual r1/599 (CW base) | 99.8% | 0.304 / 0.470 / 0.551 | 0.290 | 97.5% | 2.35 | 21% |
| Direct PPO u1/2099 | 100% | 0.332 / 0.448 / 0.485 | 0.299 | 100% | 2.74 | 0% |
| **Glideslope hybrid** | **100% (99.6–100)** | **0.280 / 0.388 / 0.419** | **0.249** | **100%** | **2.95** | **0%** |

All have 0 collisions, under both u1 and none.

**Reading it:**
- **A better classical base beats both learned policies on every metric:**
  - Δv 8% below the residual policy and 13% below CW, with the tail lower still (p95 0.388 vs 0.47–0.50);
  - 100% success at dt 1 and dt 0.1;
  - never closer than 2.95 m (it doesn't pass inside the hold point);
  - never triggers the shield.
- Under no uncertainty it is also the cheapest: 0.249, against 0.281 for CW.
- **The residual policy's gain over CW (section 16) was mostly compensation for poor terminal guidance** that a classical fix removes more cheaply. Lesson: make the base right before learning on top of it.
- **Next: residual r2 on the glideslope base** (`run_r2.sh`, same settings as r1). Does learning still add anything on a strong base?

**Residual r2 on the glideslope base** (`run_r2.sh`: r1's settings with `--base gs`, 600 iterations, about 40 min).
- **Checkpoint choice on validation (u1, 200 cases, shielded):** the base gets Δv 0.291. r2/250 gets 0.291, r2/400 0.282, **r2/599 0.275**. All have 100% success and 0 interventions. I chose 599.

**r2/599 on val2, all shielded** (`run_r2_val2.sh`; `results_r2_val2*.json`):

| Condition | Glideslope base | r2/599 | Paired Δv (share of cases lower) |
|---|---|---|---|
| u1, 1000 cases | 100%, 0.280 / p95 0.389 | 100%, **0.260** / p95 0.375 | **−0.019 (74%)** |
| none, 1000 cases | 100%, 0.248 | **60.4%**, 0.322, 28% intervened | +0.084 |
| u1, dt 0.1, 200 cases | 100%, 0.279 | 98.0%, 0.310 | +0.031 |

There were 0 collisions throughout.

**Diagnosis, validation, 200 cases** (`run_r2_diag.sh`; new presets `u1nav`, nav noise only, and `u1dyn`, thrust and disturbance errors only):

| Preset | r2/599 | Paired Δv vs base | Intervened |
|---|---|---|---|
| u1nav | 100% | −0.014 | 0% |
| u1dyn | **56%** | +0.064 | 36% |

- **r2 fails whenever navigation noise is absent.** With noise-free measurements the filter's covariance falls to its floor (R = (1e-4)² instead of (0.05)²). The policy's covariance inputs then lie far outside anything it saw in training, and its corrections go wrong.
- **This is observation distribution shift, not dynamics.** The base guidance is unaffected, and so was r1, whose CW base is more forgiving.
- **The fix is in training:**
  - randomise the nav noise level per episode, including near zero (domain randomisation);
  - or remove or normalise the covariance inputs;
  - or run the filter with the nominal sensor noise regardless of the actual one.
- **Status:** r2 is not deployable as is. The **glideslope hybrid alone is the best controller in the 20 s action space**: 100% under u1, none and dt 0.1, Δv 0.280 / 0.249, 0 shield interventions.
