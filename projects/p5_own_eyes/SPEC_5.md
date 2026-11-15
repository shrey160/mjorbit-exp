# Project 5: the servicer body's own eyes (draft for review, 14 Nov 2026)

Status: the user approved "filter first" on 14 Nov and then chose a cooperative client (§0, which overrides the text below where they differ). Built: the tolerance measurement (§3) and the filter (P5.1). Research: [RESEARCH_SERVICING.md](../../learning/RESEARCH_SERVICING.md), "P5 research (14 Nov 2026)" and "Client types (14 Nov 2026)".

## 0. Revision (14 Nov, later): cooperative client first

The user chose the simple client for now: **a cooperative, prepared client** (type A in the "Client types" research). It has the grapple bar and it reports its own state to the servicer. Harder client types are extensions for later.

What this changes in the sections below:
- **Source of the client state:** provided by the client (its attitude and spin rate from its own star tracker and gyro; relative position and velocity from navigation receivers on both satellites), not seen by the bus camera. It is modelled with errors, a low update rate, latency and dropouts, never as the simulator's exact truth.
- **Dropped for now:** the client-pose network (§5 piece 1, step P5.3) and the symmetry and lighting risks (§7).
- **Kept:** the tolerance measurement (§3), now the requirement on the provided state (§4); the filter (§5 piece 2, P5.1), which smooths the link data, carries it to the present and rides through dropouts; the monitor redesign (§5 piece 3, P5.2), plus a **stale-link monitor** that aborts when data stops arriving.
- **Unchanged:** the arm grasps from its own cameras (Project 4).
- **Claim:** "fly-in and grasp of a cooperative client on a realistic provided state", not "a client that cannot help".

Revised steps:
1. P5.1 filter (done).
2. P5.2 monitors on the filter output, with the stale-link monitor (done; fault cases move to P5.5).
3. P5.3' provided-state model (figures checked 14 Nov, RESEARCH_SERVICING.md "Provided state"): position from relative GNSS, 20 mm per axis in a good case and 60 mm per axis (10 cm 3D rms) as the realistic onboard figure, slowly varying and not growing with range; attitude from the client's star tracker, 0.05° (0.1° poor case, including bar alignment); 1–5 Hz; 0.15–0.5 s late (rate and latency are assumptions). Velocity and rate come from the filter, not the link.
4. P5.4 closed loop with the arm on the Project 4 estimators.
5. P5.5 faults.
6. P5.6 test split, once.

**P5.1 result (14 Nov, `p5_filt_closed.json`):** bus on the filter, fly-in planned from the estimated spin axis after a 30 s wait at the hold point, 25 tune seeds per level. Input error is half slow drift (5 s), at 10 Hz, 0.15 s late, with position error growing with range. Monitors and arm on the true state. Baseline on the true state: 25/25, fly-in Δv 419 mm/s.

| Input error (position at 2.7 m, attitude) | Grasps | Safe aborts | Filter error before contact, rms (position / velocity / attitude / rate) | Fly-in Δv (mm/s) |
|---|---|---|---|---|
| 5 mm, 0.25° | 25/25 | 0 | 5.3 mm / 0.30 mm/s / 0.21° / 0.007°/s | 501 |
| 10 mm, 0.5° | 24/25 | 1 | 9.8 mm / 0.37 mm/s / 0.35° / 0.009°/s | 548 |
| 20 mm, 1° | 19/25 | 6 | 17.9 mm / 0.49 mm/s / 0.47° / 0.011°/s | 604 |

No collisions. All aborts are the predicted-clearance monitor during the fly-in. The filter meets the velocity and rate targets at every level; position and attitude are no better than the input, because slow drift cannot be filtered out. So the input must itself meet about 10 mm and 0.5°. The filter is referenced to the servicer's centre of mass, whose acceleration is exactly thrust over mass before contact. After the jaws close the estimate is not valid (reported separately as `post`).

**P5.2 result (14 Nov, `p5_mon_est.json`):** bus and monitors on the filter.

*Cause of the aborts (`dbg_clear.py`).* On the true state the frozen fly-in plan leaves the predicted-clearance monitor only 0.03 m: 174 s into the 219 s path the servicer still closes at about 20 mm/s, and a 30 s coast would end 0.03 m above the limit. The true clearance at that moment is 0.66 m. So 3 cm of tracking error trips the monitor. A second cause at slow spin: the spin axis is estimated up to 15° off, the re-planned path gets a short, fast last leg, and that leg alone violates the monitor. No traced abort was a real small clearance.

*Design.*
1. After the re-plan each leg is lengthened until the path alone keeps a predicted-clearance margin equal to the allowance plus 0.08 m for tracking error (capped at 0.26 m; the pre-grasp point has 0.30 m).
2. The monitors read the estimate. The predicted clearance loses an allowance of 3 × (σ position + 30 s × σ velocity + 1.1 m × (σ attitude + 30 s × σ rate)). The filter's σ is used, with the stated input accuracy added for position and attitude, because slow drift is not averaged out.
3. New `stale_link` abort when the newest client state is older than 3 s.
4. The monitors are armed at the end of the 30 s wait, when the filter has converged.

| Input error | Grasps (P5.1 → P5.2) | Aborts | Allowance at close range | Path margin | Smallest monitor value |
|---|---|---|---|---|---|
| 5 mm, 0.25° | 25/25 → 25/25 | 0 | 0.07 m | 0.12 m | 0.11 m |
| 10 mm, 0.5° | 24/25 → 25/25 | 0 | 0.14 m | 0.17 m | 0.12 m |
| 20 mm, 1° | 19/25 → 25/25 | 0 | 0.29 m | 0.25 m | 0.07 m |

No collisions; push at most 0.60 mm/s. Fly-ins along the spin axis take 1.52 times longer (219 s → about 330 s); propellant is unchanged. Link dropout at 10 mm, 4 seeds each: 1 s gives 4/4 grasps; 6 s gives 4/4 `stale_link` safe aborts, also when it starts near the end of the fly-in.

Open: the fault cases with the monitors on the estimate (P5.5) must show the allowance has not hidden real faults; the abort back-off still reads the true client state.

**P5.3' result (14 Nov, `p5_link.json`):** provided-state input (error per axis, 90% slowly drifting over 60 s, not growing with range), bus and monitors on the filter, arm on the true state, 25 tune seeds.

| Link case | Grasps | Filter position error (3D rms) | Bus off its true station (rms) | Allowance |
|---|---|---|---|---|
| Good: 20 mm, 0.05°, 5 Hz, 0.15 s late | 25/25 | 33 mm | 35 mm | 0.08 m |
| Realistic: 60 mm, 0.05°, 1 Hz, 0.3 s late | 23/25 | 82 mm | 94 mm | 0.29 m |
| Poor: 60 mm, 0.1°, 1 Hz, 0.5 s late | 23/25 | 82 mm | 98 mm | 0.29 m |

No collisions. Attitude, rate, message rate and latency are not the limit; position is. At 60 mm per axis one case had the bus 0.17 m off station and aborted during close with a 12.9 mm/s push on the client, and one near-static client had its fly-in slowed to the 4× cap and ran out of time. So the requirement on the link is about 20 mm per axis, or the servicer's own cameras must correct the bus position at close range. That correction is the main task of P5.4.

**P5.4 result (14 Nov, `cam_eval.py`, `p5_cam_nofix.json`, `p5_cam_fix.json`):** the whole chain on the servicer's own information. Bus and monitors on the link filter, arm on the Project 4 estimators (camera mounts ±10 mm, 1°), 25 tune seeds per row.

*Hand-over.* Two seconds after the cameras first see the bar, the filter's position input switches from the link to the camera-derived client position; attitude stays on the link. The filter state is shifted at once and the opposite offset is added to its output and removed at 3 mm/s, so the bus moves over slowly. The servicer waits until the offset is gone, then reaches. At the close command position updates stop.

| Link | Hand-over | Grasps | Largest push | Bus off its true station at close (median / max) | Wait before reach |
|---|---|---|---|---|---|
| Good: 20 mm, 5 Hz | no | 25/25 | 0.90 mm/s | 37 / 73 mm | 2 s |
| Good: 20 mm, 5 Hz | yes | 25/25 | 0.84 mm/s | 17 / 48 mm | 40 s |
| Realistic: 60 mm, 1 Hz | no | 20/25 | 7.70 mm/s | 124 / 204 mm | 2 s |
| Realistic: 60 mm, 1 Hz | yes | 23/25 | 0.84 mm/s | 18 / 50 mm | 74 s |

No collisions. The hand-over removes every grasp-phase failure. The two failures left at 60 mm happen in the fly-in, before the cameras see the bar: a near-static client whose path is slowed until the run times out, and one predicted-clearance abort (the path margin of 0.23 m minus the allowance of 0.15 m leaves 0.08 m, which a 60 mm link uses up). Changes to the monitor allowance made here: the four terms are combined as a root-sum-square, and k = 2 (the monitor already carries a 0.30 m margin). The P5.2 table above used the linear sum and k = 3.

**P5.4b result (15 Nov, `p5_cam_fix_lim.json`, `p5_cam_fix_lim2.json`): the two fly-in failures are fixed, 25/25 at both links.** A trace (`dbg_fly.py`) showed the cause was not the path timing. The filter read the slow drift of the link error as velocity (2 to 4 mm/s, times the monitor's 30 s horizon), and `cam_eval.py` had been running the filter on looser noise levels than the tuned ones. The near-static client failed for a different reason: its straight path passes a corner of the client, so its predicted clearance is set by geometry and lengthening the leg could not raise it.

Three changes:
1. *Filter.* The slow part of the link position error is carried as three extra states (first-order Gauss-Markov, 60 s); differential gravity is added to the motion model; the tuned noise levels are set. Velocity error in the fly-in at a 60 mm link: 1.0 mm/s rms (was 2.3).
2. *Fly-in timing* (`fly_lim.py`). The path geometry is unchanged. At each point the speed is the largest one, up to 0.05 m/s, for which a 30 s coast keeps the margin, with a 1.5e-3 m/s² acceleration limit. This replaces the lengthening of minimum-jerk legs.
3. *Allowance.* With the drift state on, the filter's own position sigma already carries the link error, so the stated link accuracy is no longer added a second time.

| Link (hand-over on) | Grasps | Aborts | Largest push | Monitor's smallest value | Fly-in time, one leg / three legs | Fly-in propellant |
|---|---|---|---|---|---|---|
| 60 mm, 1 Hz, P5.4 | 23/25 | 1 | 0.84 mm/s | −0.005 m | 408 / 412 s | 464 mm/s |
| 60 mm, 1 Hz, P5.4b | 25/25 | 0 | 1.04 mm/s | 0.069 m | 242 / 348 s | 464 mm/s |
| 20 mm, 5 Hz, P5.4b (first acceleration limit, 4e-4) | 25/25 | 0 | 0.85 mm/s | 0.120 m | 302 / 581 s | 381 mm/s |

No collisions. The 20 mm row was run before the acceleration limit was raised and has not been repeated with the final value.

**P5.5 result (15 Nov, `p5_fault.json`, `p5_fault_nis.json`): the E2 fault set on the servicer's own information.** Chain as P5.4b at the 60 mm link with the hand-over on. One fault per case, at a random time in the fly-in, reach or approach; 25 tune seeds per row.

| Fault | Aborted | Captured | Collisions | Largest push | Detection delay (median / max) | Caught by |
|---|---|---|---|---|---|---|
| Thruster stuck open (0.1–0.4 N) | 25 | 0 | 0 | 0.02 mm/s | 0.16 / 0.28 s | thrust residual |
| Thruster weak (0–30% left) | 13 | 12 | 0 | 0.69 mm/s | 12.8 / 91.7 s | thrust residual |
| Spin axis wrong by 5–15° | 0 | 25 | 0 | 1.03 mm/s | not needed | the fly-in is re-planned from the estimated axis |
| Client kick 0.5–2°/s, monitors as in P5.4b | 18 | 6 | **1** | **15.67 mm/s** | 9.6 / 50.8 s | rate jump, tracking error |
| Client kick, with the new innovation monitor | 25 | 0 | 0 | 0.02 mm/s | 1.2 / 1.7 s | attitude innovation |
| No fault, with the new innovation monitor | 0 | 25 | 0 | 1.04 mm/s | | no false alarm |

The allowance did not hide the clearance or thruster faults. The kick was the gap: the rate-jump monitor reads the filter's rate, and the filter follows a kick slowly, so kicks during the approach led to one collision and three hard pushes without an abort. The new monitor (`nav_anom`) aborts when the attitude innovation of a link message is more than 60 times its expected square (3 when consistent). Nominal runs reach at most 17; kicks give 98 to 1431.

**P5.5b result (15 Nov, `p5_fault_est.json`): the abort back-off on the estimate.** The frozen run loop builds the abort from the true client state. The step hook now replaces the back-off reference, the thruster-fault re-pointing and the gripper retract direction by ones built from the filter and camera estimates, in the step the abort starts. All four fault rows were repeated with this and with the innovation monitor on: 100 runs, no collisions, 63 aborts, all 63 flown on the estimate and all reaching the safe state within 46 s. The outcomes per fault are the same as in the table above (kick 25 aborts, stuck 25 aborts, weak 13 aborts and 12 captures, wrong axis 25 captures), and the innovation monitor gave no false alarm. The estimate was up to 178 mm and 2.3 mm/s off when an abort started.

Open: kicks below 0.5°/s were not tried; the monitors still read the true state during the 30 s wait at the hold point; the test split has not been run.

Later extensions, simplest first: the same client but silent (state from the servicer's own camera and lidar; the original plan below) → faster spin and nutation → a client with no bar (grab the launch adapter ring).

## 1. Question and claim

**Today:** the arm grasps from camera images (Project 4), but the servicer body does not. Station keeping, the fly-in and the safety monitors read the client's true position, attitude, velocity and spin rate from the simulator.

**Question:** can the body do the fly-in and hold station for the grasp using only an estimate of the client made from its own bus camera and lidar, with the same success and safety as on the true state?

**Claim we can make:** sim-to-sim only, as in Project 4. Randomised rendering, held-out seeds, no real-image claim.

**Success:** the fly-in plus grasp criteria of Project 4 (captured, no unintended contact, no abort, client push ≤ 2 mm/s). **Safe:** the E2 definition (success, or an abort that reaches the safe state). Any collision is unsafe.

## 2. Scope

- **In:** the fly-in from the 8 m hold point, station keeping at the pre-grasp point, and the E2 monitors that use the client state (predicted clearance, tracking, rate anomaly). Scene: the arm2 camera scene. Spin 0–2°/s.
- **Out, unchanged:** the far approach (already on a filtered noisy measurement), the arm's bar-pose estimators (Project 4), everything after capture, and spin above 2°/s.
- **Frozen code:** `p3_capture` modules are not edited. The estimate enters through a subclass, as in `noise_tol.py`.

## 3. Tolerance measurement (done, 14 Nov)

`noise_tol.py` corrupts the client state the bus reads. Each error is correlated noise (correlation time 2 s) at the given 1σ per axis; delay is unpropagated. The arm stays on the true pose. 25 tune seeds (5,000,000–24) per condition. Files `p5_tol_sweep1.json`, `p5_tol_sweep2_*.json`.

**Sweep 1: one error at a time, monitors on the truth.** Baseline 25/25, fly-in Δv 419 mm/s.

| Error (1σ) | Grasps | Safe aborts | Collisions | Fly-in Δv (mm/s) | Station error near the client, rms (mm) |
|---|---|---|---|---|---|
| none | 25/25 | 0 | 0 | 419 | 8.3 |
| position 10 mm | 25/25 | 0 | 0 | 423 | 9.6 |
| position 30 mm | 23/25 | 2 | 0 | 452 | 15.7 |
| position 100 mm | 0/25 | 25 | 0 | — | — |
| attitude 0.5° | 25/25 | 0 | 0 | 454 | 11.0 |
| attitude 1° | 25/25 | 0 | 0 | 508 | 14.6 |
| attitude 3° | 0/25 | 25 | 0 | — | — |
| velocity 1 mm/s | 25/25 | 0 | 0 | 450 | 17.1 |
| velocity 3 mm/s | 19/25 | 6 | 0 | 583 | 33.5 |
| velocity 10 mm/s | 4/25 | 19 | 1 | 653 | — |
| spin rate 0.01°/s | 25/25 | 0 | 0 | 551 | 10.0 |
| spin rate 0.03°/s | 25/25 | 0 | 0 | 891 | 15.2 |
| spin rate 0.1°/s | 25/25 | 0 | 0 | 2651 | 47.1 |
| delay 0.1 s | 25/25 | 0 | 0 | 419 | 8.2 |
| delay 0.3 s | 25/25 | 0 | 0 | 419 | 9.2 |
| delay 1.0 s | 25/25 | 0 | 0 | 419 | 20.6 |

**Sweep 2: all errors together.** `good` = 10 mm, 0.5°, 1 mm/s, 0.01°/s, 0.2 s. `mid` = 20 mm, 1°, 2 mm/s, 0.02°/s, 0.2 s.

| Level | Monitors read | Grasps | Safe aborts | Abort reasons | Fly-in Δv (mm/s) |
|---|---|---|---|---|---|
| good | truth | 24/25 | 1 | predicted clearance 1 | 576 |
| mid | truth | 18/25 | 7 | predicted clearance 6, tracking 1 | 772 |
| good | estimate | 16/25 | 9 | predicted clearance 9 | 576 |
| mid | estimate | 7/25 | 18 | predicted clearance 9, rate anomaly 9 | — |

No collisions in sweep 2.

**What this says:**
1. **The control loops tolerate about 10 mm, 1°, 1 mm/s.** Velocity error is the tightest, and the only one that caused a collision (at 10 mm/s).
2. **Errors add up.** Each error at the `good` level costs nothing alone; together they cost 1 grasp in 25 and 37% more propellant.
3. **The monitors are stricter than the controller.** Reading the same `good` estimate, the predicted-clearance monitor aborts 9 of 25 fly-ins that would have succeeded. It extrapolates relative velocity over 30 s against a threshold with no allowance for estimate error. The rate-anomaly monitor (0.1°/s jump in 2 s) also false-alarms at 0.02°/s rate noise.
4. **Spin-rate noise costs propellant, not grasps,** because the fly-in differentiates the measured rate. The estimator's rate must be smooth.
5. **Delay is not critical** up to 0.3 s, even without propagation.

## 4. Requirements for the estimator (from §3)

At the output of the filter, per axis, 1σ:

| Quantity | At the pre-grasp point (about 2.7 m from the client centre) | At the 8 m hold point |
|---|---|---|
| Relative position | ≤ 10 mm | ≤ 30 mm |
| Relative attitude | ≤ 0.5° | ≤ 1° |
| Relative velocity | ≤ 1 mm/s | ≤ 1 mm/s |
| Client spin rate | ≤ 0.01°/s, smooth | ≤ 0.01°/s |
| Latency | ≤ 0.3 s | ≤ 0.3 s |

The 8 m column is an assumption: sweep 1 used one level for the whole fly-in. Step P5.3 checks it with range-dependent error.

## 5. Design

Three pieces, in the same pattern as Project 4 (a learned estimator feeding classical control):

1. **Client-pose network.** Input: bus camera (192 px) and flash lidar (128 px). Output: the client body's position and attitude in the bus frame. The Project 4 network (v3) only knows the bar at grasp range; this one must work from 8 m to 1.25 m and for every face of the client the fly-in passes. Start by retraining the v3 architecture on fly-in frames.
2. **Relative-motion filter.** Input: network poses at 10 Hz, the servicer's own gyro, accelerometer and commanded thrust. State: relative position and velocity (the Project 2 filter), client attitude, and client spin rate with a slow random walk (the client nutates, so the rate is not constant). Output at 100 Hz: the four quantities the bus and monitors read today. It also reports its own uncertainty.
3. **Monitors on the estimate.** Predicted clearance gets an allowance for estimate uncertainty and a short persistence time. The rate-anomaly monitor compares against the filter's innovation, not raw differences. Thresholds are re-tuned on tune seeds and frozen before validation. A new monitor: abort when the filter's uncertainty or innovation grows (the estimate is lost).

## 6. Steps

1. **P5.1 Filter on synthetic measurements.** Build the filter. Feed it the true pose plus white measurement noise at 10 Hz (several noise levels). Measure its output error and check it against §4. No images yet. This shows what measurement accuracy the network must reach.
2. **P5.2 Monitors.** Re-design and tune the three monitors on the filter's output. Target: no more false aborts than on the truth, and the E2 fault cases still caught.
3. **P5.3 Fly-in data and the network.** Record bus camera and lidar frames over fly-ins (about 300 episodes, demo seeds). Train on CPU. Report error against range and face.
4. **P5.4 Closed loop.** Fly-in and grasp with the bus on the filter and the arm on the Project 4 estimators (v3 plus palm). 25 validation seeds. Compare with the truth baseline.
5. **P5.5 Faults.** The E2 fault set (client kick, weak or stuck thruster, wrong spin axis) with the monitors on the estimate. 25 seeds.
6. **P5.6 Test split, once.** Then the demo is re-recorded with no true client state anywhere in the servicer.

Rules as before: one run at a time, about 25 episodes per run, CPU training only, tune on tune seeds, validation for decisions, test once.

## 7. Risks

- **Symmetry.** The client is a plain box, so some faces look alike and the attitude may be ambiguous by 90° or 180°. The bar and its posts break the symmetry only on one face. Remedies: the filter keeps the attitude continuous once initialised; or markers on the client, which is standard for a client prepared for servicing.
- **Velocity.** 1 mm/s from 10 mm position measurements needs several seconds of smoothing, which conflicts with reacting to a kicked client. The accelerometer and known thrust carry the fast part.
- **Hand-over at 8 m.** The far approach ends with 5 cm, 1 mm/s navigation noise from a different (simulated) sensor. The filter must initialise from that and from its first images.
- **Lighting.** Eclipse and glare affected Project 4; the lidar is the fallback, as there.

## 8. Decisions for the user

1. **Filter first (P5.1, P5.2) or network first (P5.3)?** Recommended: filter first. It is quick, needs no images, and turns §4 into a measurement-accuracy target for the network.
2. **Markers on the client: allowed or not?** Recommended: not at first; add them only if symmetry blocks progress.
3. **Is the initial spin axis given?** Today the fly-in plans around the true spin axis from the start. Recommended: P5 estimates it at the hold point from the first 30–60 s of images, since otherwise one true value remains.
