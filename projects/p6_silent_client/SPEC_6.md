# Project 6: silent client (type C), draft spec

Status: DRAFT for the user, 15 Nov 2026. Nothing built. Research: `learning/RESEARCH_SERVICING.md`, "Silent client (15 Nov 2026)".

## 1. Question and claim

**Today (Project 5, done):** the client reports its own state over a link. The servicer filters it, flies in, holds station, grasps from its cameras, and aborts safely on faults. Test split: 25/25 fault-free, 0 collisions in 125 runs.

**Question:** can the same mission run when the client says nothing, with the servicer estimating the client's position, attitude, velocity, spin axis and rate from its own bus camera and lidar, at the same success and safety?

**Claim:** sim-to-sim only, as in Projects 4 and 5 (randomised rendering, held-out seeds, no real-image claim).

**Success and safety:** unchanged (captured, no unintended contact, no abort, push ≤ 2 mm/s; safe = success or an abort that reaches the safe state; any collision is unsafe).

## 2. Scope

- **In:** the hold point at 8 m, a new observation phase there, the fly-in, station keeping at pre-grasp, the monitors. Client as now: box with grapple bar, spin 0-2 deg/s, slight nutation.
- **Out, unchanged:** the grasp (v3 + gated palm v2), everything after capture, spin above 2 deg/s, clients without a bar.
- **Open (decision D3):** the far leg from 30-50 m to the 8 m hold point.
- **Frozen code:** `p3_capture` and `p4_vision` modules are not edited; Project 5 code is reused by import or subclass.

## 3. What carries over

Grasp chain; `RelFilter` (position, velocity, attitude, spin rate, tide term; drift state off); speed-limited fly-in (`fly_lim.py`); predicted-clearance monitor with allowance (k = 2); thrust residual; attitude-innovation monitor; abort on the estimate; fault set; tolerance requirement (SPEC_5 §4: at pre-grasp 10 mm, 0.5 deg, 1 mm/s, 0.01 deg/s, ≤ 0.3 s); seed splits and runners.

## 4. What is new

1. **Client pose from 8 m to 1.25 m** from the bus camera and lidar. Route by decision D1.
2. **Observation phase at the hold point.** The filter watches the client until its spin axis and rate uncertainty pass a threshold (expected: minutes, not today's 30 s), with a time limit and an abort if it never converges. The fly-in is planned only after that.
3. **Symmetry and outliers.** The client box looks alike from opposite sides; only the bar and wings break it. Measurements are gated against the filter's prediction (as the palm gate), and a flip of about 180 deg is rejected or handled as a second hypothesis.
4. **Lost-track abort** in place of the stale-link abort (no accepted measurement for N s, or the filter's uncertainty grows past a bound).
5. **Monitor re-tune** on tune seeds: the innovation monitor now reads camera measurements, so its threshold and kick detection time change.

## 5. Steps (one run at a time, about 25 episodes per run)

1. **P6.1 Synthetic vision errors, no images.** Replace the link with a vision-like error model on the true pose: error growing with range, face-dependent, outliers, 180 deg flips, dropouts in eclipse or with the arm in view, 5-10 Hz, 0.1-0.2 s late. Add the observation phase and the lost-track abort. Find which error levels the chain tolerates and how long the observation must be to reach 0.01 deg/s. This sets the number the estimator must meet, as P5.1 did.
2. **P6.2 Estimator.** Route by D1. Report error against range, face and lighting on validation seeds.
3. **P6.3 Closed loop.** The whole chain with the estimator from P6.2, tune seeds, then validation; compare with Project 5.
4. **P6.4 Faults.** The four E2 faults on the new chain.
5. **P6.5 Test split, once.**

## 5b. P6.1 result (15 Nov, `vis_eval.py`, `p61_sweep.json`; tune seeds, 25 per row)

Chain: vision error model on the true pose (10 Hz, 0.15 s late; position error × range / 2.7 m; ×1.5 with the bar face turned away; half the variance correlated over 5 s; box-symmetry flips lasting 1 s on average; random outliers; dropout bursts of 1–6 s), gated filter with symmetry snapping (measurement sigma ×3 for the correlated error, rate random walk 2e-6), observation phase, **inspection hop** (user's choice), lost-track backstop 10 s, `nav_anom` after 2 s of rejected attitude. Arm on the true bar pose.

**Why the hop:** the hold point lies on the spin axis. For spin about client y or z (16 of 25 seeds) the bar face stays edge-on for the whole wait, so the attitude is known only up to the box symmetry and the fly-in could go to the wrong end (without the hop: 9/25, the rest `no_converge`). The hop moves the servicer to a point 45° off the axis at the same range, fixed in inertial axes and aimed ahead at where the estimated bar end will face. The end is resolved by seeing the bar, or by the estimated bar end facing the servicer for 3 s with no bar (then the estimate is turned 180°). The servicer then hops back to the axis and flies in as before. In every case here the bar came into view during the hop out.

| Condition (pos mm at 2.7 m, att °, outliers, flips, dropouts/min) | Captured | Safe aborts | Collisions | Push max | Observation (median / max) | Rate error at fly-in start (median / max) |
|---|---|---|---|---|---|---|
| clean 10, 1, 0, 0, 0 | 25 | 0 | 0 | 0.53 mm/s | 477 / 731 s | 0.005 / 0.066 °/s |
| nominal 10, 1, 2%, 5%, 1 | 25 | 0 | 0 | 0.56 | 478 / 541 s | 0.007 / 0.073 |
| attitude 2° | 24 | 1 (clearance) | 0 | 0.68 | 464 / 919 s | 0.017 / 0.069 |
| position 30 mm | 25 | 0 | 0 | 0.88 | 474 / 540 s | 0.005 / 0.073 |
| mid 20, 1.5, 5%, 10%, 2 | 24 | 1 (nav_anom) | 0 | 0.60 | 474 / 536 s | 0.011 / 0.070 |
| dirty 10, 1, 10%, 20%, 3 | 24 | 1 (clearance) | 0 | 0.50 | 446 / 514 s | 0.011 / 0.050 |

147/150 captured, 0 collisions, every abort safe. Both clearance aborts are seed 5000011 (near-static client, hold point to the side; the path passes a corner, as in P5.4b).

**What it says for the network (P6.2):** per-message 1σ of 30 mm at the pre-grasp range (×range beyond it) and 2° are tolerated, with up to 10% outliers, 20% symmetry flips and 3 dropouts a minute, **if** the network also reports whether the bar is in view (used to restrict the symmetry snapping and to resolve the bar end). Target set: **≤ 20 mm and ≤ 1.5° (1σ) at 2.7 m, outliers ≤ 5%, a bar-visible flag**, leaving margin. The observation phase takes about 8 minutes with the hop (30 s in Project 5); fixed time D2 is not needed, the convergence test sets it.

**Not yet checked:** faults with the vision chain (the kick is now caught by 2 s of rejected attitude, not by one message; P6.4); missed bar detections (the flag is ideal here); camera pointing during the hop (not modelled: the measurement exists whatever the servicer's attitude); the monitors still read the true state during the observation and hops.

## 6. Decisions for the user

- **D1, how the client pose is estimated (8 m to 1.25 m): DECIDED 15 Nov, (a) learned network** (user: "Learned network").
  - (a) a learned network on bus camera + lidar, the Project 4 pattern (fly-in data about 300 episodes on demo seeds, CPU training, laptop thermal limit applies). Recommended: matches the project and handles every face.
  - (b) lidar model matching (iterative closest point against the known client model): no training, robust to lighting; box symmetry is its weak point.
  - (c) either of the above plus markers on the bar face for the last metres (realistic for a prepared client; visible only from that face).
- **D2, observation phase length:** set from P6.1, or fixed (for example 120 s or 300 s).
- **D3, far leg 30-50 m → 8 m:** keep on today's filtered stand-in measurement, or add a bearing + lidar range track (simple; no attitude).

## 7. Risks

Rate accuracy (0.01 deg/s) may need long observation; symmetry flips near the bar-less faces; CPU-only training time; the stow side effect (bus off station at close, 32 / 74 mm on the test split) may add to vision error at close range.
