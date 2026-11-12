# Project 3: capture and reposition a client satellite

Status: **3A–3D done with momentum management; 3E E1 (spinning client, fly-in) and E2 (safety monitors, aborts, grip re-clamp; §12) done (10 Nov).** No collision with the monitors on in any faulted validation case. **E3 (whole-mission Monte Carlo) in progress (§13).** Then the ESPA-class servicer port through a robot description layer (§14), then Project 4. Then Project 4: camera-based imitation-learned grasp (LeRobot), shielded by the E2 monitors.

End goal (user, 8 Nov): a servicing robot that attaches itself to a satellite, then moves it to a commanded attitude or position.

**Client changed on 8 Nov (user):** from a Starlink-like bus with no fixture to a **generic rectangular satellite with something to grasp**, a grapple bar. This is a *prepared* client, like those fitted with Astroscale's docking plate or ISS grapple fixtures. The rim, adhesion and caging options in section 1 now apply only to the later unprepared-client extension. Section 2 describes the new client; the section 3 tables before "3A on the new client" are for the old Starlink-like bus.

## 1. Roadmap (revised 8 Nov after the research in [RESEARCH_SERVICING.md](../../learning/RESEARCH_SERVICING.md))

**What the research changed:**
- **The realistic client is dead, unprepared and heavy.** Starlinks carry no grapple fixture. A failed one is a removal or repositioning case, and a V2 Mini (about 800 kg) outweighs our servicer.
- **Real arms grab the launch-interface structure.** NRL/RSGS on MRV and MEV-1's launch ring do this. Our capture feature is that stand-in: a stiff rim.
- **Pose estimation and attitude-anomaly aborts matter as much as guidance.** ADRAS-J aborted twice on relative-attitude anomalies.
- **Contact results depend on timestep and contact parameters** (DLR OOS-SIM). Sensitivity checks are mandatory.
- **In the literature, learning helps with motion planning and synchronisation, while contact is classical or force-controlled.** mjorbit's own PPO Astrobee grasp (88%) is the learned reference.

| Step | What | Classical baseline | Learning angle |
|---|---|---|---|
| **3A** (done) | Approach to the pre-grasp point, box keep-out, corridor | Waypoint routing + glideslope + 9-state filter | (none needed: 100%) |
| 3A+ | Client **pose-error model** (attitude and position noise or bias); **servicer attitude** pointing the arm side at the client; abort on relative-attitude anomaly | P1 attitude control; shield with box clearance | – |
| **3B** | 6–7 DoF arm with gripper **grasps a rim** at the capture feature, real contact. First **free-flying** (base keeps station), then **free-floating** (base actuators off at contact) | Jacobian / inverse-kinematics reach, **impedance control** at contact, slow close; reaction-null-space arm motion for free-floating | PPO residual on the reach (timing); imitation (ACT / Diffusion Policy) from classical demonstrations; compare with upstream astrobee_grasp PPO |
| 3B' | Alternative capture: **adhesion pad** on the flat face (MuJoCo `adhesion` actuator, gecko-like) | Normal approach, preload, shear hold | – |
| **3C** | Post-capture: **detumble the stack**, keep grasp loads low, **identify the combined inertia and centre of mass** online | Momentum-conservation coarse estimate, then recursive least squares; compliant coordination | – |
| **3D** | **Align and move:** reorient the stack to a commanded attitude, then reposition it (metres to hundreds of metres) with thrust through the combined centre of mass; a re-entry burn as an extension | P1 pointing with identified inertia; centre-of-mass-aware thrust allocation; P2 guidance | Residual on attitude or translation under inertia uncertainty |
| **3E** | Rotating, then **tumbling client (up to 3°/s, ClearSpace-1 spec)**; motion synchronisation; **caging** option; V2 Mini-class client; a shield for every phase; whole-mission Monte Carlo and the test set once | Rotating keep-out / tube MPC; synchronised reach | PPO motion synchronisation (ESA ASTRA 2023 style), domain-randomised |

Same discipline as P2:
- classical baseline first;
- matched validation cases (seed 100);
- domain randomisation, including sensor noise (lesson from r2);
- the test set (seed 1000) once at the end;
- timestep and contact-parameter sensitivity runs at every contact stage.

## 2. 3A model (`p3a_scene.xml`, `capture_env.py`)

- **Client, current:** a generic rectangular satellite, 300 kg, 1.5 × 1.0 × 1.0 m bus (long axis = body x).
  - **Grapple bar** on the −x face: 0.30 m long, 2.5 cm radius, along body y, standing 0.15 m off the face on two posts. It is a handrail-type handle sized for a two-finger gripper, like ISS handrails or Astrobee handles.
  - ISS FRGF-style fixtures (pin, sphere and guide ramps) suit snare end-effectors like Canadarm's instead, and are kept as an option.
  - **Capture feature** = bar centre (−0.9, 0, 0). The keep-out is the bus box plus the bar envelope.
- *Earlier, until 8 Nov:* a bare Starlink-v1.5-like bus, 300 kg, 2.8 × 1.4 × 0.25 m, with the capture point on its end face.
- **Client attitude at reset:** body x along LVLH +y, rotated by a yaw about the orbit normal. Yaw 0 points the capture face at −V-bar, where the servicer starts. `yaw_mode` is `aligned` or `random` (uniform ±180°).
  - `client_rot = lvlh`: the client holds its LVLH attitude, as an operational satellite would.
  - `client_rot = inertial`: a dead client stays inertially fixed, so its face turns at the orbit rate, 0.064°/s (about 76° per 1200 s episode). The pre-grasp point then moves at about 3.6 mm/s.
- **Servicer:** 150 kg, 1 m cube, thrusters at 0.75 N per world axis (5 mm/s², the P2 authority). Attitude is assumed held by its own attitude control, as in P2. Bounding radius 0.87 m.
- **Pre-grasp point** g = feature (bar centre) + 1.8 m × face normal (servicer centre). It moves with the client. The servicer's face is then 1.3 m from the bar, within reach of an arm about 1.5 m long.
- **Keep-out:** clearance = signed distance from the servicer centre to the client (bus box and bar envelope) − 0.87 m − 0.3 m margin. Clearance < 0 ends the episode. At g the clearance is about 0.6 m.
- **Success:** |p − g| < 0.10 m and speed relative to g < 5 mm/s for the final 60 s of a 1200 s episode (60 decisions of 20 s).
- **Navigation and errors:** P2's u1 preset (thrust gain and misalignment, constant disturbance, 5 cm / 1 mm/s nav noise) and its 9-state filter. The client's pose is assumed known, as if from a pose estimator.
- **Corridor metric:** a decision is a violation if, within 5 m of the feature, the servicer is outside the 15° approach cone about the face normal.
- **Code:** P2's `ApproachEnv` gained hooks (`goal_lvlh`, `goal_vel_lvlh`, `clearance`, `proximity`, `xml_path`). `CaptureEnv` overrides them. P2 results are unchanged (regression check: CW epoch + nav9 under u1, 98%, Δv 0.332).

## 3. 3A classical baseline (`capture_guidance.py`, `evaluate_p3a.py`)

**Logic, each 20 s epoch:**
- **Inside the corridor** (within 15° of the face normal and within 11 m of the feature): glideslope straight to g, with closing speed 0.03 × range in [2 mm/s, 5 cm/s], plus feed-forward of g's motion.
- **Outside:** head for the corridor entry A = feature + 10 m × normal. If the straight segment to A comes within `seg_margin` of the keep-out volume, go instead to a waypoint on a sphere of radius `r_safe` around the client, rotated at most 60° from the current bearing towards the normal.
- Transfers use the bias-compensated CW coast velocity with t_go = distance / 0.1 m/s, clipped to [40, 400] s, re-planned each epoch.

**Tuning on validation, u1, 200 cases, random yaw, LVLH-fixed client** (`run_p3a_tune.sh`), with the filter:

| Path margin / safe radius | Success | Collisions | Δv mean / p95 | Episodes with a corridor violation |
|---|---|---|---|---|
| 1.5 m / 8 m | 100% | 0 | 0.535 / 0.678 | 14.5% |
| 3 m / 10 m | 100% | 0 | 0.550 / 0.857 | 12% |
| **4 m / 12 m (chosen)** | **100%** | **0** | 0.554 / 0.813 | **0%** |

With small margins, paths to the corridor entry skim past the side of the bus within 5 m of the feature. A 4 m margin routes them around.

**Results, validation, 200 cases, chosen guidance** (`run_p3a_cases.sh`; `results_p3a_*.json`):

| Case | Controller | Success | Collisions | Δv mean / p95 | Settle median / max | Final error max | Min clearance |
|---|---|---|---|---|---|---|---|
| u1, aligned | + nav9 | 100% | 0 | 0.468 / 0.521 | 534 / 688 s | 0.089 m | 0.56 m |
| u1, random yaw | + nav9 | 100% | 0 | 0.554 / 0.813 | 601 / 823 s | 0.075 m | 0.56 m |
| u1, random yaw, inertial client | + nav9 | 100% | 0 | 0.572 / 0.815 | 599 / **1141 s** | 0.093 m | 0.56 m |
| none, random yaw | + nav9 | 100% | 0 | 0.532 / 0.810 | 602 / 819 s | 0.045 m | 0.57 m |
| none, random yaw, inertial | + nav9 | 100% | 0 | 0.551 / 0.770 | 600 / 888 s | 0.040 m | 0.57 m |
| u1, random yaw | raw measurements | 68.5% | 0 | 0.726 | 981 / 1141 s | 0.099 m | 0.49 m |

All runs have 0 corridor violations. With random yaw, success is the same whether the face points towards the servicer or away.

**Reading it:**
- **The P2 toolkit transfers.** The filter, glideslope and bias-compensated CW give 100% approach to a moving pre-grasp point on a box-shaped client in every case, with 0 collisions.
- **The filter is essential at grasp precision.** On raw measurements, holding within 0.10 m and 5 mm/s fails in 31% of episodes. All failures are holding failures; there are no collisions.
- **Going around costs fuel.** Δv is 0.47 with the face towards the servicer and 0.55 with random yaw. That is 1.7–2× P2's hold at 3 m, from the longer path and the slow corridor.
- **Thin margin, one case:** for the inertially fixed client under u1, one episode settled at 1141 s, right at the deadline. Its pre-grasp point keeps moving (about 3.6 mm/s against a 5 mm/s tolerance), and in the worst case the corridor approach chases it. A longer episode or a faster corridor would fix it. Worth watching in 3E.
- **Not modelled yet:**
  - servicer attitude: it is assumed to keep its arm side towards the client;
  - pose-estimation error on the client;
  - plume impingement on the client;
  - a shield (P2's assumes a sphere; it needs the box clearance);
  - the 1000-case Monte Carlo.

### 3A on the new client (rectangular bus + grapple bar), 8 Nov

`run_p3a_v2.sh`, `results_p3a_v2_*.json`. Validation, 200 cases, guidance with 4 m path margin and 12 m safe radius, with the filter:

| Case | Success | Collisions | Δv mean / p95 | Settle max | Final error max | Min clearance | Corridor violations |
|---|---|---|---|---|---|---|---|
| u1, random yaw | 100% | 0 | 0.560 / 0.889 | 825 s | 0.070 m | 0.53 m | 0% |
| u1, aligned | 100% | 0 | 0.468 / 0.517 | 961 s | 0.089 m | 0.54 m | 0% |
| u1, random yaw, inertial client | 99.5% | 0 | 0.575 / 0.855 | 885 s | 0.093 m | 0.54 m | 0% |
| none, random yaw | 100% | 0 | 0.537 / 0.892 | 819 s | 0.040 m | 0.54 m | 0% |
| none, random yaw, inertial | 100% | 0 | 0.552 / 0.833 | 856 s | 0.049 m | 0.55 m | 0% |
| u1, random yaw, raw measurements | 68% | 0 | 0.736 | 1141 s | 0.099 m | 0.47 m | 0% |

- The results match those on the old bus within noise. The approach doesn't depend on the client's shape beyond the keep-out.
- The single failure is in the inertial-client case under u1. It is a hold failure, not a collision. The thin settle margin for a client turning relative to LVLH, noted above, still applies.

## 4. Next: 3B, arm reach and grasp (plan)

- **Scene:**
  - servicer, 150 kg, with a 6–7 DoF arm on its client-facing side and a two-finger gripper;
  - client: the rectangular satellite with its **grapple bar**, which the gripper closes around (a rim or unprepared surface comes later);
  - contact on; timestep ≤ 0.01 s for contact, with sensitivity checked at 0.005 / 0.002 s.
- **Start:** at the 3A pre-grasp point, servicer station-keeping, LVLH-fixed client.
- **Baseline:**
  - resolved-rate or inverse-kinematics reach to a pre-contact pose;
  - Cartesian impedance on approach;
  - close the gripper on detected contact;
  - check that the grip holds under small pulls.
- **Metrics:**
  - grasp success, i.e. holding for 30 s;
  - client velocity imparted (push-away);
  - peak contact force;
  - servicer attitude disturbance;
  - joint-limit and self-collision events.
- **Then free-floating:** base actuators off from contact, and arm motion planned to cancel base rotation (reaction null space; needs the redundant 7th joint).
- **References to read in full first:**
  - the Frontiers 2021 capture survey;
  - mjorbit `examples/ppo/astrobee_grasp.xml` and `examples/arm_reach.py`;
  - the RNS and impedance papers listed in RESEARCH_SERVICING.md.

## 5. 3B: reach and grasp the grapple bar (8 Nov)

**Scene (`p3b_scene.xml`):**
- **Servicer:** 150 kg in total, a 1 m bus. Ideal bus actuators on its free joint, as in upstream `astrobee_grasp`: thrusters at 0.75 N per world axis, reaction-wheel torques of 1 N·m in the body frame.
- **Arm** on the client-facing face, 6 revolute joints:
  - shoulder yaw and pitch, elbow, and a spherical wrist (pitch, yaw, roll);
  - upper arm 0.70 m, forearm 0.55 m, 1.93 m reach from the bus centre;
  - torque motors, 20 / 20 / 15 / 5 / 5 / 5 N·m.
- **Gripper:** parallel jaw, two flat pads on opposed slides, position-servoed and force-limited (30 N). Pads close across the bar.
- **Client:** the 3A rectangular satellite with its grapple bar, an independent free body. Nothing connects the two bodies: no weld, no equality constraint.
- **Contacts:** only gripper pads and palm against the client's bar, posts and bus. Timestep 0.002 s. mjorbit CPU backend with orbit coupling.
- **Implementation:** a plain-MuJoCo copy of the model is synced each control step (qpos, qvel, ctrl) for Jacobians, site poses and contact forces, because the mjorbit CPU data exposes only state arrays.

**Classical controller (`grasp_sim.py`), at 100 Hz:**
1. **Settle (2 s):** the bus keeps station relative to the client (PD on relative position and attitude); the arm holds its stowed pose.
2. **Reach (20 s):** inverse kinematics by damped least squares to the pre-contact pose (grip point 0.12 m in front of the bar, gripper aligned with the bar frame), then a joint-space minimum-jerk move with joint PD.
3. **Approach:**
   - Cartesian impedance at the grip point: K = 300 N/m and 30 N·m/rad, D = 60 and 6;
   - the target first blends from the current grip pose for 5 s, then slides along the approach axis at 1 cm/s;
   - every target is expressed in the client frame, so it tracks the client's motion.
4. **Close:** when the grip point is within 1 cm of the bar, both the impedance target and the bus references are frozen at the current pose, so nothing pushes on contact. Then the jaws close.
5. **Hold (30 s):** thrusters off; the wheels damp the stack's rotation relative to LVLH. Controlling the stack is 3C's job.

**Modes:**
- **free-flying:** bus actuators active throughout;
- **free-floating:** bus actuators off from the start of the approach until capture.

**Cases** (`evaluate_p3b.py`, seeds 100000+; `run_p3b_eval.sh`):
- servicer at the pre-grasp point with ±8 cm position error per axis, ±3 mm/s velocity error, ±3° attitude error;
- client tilted up to ±3° and LVLH-fixed (spinning at the orbit rate).

**Success** = all of:
- both pads on the bar at the end of the close and at the end of the hold;
- grip point within 2 cm of the bar axis;
- bar slip relative to the grip point < 2 mm/s.

**Results, validation, 200 cases each:**

| Setting | Success | Other contact | Client push Δv median / max | Peak contact force | Clamp force | Bus attitude disturbance max | Bar shift in grip max | Slip max |
|---|---|---|---|---|---|---|---|---|
| free-flying, dt 0.002 s | **100%** | 0 | 0.45 / 0.73 mm/s | 7.7 N | 3.2 N | 1.5° | 0.46 mm | 0.017 mm/s |
| **free-floating**, dt 0.002 s | **100%** | 0 | 0.27 / 0.75 mm/s | 7.7 N | 3.2 N | **6.7°** | 0.48 mm | 0.017 mm/s |
| free-flying, dt 0.001 s | 100% | 0 | 0.45 / 0.74 mm/s | 7.5 N | 3.2 N | 1.5° | 0.46 mm | 0.017 mm/s |
| free-flying, dt 0.005 s | 100% | 0 | 0.42 / 0.71 mm/s | 6.1 N | 3.2 N | 1.5° | 0.45 mm | 0.017 mm/s |

Other numbers:
- approach tracking error ≤ 1.3 cm;
- grasp completed at about 41 s;
- inverse-kinematics error < 0.01 mm.

**Bugs found and fixed on the way:**
1. **The first controller pushed the client to about 6 mm/s at closing.** The impedance target was still advancing and the bus was holding its pre-capture reference. Fix: freeze both at the close command.
2. **After capture, the bus kept station-keeping relative to the now-attached client** and slowly accelerated the stack. Fix: thrusters off after capture.
3. **Contact forces read near zero,** because the kinematics copy was missing ctrl. Fix: sync ctrl.
4. **The first success test compared centre-of-mass speeds of the two bodies.** A rigidly joined, slowly rotating stack fails that test without any slip. Replaced by bar slip relative to the grip point.
5. **The approach target jumped 14 cm** after the reach, because the bus kept correcting during the reach. Fix: Cartesian blend.
6. **Parallel workers raced on mjorbit's temporary compile file.** Fix: staggered retry.

**Reading it:**
- **A prepared bar makes the grasp easy for a classical controller.** It works first time once the push-on-contact and post-capture bugs are fixed, at all three timesteps, with sub-mm/s push and no unintended contact.
- **Free-floating capture works but disturbs the base 4–5× more** (6.7° against 1.5°): with the bus passive, the arm's reaction torque turns it. This is exactly what reaction-null-space motion addresses. It needs a redundant (7th) joint or a planned path, and is the next 3B item.
- **Untested:**
  - **the grip under load.** The clamp is only 3.2 N, from a small position-servo error, and the hold applies no pull. 3C's detumble will load it, so a pull and torque test, with a higher preload if needed, comes before 3C;
  - sensitivity to contact parameters other than the timestep (friction, solref);
  - client pose-estimation error (the controller uses the true bar pose);
  - a rotating or tumbling client beyond the orbit rate;
  - learning. Not needed for this case; it becomes relevant with pose error and tumbling (3E).

## 6. 3B grip load test (8 Nov)

**What 3C will ask of the grip (estimate).** Once the two are joined, the stack is about 450 kg.
- Client inertia about its centre: about 50 / 81 / 81 kg·m².
- The stack's centre of mass is about 0.9 m from the client centre, so the stack inertia is about 835 kg·m², of which the client carries about 0.39.
- **Translation:** the servicer's 0.75 N thrust accelerates the client through the grip with 0.75 × 300/450 ≈ **0.5 N**.
- **Rotation:** the 1 N·m wheel torque passes about 0.39 × 1 ≈ **0.4 N·m**, plus about 0.3 N of force, through the grip.
- **Requirement with a 3–4× margin:** hold **2 N** and **1.5 N·m** in every direction.

**Test (`loadtest_p3b.py`, `run_p3b_load.sh`):**
- Grasp as in 3B (free-flying), wait 5 s.
- Ramp an **internal** wrench, equal and opposite on client and servicer, applied by test actuators on both free joints, so the stack doesn't accelerate and the full load goes through the arm and grip.
- Directions ±x, ±y, ±z of the client frame: x = into the capture face, y = bar axis, z = across the bar.
- Forces at 1 N/s up to 30 N; torques at 0.5 N·m/s up to 15 N·m. 10 cases per direction.
- **Slip** = sustained motion of the bar relative to the gripper (> 2 mm/s or > 1°/s, averaged over 0.2 s), gross motion > 15 mm or 10°, or a pad losing contact.

**Finding 1: MuJoCo friction creep.**
- With MuJoCo's default **pyramidal** friction cones (the original 3B scene), slip set in at about 6–9 N and 3.4–4.1 N·m **whatever the clamp**: 3.2 N, 9.6 N and 17.5 N gave nearly the same limits.
- An earlier version of the test used a 3 mm threshold, which also counted soft-contact deflection as slip.
- Physical friction capacity scales with the clamp, so this was the simulator's soft frictional constraint creeping under a steady sideways load, not physics.
- **Elliptic cones with `impratio` 10** (MuJoCo's recommended setting for grasping) make the capacity scale with the clamp. These are now the default in `p3b_scene.xml`; the old setting is in `p3b_scene_pyramidal.xml`.

**Finding 2: load capacity with elliptic cones** (`results_p3b_load_elliptic.json`). Load at first slip, median over 10 cases:

| Load | Squeeze 0.012 (clamp 3.2 N) | **Squeeze 0.03 (clamp 17.6 N), new default** |
|---|---|---|
| force +x (pull out of the jaws) | 3.6 N | **27.0 N** |
| force −x (push into the jaws) | 5.9 N | 10.1 N |
| force ±y (along the bar) | 5.5–6.0 N | 14.3–14.4 N |
| force ±z (across the bar) | 5.1–7.9 N | 8.3–8.9 N |
| torque ±x | 3.1–3.7 N·m | 5.3–5.4 N·m |
| torque ±y (about the bar) | 2.7–3.6 N·m | 3.7–7.2 N·m |
| torque ±z | 2.7–2.8 N·m | 6.7 N·m |
| **weakest** | 3.4 N / 2.5 N·m (1.7× the requirement) | **8.3 N / 3.7 N·m (4× / 2.4×)** |

- Elastic deflection at the requirement (2 N, 1.5 N·m) is ≤ 0.1 mm and ≤ 0.1°.
- No arm joint saturated before the grip slipped in any test.
- Most failures are "pad lost": the load pushes the bar into one jaw's force-limited servo (30 N), and the other jaw separates.

**3B re-run with the new defaults** (elliptic cones, squeeze 0.03; `*_v2.json`):

| Setting | Success | Push on client max | Peak contact force | Base attitude disturbance max | Slip max |
|---|---|---|---|---|---|
| free-flying, dt 0.002 s | 100% | 0.77 mm/s | 29 N | 1.5° | 0.012 mm/s |
| free-floating | 100% | 0.78 mm/s | 33 N | 6.7° | 0.012 mm/s |
| free-flying, dt 0.001 / 0.005 s | 100% / 100% | 0.78 / 0.75 mm/s | 33 / 40 N | 1.5° | 0.013 mm/s |

The peak force is the closing impact of the firmer squeeze, and it depends on the timestep (29–40 N).

**Reading it:**
- **The grip is good enough for 3C** with a 2.4–16× margin, depending on direction.
- **The weakest direction is torque about the bar axis or across it (3.7 N·m),** because a round bar between flat pads resists rotation mainly by friction. A V-groove or a two-bar fixture would be the hardware fix if 3C needs more.
- **Lesson: test the simulator's contact model before trusting grasp results.** The default friction setting would have under-reported grip strength by up to 4× and hidden the effect of clamp force.
- **Not yet done:**
  - load-test sensitivity to timestep and friction coefficient;
  - dynamic (impulsive) loads;
  - repeating the test free-floating.

## 7. 3C: stabilise and identify the captured stack (8 Nov)

**Setup (`stack_sim.py`, `evaluate_p3c.py`, `run_p3c_eval.sh`, `run_p3c_final.sh`).** Each case:
1. **Randomise the client's mass properties.** These are unknown to the controller and set by an explicit `<inertial>`:
   - mass 200–400 kg;
   - centre of mass offset ±10 cm per axis;
   - principal inertias ×0.8–1.2 of a uniform box, principal axes tilted up to 10°.
2. **Grasp** as in 3B.
3. **Brakes:** the arm joints and gripper jaws are pinned by stiff joint-limit constraints.
4. **Tumble:** a random rigid-body tumble of 0.5–2°/s is imposed on the stack.

The servicer then:
1. **Detumbles:** reaction-wheel rate damping to the LVLH rate (1 N·m wheels).
2. **Identifies rotation:** wheel-torque pulses of ±0.5 N·m, 12 s each, about each bus axis. The momentum form of Euler's equation in the bus frame, I Δω + ∫ω × Iω dt = ∫τ dt, is solved by batch least squares over 4 s windows for the 6 inertia parameters. Integrating instead of differentiating the gyro keeps noise small.
3. **Identifies translation:** body-fixed thrust pulses of ±0.4 N along each bus axis while the wheels hold attitude. The accelerometer Δv at the bus centre is Δv = ∫F dt/m + Δω × d + ∫ω × (ω × d) dt, and torque balance gives d × ∫F dt = I Δω + ∫ω × Iω dt − ∫τ dt. Least squares gives 1/m and d (bus centre minus stack centre of mass).
4. **Validates:** 40 s of thrust across the bus-to-centre-of-mass line (about 0.7 N·m of torque) with a soft wheel attitude hold, first without, then with the counter-torque feed-forward −d̂ × F.

**Sensors:** gyro with σ = 1e-5 rad/s and accelerometer Δv with σ = 1e-6 m/s per 100 Hz sample. Truth (composite properties about the stack centre of mass, in the bus frame) is computed from the model, for scoring only.

**What it took (each fix verified):**
1. **A PD "lock" on the arm joints at 100 Hz was unstable** for the light wrist joints. It shook a pad off the bar within a second. With inertia-scaled gains it was stable but too soft (2° deflection under a 300 kg client). **Fix: brakes,** modelled as joint ranges pinned to the captured angles (solreflimit 0.004) in a rebuilt model with the state copied across.
2. **The stack was not rigid**, as checked against the true inertia with known pulses:
   - **Friction creep below the slip limit** (MuJoCo soft friction). Fix: `noslip_iterations` 20.
   - **Servo-driven jaws act as 400 N/m springs** the client bobs on. Fix: lock the jaws too (non-backdrivable, like a lead-screw gripper; the squeeze remains as contact preload).
   - **No torsional friction** with condim 3. Fix: condim 4.
   - **A round bar between flat pads twists by friction only.** Fix: a **square-section bar** (form closure), now the default in `p3b_scene.xml` and `p3a_scene.xml`; the round version is in `p3b_scene_round.xml`.
3. **1 s integration windows were 3–50× worse than 4 s windows** with 12 s pulses.

Identification error, 24 cases each (medians):

| Variant | Mass | Inertia (relative) | Centre of mass | Validation with counter-torque |
|---|---|---|---|---|
| round bar, 1 s windows (after the creep fixes) | 5.5% | 15% | 119 mm | 0.64° |
| round bar, 4 s windows | 3.1% | 6.1% | 50 mm | 0.26° |
| square bar, 1 s windows | 0.4% | 6.7% | 25 mm | 0.13° |
| **square bar, 4 s windows** | **0.13%** | **0.3%** | **2.6 mm** | **0.02°** |

3B re-run on the square bar (200 cases each, `results_p3b_sq_*.json`):
- free-flying and free-floating both **100%**, with no unintended contact;
- push on the client ≤ 0.82 mm/s;
- the bar shifts up to 1.9–2.9 mm in the grip (flat-on-flat contact settling at closing).

Square-bar load test (`results_p3b_load_sq.json`): weakest 8.2 N (force −z) and 3.6 N·m (torque −y). Torque ±z reaches the shoulder-yaw limit at 7.8 N·m. Deflection at the requirement is ≤ 0.1 mm and ≤ 0.01°.

**Final results, validation, 100 cases** (`results_p3c_final.json`; the 10× noise run is `results_p3c_noise10.json`):

| Metric | Nominal sensors | 10× sensor noise |
|---|---|---|
| grasped / pads lost | 100 / 0 | 100 / 0 |
| detumble time (tumble ≤ 2°/s) median / max | 5.1 / 7.1 s | – |
| mass error median / max | **0.11% / 0.48%** | 1.2% / 3.9% |
| inertia error (relative) median / max | **0.28% / 1.1%** | 1.9% / 4.3% |
| principal-inertia error max | 0.56% | 5.7% |
| centre-of-mass error median / max | **2.5 / 12.6 mm** | 17 / 45 mm |
| off-axis thrust peak attitude error, no compensation | 10.3° | 10.3° |
| … with counter-torque from the estimated centre of mass, median / max | **0.020° / 0.057°** | 0.066° / 0.20° |
| wheel momentum stored, max | **84 N·m·s** | – |

**Reading it:**
- **The grip and stack hold through detumble, identification and thrust.** The servicer learns the unknown client's mass properties well enough that off-axis thrust is almost torque-free once compensated: a 500× smaller attitude error.
- **Most of the work was making the simulated stack rigid,** as on real hardware:
  - brakes on the arm;
  - non-backdrivable jaws;
  - friction without creep;
  - a fixture with form closure.

  Each was found by checking the measured response to known pulses against the true inertia. Without that check, a 15% inertia error would have been blamed on the estimator.
- **Wheel momentum is the next constraint.** 84 N·m·s comes from the detumble (up to about 30), the identification pulses, and mostly from **holding attitude against off-axis thrust** (0.7 N·m × 40 s ≈ 28). A 150 kg servicer's wheels would typically store far less. 3D must thrust through the stack centre of mass, either with thrusters placed or gimballed for it or by moving the arm to put the centre of mass on the thrust line, or dump momentum with thrusters that have torque authority.
- **Not modelled:**
  - wheel bodies and saturation (wheels are ideal torques with momentum only tallied);
  - flexible appendages;
  - stack attitude disturbance torques;
  - bias in gyro or accelerometer (white noise only);
  - identification while tumbling (identification starts after the detumble).

## 8. 3D: reorient and reposition the stack (8 Nov)

**Research for this step:**
- MEV-1 took over the docked stack's propulsion and attitude, using momentum wheels and its thrusters ([Wikipedia](https://en.wikipedia.org/wiki/Mission_Extension_Vehicle), [eoPortal](https://www.eoportal.org/satellite-missions/mev-1)). I found no public detail on how it keeps thrust through the combined centre of mass.
- The standard slew law is rate-limited eigenaxis quaternion feedback, with wheel momentum unloaded by thrusters that have torque authority ([NASA NTRS](https://ntrs.nasa.gov/api/citations/20040105588/downloads/20040105588.pdf), [NPS thesis](https://calhoun.nps.edu/entities/publication/edcbc042-55d5-4f25-b3c0-c71b717e2567)).
- Our servicer's thrusters act at the bus centre, so they cannot unload momentum without spending Δv. The practical answer is to **point the stack** so the burn passes through the combined centre of mass.

**Task (`maneuver3d.py`, `evaluate_p3d.py`, `run_p3d_eval.sh`).** After 3C on the same randomised client:
1. **Translate** the stack centre of mass by a commanded offset (5–15 m, random 3D direction) in LVLH. Use CW targeting every 20 s epoch to arrive at 400 s, with 0.6 N maximum thrust, then keep station.
2. **Reorient** to a commanded LVLH attitude 30–120° away, while keeping station.
3. **Hold** for 60 s.

**Controllers:**
- **Attitude:** rate-limited eigenaxis quaternion feedback, ω_des = sat(kp/kd · e, 0.5°/s), τ = Î kd (ω_des − ω_rel) + ω × Îω + ff. Bandwidth 0.03 rad/s, ζ 0.9, 1 N·m wheels.
- **Translation strategy "align":** first slew so the bus-to-centre-of-mass line lies along the transfer direction; main burns then pass through the centre of mass. Corrections across the line get the counter-torque −d̂ × F.
- **Translation strategy "fixed":** no slew; every burn gets the counter-torque.

**Navigation:**
- star tracker, 0.003° noise;
- the 3C gyro;
- centre-of-mass position from the bus position (2 cm / 0.2 mm/s noise) and d̂.

**Estimates compared:**
- **id:** from 3C;
- **prior:** a nominal 300 kg uniform client with its centre of mass at the box centre, i.e. no identification;
- **truth.**

**Success criteria:**
- at the end of the station-keeping hold: centre of mass within 0.2 m of target and moving < 2 mm/s;
- after a further 30 s coast with attitude control only and no burns: attitude error < 0.5° and rate < 0.01°/s;
- grip held.

**The protocol was revised twice after seeing results**, both times for measurement reasons, not to tune controllers:
1. Rate was first taken at an instant that could be mid-burn, so 40% of cases "failed" at up to 0.024°/s.
2. Velocity was then taken after the coast, but a stack held up to 15 m off the chief drifts under CW dynamics without station keeping (3n²x × 30 s ≈ 1.8 mm/s). Velocity is now judged before the coast and rate after it.

Every version is kept (`results_p3d.json`, `_v2`, `_v3`).

**Results, validation, 40 matched cases per variant** (seeds 300000+, `results_p3d_v3.json`):

| Variant | Success | Final position error med / max | Final velocity med / max | Final attitude error med / max | Peak attitude error during transfer med / max | Peak wheel momentum med / max | Above 30 N·m·s | Δv | 3D time |
|---|---|---|---|---|---|---|---|---|---|
| **id / align** | **100%** | 3.1 / 6.6 cm | 0.76 / 1.4 mm/s | 0.11 / 0.26° | 0.18 / 0.26° | 16.5 / 44 N·m·s | 12.5% | 0.084 m/s | 1080 s |
| id / fixed | 97.5% | 3.5 / 7.3 cm | 0.76 / 2.1 mm/s | 0.10 / 0.21° | 0.22 / **10.2°** | 20.5 / 39 N·m·s | 20% | 0.088 m/s | 830 s |
| prior / align | 80% | **13 / 29 cm** | 0.82 / 1.7 mm/s | 0.19 / 0.90° | **1.6 / 4.7°** | 16.2 / 36 N·m·s | 12.5% | 0.096 m/s | 1130 s |
| truth / align | 100% | 3.1 / 6.6 cm | 0.79 / 1.4 mm/s | 0.12 / 0.26° | 0.16 / 0.21° | 16.4 / 44 N·m·s | 12.5% | 0.085 m/s | 1080 s |

Failures: id/fixed 1 (velocity 2.1 mm/s). prior/align 8: 6 on position, 2 on attitude, 1 on rate.

**Reading it:**
- **The servicer can now "attach and align":** capture, identify, move 5–15 m and turn 30–120°, with every identified case ending within 7 cm and 0.26°.
- **Identification is worth it, and is as good as knowing the truth.** id and truth are indistinguishable. With the nominal prior, success falls to 80%, the final position is 4× worse (the centre of mass is wrongly located, so the stack is steered to the wrong point), and the attitude error during transfer is 10× worse (wrong counter-torque).
- **Aligning the thrust through the centre of mass beats compensating it.** Fixed attitude reaches the targets almost as often, but needs about 25% more wheel momentum. In one case the wheels saturated during a cross-track burn, giving a 10° attitude excursion. Aligning costs an extra slew of about 4 min.
- **Wheel momentum is the binding constraint.** A 0.5°/s slew of a ~1000 kg·m² stack stores about 9 N·m·s, and nothing unloads it: bus-centre thrusters cannot make torque without Δv. 12.5–20% of cases exceed 30 N·m·s, with a peak of 44. Options, none built yet:
  - lower slew rates (slower maneuvers);
  - thrusters with torque authority (offset pairs) for unloading;
  - using the centre-of-mass offset to unload while thrusting, by choosing cross-track correction directions;
  - magnetorquers.
- **Not modelled yet:**
  - wheel saturation (momentum is only tallied);
  - fuel and thruster minimum impulse;
  - long-range relocation (this is relative repositioning of metres; a re-entry burn is a later extension);
  - pose-estimate error for the client;
  - flexible arm or brake slip under maneuver loads (the grip held in every case).

## 9. Momentum management (8 Nov)

**Model changes (`maneuver3d.py`, `Stack3D.actuate`).** The wheels in §8 were ideal external torques with momentum only tallied. Now:
- **Wheels saturate:** torque limit 1 N·m, momentum limit 30 N·m·s per body axis. A saturated wheel cannot add momentum in that direction.
- **Gyroscopic reaction:** the stored momentum h acts on the bus as −ω × h.
- **3D starts with 3C's wheel momentum** (detumble and identification) instead of zero: median 14, max 26 N·m·s.
- **RCS unloading (option):** a pure couple τ_rcs = −k_u h_body (k_u = 0.05/s, 0.5 N·m per axis, i.e. 0.5 N thrusters on a 1 m lever, deadband 0.5 N·m·s). The wheels supply τ_cmd − τ_rcs, so attitude is held while the wheels are driven toward zero. Propellant is computed at Isp 220 s.
- **Attitude law:** gyroscopic compensation now uses the **total** momentum, ω × (Îω + h).
  - Without it, on the first test case the stored 13 N·m·s made both slews fail to converge, with a 7.5° final error.
  - With it, the same case ended at 0.07°.
- The bus torque actuator range in `p3b_scene.xml` was widened to ±3 N·m, so the wheel, RCS and gyroscopic torques are not clamped together. The 3B/3C controllers still clip to 1 N·m.

**Results, validation, 40 matched cases, identified estimates** (`run_p3d_unload.sh`, `results_p3d_unload.json`):

| Variant | Success | Episodes with wheel saturation | Wheel momentum peak med / max | at end med / max | RCS propellant med / max | Final attitude error med / max | 3D time |
|---|---|---|---|---|---|---|---|
| align, no unloading | 95% | 2 (591 s saturated; attitude lost, 111° error) | 20 / 38 N·m·s | 13.9 / 42.9 | 0 | 0.12 / 111° | 1080 s |
| **align + RCS unloading** | **100%** | **0** | 14.2 / 26.1 | **0.57 / 0.81** | **29 / 45 g** | 0.11 / 0.27° | 1080 s |
| align, slower slews (0.2°/s), no unloading | 65% | 2 | 19 / 34 | 16.8 / 40.4 | 0 | 0.11 / 117° | 1460 s |
| fixed + RCS unloading | 97.5% | 0 | 15.3 / 26.3 | 0.59 / 0.82 | 33 / 59 g | 0.10 / 0.22° | 830 s |

Failures:
- **align, no unloading:** 2 cases. Both are wheel saturation leading to loss of attitude.
- **slow slews:** 14 cases. Most hit the 500 s slew limit before converging; 2 also saturated.
- **fixed + unloading:** 1 case on velocity (2.07 mm/s). Fixed attitude still shows a 15° attitude excursion during transfer in one case: the wheel torque limit during cross-track burns, not momentum.

**Reading it:**
- **Unloading is necessary, and cheap.** Without it, the momentum left from capture and identification plus the slews saturates the wheels in 5% of cases. The stack then loses attitude completely.
  - Continuous RCS unloading prevents saturation in every case.
  - It ends with less than 1 N·m·s in the wheels, against up to 43 without unloading.
  - It costs about 30 g of hydrazine per mission (45 g worst case), about 0.02% of the servicer's mass.
- **Slowing the slews is not a substitute.** It lowers the slew-induced momentum only slightly (the 3C momentum is still there) and makes the slews too slow for the time budget (65%).
- **Including wheel momentum in the gyroscopic term is required** once wheels carry real momentum. Otherwise the controller fights −ω × h.
- **align + unloading is the default configuration** (`Man3DCfg.unload = True`).
- **Not modelled:**
  - RCS minimum impulse and pulse-width modulation (the couple is continuous);
  - thruster misalignment, so the RCS couple adds no translation here;
  - wheel friction and speed limits;
  - unloading during 3C itself (3C still uses ideal wheels).

## 10. 3E step E1: a spinning client (8 Nov)

**Research for this step:**
- Concept of operations for tumbling targets: final approach along the target's angular-momentum (spin) axis inside a cone, a separate **angular-rate synchronisation** phase, then capture ([CBF for tumbling-target capture, arXiv 2609.25905](https://arxiv.org/pdf/2609.25905)).
- Synchronisation is limited by actuator authority ([spinning-base capture, arXiv 2402.01959](https://arxiv.org/pdf/2402.01959)). ClearSpace-1 designs for ≤ 3°/s, and observed debris reaches 3.5°/s ([Polimi](https://re.public.polimi.it/bitstream/11311/1146010/1/BOREG01-20.pdf)).
- After capture, the stack's momentum goes into the wheels, with thrusters when it is large ([patent](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/12479603)).
- **Our feasibility:** holding the pre-grasp point 2.7 m from a client spinning at ω needs a centripetal force of m ω² r: 0.12 N at 1°/s, 0.50 N at 2°/s and 1.1 N at 3°/s, against 0.75 N of thrust. **About 2°/s is the ceiling for this servicer.**

**Changes:**
- **Client spin** (`GraspSim.reset(spin_deg_s, spin_axis)`): the client spins at a given rate about a random body axis, on top of the LVLH rate. Principal axes are tilted up to 10°, so off-axis spins wobble.
- **The servicer starts synchronised:** co-rotating at the pre-grasp point with the usual errors, as at the end of a synchronisation manoeuvre. **The fly-in from a spin-axis hold into that state is not built yet.**
- **Synchronised station keeping** (`bus_ctrl`): the reference point moves with the client, v_ref = v_c + ω × r, with centripetal feed-forward m ω × (ω × r). The arm's approach targets were already in the client frame.
- **One wheel model for 3C and 3D** (`StackRun.actuate`): saturation, gyroscopic reaction and RCS unloading now act through detumble, identification and validation too. 3D no longer carries an ideal-wheel tally into realistic wheels. `StackCfg(spin_deg_s, impose_tumble=False)` runs 3C on the stack's own spin.

**Grasp of a spinning client, validation, 100 cases each, free-flying** (`run_p3e_grasp.sh`, `results_p3e_grasp_s{0,1,2}.json`):

| Spin | Success | Other contact | Client push max | Peak contact force | Bar shift in grip max | Slip max |
|---|---|---|---|---|---|---|
| 0°/s | 100% | 0 | 0.82 mm/s | 29.4 N | 2.0 mm | 0.10 mm/s |
| 1°/s | 100% | 0 | 0.89 mm/s | 29.4 N | 6.5 mm | 0.22 mm/s |
| 2°/s | 100% | 0 | 1.01 mm/s | 29.4 N | **11.3 mm** | 0.38 mm/s |

- The "bus attitude disturbance" metric (20–40° with spin) only measures absolute attitude change, and a synchronised servicer rotates by design, so it is meaningless here.
- The bar shift grows with spin: the brief post-capture hold already damps the stack at full wheel torque, which loads the grip. It holds, but the margin shrinks.

**Detumble and identification of the spinning stack, 40 cases each** (`run_p3e_stack.sh`, `results_p3c_{wm_regress,spin1,spin2,spin2_diag}.json`):

| Case | Detumble median / max | Wheel peak per axis max | Saturated | RCS propellant med / max | Mass err max | Inertia err median / max | CoM err median / max | Grip |
|---|---|---|---|---|---|---|---|---|
| imposed tumble ≤ 2°/s (regression, wheel model now in 3C) | 4.7 / 8.6 s | 15.4 N·m·s | 0 | 18 / 30 g | 0.47% | 0.50 / 1.9% | 2.3 / 7.4 mm | held |
| client spin 1°/s | 5.0 / 8.3 s | 10.2 N·m·s | 0 | 16 / 20 g | 0.39% | 0.44 / 1.4% | 2.4 / 7.6 mm | held |
| client spin 2°/s | 4.9 / 9.0 s | 20.2 N·m·s | 0 | 25 / 30 g | 0.34% | 0.42 / 1.4% | 2.2 / 6.0 mm | held* |

\* In 16 of 40 runs at 2°/s, a pad contact dropped for 0.01–0.03 s in total, always in the first 0.02 s after the brakes engaged. That is the model rebuild and state copy resettling the contacts, not grip physics. The grip check now has a 0.05 s grace window after brake engagement. The client stayed attached: the mass is identified to 0.3%, and the bar shifted ≤ 0.8 mm during 3C.

With the wheel model in 3C, identification is slightly worse than with ideal wheels (inertia 0.5% vs 0.28% median), because the RCS couple acts during the excitation. The counter-torque validation gives 0.14° instead of 0.02°. Both are still far inside need.

**Reading it:**
- **A spinning client up to 2°/s is capturable** once the servicer is synchronised. The grasp succeeds in 100% of cases, and the spinning stack is detumbled in ≤ 9 s and identified as accurately as a still one.
- **Wheels plus RCS absorb a 2°/s stack without saturating:** about 25 g of propellant, wheel peak 20 N·m·s.
- **The physical limit is the servicer's thrust for synchronised station keeping** (0.5 of 0.75 N at 2°/s). At 3°/s it cannot hold the pre-grasp point. Faster clients would need more thrust, a pre-capture detumble (e.g. plume impingement), or a capture point on the spin axis.
- **Not done yet:**
  - the synchronisation fly-in itself (from a hold on the spin axis to the co-rotating pre-grasp point);
  - nutating, non-principal tumbles beyond the slight wobble;
  - 3D re-run after the wheel-model refactor;
  - E2 safety monitors;
  - E3 whole-mission Monte Carlo.

## 11. 3E step E1b: fly-in to a spinning client (8 Nov)

**Concept** (from the §10 research: approach along the spin axis, then synchronise):
- A point on the client's spin axis does not move as it rotates, so 3A's guidance can deliver the servicer to a **hold point 8 m out on the spin axis**, inertially still.
- From there `flyin.py` defines a path in the **client's rotating body frame**, using minimum-jerk legs at ≤ 5 cm/s:
  1. along the spin axis to a safe height (2.4 m);
  2. radially out at that height to the pre-grasp point's radius (an inertial spiral);
  3. along the axis to the pre-grasp point.
- If the pre-grasp point is on the axis (spin about the face normal), only the axial leg is flown.
- The station-keeping reference carries the full rotating-frame kinematics: v = v_c + ω × r + R ṙ_b and a = ω × (ω × r) + 2ω × (R ṙ_b) + R r̈_b (Coriolis included), with mass × a as feed-forward.
- **Attitude** blends from the servicer's initial attitude to the synchronised (aligned) one over the first leg, with rate reference ω_c.
- After a 20 s settle the normal 3B grasp sequence runs.
- Code: `GraspCfg(flyin=True, flyin_D, flyin_v, flyin_h_safe)`, `evaluate_p3b.py --spin S --set flyin=true`, `run_p3e_flyin.sh`. Clearance is the servicer bounding sphere (0.87 m) plus a 0.3 m margin against the client box and bar envelope.

**Results, validation, 100 cases per spin rate, random spin axis, free-flying** (`results_p3e_flyin_s{0,0.5,1,2}.json`):

| Spin | Success | Fly-in time median / max | Minimum clearance during fly-in | Thrust saturation | Arrival error max: position / velocity / attitude | Client push max | Bar shift in grip max | Slip max |
|---|---|---|---|---|---|---|---|---|
| 0°/s (along the face normal) | **100%** | 199 / 202 s | 0.60 m | 0% | 0.9 mm / 0.17 mm/s / 0.0003° | 0.47 mm/s | 1.7 mm | 0.08 mm/s |
| 0.5°/s | **100%** | 399 / 404 s | 0.60 m | 0% | 2.8 mm / 0.17 mm/s / 0.01° | 0.49 mm/s | 3.7 mm | 0.14 mm/s |
| 1°/s | **100%** | 399 / 404 s | 0.60 m | 0% | 6.3 mm / 0.26 mm/s / 0.04° | 0.51 mm/s | 6.4 mm | 0.22 mm/s |
| 2°/s | **100%** | 399 / 404 s | 0.60 m | 0% | 19 mm / 0.79 mm/s / 0.15° | 0.58 mm/s | 11.2 mm | 0.37 mm/s |

The fly-in time is about 200 s for the axial case (spin about the face normal, a third of the cases) and about 400 s for the three-leg spiral. The minimum clearance (0.60 m) is reached at the pre-grasp point itself; the path never comes closer.

**Reading it:**
- **The rotating-target approach closes the loop from 3A to the grasp.** The servicer goes from a still hold on the spin axis to synchronised capture of a 2°/s client, in every case, with no thrust saturation and positive clearance throughout.
- **Rotating-frame feed-forward does the work.** The arrival error at 2°/s is 2 cm and 0.8 mm/s, with the same gains as for a still client.
- **The grip margin still shrinks with spin** (bar shift 11 mm at 2°/s), set by the post-capture detumble load, as in §10.
- **Not done:**
  - 3A delivery to the spin-axis hold point with a spinning client: the 3A goal would be the hold point instead of the pre-grasp point, but this has not been run;
  - spin-axis estimation error (the controller uses the true client spin);
  - nutating, non-principal tumbles;
  - E2 monitors;
  - E3 whole-mission Monte Carlo.

## 12. 3E step E2: safety monitors, aborts and injected faults (10 Nov)

**Research for this step:**
- Proximity operations rely on **passive safety first**: a trajectory that stays collision-free if thrust is lost. An **active abort**, triggered by fault detection, covers the rest ([passively safe rendezvous, Goggin & McClain](https://spacefoundation.org/wp-content/uploads/2019/07/Paper-Goggin-McClain-Rendezvous-Mission-Risk-Reduction-Through-Passively-Safe-Orbital-Rendezvous-_0.pdf); [convex passively safe guidance, arXiv 2608.03060](https://arxiv.org/pdf/2608.03060); [MERL abort-safe MPC](https://merl.com/publications/docs/TR2021-054.pdf), which survives thruster failures).
- **ADRAS-J aborted itself twice in 2024** (July, during the fly-around; November, 15 m from the debris), each time on an **unexpected relative attitude anomaly**, and safely backed away ([Astroscale](https://astroscale.com/en/news/astroscales-adras-j-achieves-historic-15-meter-approach-to-space-debris)).
- **Post-grasp work** emphasises limiting interaction forces and keeping the stack configuration safe ([TU Wien](https://repositum.tuwien.at/handle/20.500.12708/223615); [DLR OOS-Sim](https://elib.dlr.de/216816/1/root_copyright.pdf)).

**Monitors** (`safety.py`, `GraspCfg(mon=True)`), run every control step through the fly-in, settle, reach, approach, close and hold phases:

| Check | Signal | Threshold | Nominal max (100 tune runs) |
|---|---|---|---|
| `clear_pred` | predicted clearance of the bus sphere if it coasts 30 s, against the client box and bar rotating at the measured rate | < 0 | never negative |
| `rate_anom` | the client's measured inertial angular velocity minus its model at start (the ADRAS-J trigger) | 0.25°/s | 0.14°/s (0.085 before contact) |
| `thr_resid` | thruster FDI: centre-of-mass acceleration (accelerometer) × mass minus the commanded thrust, 1 s window, only without contact | 0.04 N | 0.003 N |
| `track` | bus position / attitude tracking error, **only when near the client** (clearance < 1.5 m) | 0.25 m / 8° | 3.7 cm / 1.9° |
| `thrust_sat` | saturated thrust > 60% of 5 s, near the client **and** tracking error above half its limit | — | 0 |
| `contact` | any servicer–client contact except the gripper on the bar | any | none |

Thresholds were set on a separate tuning split (seeds 3,000,000+) with the monitors recording only. Two nuisance findings led to the range and tracking gates:
1. **Spin-axis error is a nuisance case, not a hazard.** It trips `track` and `thrust_sat` at the hold point, 8 m out, yet every case captured. Hence the near-client gates on those two checks.
2. **Tracking error catches a stuck thruster only after 15–40 s.** The acceleration residual (`thr_resid`) catches it in under 1 s.

**Abort** (the same in every phase, chosen not to depend on the client spinning as modelled):
1. Open the jaws.
2. If the arm is out, pull the gripper straight back 0.25 m in 2 s, in the inertial frame rather than co-rotating, so a kicked client's bar cannot drag the jaws. Then stow the arm with a minimum-jerk move.
3. The bus stops co-rotating and backs off along the line from the client centre to the servicer, to a point 6 m away that is still relative to the client's centre of mass.
   - **Path:** a cubic Hermite from the current relative state, with duration chosen so the feed-forward stays ≤ 2.5 mm/s².
   - **Why this is safe:** outside the client's swept sphere (client radius 1.1 m plus the servicer sphere), no client rotation can reach the servicer.
4. **On a thruster alarm**, the faulty body axis is isolated from the residual direction, and the bus re-points so that axis lies normal to the back-off plane (the span of the relative position and velocity). The healthy axes then carry the manoeuvre.

**Safe state:** the bus at least 3.2 m from the client centre, arm stowed, jaws open, not closing on the client.

**Injected faults** (one per case, random phase among fly-in, reach and approach, random time and size; `GraspCfg(fault_kind=…)`):
- `client_kick`: the client's rate changes by 0.5–2°/s about a random axis.
- `thruster_weak`: one body axis gives 0–30% of the commanded thrust.
- `thruster_stuck`: one valve stuck open, adding 0.1–0.4 N along +axis.
- `spin_axis_err`: the fly-in is planned with the spin axis wrong by 5–15°.

Every case flies in from the spin-axis hold, with client spin uniform in 0–2°/s. "No monitor" means the same seeds with the monitors recording only, so it also gives each fault's detection rate. "Detect" counts cases where the fault fired.

**Results, validation, 100 cases each** (`results_p3e_safety_val_*.json`; the thruster faults on the fresh val2 split, see below):

| Fault | With monitors (abort) | No monitor | Detected | Detection delay median / max | Time to safe median / max | False alarms |
|---|---|---|---|---|---|---|
| none | **100 captured** | 100 captured | — | — | — | **0** |
| client kick | 97 safe abort, 3 captured, **0 collisions** | 94 captured, **5 collisions** (up to 254 N), 1 failed | 97/97 (`rate_anom`) | 0 / 0 s | 23 / 36 s | 0 |
| thruster weak (val2) | 40 safe abort, 60 captured, **0 collisions** | 80 captured, **3 collisions** (up to 191 N), 17 failed | 40/98 (`thr_resid`) | 1.9 / 207 s | 26 / 38 s | 0 |
| thruster stuck (val2) | 95 safe abort, 5 captured, **0 collisions** | 98 captured, 2 failed; keep-out violated (clearance −0.45 m) | 95/98 (`thr_resid`) | 0.18 / 0.39 s | 24 / 33 s | 0 |
| spin-axis error | 100 captured, no alarm | 100 captured | 0 (none needed) | — | — | 0 |

- **Minimum bus clearance with the monitors on:** ≥ 0.48 m in every row; with no monitor it went to −1.5 m.
- **Faults "fired" but not detected** are a weak thruster with too little commanded thrust on that axis to show a residual. Those runs captured normally.

**How the validation was used:** the first validation pass for `thruster_weak` with the monitors on had 3 collisions and 2 aborts that never reached a safe state. All 5 had a thruster at 0–5% strength, an axis effectively lost, and the abort had assumed full authority. The re-pointing step (4 above) was added after seeing them; it rescues all 5. Because it was designed from validation failures, both thruster faults were re-run on a **fresh split, val2** (seeds 4,000,000+), which gives the table rows above. The kick, spin-axis and nominal rows are unaffected, since re-pointing acts only on thruster alarms.

Three bugs were found and fixed along the way:
- The first "stuck" fault replaced the axis command, which left nothing to steer with; it now adds a push on top.
- An abort during the approach let a kicked bar brush the open jaws about 5 s later; fixed by the gripper retract.
- The contact grace window was too broad; it now applies only to releases from a closed grip.

**Post-capture grip monitor** (`StackCfg(grip_mon=True)`, during the 3C detumble and identification):
- **Signals:** bar shift in the gripper frame since the brakes engaged (> 3 mm), 0.2 s average bar slip speed (> 2 mm/s), or a pad off the bar for ≥ 0.2 s.
- **Fault:** the jaw brakes engage `grip_loose_mm` looser per jaw, i.e. preload loss.

| Looseness (16 cases each, 2°/s spin, no action) | 0 | 0.05 mm | 0.1 mm | 0.2 mm and above |
|---|---|---|---|---|
| client retained | 16 | 16 | 5 | 0 |

The grip lives on well under a millimetre of pad compression.

Responses compared on validation (seeds 210,000+, 50 cases, looseness uniform 0.05–0.3 mm, client spinning at 2°/s; `results_p3e_grip_val_*.json`):

| Response | Retained | Lost |
|---|---|---|
| none | 8 | 42 |
| `safe`: stop all actuation (only the wheels' gyroscopic reaction acts) for 60 s | 24 | 26 |
| **`reclamp`: release the jaw brake, the jaw drive squeezes again (17.6 N), stop actuation** | **50** | **0** (bar shift ≤ 3 mm) |

- **Detection:** the monitor alarmed in 46/50 faulted cases. The other 4 held without help.
- **False alarms:** 0 in 100 nominal runs (50 at 2°/s spin, 50 under 3C's imposed tumble). Nominal maxima were 0.22 mm shift and 0.48 mm/s slip.

**Reading it:**
- **The monitors turn every collision into a safe abort:**

  | Fault | Collisions with no monitor | Collisions with monitors |
  |---|---|---|
  | kick | 5 | 0 |
  | weak thruster | 3 | 0 |
  | stuck thruster | 0, but the keep-out was violated | 0 |

  There were no false alarms, and the time to a safe state is 25–40 s.
- **Thruster FDI by acceleration residual is the key detector:** sub-second, with a 13× margin over nominal noise. Tracking error alone is 100× slower.
- **Abort conservatism costs captures.** With a stuck valve the controller could have overpowered it and captured 98 of 100, but the monitor aborts 95. A next step is **fault-tolerant continuation**: after isolating the fault, estimate the bias, add it as feed-forward and resume. That is a policy choice, and aborting is the conservative default real missions take.
- **After capture, detection alone is not enough.** Safing (stop loading the grip) saves only half; re-squeezing saves all. So the gripper needs an independent drive that can re-close through a slipped brake.
- **Not done:**
  - nav faults (the controller still uses the true client pose; this needs the pose estimator, which Project 4 brings);
  - wheel-saturation and RCS faults during 3D;
  - abort of a failing grip mid-manoeuvre (3D);
  - nutating tumbles;
  - the test split.

## 13. 3E step E3: whole-mission Monte Carlo (10–11 Nov)

**Research:** industry verifies GNC with Monte Carlo campaigns over dispersions, with pass/fail criteria fixed in advance ([NASA JSC proximity-ops MC and covariance](https://ntrs.nasa.gov/api/citations/20160000212/downloads/20160000212.pdf); [lander GNC criteria](https://arxiv.org/pdf/1907.10955)). DLR's end-to-end OOS GNC and robotics simulation is the closest precedent ([DLR OOS](https://www.dlr.de/en/rb/research-operation/portfolio/dlr_rb_portfolio_oos.pdf/@@download/file)).

**Mission chain** (`evaluate_e3.py`):
- **Stage A (GPU):** 3A under u1 to a hold point 8 m out on the client's **angular-momentum axis**.
- **Hand-off:** the arrival error, in the client frame, starts stage B.
- **Stage B:** fly-in → grasp (E2 monitors) → 3C (grip monitor, re-clamp) → 3D.

**Per-case dispersions:**
- client spin 0–2°/s about a random body axis;
- client yaw;
- client mass, inertia and centre of mass (3C ranges);
- start state 20–50 m;
- navigation and actuator uncertainty (u1);
- with probability 0.2, one injected fault.

**Mission success (fixed before running):** the approach arrives; the capture has no unintended contact; 3C keeps the client; 3D ends within 0.2 m, 2 mm/s, 0.5° and 0.01°/s with both pads on the bar.
- **Safe outcomes:** success, a safe abort, or a grip re-clamp that keeps the client.
- **Unsafe outcomes:** a collision, an abort that does not reach safety, or a lost client.

**Findings so far (tuning split only):**
- **The hold point has to be on the angular-momentum axis, not the body spin axis.**
  - The client's spin combined with the LVLH rotation is not about a principal axis, so the body axis nutates. A hold point on it swings about 0.3 m at 8 m.
  - 40/200 approaches never settled within 10 cm and 5 mm/s. On the momentum axis, which is inertially fixed, 199/200 arrive with 0 collisions; median settle 469 s, maximum 2243 s, arrival error ≤ 6.5 cm.
  - A real servicer would estimate this axis as the mean measured rotation axis over a nutation period; here it comes from the true rate and inertia.
- **The E2 `rate_anom` monitor had to change.**
  - E3's clients have principal axes tilted up to 10°, so their angular velocity drifts slowly even with no fault. Comparing against the rate at the start false-aborted 6/20 nominal missions.
  - It now flags a jump of more than 0.1°/s within 2 s, which is about 1000 times faster than the drift. Kicks are still caught instantly (40/40), with 0 false alarms in 40 nominal runs (E2 tuning seeds).
- **`thrust_sat` is now record-only.** It aborted a 1.9°/s mission that, with no abort, succeeded with ≥ 0.58 m clearance.
- **Smoke test, 20 nominal missions:** 19 succeeded and 1 aborted on `thrust_sat` (that trigger is now removed). Final errors ≤ 7.7 cm and 0.18°; mass error ≤ 0.4%; RCS about 25 g; about 3 minutes of wall time per mission per core.

**Tuning pass (200 missions, 11 Nov):**
- **Outcomes:** 179 success, 12 safe aborts, 8 grip re-clamps, 1 approach failure.
- **Success** 89.5% (84.5–93.0%); **safe** 99.5%; **unsafe 0**.
- **Nominal** 168/169; **faulted** 31/31 safe.
- No changes needed. Code frozen as freeze v1 (`e3_freeze_v1.sha256`).

**Validation (1000 missions, freeze v1, 11 Nov):**
- **Outcomes:** 850 success, 100 safe aborts, 46 grip re-clamps, 3 failures in the final 3D move, 1 approach failure.
- **Success** 85.0% (82.7–87.1%); **safe** 99.6% (99.0–99.8%); **unsafe 0**.
- **Nominal** 773/776 (99.6%); **faulted** 221/224 safe.
- **Final error** ≤ 8.9 cm and 0.35°; mass error ≤ 0.56%; fly-in minimum clearance 0.51 m.
- Lower overall success than tuning comes from the fault mix (22% faulted vs 15.5%): faulted missions mostly end in a safe abort by design.
- The run was lost once to a power-off. Stage B now appends each mission to `<out>.partial.jsonl` and resumes from it.

**Weaknesses found in validation, and fixes (freeze v2):**
1. **Grip monitor missed a barely loose grip.**
   - All 3 failures in the final 3D move had `grip_loose` ≈ 0.07 mm, the bottom of the 0.05–0.3 mm range. Every case ≥ 0.09 mm was caught.
   - A replay showed isolated 10 ms pad-contact drops, 2–3 per mission, spread over detumble, ID and 3D. Each is far shorter than `pad_loss_s` = 0.2 s. The pads then slipped in 3D, which fails the success criterion.
   - No healthy grip showed a single drop after the brakes engaged (all 850 successes).
   - **Fix:** a `pad_chatter` trigger. The first separate pad-contact drop of any length trips the grip monitor, so the monitor matches the success criterion.
   - A 2-drop version missed one case once the fly-in changed (only 1 drop).
2. **Tracking false alarms at the top of the spin range.**
   - 2 nominal missions with a client spinning about 2.0°/s about body axis 2 aborted on `track` (0.25 m) during the fly-in.
   - **Diagnosis, step by step:**
     1. The planned feedforward was near the 0.75 N per-axis limit, but the thrust was not saturated while the error grew. So the limit alone was not the cause.
     2. The client nutates: the measured rate rose from 1.98 to 2.19°/s during the fly-in. The fly-in feedforward neglected the client's angular acceleration. With the ẇ × r term added, one case was fixed.
     3. In the other case, on the last (axial) leg the rate vector had wandered from the planning axis. The 2ω × v Coriolis term, which is zero for an axial leg about the true spin axis, pushed the feedforward to 0.96 N and saturated the thrusters.
     4. A one-off leg stretch planned with the start rate cannot anticipate this.
   - **Fixes:**
     - **ẇ × r feedforward,** with ẇ from a 2 s low-passed difference of the measured client rate (no client inertia needed).
     - **Online path-rate scaling.** Once a second, the path clock speed is set to the largest k in {1, 0.85, 0.7, 0.55, 0.4, 0.25} whose feedforward, at the measured rate over the next 20 s, stays within 0.65 N (norm). k is low-passed with a 3 s time constant. The path terms scale by k and k², while the centripetal term is fixed by the geometry.
3. **Checks:**
   - **The 8 affected or reference missions:**
     - all 3 loose grips are now caught and re-clamped;
     - both false-alarm missions now succeed, with k down to 0.25–0.4 and the fly-in up to 586 s instead of 420 s;
     - the spin-axis-error case and 2 high-spin nominal cases still succeed.
   - **Regression, validation seeds 6000000–6000199:**
     - every outcome is unchanged except 6000148, which moved from a 3D failure to a re-clamp;
     - nominal 163/163; safe 200/200;
     - 10/200 fly-ins were slowed.
   - Wall time per mission was about 1.4× higher in this run, including missions that were not slowed. That points to machine load, not the change.
- Freeze v2: `e3_freeze.sha256`.

**Test split:** freeze v2. Stage A ran 1000 (999 at the hold point, 0 collisions). Stage B was cut to the first 300 missions (seeds 7000000–7000299) at 22:25, with 16 done and before any outcome was looked at: this servicer is a stand-in for the ESPA one, and 300 is enough to confirm v2 on unseen missions (0 unsafe would bound the rate at about 1%).

**Test result (v2, 300 missions, `results_e3_test.json`):** success 257 (85.7%, 81.3–89.2%); safe 300/300 (98.7–100%); **0 unsafe**; nominal 244/244; faulted 56/56 safe. Final error ≤ 9.1 cm and 0.33°; mass error ≤ 0.6%; fly-in clearance ≥ 0.53 m; RCS median 26 g; 21/300 fly-ins slowed by the path-rate scaling; all 10 loose-grip re-clamps tripped `pad_chatter` at 0.05 s.

**To do:** re-run the E2 kick and nominal validation rows with the new `rate_anom`.

## 14. Next: an ESPA-class servicer through a robot description layer (prepared, 10 Nov)

**Why:** the user's final models will differ from the test servicer, and a practical servicer should fly as a rideshare secondary. A standard ESPA port allows 61 × 71 × 97 cm and 181 kg ([ESPA](https://en.wikipedia.org/wiki/EELV_Secondary_Payload_Adapter), [Moog](https://moog.com/products/spacecraft-payload-interfaces/espa.html), [USU](https://digitalcommons.usu.edu/smallsat/2018/all2018/414)).

**`p3e_espa_scene.xml`:**
- **Bus:** 0.5 × 0.6 × 0.9 m, 140 kg; the servicer is 150 kg in total.
- **Arm:** 6 joints, shoulder 0.25 m below the face centre, links 0.55 m and 0.45 m.
- **Stowed:** the elbow closes fully (±3.05 rad) and the gripper points down, giving a stowed size of **0.70 × 0.60 × 0.90 m, inside the envelope**.
- **Reach:** the arm reaches the pre-contact and bar poses exactly from 1.35 m when IK starts from a ready pose; the folded pose is singular.
- **Bounding radius:** 0.60 m, set to 0.62 m.
- **Same as before:** client, gripper, thrusters and wheels.
- `check_espa.py` checks all of this.

**`robot_spec.py`:**
- Reads from the model, by name and transmission:
  - free joints;
  - arm and gripper joints;
  - actuator groups (thrusters, bus torques, arm, gripper, load-test rig);
  - limits and masses;
  - sites;
  - client keep-out boxes from the colliding geoms.
- Reads from `<custom><numeric>` in the scene: stow and ready poses, jaw positions, squeeze, pre-grasp distance and bounding radius.
- On `p3b_scene.xml` it reproduces every hard-coded constant. The exception is the feature keep-out box, which comes out 2.3 cm narrower than the hand-written one, so add padding when wiring it in.

### 14.1 Wiring (12 Nov)

- **`GraspSim._apply_spec()`** builds the spec from the scene and takes from it: stow pose, jaw-open pose, IK seed (the scene's ready pose, if given; otherwise IK starts where the arm is, as before), pre-grasp distance, bounding radius (`GraspCfg.servicer_radius = -1` means "from the scene"), thrust limit, arm joint ranges and the keep-out boxes. `safety.py` monitors use `sim.boxes` and `sim.r_serv`. `StackRun` and `Stack3D` inherit all of it through `StackCfg.scene`.
- **The index layout** (free joints, arm, jaws, actuator groups) is still the module constants. `_apply_spec` checks the scene against them and stops with an error if a model differs. Both current scenes have the same layout; a 7-joint arm would need this generalised.
- **Keep-out pad:** the `keepout_pad` numeric (default 0, 2.3 cm, 0) widens the merged feature box along the bar, reproducing the hand-written envelope exactly.
- **Gains from mass and inertia (`GraspCfg.gain_scale`, default on):** the tuned bus gains are taken as the reference servicer's (148.4 kg, bus inertia 22.67 kg·m² per axis). Position gains scale with the mass ratio; attitude torques scale per body axis with the bus inertia ratio, giving the same bandwidth and damping. Ratios within 0.1% of 1 snap to exactly 1, so p3b is bit-identical. ESPA: mass ×1.011, inertia ×(0.60, 0.55, 0.31). The wheel torque limit stays 1 N·m (`tau_bus_max`).
- **`evaluate_e3.py --scene`:** stage A takes the servicer radius from the scene (p3b keeps the frozen settings), and stage B builds every mission on the scene.
- **No-change check on p3b:** `check_spec_wiring.py` shows every derived value equal to the old constants. 20 E3 validation missions (`run_e3_wiring.sh`, chosen by `cmp_e3.py pick` to cover every outcome and fault) re-ran **20/20 identical** to `results_e3_regress.json`, in outcome and every compared metric.

### 14.2 First ESPA runs (12 Nov)

- **Pre-grasp distance 1.35 → 1.25 m.** At 1.35 m the bar was at 96% of the arm's reach: the shoulder sits 0.25 m below the bus centre, and shoulder to bar was 1.13 m against 1.18 m maximum. When the jaws closed, the arm could not comply, and the bus pitched at 1.4°/s with the wheels well below their limit (0.4 N·m). The client was pushed at 3.8 mm/s, against 0.8 on p3b.
- **Variants, 4 still-client grasps each:**

  | Variant | Push | Bus attitude error | Result |
  |---|---|---|---|
  | 1.45 m | 22 mm/s | 35° | 0/4 |
  | 1.35 m | 3.8 mm/s | 3.7° | 4/4 |
  | 1.25 m | 0.9 mm/s | 1.7° | 4/4 |
  | shoulder on the centre line | 0.2 mm/s | 1.4° | 4/4 |

  The scene parameter was changed rather than the shoulder, which the stowed fold needs where it is. The client push is not a gain issue: unscaled gains gave 3.6 mm/s at 1.35 m.
- **Smoke tests at 1.25 m (8 cases each, validation seeds):**
  - still client: 8/8, push ≤ 0.9 mm/s;
  - fly-in and grasp at 2°/s: 8/8, no thrust saturation, arrival error ≤ 7.5 mm, closest approach 0.30 m (the pre-grasp point itself, by design).
- **Lesson:** check the arm's reach margin at the grasp pose, not just whether IK converges. Close to full reach the arm cannot comply along the approach axis, and bus errors go straight into the client.

### 14.3 E3 on the ESPA servicer (`run_e3_espa.sh <split> <n>`, 12 Nov)

| Run | Missions | Success | Safe | Unsafe | Nominal success |
|---|---|---|---|---|---|
| p3b tune (v1, reference) | 200 | 89.5% | 99.5% | 0 | 168/169 |
| **ESPA tune** | 200 | **89.0% (83.9–92.6%)** | **99.5%** | **0** | **167/169** |

- **Stage A:** 199/200 reached the hold point, 0 collisions.
- **Faulted missions:** 31/31 safe.
  - client kick: 5 aborts;
  - weak thruster: 3 aborts, 4 successes;
  - stuck thruster: 4 aborts;
  - spin-axis error: 7 successes;
  - loose grip: 8 re-clamps.
- **Nominal non-successes:**
  - one approach failure (stage A);
  - one safe abort: `clear_pred` during a 1.9°/s fly-in, the same kind as p3b's.
- **Physical results:**
  - final error ≤ 8.2 cm and 0.15°; mass error ≤ 0.47%;
  - RCS median 18.6 g (p3b 26 g): the lighter bus inertia needs less unloading.
  - Fly-in clearance ≥ 0.23 m, the lowest at spins near 2°/s. The design value is 0.30 m at the pre-grasp point, and the figure is after the 0.62 m sphere and 0.3 m margin, so the physical gap is ≥ 0.53 m.
- **Decision:** no tuning changes. Code frozen as ESPA v1 (`e3_espa_freeze.sha256`). Validation is 500 missions (seeds 6000000+), then test 300.

**Validation (500, 12 Nov, 140 min):**

| Run | Missions | Success | Safe | Unsafe | Nominal success |
|---|---|---|---|---|---|
| p3b validation (reference) | 1000 | 85.0% | — | 0 | 773/776 |
| **ESPA validation** | 500 | **84.8% (81.4–87.7%)** | **99.8% (98.9–100%)** | **0** | **395/399 (97.5–99.6%)** |

- **Stage A:** 499/500 reached the hold point, 0 collisions.
- **Faulted missions:** 101/101 safe.
  - client kick 19/19 aborted (`rate_anom`);
  - loose grip 25/25 re-clamped (`pad_chatter`);
  - stuck thruster: 10 aborts (`thr_resid`), 1 success;
  - weak thruster: 13 aborts (12 `thr_resid`, 1 `clear_pred`), 9 successes;
  - spin-axis error: 19 successes, 5 `clear_pred` aborts.
- **Nominal non-successes:** one stage-A failure, and three `clear_pred` safe aborts during the fly-in (6000111, 6000369, 6000455).
- **Difference from p3b:** `clear_pred` trips more often on ESPA. Spin-axis error gives 5/24 aborts (p3b 1/53) and nominal 3/399 (p3b 0 of its 2). The likely cause, not yet checked per mission, is the ESPA servicer's keep-out geometry (its spec radius and padded feature box), which leaves less room in the predicted sweep near 2°/s. Every case is a safe abort, never a contact. **Not tuned on validation:** the margin stays at 0.3 m, and the abort rate is reported as a cost of the ESPA geometry.
- **Physical results:** final error ≤ 7.0 cm and 0.15°; mass error ≤ 0.58%; RCS median 18.0 g; fly-in clearance ≥ 0.23 m.
- **Before the test split:** the p3b identity re-check after adding `step_hook` came out 20/20 identical (12 Nov).

**Test (300, seeds 7000000–7000299, the same seeds as the p3b test; run once on 12 Nov, 65 min, `results_e3_espa_test.json`):**

| Run | Missions | Success | Safe | Unsafe | Nominal success |
|---|---|---|---|---|---|
| p3b test (v2, reference) | 300 | 85.7% | 100% | 0 | 244/244 |
| **ESPA test** | 300 | **84.7% (80.2–88.3%)** | **100% (98.7–100%)** | **0** | **242/244 (97.1–99.8%)** |

- **Stage A:** 300/300 reached the hold point, 0 collisions.
- **Faulted missions:** 56/56 safe.
  - client kick 11/11 aborted;
  - stuck thruster 12/12 aborted;
  - weak thruster: 10 aborts, 1 success;
  - spin-axis error: 10 successes, 1 abort;
  - loose grip: 10 re-clamped, 1 success (the loosening was too small to matter).
- **Nominal non-successes:** two `clear_pred` safe aborts during the fly-in (7000002 at 1.88°/s, 7000135 at 1.22°/s). This is the one systematic difference from p3b, as in validation.
- **Physical results:** final error ≤ 7.1 cm and 0.15°; mass error ≤ 0.64%; RCS median 17.3 g; fly-in clearance ≥ 0.24 m after the margins.
- **Conclusion: the ESPA port is complete.** The robot-description layer and mass/inertia gain scaling carry E3 to a servicer of different shape and inertia with no tuning. Success matches p3b within the confidence intervals, and there is no unsafe outcome in 1000 ESPA missions (tune, validation, test). The cost is a higher `clear_pred` abort rate (about 1% of nominal missions), from the ESPA keep-out geometry.
