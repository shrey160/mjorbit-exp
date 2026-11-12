# What we learned so far

This chapter records what actually happened when the projects were built and trained (6–7 November 2026), as distinct from the plans in the earlier chapters. Every number is measured on held-out validation cases unless marked "test". Full records are in `projects/p1_detumble/SPEC.md`, `SPEC_1B.md` and `projects/p2_approach/SPEC_2.md`.

## Setup that worked

- The simulator runs in WSL2 Ubuntu through pixi environments. The GPU backend (`mjorbit_warp`, float32, many worlds in parallel) trains policies; the CPU backend (float64) checks the physics.
- Throughput on the RTX 4060 laptop GPU: about 4.6k physics steps/s at 256 worlds, rising to 41k at 4096 worlds, using about 1 GB of VRAM.
- The repo's own truss-pointing PPO example trained in 18 minutes and reached 3.3° mean pointing error.
- Live 3D replays in the browser (viser) replaced video recording, which would have needed extra system packages.

## Method rules that paid off

1. **Measure the physics before training.** Check frames, signs and magnitudes against hand calculations. Every project started with a `measure.py` that confirmed what the actuators do and how large the disturbances are.
2. **Build a classical baseline first, and make it strong.** In Project 2 the first baseline (a PD controller) was too weak. Against it, a mediocre policy would have looked like a win. The fair benchmark was CW targeting, the standard orbital-mechanics answer.
3. **Separate validation from test.** Every choice is made on validation (seed 100). The test set (seed 1000) is used once and reported as such, including the one time it was used twice.
4. **Compare on matched cases.** Every controller sees the same start states and the same random errors, so per-case differences are meaningful.
5. **Predict, then check.** Writing a prediction before each experiment exposed wrong mental models: the pointing offset was not reward satisficing, and the fine-precision reward did not leave settle time unchanged.

## Project 1: stabilise, then point

| Stage | Result | Lesson |
|---|---|---|
| 1A detumble, 0.01 N·m | test: PPO 100%, settle 5.15 s, vs damping baseline 100%, 5.2 s | for a problem with a simple optimal law, PPO ties the classical controller; it does not beat it |
| 1A hard, 0.002 N·m | PPO 47% vs baseline 48%; 31 of 200 test cases are physically impossible | check feasibility: saturated damping is already the best greedy torque, so the gap to 100% is physics, not learning |
| 1B pointing, first try | 100% success but a steady 2.8° offset | a network bias created a zero-torque point slightly off target |
| 1B with zoomed error input | final error 1.17°, settle close to baseline | give the network high gain near the goal by adding a magnified copy of the small error |
| 1B with fine-precision reward (final) | test: 100%, final error 0.028°, settle 16.3 s vs baseline 11.9 s | a narrow precision bonus buys accuracy at the cost of speed; neither policy dominates |

Key ideas:
- Free-joint angular velocity is in the body frame, and torques are body-frame.
- A body spinning freely keeps its angular momentum fixed in space while its body-frame rates wander. That gyroscopic coupling is why detumbling is not three independent axes.
- The error quaternion, sign-fixed to the short way round, is a good attitude-error observation.

## Project 2: approach a co-orbiting satellite

**Physics.**
- Relative motion in orbit follows the Clohessy–Wiltshire (CW) equations. A point directly behind the target along its track (V-bar) is a natural resting point: holding there costs no fuel. A radial offset drifts.
- The simulator matched the CW solution to 0.1 mm over 10 minutes. A 1 s physics step was accurate enough, which made training 10× cheaper.

**What failed and why:**
- **Thrusting every second with a fuel penalty.** Exploration noise costs fuel immediately, while the payoff (holding precisely at the end) is hundreds of steps away. PPO collapsed to "never fire".
- **Strong reward shaping.** Rewarding progress toward the goal (potential-based shaping, which leaves the optimal policy unchanged) made learning work. But PPO's advantage estimate only looks about 20 steps ahead, so the shaping still biased the policy toward arriving early, which wastes fuel.
- **A minimum exploration noise.** Keeping the noise from collapsing helped a little. Nothing fixed the long horizon.

**What worked:**
- **One Δv decision every 20 s** (30 decisions per episode, not 600), each flown as a burn followed by a coast. This matches how real rendezvous manoeuvres are flown and makes the fuel trade-off learnable.
- Weakening the shaping once the task was learned, with GAE λ 0.99, let the policy learn to arrive late and save fuel.

| Setting | Best classical | Learned policy |
|---|---|---|
| Ideal (perfect knowledge) | CW targeting: 100%, Δv 0.23 (1 Hz) / 0.29 (20 s epochs) | 100%, Δv 0.32: beats the PD controller, not CW |
| Uncertainty u1 (thrust errors, disturbance, sensor noise), raw measurements | CW 1 Hz: 100% but Δv 2.4 (it chases noise); CW epochs: 83% | trained ideal-only: 98%, Δv 0.36 |
| Uncertainty u1, classical controllers given a Kalman filter | CW 1 Hz + filter: 100%, Δv 0.27; CW epochs + filter that models the finite burns and estimates the bias: 98%, Δv 0.334 (an earlier 77% was a filter bug) | trained under u1: 100%, Δv 0.344; reliability edge only 4 of 200 cases |

Lessons:
- In an idealised world with a known model, the classical optimal-control answer is very hard to beat. RL spends its effort rediscovering it.
- Learning earns its place under uncertainty. Its robustness in the coarse 20 s action space was real.
- A fair comparison under noise needs a navigation filter for the classical controllers. Without one they look artificially bad.
- Policies without memory cannot estimate hidden errors. Next step: give them the filter's estimate, or a recurrent network.
- **Audit your own baseline as hard as the policy.** The epoch-CW benchmark looked weak (77%) because my filter applied each Δv instantly, while the env flies it as a burn of up to 20 s. Modelling the burn as flown raised it to 98% and erased most of the policy's apparent advantage.
- A Kalman filter that also estimates the constant disturbance passed its consistency checks: innovation and error statistics matched theory, and the bias was known to about 14% (median) after 200 s with 1 Hz measurements. One measurement per 20 s was much slower (38%).
- **Residual RL works when the base is strong.** A policy that adds bounded corrections (up to 2 cm/s) to filtered CW targeting, initialised at zero so it starts at CW performance, cut fuel by 6% against CW in the same 20 s action space and was lower in 94% of paired cases, at 99% success. A policy issuing Δv directly never got below CW's fuel use.
- **A safety shield must be feasible for the mission, not just safe.** The textbook test (would we still miss the target if the thrusters failed for 5 minutes?) forbade even sitting at the 3 m hold point, because the unknown disturbance drifts the vehicle about 0.9 m in that time. Every controller dropped to 0% success, with 0 collisions. Checking instead that a backup (stop, then station-keep) can always recover gave 0 collisions and raised success to 100% for CW and the residual policy. It cost about 4% more fuel. The shield acted in about a quarter of the episodes.
- **The shield showed where the danger came from: the guidance.** Every intervention happened in the last 20–40 s, because CW targeting arrives at the hold point still closing at 7–9 cm/s and has to brake hard only 1 m from the keep-out zone. The directly trained policy had learned a slower final approach and never triggered the shield. Real approaches cap closing speed near the target (glideslope guidance) for exactly this reason.
- **Rare failures need large samples.** At 200 cases no unshielded controller collided. At 1000 cases each CW controller collided once. Final test set, 1000 cases, run once: the shielded residual policy got 99.2% success with 0 collisions and used 5.7% less fuel than shielded CW. It did not match shielded CW's reliability (99.5–100%), so the reliability gate was missed.
- **Change the simulator step to find hidden fragility.** With a 0.1 s physics step, the CW-based controllers fell to 91–97% success. CW's fixed arrival at 480 s leaves only about 20 s before the settle deadline. Moving the arrival to 440 s restored 100%. The directly trained policy stayed at 100% because it had learned to arrive earlier.
- **Fix the selection rule before looking at the results.** The rule chose the residual policy on fuel, because the success intervals overlapped. The directly trained policy was the more robust of the two. Both facts are reported, and the test set was used only once.
- **Fix the base before learning on top of it.** After the test set, I replaced the last 3 m of CW guidance with a glideslope that slows the approach as the vehicle nears the hold point. This purely classical controller got 100% success everywhere, used 8% less fuel than the residual policy and 13% less than CW, and never triggered the shield. Most of the residual policy's earlier gain had been compensating for poor terminal guidance.
- **A learned layer can be brittle to inputs it never saw.** A residual policy trained on the glideslope base saved another 7% of fuel under the training conditions. With perfect sensors it failed 40% of episodes, because the filter's covariance inputs fell far outside their training range. Better sensors broke the policy. Randomise what the policy sees, or don't feed it inputs whose scale depends on the sensor.

## How real spacecraft do it, and the next design

Flight rendezvous systems use three layers plus an independent safety monitor:
- **Navigation:** a Kalman filter that also estimates biases and unmodelled accelerations.
- **Guidance:** CW, multi-impulse or glideslope targeting through hold points and approach corridors, choosing passively safe trajectories that would not hit the target if the thrusters failed.
- **Control:** thruster selection and pulse timing; burns are cut off when the accelerometer says the requested Δv has been delivered.
- **Safety monitor:** watches the keep-out zone and closing speed, and can trigger an abort or collision-avoidance manoeuvre.

Learning research typically keeps navigation and safety classical and puts the policy in guidance:
- velocity or Δv commands tracked by a classical controller ("deep guidance");
- recurrent meta-RL to adapt to hidden parameters;
- run-time assurance, a simplex "shield" that checks every learned command and swaps in a verified backup when needed.

**Project 2B plan:**
1. A shared bias-estimating Kalman filter.
2. Classical CW targeting on top of it, as the benchmark.
3. Two learned options: a policy that reads the filter estimate, and a residual policy that learns small corrections to CW targeting, starting from its performance.
4. A safety shield applied to every controller (done 8 Nov: 0 collisions, 100% validation success for the shielded residual policy).
5. A 1000-case Monte Carlo, then one test-set run (done 8 Nov: 99.2%, 0 collisions, −5.7% Δv vs shielded CW).

## Project 3A: approach a Starlink-like satellite (8 Nov)

- **The Project 2 toolkit carried over directly.** On a box-shaped client the filter, glideslope and bias-compensated CW reach a pre-grasp point 1.8 m in front of the capture face. The cases were random client orientation and a dead client turning slowly relative to the orbit frame: 100% success, 0 collisions.
- **Grasp precision needs filtering.** Holding within 10 cm and 5 mm/s on raw measurements failed in about a third of episodes.
- **Path safety is a separate requirement from "no collision".** With small routing margins every episode succeeded, yet 14% passed within 5 m of the capture point from the side. Real approaches use a corridor, so the routing margin was raised until no episode violated it, at about 4% more fuel.

## Project 3B: grasp the grapple bar (8 Nov)

- **With a prepared handle, a classical arm controller grasps reliably:** inverse-kinematics reach, impedance approach, close. It got 100% over 200 randomised cases, both with the servicer holding station and with its thrusters off.
- **The hard part was what happens at and after contact.**
  - At closing, an arm still pushing toward its target and a bus still holding its old reference shoved the client at 6 mm/s.
  - After capture, a bus that keeps station-keeping relative to the client fights the stack it is now part of.
  - Rule: **at contact, freeze every reference; after capture, switch controllers.**
- **Measure the right thing.** A "relative speed" test failed rigidly joined bodies because the stack rotates. Slip at the grip point is the right metric.
- **Free-floating capture costs base attitude:** 6.7° against 1.5° with the servicer holding station. This is why flight systems use reaction-null-space arm motion.

- **Test the simulator's contact model before trusting grasp results.** Under a steady load, MuJoCo's default (pyramidal) friction let the grip creep at about 7 N however hard it squeezed. That hid the effect of clamp force and under-reported strength by up to 4×. Elliptic cones with a higher `impratio` fixed it, and capacity then scaled with clamp force as physics says. With a 17.6 N clamp the grip holds ≥ 8 N and ≥ 3.7 N·m, comfortably above what stabilising the joined stack should need (about 0.5 N and 0.4 N·m).

## Project 3C: stabilise and identify the captured stack (8 Nov)

- **Identify mass properties with momentum, not acceleration.** Integrating Euler's equation over 4 s windows of known wheel and thrust pulses gave the unknown client's stack mass, inertia and centre of mass to about 0.1–0.3% and 2.5 mm. Then off-axis thrust could be compensated almost perfectly: attitude error 0.02° instead of 10°.
- **Most of the error budget was the stack not being rigid.** In order of discovery:
  1. an arm "locked" by software PD that was unstable or too soft;
  2. friction creeping below its limit;
  3. servo-driven jaws acting as springs;
  4. no torsional friction;
  5. a round handle that twists between flat pads.

  Real hardware uses brakes, non-backdrivable grippers and fixtures shaped to lock (form closure). The simulation needed the same. Checking the response to known pulses against the true inertia exposed each one; otherwise a 15% error would have looked like an estimator problem.
- **Off-axis thrust costs wheel momentum.** Holding attitude while pushing a stack whose centre of mass is 1.8 m from the thrusters stored about 28 N·m·s in 40 s. Moving a captured satellite means thrusting through the combined centre of mass, or dumping momentum.

## Project 3D: move the captured satellite (8 Nov)

- **"Attach and align" works end to end in simulation:** capture an unknown client, identify the stack, move it 5–15 m and turn it 30–120°. It succeeded in 40 of 40 cases, within 7 cm and 0.26°.
- **Identification pays for itself.** Using the measured properties was as good as knowing the truth. Assuming a nominal client cut success to 80%: the centre of mass was in the wrong place, so the stack was steered to the wrong point and counter-torqued wrongly.
- **Point the thrust through the centre of mass instead of fighting it.** Slewing so the burns pass through the combined centre of mass used less wheel momentum, and avoided a wheel-saturation excursion that compensating the torque produced.
- **The binding constraint is momentum, not accuracy.** Each slew stores momentum that bus-centre thrusters cannot unload, and some cases exceeded a plausible wheel budget. Real servicers carry torque-capable thrusters or magnetorquers for this.
- **Define "end state" carefully.** My first success test sampled rate mid-burn, the second sampled velocity after a coast that orbital mechanics turns into drift. Each wrong test turned good runs into apparent failures. Judge position and velocity while station keeping, and attitude and rate after the burns stop.

- **Momentum must be unloaded, and that needs torque-capable thrusters.**
  - With realistic, saturating wheels and the momentum left over from detumbling and identification, 5% of missions saturated the wheels and lost attitude completely.
  - A pure RCS couple bleeding the wheels while they hold attitude fixed every case, for about 30 g of propellant.
  - Slowing the slews didn't help enough and made the maneuvers too slow.
  - Once wheels hold real momentum, the attitude law must compensate the gyroscopic term of the total momentum, ω × (Iω + h). Without it, the controller fights the wheels.

## Project 3E, E1: a spinning client (8 Nov)

- **Synchronise first, then grasp.** With the servicer co-rotating at the pre-grasp point (velocity ω × r plus centripetal feed-forward), the same arm controller grasped a client spinning at 1–2°/s in every case. After capture, wheels plus RCS stopped the stack in under 9 s.
- **The limit is thrust, not control.** Station keeping 2.7 m from a 2°/s client already takes two thirds of the servicer's thrust, and 3°/s is out of reach. Faster tumblers need more thrust, a contactless pre-detumble, or a grasp point on the spin axis.
- **Spin costs grip margin.** At 2°/s the bar shifted up to 11 mm in the gripper while the stack was being slowed, against 2 mm for a still client.

- **Approach a spinning target from its spin axis, and plan in its rotating frame.** Points on the spin axis are still. A path drawn in the client's own frame (down the axis, out radially, down to the grasp point) becomes a spiral seen from outside. With the rotating-frame kinematics as feed-forward, including the Coriolis term, the servicer arrived synchronised within 2 cm and 0.8 mm/s at 2°/s and grasped in every case.

## Project 3E, E2: safety monitors and aborts (10 Nov)

- **Detect the fault, not its symptom.** A stuck thruster shows up in tracking error only after 15–40 s. Comparing the accelerometer with the commanded thrust catches it in 0.2 s, with a 13× margin over nominal noise.
- **Gate nuisance checks by range.** Tracking and saturation transients far from the client are harmless, and aborting on them would have thrown away good captures. Checks that matter everywhere (spin anomaly, predicted clearance, thruster residual) stay on everywhere.
- **Make the abort not depend on the model that just failed.** Backing off radially to outside the client's swept sphere and stopping co-rotation works whatever the client's spin is doing. Pull the gripper back first, because the arm is the part inside that sphere.
- **An abort needs authority too.** With one thruster axis nearly dead, the first abort design still collided. Turning the bus so the dead axis lies out of the manoeuvre plane fixed it.
- **Grip faults need a physical recovery, not just a stop.** Stopping the load after slip detection kept half the clients; re-squeezing with the jaw drive kept all of them.
- **Validation hygiene again.** A fix designed from validation failures means a fresh split for the number you report.

## Project 3E, E3: whole mission (10–11 Nov)

- **A spinning body's "spin axis" is its angular momentum, not a body axis.** Add the slow LVLH rotation to a spin and the body axis nutates. A hold point on it swung about 0.3 m and the approach could never settle; on the momentum axis, 199/200 settled.
- **Monitors tuned on idealised targets false-alarm on realistic ones.** A client whose inertia axes are tilted drifts in angular velocity with no fault. A check against the starting rate aborted 30% of nominal missions; a check for a sudden jump separates a kick from that drift by about 1000 times.
- **Chain the whole mission before trusting the parts.** Both problems only appeared once approach, fly-in, capture and stack control ran together on realistic clients.
- **Check for duplicate background runs before launching.** Twice, two copies of a campaign wrote to the same results file.
- **Result:** test split (300 missions, frozen code v2): 0 unsafe, 100% safe, 85.7% success, all 244 nominal missions succeeded. Most of the shortfall in success is by design: faulted missions end in a safe abort.
- **Rare corners only show up at scale.** 200 tuning missions were clean. 1000 validation missions found a grip slightly too loose to trip the pad-loss rule, and false aborts near 2°/s about one body axis. Each was a handful of cases at the edge of a dispersion range.
- **A metric that fails a mission and the monitor that should catch it must agree.** "Any pad loss" failed the mission, yet the monitor waited for 0.2 s of loss. A barely loose grip chattered in single 10 ms drops that the monitor never saw. A healthy grip shows none, so the fix was to make any drop trip the monitor.
- **Check that the actuator is actually saturated before blaming the limit.** The planned thrust was near the limit, but the error grew while the thrust was not saturated. The real cause was a neglected term: the client's angular acceleration while it nutates. Two wrong guesses were caught by replaying 2–8 missions with logging, not by re-running the campaign.
- **Plan against the measured state, not the start state.** A nutating client's rate vector wanders off the planning axis, so a leg that was feasible at planning time became thrust-infeasible later (Coriolis on an axial leg). Scaling the path speed online fixed what a one-off plan could not.
- **Make long runs resumable.** A power-off lost 1 h 10 min of results kept in memory. Stage B now writes one line per finished mission.
- **Size the test to the claim.** 0 unsafe in 1000 bounds the rate at about 0.3%; in 300, about 1%. For a stand-in servicer, 300 was enough. Fix the size before looking at outcomes.

## Porting to the ESPA-class servicer (12 Nov)

- **Read the robot from its description, then prove nothing changed.** Every hand-written constant (stow pose, jaw limits, pre-grasp distance, keep-out boxes, servicer radius) now comes from the scene through `robot_spec`. The proof is bit-identical results on the old servicer, 20/20 missions, before trying the new one.
- **Scale gains by physics, and snap near-1 ratios to exactly 1.** Position gains scale with mass and attitude torque per axis with inertia. Ratios within 1e-3 of 1 become 1, so the old servicer stays bit-identical.
- **A changed geometry can break a hidden assumption.** The ESPA shoulder sits 0.25 m below the bus centre. At the old 1.35 m pre-grasp distance the arm was at 96% of its reach and pushed the client at 3.8 mm/s. Gains, singularity and wheel saturation were each checked and ruled out first. Moving the pre-grasp point to 1.25 m gave 0.9 mm/s.
- **Result:** with no tuning, E3 on the ESPA servicer gave 1000 missions (tune 200, validation 500, test 300) and 0 unsafe. Test success was 84.7% against 85.7% on the old servicer for the same seeds, and nominal 242/244.
- **The cost shows up as aborts, not contacts.** The predicted-clearance monitor trips on about 1% of nominal missions, against nearly none before. A conservative monitor turns a geometry change into lost missions rather than collisions.

## Project 4 preparation (12 Nov)

- **Hide markers before rendering for learning.** A green grip-site marker showed in the first images, which would hand the policy its target.
- **Randomise within the physics of real operations.** Uniform sun directions left half the episodes with the grasp face in shadow. Real approaches are planned with the face lit, so the sun is kept within 70° of the face normal, and one lighting cone is held out for evaluation.
- **A sensor model can erase the target.** Dropping every lidar depth-edge pixel removed the 3-pixel-tall bar. Edge dropout is now 25%.
- **Check a range sensor where nothing can block it.** The first lidar check read 7–13 cm short because the arm sat between the sensor and the bar. With the arm stowed, the reading was 2.3–2.8 cm short, which is the bar's near face.
- **demos_v1:** 500 of 500 expert episodes succeeded, 231,500 frames, 9.7 GB, 22 minutes on 20 workers.

## Toward Project 3 and imitation learning

- Vision-language-action models such as π0.5 (about 3B parameters) learn from demonstrations, not rewards. They are too large to fine-tune on an 8 GB GPU and add little when the exact state is already known.
- Smaller options fit the laptop: ACT (about 80M), Diffusion Policy, Octo (27–93M) and SmolVLA (about 450M). All except Octo are available in Hugging Face's LeRobot library.
- Planned route for Project 3:
  1. Train a state-based expert (RL or the repo's MPPI planner).
  2. Record image demonstrations in simulation.
  3. Distil them into ACT first.
  4. Then compare SmolVLA, to measure what pretraining adds.
