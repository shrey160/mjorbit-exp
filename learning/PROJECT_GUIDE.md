# From MuJoCo to an orbital servicing robot

This is the source of `output/pdf/mjorbit_project_guide.pdf` (build: `python tmp/pdfs/build_guide.py`). It takes a reader from "what is a physics engine" to every design decision and result in this project, as of 12 November 2026.

## How to read this guide

This guide has three parts.

- **Part I, foundations (chapters 2–5):** what MuJoCo is, the space physics you need, what mjorbit adds to MuJoCo, and the control and learning tools used throughout. Read these if any term later on is unfamiliar.
- **Part II, the journey (chapters 6–9):** the learning programme and its method, Project 1 (stop a spin, point), Project 2 (approach a satellite) and Project 3 (capture, stabilise and move a client satellite), in the order they were built, with what failed and why.
- **Part III, where it stands (chapters 10–12):** the ESPA-class servicer and robot description layer, the cross-cutting lessons, and what comes next. The appendices map the code, explain how to run things and define terms.

Every number in this guide is measured on held-out validation cases unless it says "test" or "tuning". The project files are the full record: `CONTEXT.md` (status), `projects/*/SPEC*.md` (designs and results) and `learning/LEARNINGS.md` (lessons).

> **The end goal.** A servicing robot that flies to a client satellite, grasps it by a grapple bar, takes control of the combined "stack", works out the client's unknown mass properties, and then turns and moves the stack to a commanded attitude and position. Everything here builds toward that, one hard problem at a time.

[[fig:mission|The whole mission as it runs today, one Monte Carlo case per row of the E3 campaign.]]

## MuJoCo: a physics engine for robots

### What a physics engine does

A physics engine answers one question over and over: given where everything is, how fast it moves and what forces act on it, where will everything be a moment later? Repeating that tiny step thousands of times gives a simulation.

For robots, the hard parts are:

- **Articulated bodies.** An arm is a chain of rigid links connected by joints. Pushing on the hand moves every link; the engine must solve the coupled equations of motion.
- **Contact.** When a gripper pad touches a bar, the bodies must not pass through each other, and friction must resist sliding. Contact switches on and off abruptly and is the main source of numerical trouble.
- **Speed.** Learning and Monte Carlo testing need millions of steps, so each step must be cheap.

**MuJoCo** (Multi-Joint dynamics with Contact) is a free, open-source engine built for exactly this. Originally from Emo Todorov's lab and now maintained by Google DeepMind, it is the standard simulator in robot learning research. This project uses MuJoCo 3.7.0.

### The model: MJCF

A MuJoCo world is described in an XML file called **MJCF**. The main elements:

| Element | What it is | Example in this project |
|---|---|---|
| `body` | a rigid body with a pose, nested to form a tree | `chaser` (servicer bus), `upper` (upper arm), `target` (client) |
| `joint` | how a body moves relative to its parent | `freejoint` (6 DoF, for a spacecraft), `hinge` (arm joints), `slide` (gripper jaws) |
| `geom` | a shape for collision and mass | the bus `box`, arm `capsule`s, pad `box`es, the square grapple bar |
| `site` | a named marker point, no physics | `grip_site` between the jaws, `bar_site` at the bar centre |
| `inertial` | explicit mass, centre of mass and inertia | the randomised client in 3C |
| `actuator` | something that applies force through a joint | `motor` (thrusters, wheels, arm torques), `position` (jaw servo) |
| `custom/numeric` | free-form named numbers the model carries | stow pose, pre-grasp distance (the robot description layer) |

The body tree matters: a child body's pose is defined relative to its parent, and a joint says how the child may move. The servicer is a `freejoint` body (the bus) whose descendants are the six arm links and two jaws; the client is a separate `freejoint` body. **Nothing connects the two**: capture happens only through contact forces between pads and bar.

```
<body name="chaser">            <- servicer bus, free to fly
  <freejoint/>
  <geom type="box" size="0.5 0.5 0.5" mass="136"/>
  <body name="arm_base">        <- arm link 1
    <joint type="hinge" axis="0 0 1"/>   <- shoulder yaw
    ...
<actuator>
  <motor joint="chaser_joint" gear="1 0 0 0 0 0" ctrlrange="-0.75 0.75"/>  <- x thruster
```

### Model and data

MuJoCo splits a simulation into two objects:

- **`mjModel`**: everything constant — geometry, masses, joint limits, actuator gains, solver settings. Compiled once from MJCF.
- **`mjData`**: everything that changes — positions `qpos`, velocities `qvel`, controls `ctrl`, contact list, computed forces, body poses `xpos`/`xmat`.

`qpos` and `qvel` are **generalized coordinates**: one number per joint degree of freedom, not a Cartesian pose per body. A free joint takes 7 numbers in `qpos` (position plus a unit quaternion) but 6 in `qvel` (linear velocity plus angular velocity), because a quaternion has one redundant component. The arm's hinges add one number each. MuJoCo then computes every body's world pose from them (forward kinematics).

One detail caused real confusion early on and is worth fixing in memory: **a free joint's angular velocity in `qvel` is in the body frame**, while its linear velocity is in the world frame.

### One time step

`mj_step` advances the simulation by `timestep` seconds. Inside it:

[[fig:mjstep|What happens inside one MuJoCo step.]]

1. **Kinematics:** world poses of all bodies and geoms from `qpos`.
2. **Collision detection:** find pairs of geoms that touch or nearly touch, producing contacts.
3. **Dynamics:** the mass matrix M(q), Coriolis and gravity terms, actuator forces.
4. **Constraint solve:** contacts, joint limits and equality constraints become constraint forces, found by a convex optimisation.
5. **Integrate:** update `qvel` and `qpos` with the chosen integrator.

The equation behind it is the rigid-body equation of motion, M(q) v′ + c(q, v) = τ + J<sup>T</sup> f, where v is `qvel`, the prime marks a time derivative, τ is applied force (actuators) and J<sup>T</sup> f is the constraint force (contacts and limits) mapped through the constraint Jacobian J.

### Contact in MuJoCo: soft constraints

MuJoCo's contacts are **soft**: a contact may penetrate very slightly, and the constraint force behaves like a stiff, damped spring that the solver computes exactly for each step. Two parameters shape it:

- `solref` = (time constant, damping ratio): how fast penetration is corrected. Smaller time constant means stiffer.
- `solimp`: how the constraint's "impedance" varies with penetration depth.

Other contact settings that ended up mattering a great deal in this project:

- **`condim`** — the contact's dimensionality. 3 means normal force plus 2D sliding friction; **4 adds torsional friction** (resistance to twisting about the contact normal). The grip on the bar needed 4.
- **Friction cones.** Coulomb friction says the friction force must stay inside a cone around the normal. MuJoCo's default approximates it as a **pyramid**; the **elliptic** cone is the true shape. With `impratio` above 1, friction is made stiffer relative to the normal force. Grasping needs elliptic cones with `impratio` ≈ 10.
- **`noslip_iterations`** — an extra post-solve pass that removes the small residual slip that soft friction allows under steady sideways load.
- **`contype` / `conaffinity`** — bitmasks deciding which geoms may collide. Here the gripper geoms (contype 2, conaffinity 4) and the client geoms (4, 2) collide only with each other; the servicer's bus and arm links have no contact at all, so any servicer–client contact other than the gripper is caught by a separate check.

> **Key lesson (3B, 3C).** The simulator's default contact settings under-reported grip strength by up to 4× and made a rigidly clamped stack measurably springy. Before trusting any contact result, test the contact model itself against a known load.

### Integrators and the timestep

The integrator turns accelerations into new velocities and positions. MuJoCo offers semi-implicit Euler, `implicitfast` (implicit in velocity-dependent forces such as damping, which makes stiff joints stable) and RK4. This project uses `implicitfast` with a 0.002 s step for anything with contact, and 1 s steps for contact-free orbital approach, where accuracy was checked against the analytic solution. A smaller step is more accurate and slower; **every contact result was checked at more than one timestep**.

### Plain MuJoCo vs. mjorbit in this project

mjorbit (next chapters) wraps MuJoCo's native model, but its CPU data object exposes only state arrays (`qpos`, `qvel`, `xpos`, `xmat`, `ctrl`). The grasp controller needs Jacobians, site poses and contact forces, so `GraspSim` keeps a **plain-MuJoCo copy of the same model**, syncs `qpos`, `qvel` and `ctrl` into it each control step, and asks it for those quantities (`mj_jacSite`, `mj_contactForce`). Forgetting to sync `ctrl` once made all contact forces read zero.

## The space physics you need

### Why things float: free fall

A spacecraft at 400 km altitude feels about 90% of surface gravity. It "floats" because it is falling all the time: its sideways speed (7.7 km/s) is so high that it falls around the Earth instead of into it. Everything inside falls together, so nothing presses on anything. Two consequences drive the whole project:

- **Nothing stops by itself.** Switch off a thruster and the spacecraft keeps drifting; braking costs as much fuel as accelerating.
- **Relative motion is subtle.** Two satellites near each other are both falling, but on slightly different orbits, so they drift apart in curved, non-intuitive paths.

### Frames

[[fig:frames|The three frames used throughout: absolute ECI, mjorbit's chief-centred world, and the rotating LVLH frame.]]

- **ECI (Earth-centred inertial):** origin at Earth's centre, axes fixed relative to the stars. Orbits are computed here.
- **The chief and the local world:** mjorbit propagates one reference orbit, the **chief**, and puts MuJoCo's world origin on it with axes parallel to ECI. The robots' `qpos` are metre-scale offsets from the chief, which keeps numbers small.
- **LVLH (local vertical, local horizontal):** a frame that rotates with the orbit. x points radially out (R-bar), y along the direction of travel (V-bar), z along the orbit normal. It turns once per orbit (0.064°/s at 400 km). Relative-motion guidance is easiest here, but converting velocity into it needs the rotation term ω × r.
- **Body frames:** fixed to each spacecraft. Wheel torques and the gyro live here.

### Relative motion: the Clohessy–Wiltshire equations

For two spacecraft a few kilometres apart or less on a near-circular orbit, the relative motion in LVLH is described by the linear **Clohessy–Wiltshire (CW)** equations (n is the orbit rate):

x″ − 2n y′ − 3n² x = a<sub>x</sub>;   y″ + 2n x′ = a<sub>y</sub>;   z″ + n² z = a<sub>z</sub>  (primes are time derivatives)

Their useful consequences:

- **A point on V-bar (ahead or behind along track) is an equilibrium.** Holding there costs no fuel. This is why approaches end at a V-bar hold point.
- **A radial offset drifts.** Something placed slightly above the target falls behind; below, it pulls ahead.
- **CW targeting:** because the equations are linear, the velocity needed to coast from here to a target in time T can be solved in closed form. This is the classical way to plan a burn and the benchmark every learned approach controller had to beat.

### Attitude: how bodies rotate

A rigid body's rotation follows Euler's equation, I ω′ + ω × (I ω) = τ, where I is the inertia matrix, ω the angular velocity in the body frame and τ the applied torque. Key consequences:

- **Angular momentum H = I ω is conserved in inertial space** when no torque acts. ω in the body frame can wander while H stays fixed. That gyroscopic coupling (the ω × Iω term) is why stopping a spin is not three independent one-axis problems.
- **Spin about a principal axis is steady; spin about any other axis nutates.** The body's spin axis cones around the fixed momentum axis. This mattered in E3: a hold point on the client's body spin axis swung by 0.3 m, while one on its momentum axis stays still.
- **Attitude is represented by a unit quaternion.** The error between current and desired attitude is another quaternion; its vector part, sign-fixed to take the short way round, is a good error signal for both PD control and neural networks.

### Actuators: thrusters, wheels and momentum

- **Thrusters** push mass out. They change the spacecraft's velocity (Δv) and cost propellant, m<sub>prop</sub> ≈ m Δv / (I<sub>sp</sub> g<sub>0</sub>). Δv is the standard measure of fuel cost. A thruster pushing through the centre of mass gives pure translation; off-centre it also makes torque.
- **Reaction wheels** spin up internally to turn the spacecraft the other way. They need only electricity but **store momentum**: every N·m·s of disturbance absorbed accumulates in the wheels until they **saturate** at their speed limit and lose authority in that direction. Unloading the stored momentum needs an external torque, usually thruster pairs (RCS) or magnetorquers.
- Stored wheel momentum h also acts back on the bus as a gyroscopic torque −ω × h, which the attitude controller must compensate.

### Docking and capture vocabulary

| Term | Meaning here |
|---|---|
| servicer | our robot spacecraft: bus, arm, gripper |
| client | the satellite to capture: a rectangular bus with a grapple bar |
| prepared client | one fitted with a fixture made for capture (our grapple bar); unprepared = no fixture |
| pre-grasp point | where the servicer bus parks before reaching with the arm |
| keep-out zone | a volume around the client the bus must never enter |
| free-flying / free-floating | bus actively controlled during capture / bus actuators off, so the arm's reaction turns the base |
| stack | servicer and client joined after capture |
| Δv | total velocity change from thrust, a fuel measure |

## mjorbit: MuJoCo in orbit

### What it adds

mjorbit (Zhang, Verhagen, Vega, McKeen and Manchester, Carnegie Mellon, 2026; arXiv 2609.08010) extends MuJoCo with **orbital dynamics and spacecraft physics**, so an articulated, contacting robot can be simulated on a real orbit:

- **A propagated reference orbit (the chief)** in ECI, in km and km/s, with the robot world centred on it in metres.
- **Differential gravity:** each body feels the gravity at its own position minus that at the chief. Because the offsets are metres against orbits of thousands of km, a naive subtraction loses all precision; mjorbit uses the **Encke formulation**, algebraically the same but without the cancellation. The simulation steps this nonlinear model, not the linear CW equations, which is why CW makes a fair independent check.
- **Environment loads (switchable):** J2 (Earth's oblateness), gravity-gradient torque, atmospheric drag, solar radiation pressure and magnetic torque, each needing explicit surface or magnetic models. This project switched them off for clarity, after measuring that gravity gradient was negligible at these sizes (about 2.6 × 10<sup>-7</sup> N·m in Project 1).
- **Spacecraft actuators and sensors:** reaction wheels, magnetorquers, thrusters and control-moment gyros, with noisy sensors on the CPU side.
- **Two backends:** `mjorbit` (CPU, float64, the reference) and `mjorbit_warp` (GPU via NVIDIA Warp and MuJoCo Warp, float32, thousands of worlds in parallel for learning).
- **Tools:** a browser viewer, an MPPI planner, examples (free drift, arm reach, docking, capture, PPO tasks) and the paper's experiments.

[[fig:arch|mjorbit's layers: MJCF plus an orbit block compiles to a model; each data object steps MuJoCo physics plus orbit coupling.]]

The workflow mirrors MuJoCo's: `MjoModel.from_xml_path(...)` → `model.make_data(orbit=OrbitInit(...))` → `mjo_step(model, data)`. The `<mjorbit>` element in the scene XML switches the environment terms on or off.

### This installation

| Item | Version or setting |
|---|---|
| repository | github.com/johnzhang3/mjorbit, revision a0909067, unmodified |
| host | Windows 11 laptop, Ryzen AI 9 HX 370, RTX 4060 Laptop GPU (8 GB) |
| runtime | WSL2 Ubuntu; environments from pixi, stored on the Linux filesystem (detached) for speed |
| CPU environment | Python 3.12.13, MuJoCo 3.7.0, C++ plugin built with Ninja |
| GPU / RL environment | PyTorch 2.13 (CUDA 13.0), Warp 1.15, mujoco-warp 3.7.0.1, rsl-rl 5.4.1 |
| checks | API tests 8 passed; GPU tests 41 passed, 3 skipped; free drift matches CW to 7.8 μm over 60 s |
| GPU throughput | about 15.5k steps/s at 1024 worlds, 41k at 4096, about 1.1 GB VRAM (contact-rich task) |

Installation lessons: building on the Windows-mounted drive was far too slow, so environments live on the Linux side; CMake needed `CMAKE_GENERATOR=Ninja`; the lockfile is kept as committed (`--locked`).

### How this project uses the two backends

- **GPU (`mjorbit_warp`)**: PPO training in Projects 1 and 2, and every batched approach run, including E3's stage A (200–1000 approaches at once).
- **CPU (`mjorbit`)**: physics checks, and all the contact-rich capture work (3B–3E), run as many independent cases in parallel worker processes (20 at a time).

## The control and learning toolkit

This chapter introduces each method once. Later chapters say where and why it was used.

### Feedback control

- **PD control:** command = −K<sub>p</sub> × error − K<sub>d</sub> × rate. The workhorse for station keeping, joint control and attitude.
- **Rate-limited eigenaxis slew:** turn about the single axis that connects current and desired attitude, with the commanded rate capped so the wheels do not saturate. Used for pointing (Project 1) and stack reorientation (3D).
- **Feed-forward:** add the force you know will be needed (centripetal force when co-rotating with a spinning client, the counter-torque from a known centre-of-mass offset) instead of waiting for feedback to notice the error.

### Guidance and navigation

- **CW targeting** (chapter 3) plans each coast arc.
- **Glideslope:** close on the target at a speed proportional to the remaining distance (here 0.03 × range, clipped), so arrival is slow and safe.
- **Kalman filter:** blends noisy measurements with a motion model to estimate the true state and its uncertainty. The **9-state filter** used from Project 2 on estimates relative position, velocity and an unknown constant disturbance acceleration (bias), which a 6-state filter cannot see.

### Robot arm control

- **Jacobian:** the matrix J that maps joint velocities to the hand's linear and angular velocity.
- **Inverse kinematics (damped least squares):** iterate Δq = J<sup>T</sup>(J J<sup>T</sup> + λ² I)<sup>-1</sup> e to find joint angles that put the hand at a target pose; the damping λ keeps it stable near singular poses.
- **Impedance control:** make the hand behave like a spring-damper around a target (here K = 300 N/m, D = 60 N·s/m), so contact produces gentle, bounded forces instead of fighting a rigid position command.
- **Free-floating base:** with the bus uncontrolled, every arm motion turns the bus (momentum is conserved). Reaction-null-space motion is the classical answer; noted, not yet built.

### System identification

After capture the client's mass, centre of mass and inertia are unknown. **Least squares** fits them from measured responses to known pulses. Writing Euler's equation in momentum form, I Δω + ∫ω × Iω dt = ∫τ dt, integrates the gyro instead of differentiating it, which keeps noise small.

### Reinforcement learning

- **PPO** (Proximal Policy Optimisation) trains a neural-network policy by trial and error in thousands of parallel simulated worlds, nudging it towards actions that earned more reward, with a clip that stops any single update from changing the policy too much. Implementation: rsl-rl.
- **Reward shaping:** extra reward for progress. Potential-based shaping leaves the optimal policy unchanged in theory, but with a short effective horizon it still biases behaviour.
- **Residual RL:** the policy adds a small correction on top of a classical controller instead of replacing it.
- **Curriculum:** start with easy cases, widen the range as the policy improves.
- **Imitation learning** (Project 4): learn from demonstrations instead of reward. LeRobot provides ACT and Diffusion Policy.

### Safety

- **Shield (explicit simplex):** before executing an action, predict its consequence and fall back to a certified safe action if it could violate the keep-out zone. Built in Project 2.
- **Monitors and aborts (FDIR, fault detection, isolation and recovery):** watch signals that reveal faults, and on an alarm fly a pre-planned escape to a safe state. Built in E2.

### Evaluation method

- **Splits:** tuning seeds (adjust thresholds, gains), validation seeds (choose between designs), test seeds (report once, never used for choices). Fresh splits (val2) are used when a design was changed after seeing validation failures.
- **Matched cases:** every controller sees exactly the same start states and random errors, so per-case comparisons are fair.
- **Monte Carlo with confidence intervals:** run hundreds to a thousand random cases and report rates with 95% Wilson intervals. 0 failures in 200 cases still allows a true rate of up to about 1.9%.
- **Success criteria fixed before running.** Revisions are allowed only for measurement mistakes, and every version is kept.

## The programme and its method

### Goal and progression

The user (comfortable with Python and AI, new to robotics and RL, 10+ hours per week, on a laptop) wanted to understand space robotics and train their own controllers in mjorbit. The programme was set up on 6 November 2026 as three projects, each adding one hard thing:

1. **Stabilise:** stop a spinning spacecraft, then point it (rotation).
2. **Approach:** fly to a nearby satellite and hold position (relative translation in orbit).
3. **Capture:** reach, grasp and control a client satellite (articulated contact, then the coupled stack).

On 8 November the end goal was made concrete (grasp a rectangular client by a grapple bar, then reorient or reposition it) and Project 3 grew into five steps, 3A–3E.

### The working loop

Each milestone ran the same loop: **predict the motion, build a small experiment, measure the physics, explain the result, then make it harder.** Five rules came out of the first two projects and held throughout:

1. **Measure the physics before training.** Every project began by checking frames, signs and magnitudes against hand calculations.
2. **Build a strong classical baseline first.** A weak baseline makes a mediocre policy look good.
3. **Keep validation and test apart,** and report every use of the test set.
4. **Compare on matched cases.**
5. **Write a prediction before each experiment.** Wrong predictions exposed wrong mental models.

Research came before each stage: missions (MEV, ClearSpace-1, ADRAS-J, Orbital Express, ETS-VII, RSGS) and papers were read and recorded in `learning/RESEARCH_SERVICING.md` before designing.

## Project 1: stop a spin, then point

### Task

A 10 kg box spacecraft (inertia 0.108 / 0.242 / 0.283 kg·m²) with ±0.01 N·m body torques at 10 Hz. **1A detumble:** from a random spin up to 0.3 rad/s per axis, bring the rate to near zero within 30 s. **1B pointing:** turn to a requested orientation (up to 180° away) and hold.

### Results

| Stage | Result | Lesson |
|---|---|---|
| 1A detumble | test: PPO 100%, settles in 5.15 s; damping baseline 100%, 5.2 s | for a problem with a simple optimal law, PPO ties the classical controller |
| 1A hard (0.002 N·m) | PPO 47% vs baseline 48%; 31/200 test cases are physically impossible | check feasibility: saturated damping is already the best greedy torque |
| 1B, first policy | 100% but a steady 2.8° offset | not reward hacking: the network's bias made a zero-torque point off target |
| 1B, zoomed error input | final error 1.17° | give the network a magnified copy of small errors |
| 1B final (fine-precision reward) | test: 100%, error 0.028°, settle 16.3 s vs baseline 11.9 s | precision costs speed; neither dominates |

### What it taught

- Body-frame rates wander while momentum stays fixed: the gyroscopic coupling is real and visible.
- **A learned policy cannot beat an optimal simple law; it can only match it.** Learning earns its place where the classical answer is weak or unknown.
- The quaternion error vector is a good observation; networks need extra gain near the goal.
- Practical: GPU training of a curriculum took about 9 minutes at about 30k steps/s.

## Project 2: approach a co-orbiting satellite

### Task

A 10 kg chaser with ±0.05 N thrusters starts 20–50 m from an inert 100 kg target in a 400 km orbit and must reach a hold point 3 m behind it on V-bar, holding within 0.25 m and 0.01 m/s for the last 60 s of a 600 s episode, without entering a 2 m keep-out sphere. The learning target is **fuel (Δv)**, since a simple PD controller already succeeds 100%.

### What failed

- **Thrusting every second with a fuel penalty:** exploration noise costs fuel immediately while the payoff is hundreds of steps away; PPO collapsed to "never fire".
- **Strong shaping** made learning work but biased the policy to arrive early, wasting fuel.
- **Raw navigation measurements:** 1 Hz controllers chased noise and burned 1.5–2.4 m/s.

### What worked

- **Manoeuvre-epoch actions:** one Δv decision every 20 s (30 per episode, not 600), each flown as a burn and a coast, like real rendezvous. The fuel trade-off became learnable.
- **The 9-state bias filter** for navigation under uncertainty (thrust gain ±10%, misalignment ≤ 3°, constant disturbance, 5 cm / 1 mm/s noise: the **u1** preset).
- **A shield** that certifies each burn against the keep-out sphere (2 m plus 3σ) by predicting the burn, a coast and a backup stop.
- **A glideslope hybrid** (bias-compensated CW to 3 m out, then glideslope): 100% under every condition on a fresh split, Δv 0.280 — the best controller in this action space, classical.

### Final numbers

| Controller (u1 uncertainty) | Success | Δv (m/s) |
|---|---|---|
| test, 1000 cases: residual PPO r1 + shield | 99.2% (98.4–99.6), 0 collisions | 0.303 |
| test: shielded CW + filter benchmark | 99.5% | 0.322 |
| validation: direct PPO | 100% in every validation run, never needs the shield | 0.344 |
| fresh split: glideslope hybrid + shield | 100% (also with no uncertainty and dt 0.1 s) | 0.280 |

### What it taught

- **The shield matters at scale:** unshielded CW collided 1–2 times per 1000 cases; no shielded run ever collided.
- **Structure beats raw learning:** the action space (epochs), the filter and the guidance law did more than network size.
- Learning helped most where it found a better fuel trade-off around a good classical base, and it was fragile out of distribution (a residual trained with noise failed when the noise was removed).

The tools from Project 2 — filter, glideslope, CW targeting, shield — carried directly into Project 3.

## Project 3: capture and move a client satellite

### The two spacecraft

[[fig:robots|Top view of the servicer (p3b) at the pre-grasp point and the client with its grapple bar, drawn to scale.]]

**Servicer (`p3b_scene.xml`):**

- About 150 kg in total: a 1 m cube bus (136 kg) plus about 12 kg of arm and gripper.
- **Thrusters:** 0.75 N per world axis on the bus free joint (about 5 mm/s²).
- **Reaction wheels:** 1 N·m per body axis, 30 N·m·s momentum limit, with gyroscopic reaction; **RCS** couples of 0.5 N·m to unload them.
- **Arm:** 6 revolute joints on the client-facing face — shoulder yaw and pitch, elbow, and a spherical wrist (pitch, yaw, roll). Upper arm 0.70 m, forearm 0.55 m. Torque motors 20 / 20 / 15 / 5 / 5 / 5 N·m.
- **Gripper:** parallel jaws, two flat pads on opposing slides, position-servoed and force-limited at 30 N. The standard squeeze gives a 17.6 N clamp.
- **Brakes:** after capture, arm joints and jaws are pinned (modelled as joint ranges collapsed onto the captured angles).
- **Sensors (modelled):** gyro, accelerometer, star tracker and relative navigation, each with noise.

**Client (`p3a_scene.xml`, `p3b_scene.xml`):**

- A generic rectangular satellite, 1.5 × 1.0 × 1.0 m, nominally 300 kg. In 3C and later its mass (200–400 kg), centre of mass (±10 cm) and inertia (±20%, axes tilted up to 10°) are **randomised and unknown** to the controller.
- A **grapple bar** on its −x face: 0.30 m long, a 5 × 5 cm square section, standing 0.15 m off the face on two posts — a handrail-type handle sized for a two-finger gripper. The square section (not round) was chosen in 3C for form closure.
- Passive: it may drift, rotate at the orbit rate, or spin up to 2°/s about a random axis.

### The roadmap

| Step | What it adds | Status |
|---|---|---|
| 3A | approach to the pre-grasp point around a box-shaped keep-out | done, 100% |
| 3B | arm reach and grasp with real contact | done, 100% |
| 3C | stabilise the stack and identify the client's mass properties | done |
| 3D | reorient and reposition the stack, with momentum management | done, 100% |
| 3E E1 | spinning client: synchronised grasp, fly-in from the spin axis | done, 100% up to 2°/s |
| 3E E2 | safety monitors, aborts, injected faults | done, 0 collisions |
| 3E E3 | whole-mission Monte Carlo | test 300: 0 unsafe, 100% safe, 85.7% success |

### 3A: approach around a box

**Problem.** Project 2's target was a sphere; the client is a box with a bar sticking out, and the servicer must arrive at a point 1.8 m in front of the bar, inside a 15° approach corridor about the face normal, without clipping the corners. The client may face any direction (random yaw).

**Design.** Each 20 s epoch: inside the corridor, glideslope straight in; outside, head for a corridor entry point 10 m out, and if the straight path would pass within 4 m of the keep-out, go via a waypoint on a 12 m safety sphere instead. Transfers use bias-compensated CW arcs; navigation is the 9-state filter.

**Results (200 validation cases each).** 100% success with 0 collisions and 0 corridor violations under u1 and with no uncertainty, for aligned and random yaw; 99.5% for a client fixed in inertial space (its face turns relative to LVLH). Δv about 0.47–0.58 m/s. **On raw measurements only 68% succeed** (all hold failures), so the filter is essential at grasp precision.

### 3B: reach and grasp the bar

[[fig:grasp|The grasp sequence. Everything after the reach is expressed in the client's frame, so it tracks client motion.]]

**Controller, at 100 Hz:**

1. **Settle (2 s):** the bus keeps station relative to the client.
2. **Reach (20 s):** inverse kinematics (damped least squares) to a pre-contact pose 12 cm in front of the bar, aligned with it; a minimum-jerk joint move.
3. **Approach:** Cartesian impedance at the grip point, blending in over 5 s and then sliding forward at 1 cm/s.
4. **Close:** within 1 cm of the bar, **freeze** both the impedance target and the bus reference, then close the jaws.
5. **Hold (30 s):** thrusters off; wheels damp the stack.

**Results (200 cases each).** 100% success for free-flying and free-floating, at 0.001, 0.002 and 0.005 s steps, with no unintended contact and a push on the client below 1 mm/s. Free-floating disturbs the bus 6.7° against 1.5° free-flying.

**Bugs that taught something:** the first controller pushed the client away at 6 mm/s on contact (targets still advancing — hence the freeze); after capture the bus kept station-keeping on the now-attached client (thrusters off after capture); and the first success test compared the two bodies' speeds, which a rotating rigid stack fails without any slip (now slip of the bar relative to the gripper).

**Grip load test.** An internal wrench, equal and opposite on the two bodies, ramps until the bar slips. With MuJoCo's default pyramidal friction, slip began at 6–9 N **whatever the clamp force** — a simulator artefact. With elliptic cones (`impratio` 10) capacity scales with the clamp; at the 17.6 N clamp the grip holds at least 8.3 N and 3.7 N·m in every direction, against an estimated need of 2 N and 1.5 N·m.

### 3C: stabilise and identify the stack

After grasping a randomised, unknown client, braking the arm and imposing a tumble of up to 2°/s, the servicer:

1. **Detumbles** with the wheels (≤ 7 s).
2. **Identifies rotation:** ±0.5 N·m wheel pulses about each axis; least squares on the momentum form of Euler's equation over 4 s windows gives the stack's six inertia parameters.
3. **Identifies translation:** ±0.4 N thrust pulses; the accelerometer response and torque balance give the mass and the bus-to-centre-of-mass offset.
4. **Validates:** thrusts across the centre-of-mass line with and without the counter-torque computed from the estimate.

**Results (100 cases):** mass error 0.11%, inertia 0.28%, centre of mass 2.5 mm (medians). The counter-torque cuts the attitude error during off-axis thrust from 10.3° to 0.02°, a 500× improvement. With 10× sensor noise: about 1–2% and 17 mm.

**Most of the work was making the simulated stack rigid,** exactly as on hardware. A PD "lock" on the arm shook a pad loose, so the arm got **brakes**; the servo-driven jaws acted as 400 N/m springs, so the jaws got **non-backdrivable locks**; soft friction crept, so `noslip_iterations` went to 20; contacts could not resist twist, so `condim` went to 4; and a round bar twisted between flat pads, so the bar became **square**. Each problem was found by comparing the identified inertia with the true one under known pulses. Without that check, a 15% error would have been blamed on the estimator.

### 3D: reorient and reposition the stack

**Task.** Move the stack's centre of mass 5–15 m in a random direction (CW targeting, 400 s transfers) and turn it 30–120°, then hold, ending within 0.2 m, 2 mm/s, 0.5° and 0.01°/s.

**Key design choice — align, don't fight.** The thrusters sit at the bus centre, not at the stack's centre of mass, so every burn makes a torque. Strategy **align** first slews the stack so the bus-to-centre-of-mass line lies along the transfer direction; burns then pass through the centre of mass.

| Variant (40 cases) | Success | Final error | Note |
|---|---|---|---|
| identified properties, align | 100% | 3.1 cm, 0.11° (median) | identical to knowing the truth |
| identified, fixed attitude + counter-torque | 97.5% | similar | 25% more wheel momentum; one 10° excursion |
| nominal prior (no identification) | 80% | 13 cm | steered to the wrong point; 10× attitude error in transfer |

**Momentum management.** Once wheels were given real limits (30 N·m·s) and gyroscopic reaction, momentum left from capture and identification plus the slews saturated the wheels in 5% of cases, and the stack lost attitude completely. Continuous **RCS unloading** (a 0.5 N·m couple driving wheel momentum to zero) fixed every case for about 30 g of propellant per mission. Slowing the slews instead did not help (65%). The attitude law also had to include stored wheel momentum in its gyroscopic term, ω × (Îω + h).

### 3E step E1: a spinning client

**Feasibility first.** To hold the pre-grasp point (2.7 m from the client's centre) while co-rotating at rate ω, the servicer needs centripetal force m ω² r: 0.12 N at 1°/s, 0.50 N at 2°/s, 1.1 N at 3°/s, against 0.75 N of thrust. **About 2°/s is this servicer's ceiling,** set by physics, not control.

**Synchronised station keeping** moves the reference with the client (v = v<sub>c</sub> + ω × r) and feeds forward the centripetal force. The grasp then works unchanged because its targets were already in the client frame: **100% at 1 and 2°/s**, and the spinning stack is detumbled in ≤ 9 s and identified as well as a still one. The bar shifts up to 11 mm in the grip at 2°/s, so the grip margin shrinks with spin.

**E1b, the fly-in.** A point on the spin axis does not move as the client rotates, so the approach (3A) can deliver the servicer to a still **hold point 8 m out on the axis**. From there the servicer flies a path defined in the client's rotating frame: down the axis to a safe height, a radial spiral out to the pre-grasp radius, then in along the axis. The reference carries the full rotating-frame kinematics, including Coriolis terms, as feed-forward.

[[fig:flyin|The E1b fly-in, seen in the client's rotating frame (left) and as an inertial spiral (right).]]

**Results (100 cases per rate):** 100% at 0, 0.5, 1 and 2°/s; 200–400 s fly-in; clearance never below 0.60 m; no thrust saturation; arrival within 2 cm and 0.15° at 2°/s.

### 3E step E2: safety monitors, aborts and faults

**Why.** Astroscale's ADRAS-J aborted itself twice in 2024 on unexpected relative-attitude anomalies and backed away safely. A servicer must detect trouble and escape on its own.

**Monitors** (`safety.py`), every control step:

| Monitor | Watches | Trips at |
|---|---|---|
| predicted clearance | if the bus coasts 30 s from now, does it hit the rotating client? | < 0 |
| client rate jump | a jump in the client's inertial rate within 2 s (the ADRAS-J trigger) | 0.1°/s |
| thruster residual | measured acceleration × mass minus commanded thrust, 1 s window, no contact | 0.04 N |
| tracking error | bus position / attitude error, only within 1.5 m of the client | 0.25 m / 8° |
| unexpected contact | any servicer–client contact except gripper on bar | any |
| grip slip (after capture) | bar shift > 3 mm, slip > 2 mm/s, or a pad off the bar ≥ 0.2 s | — |

**Abort sequence** (the same in every phase):

[[fig:abort|The abort sequence. Each step was added because a validation or tuning case needed it.]]

**Injected faults:** a client kick (rate change 0.5–2°/s), a weak thruster axis (0–30% of command), a stuck-open valve (0.1–0.4 N extra), a spin-axis knowledge error (5–15°) and, after capture, a loose jaw brake (preload loss).

[[fig:e2bars|E2 validation: collisions per 100 faulted cases without and with the monitors.]]

| Fault (100 cases) | With monitors | No monitor | Detected |
|---|---|---|---|
| none | 100 captured, 0 false alarms | 100 captured | — |
| client kick | 97 safe aborts, 3 captured, 0 collisions | 94 captured, 5 collisions (up to 254 N) | 97/97 instantly |
| weak thruster | 40 safe aborts, 60 captured, 0 collisions | 80 captured, 3 collisions, 17 failed | 40/98 |
| stuck thruster | 95 safe aborts, 5 captured, 0 collisions | 98 captured, keep-out violated | 95/98 in < 0.4 s |
| spin-axis error | 100 captured, no alarm | 100 captured | not needed |

**After capture, detection is not enough.** With a loose brake, doing nothing kept 8 of 50 clients and "safing" (stop all loading) kept 24; **re-clamping** (release the brake, let the jaw drive squeeze again) kept all 50.

**What E2 taught:**

- **Thruster fault detection by acceleration residual is the key detector:** under a second, with a 13× margin over nominal noise. Tracking error alone takes 15–40 s.
- **Aborting is conservative.** The stuck-thruster cases could mostly have been captured by overpowering the fault; fault-tolerant continuation is a later extension.
- **Validation hygiene:** the bus re-pointing step was added after validation failures with a near-dead thruster, so both thruster faults were re-run on a fresh split.

### 3E step E3: the whole-mission Monte Carlo

**The chain, per case:**

- **Stage A (GPU, batched):** approach from 20–50 m under u1 to a hold point 8 m out on the client's **angular-momentum axis**.
- **Hand-off:** the arrival error, in the client's body frame, becomes stage B's start.
- **Stage B (CPU, 20 workers):** fly-in → grasp with the E2 monitors → 3C with the grip monitor and re-clamp → 3D.

**Dispersions:** client spin 0–2°/s about a random axis; client yaw; client mass, inertia and centre of mass; start 20–50 m away; u1 navigation and actuator errors; and, with probability 0.2, one injected fault.

**Outcome classes (fixed before running):** *safe* = success, safe abort, or re-clamped grip; *unsafe* = collision, an abort that never reached safety, or a lost client; *neutral* = an approach, grasp or 3D that failed without danger.

**Two discoveries during tuning:**

1. **The hold point must sit on the angular-momentum axis, not the body spin axis.** A client spinning in an orbiting frame is not spinning about a principal axis, so its body axis nutates; a hold point on it swings by 0.3 m at 8 m, and 40 of 200 approaches never settled. On the momentum axis, which is fixed in space, 199/200 arrive with 0 collisions.
2. **The client-rate monitor had to look for jumps, not drift.** Nutating clients drift slowly with no fault, and comparing against the start rate false-aborted 6 of 20 nominal missions. A jump of 0.1°/s within 2 s still catches every kick instantly (40/40), with 0 false alarms in 40 nominal runs. The saturation monitor became record-only after it aborted a mission that succeeds without the abort.

**Tuning pass (200 missions, 11 November):**

[[fig:e3bars|E3 tuning outcomes, 200 missions.]]

| | Count | Rate (95% interval) |
|---|---|---|
| mission success | 179 | 89.5% (84.5–93.0%) |
| safe outcomes | 199 | 99.5% (97.2–99.9%) |
| unsafe | **0** | |
| nominal missions succeeding | 168 of 169 | the 1 miss was the approach not settling |
| faulted missions ending safe | 31 of 31 | 11 still completed the mission |

Final errors at most 7.6 cm and 0.28°; client mass estimated within 0.5%; closest approach during fly-in 0.55 m; median propellant about 27 g; about 5 minutes of computer time per mission per core. Nothing needed fixing, so the code was frozen (freeze v1, file hashes in `projects/p3_capture/e3_freeze_v1.sha256`).

**Validation (1000 missions, freeze v1):** 0 unsafe; safe 99.6% (99.0–99.8%); success 85.0% (82.7–87.1%); nominal 773 of 776. Success is lower than in tuning because more missions drew a fault (22% against 15.5%), and faulted missions mostly end in a safe abort by design. The run also showed two weaknesses, each in a handful of cases at the edge of a dispersion range:

1. **A barely loose grip went unnoticed.** All 3 missions that failed in 3D had the smallest injected looseness (about 0.07 mm). A replay showed single 10 ms pad-contact drops, 2–3 per mission, each far shorter than the 0.2 s pad-loss rule; the pads then slipped during the move. No healthy grip had a single drop. **Fix:** any pad-contact drop now trips the grip monitor (`pad_chatter`), which matches the success criterion.
2. **False aborts near 2°/s.** Two nominal missions aborted on tracking error during the fly-in. The first guess, a path that needs more than the 0.75 N thrust limit, was wrong: the error grew while the thrust was not saturated. The client nutates (its rate rose from 1.98 to 2.19°/s), and the feedforward left out its angular acceleration. Adding that term fixed one case. In the other, on the axial leg the rate vector had wandered off the planning axis, and the Coriolis term pushed the needed thrust to 0.96 N. **Fixes:** feed forward ẇ × r, with ẇ from the filtered measured rate, and **slow the path online**. Once a second, the path speed is set to the largest factor (1 down to 0.25) whose feedforward over the next 20 s fits 0.65 N.

Both fixes were checked on the affected missions (all now safe or successful) and on 200 validation missions, where nothing else changed. The code was then re-frozen (v2, `e3_freeze.sha256`). Stage B now also saves each mission as it finishes, after a power-off lost a run.

**Test split (freeze v2, run once):** stage B was cut to the first 300 missions before any outcome was seen. This servicer is a stand-in, and 300 bounds the unsafe rate at about 1%.

[[fig:e3test|E3 test outcomes, 300 missions on the frozen v2 code.]]

| | Count | Rate (95% interval) |
|---|---|---|
| mission success | 257 | 85.7% (81.3–89.2%) |
| safe outcomes | 300 | **100%** (98.7–100%) |
| unsafe | **0** | |
| nominal missions succeeding | 244 of 244 | |
| faulted missions ending safe | 56 of 56 | kicks and stuck thrusters all aborted safely; 10 of 11 loose grips re-clamped |

Final errors at most 9.1 cm and 0.33°; client mass within 0.6%; fly-in clearance at least 0.53 m; median propellant 26 g; 21 of 300 fly-ins were slowed by the online scaling.

## The ESPA-class servicer and the robot description layer

### Why a new servicer

The servicer so far is a test vehicle; the final models will differ. A practical servicer should launch as a **rideshare secondary** on a standard ESPA ring port, whose envelope is 61 × 71 × 97 cm and 181 kg.

[[fig:espa|The ESPA-class servicer: arm folded flat for launch inside the 61 × 71 × 97 cm envelope (side view).]]

**`p3e_espa_scene.xml`:**

- Bus 0.5 × 0.6 × 0.9 m, 140 kg; 150 kg in total — 31 kg of margin.
- One 6-joint arm (0.55 m + 0.45 m links, 1.18 m from shoulder to grip point), shoulder 0.25 m below the face centre so it can fold.
- Stowed: elbow fully closed (±3.05 rad), gripper pointing down: **0.70 × 0.60 × 0.90 m, inside the envelope**.
- Pre-grasp distance 1.25 m (bar to bus centre); the arm reaches the pre-contact and bar poses exactly. It was first 1.35 m, but there the arm worked at 96% of its reach and pushed the client at 3.8 mm/s on closing; at 1.25 m the push is 0.9 mm/s. The folded pose is singular for inverse kinematics, so a separate "ready" pose seeds it.
- Same gripper, thrusters, wheels and client.

### The robot description layer

Until now, controller code held hard-coded facts: which `qpos` indices are the arm, which actuators are thrusters, the stow pose, the bounding radius. Changing the robot would mean editing dozens of places. `robot_spec.py` reads them from the model instead:

- **By structure:** free joints by body name; arm joints as the hinges in the servicer's subtree; gripper joints as its slides; actuators grouped by what they drive (translational gear on the bus = thrusters, rotational = wheels, arm, gripper, test rig).
- **From `<custom><numeric>` entries in the scene:** stow and ready poses, jaw positions, squeeze, pre-grasp distance, bounding radius.
- **Derived:** masses, limits, and the client's keep-out boxes from its colliding geoms.

On the old servicer it reproduces every hand-written constant once the bar's keep-out box is padded by 2.3 cm (a `keepout_pad` entry).

### Wiring, gains and the E3 re-run (12 Nov)

- **Wired in:** the grasp, safety and mission code now read the robot from `robot_spec`. On the old servicer, 20 of 20 E3 missions came out bit-identical.
- **Gains from physics:** position gains scale with mass, and attitude torque per axis with the bus inertia. A ratio within 0.1% of 1 is snapped to 1, so the old servicer is unchanged. For ESPA the inertia ratios are 0.60, 0.55 and 0.31.
- **E3 on ESPA, with no tuning:**

| Run | Missions | Success | Safe | Unsafe | Nominal success |
|---|---|---|---|---|---|
| Tune | 200 | 89.0% | 99.5% | 0 | 167 of 169 |
| Validation | 500 | 84.8% | 99.8% | 0 | 395 of 399 |
| Test (same seeds as the old servicer's test) | 300 | **84.7%** | **100%** | **0** | **242 of 244** |

The old servicer's test gave 85.7% and 244 of 244 nominal. The one systematic difference is the predicted-clearance monitor, which stops about 1% of nominal fly-ins on ESPA as a safe abort; it is left conservative rather than tuned on validation. Final errors at most 7.1 cm and 0.15°, client mass within 0.64%, median propellant 17 g (the lighter bus needs less wheel unloading).

## Cross-cutting lessons

1. **Check the simulator before trusting it.** Friction creep, springy jaws and missing torsional friction were all simulator settings, not physics, and each would have been misread as a controller or estimator problem.
2. **Classical first, and make it strong.** In every project so far, a well-designed classical controller set the bar; learning matched it (P1), improved fuel around it (P2), or was not needed (P3).
3. **Structure the problem for the physics.** Manoeuvre epochs instead of 1 Hz thrust; rotating-frame paths with feed-forward; a hold point on the momentum axis; thrust aligned through the centre of mass.
4. **Know the physical limits.** Feasibility checks (31 impossible detumble cases, the 2°/s synchronisation ceiling, wheel momentum budgets) separated "the controller is bad" from "it cannot be done".
5. **Freeze, hold, release.** At contact, stop pushing; after capture, stop station keeping; on an abort, open the jaws and pull back in the inertial frame.
6. **Detect fast, from the right signal.** An acceleration residual finds a stuck thruster in under a second; tracking error takes 40.
7. **Responses must be able to act.** Detecting grip slip saved half the clients; re-clamping saved all of them.
8. **Keep score honestly.** Tuning, validation, fresh validation and a test set used once; every revision of a protocol kept and explained.

## What comes next

### E2 follow-up

E3 is complete. One loose end: re-run the E2 kick and nominal rows with the new rate-jump monitor before quoting the E2 numbers as final.

The E2 per-fault rows can also be re-run on the ESPA servicer.

### Project 4: see and learn the grasp

Spec in `projects/p4_vision/SPEC_4.md`. Built so far (12 Nov): a camera scene, randomised rendering, a flash lidar model, and 500 expert demonstrations (`demos_v1`, all successful).

- **Sensors:** a wrist camera and a bus camera (128 × 128 RGB) and a bus flash lidar (64 × 64 range), as on NASA's Raven. Sun direction, materials, mounts, exposure and noise are randomised per episode, with one lighting cone held out for evaluation.
- A **pose estimator** for the bar (P4.3), so the classical grasp on an estimated pose is the fair baseline for the learned policy.
- **Demonstrations** from the classical controller, then **imitation learning** with LeRobot (ACT or Diffusion Policy first; a vision-language-action model such as SmolVLA only if language commands are wanted).
- **Cartesian end-effector actions**, so the learned policy is not tied to one arm.
- The **E2 monitors stay on as the shield** around the learned policy.

### Later extensions

Nutating tumbles beyond a slight wobble; faster clients; fault-tolerant continuation instead of always aborting; RCS minimum-impulse modelling; unprepared clients (launch-adapter rings, adhesion, caging).

## Appendix A: code map

| Path | What it holds |
|---|---|
| `simulator/` | the upstream mjorbit repository, unmodified |
| `projects/p1_detumble/` | Project 1: `detumble_env.py`, `pointing_env.py`, `train.py`, specs |
| `projects/p2_approach/` | Project 2: `maneuver_env.py`, `nav_filter.py`, `shield.py`, `guidance.py`, `evaluate.py` |
| `projects/p3_capture/p3a_scene.xml`, `capture_env.py`, `capture_guidance.py` | 3A approach (GPU), also E3 stage A (`goal_mode="hold"`) |
| `p3b_scene.xml`, `grasp_sim.py` | servicer, arm, gripper, client; fly-in, grasp, monitors, faults, aborts |
| `stack_sim.py` | 3C: brakes, detumble, identification, grip monitor, re-clamp |
| `maneuver3d.py` | 3D: `Stack3D`, slews, CW transfer, wheel model, RCS unloading |
| `flyin.py` | E1b rotating-frame fly-in path |
| `safety.py` | E2 monitors (`Monitor`), abort path (`AbortRef`) |
| `evaluate_p3*.py`, `evaluate_e3.py` | evaluators for each step and the whole mission |
| `p3e_espa_scene.xml`, `check_espa.py`, `robot_spec.py` | ESPA servicer and the robot description layer |
| `export_replay.py`, `replay/index.html` | three.js browser replay |
| `SPEC_3.md` | the full Project 3 record |

## Appendix B: how to run things

All runs go through WSL from the simulator root, for example:

```
wsl -d Ubuntu -- bash -c "cd /mnt/c/Shrey_Projs/robotics/mjorbit/simulator && bash ../projects/p3_capture/run_e3.sh validation 1000 20"
```

- E3: `run_e3.sh <split> <n> [workers]`; E2: `run_p3e_safety.sh`, `run_p3e_grip.sh`; single cases: `run_dbg_e2.sh <seed> <fault>`.
- ESPA checks: `run_check_espa.sh`, `run_robot_spec.sh`.
- Put shell loops in `.sh` files, strip Windows line endings, and check `ps` for leftover runs before launching anything.
- The mjorbit viewer: `pixi run --locked viewer --task free_drift --no-track` (port 8080).
- The replay page: export with `run_export_replay.sh`, then republish `replay/index.html`.

## Appendix C: glossary

| Term | Meaning |
|---|---|
| CW | Clohessy–Wiltshire relative-motion equations |
| ECI / LVLH | Earth-centred inertial frame / local rotating orbit frame |
| V-bar / R-bar | along-track / radial direction in LVLH |
| Δv | total velocity change from thrust; a fuel measure |
| DoF | degree of freedom |
| MJCF | MuJoCo's XML model format |
| qpos / qvel | generalized positions / velocities |
| impratio | MuJoCo ratio of frictional to normal constraint stiffness |
| condim | contact dimensionality (3: sliding friction; 4: plus torsion) |
| PPO | Proximal Policy Optimisation, the RL algorithm used |
| u1 | the uncertainty preset: thrust errors, disturbance bias, navigation noise |
| FDIR | fault detection, isolation and recovery |
| RCS | reaction control system: small thrusters used for torque |
| nutation | the coning of a body's spin axis around its fixed momentum axis |
| ESPA | EELV Secondary Payload Adapter, a rideshare ring with standard ports |
| stack | servicer plus captured client |
| Wilson interval | a confidence interval for a success rate that behaves well near 0% and 100% |

## Appendix D: sources

- mjorbit: J. Z. Zhang et al., *mjorbit: A Simulation Framework for Space Robotics*, arXiv 2609.08010 (2026); johnzhang3.github.io/mjorbit.
- MuJoCo documentation: mujoco.readthedocs.io (modelling, contact, solver parameters).
- Missions and capture research: `learning/RESEARCH_SERVICING.md` (MEV, MRV/RSGS, ClearSpace-1, ADRAS-J, Orbital Express, ETS-VII, ELSA-d).
- Monte Carlo verification practice: NASA JSC proximity-operations Monte Carlo and covariance analysis (NTRS 20160000212).
- ESPA envelope: EELV Secondary Payload Adapter user information (Moog; Utah State SmallSat 2018).
- Project records: `CONTEXT.md`, `projects/*/SPEC*.md`, `learning/LEARNINGS.md`.
