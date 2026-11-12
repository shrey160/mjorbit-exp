# Three project briefs

These are proposed learning projects. Numerical thresholds are initial design targets; see [evaluation](EVALUATION.md). The source simulator already provides examples, but custom Project 1 and Project 2 RL tasks still need implementation later. Do not mistake an MPPI demonstration for an existing trained RL environment.

## Shared mission and interfaces

Use one simple chaser design and one passive target wherever practical. Choose modest mass/inertia and clearly bounded actuators after checking whether each proposed manoeuvre fits the episode duration. A 10 kg chaser and a 5 kg target are reasonable illustrative starting masses, not required specifications. Document geometry and compute inertia consistently with it. Increase target mass only after the light-target capture works.

Use a circular Earth reference orbit near 400 km. Begin with simulator-provided state. Keep task/control rates separate from physics timesteps: an initial candidate is 10 Hz control with 0.01 s physics steps, subject to contact validation. Smaller timesteps must be assessed for accuracy and throughput, not assumed universally better.

Normalize policy inputs and map normalized actions to physical limits explicitly. The action contract includes frame, sign convention, units, bounds, and holding period. The observation contract includes relative pose, velocity convention, and any actuator memory required to predict the next state.

Initially use ideal bounded body forces and torques as an educational actuator model. A signed force along an axis is not one physical thruster: physical thrusters push in their installed direction with nonnegative command. Later use an explicit thruster arrangement/allocation. mjorbit provides wheel and thruster models with limits; wheel commands exchange momentum with the bus and have finite storage when limits are configured. [Actuators](https://johnzhang3.github.io/mjorbit/docs/actuators-sensors.html)

Keep optional perturbations simple initially; log exactly which are active. Disabling drag/J2/SRP flags does not automatically turn off all orbit coupling. [Model configuration](https://johnzhang3.github.io/mjorbit/docs/modeling.html)

## Project 1: detumble, then point

### Question

Can a policy stop a randomly spinning spacecraft efficiently, then keep a requested orientation?

Detumbling means reducing angular velocity. It does not determine final orientation. Treat pointing as a second milestone.

### Task contract

| Item | Initial definition |
|---|---|
| System | One rigid spacecraft, no arm or target contact |
| Observation | Body angular velocity; previous action. Add orbital/environment information if needed for enabled disturbances |
| Pointing extension | Add sign-consistent orientation error relative to requested attitude |
| Action | Three bounded body torque commands through the declared ideal model |
| Reset | First one-axis spin, then random three-axis spin, initially up to 0.1 rad/s per axis; widen toward 0.3 only after authority checks |
| Episode | Initially 30 s, longer if the physical torque/inertia combination needs it |
| Baseline | Saturated angular-rate damping; orientation-plus-rate feedback for pointing |
| Training signal | Low angular rate, modest effort and action changes; add orientation accuracy only for pointing |
| Main measurements | Time to settle, residual rate, attitude error, command saturation, torque effort |

With disturbances disabled, angular-rate observations are adequate for an ideal spherical-inertia rate-control exercise. For asymmetric inertia, randomized hidden properties, actuator dynamics, or orientation-dependent disturbances, revisit observation sufficiency rather than calling an incomplete state fully observed.

### Training stages

1. Verify zero-command coast, axis/sign conventions, and one torque pulse against expected angular acceleration.
2. Make the damping baseline succeed across feasible initial spins.
3. Train one-axis detumbling, then three-axis detumbling.
4. Add attitude holding with an initially small orientation error, then wider errors.
5. Add modest inertia variation, disturbances, and observation noise one at a time.
6. Optional realism: replace ideal torques with actual wheels or torque-producing thrusters. Include wheel speed/momentum in observations, configure limits, and ensure capacity is sufficient for initial angular momentum. Wheels store momentum; they cannot absorb unlimited spin or sustained external torque indefinitely.

### Failure modes worth studying

- Rate decreases, but there is no pointing accuracy: correct detumbling, incomplete pointing task.
- Alternating maximum commands: poor tuning, saturation, exploration, or control-frequency issues.
- Reward rises while settling slows: effort terms may outweigh the task.
- Performance disappears with a different inertia: memorized dynamics or an unobservable randomization.

### Deliverable

A baseline-versus-policy comparison, angular-rate plots, a separate attitude-hold result, and a written explanation of the actuator model. There is no requirement that PPO beat damping on this simple task.

## Project 2: approach a nearby co-orbiting satellite

### Question

Can the chaser reduce relative separation, match relative velocity, align appropriately, and hold at a specified offset without contact?

The first version starts metres away in the same orbital neighbourhood. Transfers across distant orbital planes/altitudes require separate guidance and are outside this initial task. Rendezvous/proximity work includes collision-avoidance and relative-navigation concerns; here we use known simulator state to isolate control. [NASA overview](https://www.nasa.gov/reference/jsc-rendezvous-prox-ops-docking-subsystems/)

### Task contract

| Item | Initial definition |
|---|---|
| System | Chaser plus passive target; neither is fixed in the inertial world |
| Goal | Hold a target-relative offset, initially about 2 m from its centre, outside the geometry-derived exclusion region |
| Observation | Relative position and velocity in a named frame, chaser attitude error/angular rate, desired offset, previous action; add target attitude/rate if goal follows its body |
| Orbital information | Known orbit rate/frame information as required by the chosen representation; explicitly distinguish a target-body goal from an LVLH goal |
| Action, first version | Three bounded force commands while an independently verified attitude controller runs |
| Action, later version | Translation and rotation together through body-frame commands |
| Reset | Start about 3-5 m away with low relative speed; widen to 5-20 m and up to approximately 0.05 m/s relative speed |
| Episode | Start around 120 s; allow up to 300 s for wider cases only after a reachability/braking check |
| Baseline | Relative position/velocity feedback, then an orbit-aware linear controller or existing MPPI where appropriate |
| Training signal | Progress toward the holding point, low relative speed there, approach-axis alignment, effort, and violations |

When a goal frame rotates, desired position and desired velocity must be consistent. A stationary offset in LVLH generally has nonzero relative velocity in an ECI-aligned frame. The simulator's frame conversions account for this distinction. [Frames](https://johnzhang3.github.io/mjorbit/docs/frames.html)

### Training stages

1. Hold a relative offset; test frame conversions and a zero-control coast.
2. Approach along one axis with attitude stabilized, learning to brake rather than fly through.
3. Approach in three dimensions with small initial offsets.
4. Introduce a geometric corridor and holding points, then widen distance and relative velocity.
5. Couple translation and attitude. A Project 1 policy is reusable knowledge/baseline, but changed dynamics or interfaces may require retraining.
6. Evaluate with orbital coupling active and longer coasts. Add perturbations and noise separately.
7. Extension: approach a slowly rotating target while matching the handle-frame motion. This prepares Project 3 without making first rendezvous a tumbling-capture problem.

Define prohibited geometry before training. Use collision shapes or conservative bounds for clearance; centre-to-centre distance alone is insufficient. A supervisory stop/retreat can be useful, but log its interventions and report supervised performance separately from standalone policy performance.

### Failure modes worth studying

Fly-through; oscillation around the holding point; pointing the wrong way before applying body thrust; using inertial instead of rotating-frame velocity; cutting through excluded geometry; and succeeding only from one direction.

### Deliverable

A three-dimensional approach trajectory, distance/relative-speed/clearance plots, success and failure counts, and a comparison with a controller using the same actuator limits.

## Project 3: arm capture and post-capture stabilization

### Question

Can the chaser reach a prepared satellite handle, grasp gently, and keep the satellite attached while stabilizing both bodies?

This adds a moving base, arm joints, contact, friction, capture timing, and changed mass/inertia. Begin with a stationary-in-relative-attitude target and an accessible handle. A two-joint arm is suitable for a restricted aligned or planar first task. Arbitrary three-dimensional handle pose may require more arm joints and/or explicit spacecraft repositioning; verify reachability before choosing the action space.

### Task contract

| Item | Initial definition |
|---|---|
| System | Free chaser, articulated arm and gripper, independent free target with a prepared capture feature |
| First reset | Close, reachable, aligned pre-grasp state with very low relative motion |
| Observation | Arm positions/velocities; gripper-to-handle pose and relative velocities; base attitude/rates; target relative motion; gripper state; previous action |
| Actions | Arm joint targets/velocities and grip open/close; use a verified base controller first, then introduce bounded base-control policy commands |
| Baseline | Staged approach, kinematic reach, slow closing, and post-capture feedback |
| Training signal | Reach and alignment progress, low contact speed, valid retained grasp, and stable hold; effort and abrupt-motion penalties |
| Episode | Begin with short close-range attempts; target approximately 60-120 s for combined approach/capture/hold if physically reachable |
| Capture | Contact-supported grasp around the handle; no invisible weld for the grasping claim |

The repository has Astrobee and truss PPO examples that can provide reference environments. Inspect their collision filtering and automatic controls before adapting them. Some example bodies do not participate in all possible collision pairs, so their successful grasp does not establish whole-spacecraft collision avoidance. Their close-to-far training progression is useful precedent. [Example guide](https://github.com/johnzhang3/mjorbit/blob/main/examples/ppo/README.md)

### Training stages

1. Fixed-base arm reaches a marker: teach geometry separately.
2. Free-base arm reaches the same marker: demonstrate reaction motion.
3. Align the open gripper around a prepared handle without touching other geometry.
4. Execute a slow scripted grasp and verify contact/retention physically.
5. Learn grip timing from favourable poses, then vary small offsets and speed.
6. Extend retention to a 20 s hold with mild specified disturbances.
7. Combine Project 2 approach, pre-grasp alignment, capture, and post-capture stabilization with explicit stage transitions.
8. Widen distance, target mass and inertia, handle pose, friction, and slow target rotation progressively. More complex tumbling is an extension after the simpler capture passes.

First implement a staged controller. A single end-to-end policy is an optional later comparison, not a prerequisite. Do not assume separately trained policies compose reliably: test their handoff states and include recovery/failure transitions.

### Failure modes worth studying

Closing beside the handle; brief contact followed by release; shoving the target away; interpenetrating geometry; arm/base counteracting each other; joint-limit contact; and post-capture oscillation or actuator saturation.

### Deliverable

A contact-supported capture and hold, contact/relative-motion traces, a stage-by-stage success breakdown, and ordinary evaluation clips alongside failures. State whether the target was non-spinning, slowly rotating, or tumbling.
