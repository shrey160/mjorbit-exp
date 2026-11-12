# Curriculum: understand, predict, experiment, explain

Audience: Python-comfortable beginner in robotics/RL, 10+ hours/week, with AI implementation assistance. [Project briefs](PROJECTS.md) and [evaluation rules](EVALUATION.md) define the practical work.

## How each lesson works

Spend roughly 2 hours learning, 1 hour predicting and drawing, 4-6 hours on a small experiment with AI help, and 2-3 hours interpreting results. Leave additional time for setup and failed experiments. Coding difficulty should not become the main assessment.

For every lesson, produce four things: a prediction made before running, one labelled plot or animation, an explanation of any mismatch, and one follow-up question. Ask AI for help after recording your own prediction.

## Twelve-week first pass

| Week | Concept focus | Experiment or deliverable | Understanding gate |
|---|---|---|---|
| 1 | Free fall; position versus velocity; force versus torque; mass versus inertia; body/world/orbital frames | Frame-and-units diagram; free-drift and one-pulse predictions | Explain why zero relative velocity depends on the reference frame |
| 2 | Feedback; damping; orientation error; RL observation/action/reward/episode; actor and critic | Project 1 contract; simple damping controller specification; environment plan | Explain why stopping spin and pointing in a direction are different tasks |
| 3 | Angular momentum; bounded commands; PPO rollout/update loop | One-axis detumbling, then three-axis baseline and first learned policy | Predict the effect of doubling inertia or reducing torque |
| 4 | Generalization; training curves; validation; actuator limits | Project 1 evaluation and an attitude-hold extension | Explain why a larger reward may coexist with worse task success |
| 5 | Relative orbital motion; rotating frames; braking and stopping distance | Short-versus-long coast comparison; Project 2 holding-point geometry | Explain why flying straight at the target can cause a flyby or collision |
| 6 | Position/velocity feedback; target-relative goals; constrained approaches | One-axis approach, then three-dimensional approach with attitude stabilized | Distinguish velocity matching from merely reaching a position |
| 7 | Coupled rotation/translation; body-mounted thrust; staged policies | Project 2 joint control and held-out tests | Explain what a wrong thrust-frame conversion would look like |
| 8 | Arm joints; forward/inverse kinematics; joint limits; moving-base reaction | Reach a marker with a fixed base, then with a free base | Explain why moving the arm moves the spacecraft |
| 9 | Relative end-effector pose/velocity; capture geometry; compliant contact; friction | Pre-grasp alignment; slow contact; scripted grasp reference | Explain why being close is insufficient for grasping |
| 10 | Sparse success; shaping; distance curriculum; contact-supported retention | Learn a close-range grasp from favourable initial poses | Identify a false grasp and a reward loophole |
| 11 | Post-capture momentum and inertia; loss of grasp; staged coordination | Approach + grasp + hold sequence; progressively varying target mass | Explain why the pre-capture controller may perform differently after capture |
| 12 | Reproducibility; matched baselines; failure analysis; transfer limits | Final evaluation, ordinary and failed clips, and a short technical report | Defend what the results establish and what remains untested |

If Project 3 takes longer, extend weeks 9-12 rather than dropping evaluation. First success is a learning milestone, not evidence of reliability.

## Lessons and questions

### 1. What actually moves in the simulator?

Learn position, velocity, acceleration, orientation, angular velocity, force, torque, mass, and inertia. Keep linear and rotational quantities distinct. The reference orbit and robot offsets are separate states. Read [mjorbit frames and units](https://johnzhang3.github.io/mjorbit/docs/frames.html): orbit storage uses km and km/s, while robot offsets use m and m/s. The world axes remain parallel to ECI; LVLH rotates with the orbit.

Prediction exercise: give a spacecraft a short force pulse, then remove the command. Sketch velocity and position versus time. Repeat with a torque pulse and sketch angular velocity. Explain the role of external disturbances in an orbital run.

Checkpoint: identify the frame and units of every quantity in one proposed policy observation. A vector without a frame label is incomplete.

### 2. What is a controller, and what is a policy?

Learn feedforward versus feedback, proportional and derivative feedback, overshoot, saturation, and settling time. Use rate damping for detumbling; use orientation error plus angular velocity for pointing. Introduce quaternions through their purpose before studying their algebra: represent orientation without Euler-angle singularities, normalize them, and remember that q and -q describe the same rotation.

Then read [RL key concepts](https://spinningup.openai.com/en/latest/spinningup/rl_intro.html). Be able to distinguish state, observation, action, reward, return, policy, value estimate, rollout, and episode. Your reward is a training aid; your success predicate is the test of the task.

Prediction exercise: describe how a robot could exploit a reward for proximity while failing to stop at the target.

### 3. How does PPO learn?

Read the intuitive parts of the [PPO guide](https://spinningup.openai.com/en/latest/algorithms/ppo.html), then the abstract and algorithm of the [original paper](https://arxiv.org/abs/1707.06347). Understand the cycle: collect experience, estimate which actions did better than expected, update actor/critic, then collect fresh experience. Explain why clipping discourages overly large changes but does not guarantee stability or task safety.

Understand learning rate, exploration, rollout size, minibatches, discounting, and normalization through one-at-a-time experiments. Use an existing implementation. Implementing PPO from scratch is optional later study.

Checkpoint: explain why a rollout of 32 control steps is not necessarily a whole episode, and why reducing parallel worlds changes experience per update.

### 4. Why do nearby spacecraft move strangely?

Learn radial, along-track, and cross-track directions. Use [MIT's relative-motion lecture](https://ocw.mit.edu/courses/16-346-astrodynamics-fall-2008/e4f0632a9f1c98f7e9b25492e1a30eb1_lec_26.pdf) as a reference, not a prerequisite to derive everything. The linear CW model is a local approximation around a circular reference orbit; use it for intuition and a matched simple comparison, not as universal orbital truth.

Prediction exercise: compare coasting for seconds with coasting for a substantial fraction of an orbit. Explain when an inertial double-integrator approximation becomes inadequate. Maintain orbital coupling in the final Project 2 evaluation.

Checkpoint: explain that rendezvous requires both relative position and relative velocity conditions. A co-orbiting target still moves around Earth.

### 5. What changes when an arm touches another free body?

Learn joints, links, end-effector pose, reachable workspace, Jacobian intuition, reaction motion, relative contact velocity, friction, and post-capture inertia. Begin with a prepared handle and simple supported collision primitives. Read the [mjorbit PPO examples](https://github.com/johnzhang3/mjorbit/blob/main/examples/ppo/README.md) critically: note what the policy controls, what is automatic, what contacts are enabled, and how resets simplify the task.

Prediction exercise: move the arm with the base control disabled. Explain the resulting base reaction. Close a gripper beside a handle and decide whether closed fingers alone prove a grasp.

Checkpoint: define retained capture using geometry, contact, and a hold interval. Specify how an initially captured target can escape.

## What you should own, and what AI can implement

You should be able to explain the task contract, frames, command limits, reward terms, success metrics, plots, and failure modes. AI can help with installation, XML construction, wrappers, plotting, library integration, and debugging. You do not need to memorize APIs or write every line manually. Follow the [AI workflow](AI_WORKFLOW.md).

Defer full orbital-transfer design, advanced optimal control proofs, detailed reaction-wheel engineering, camera navigation, recurrent policies, and complex contact mechanics until a concrete project question requires them.
