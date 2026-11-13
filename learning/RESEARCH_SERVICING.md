# Research: from where we are to the servicing goal (8 Nov 2026)

> **Update, later on 8 Nov:** the user changed the client to a generic rectangular satellite with a grapple bar, a *prepared* client. Sections 1–2 on Starlinks and unprepared capture now inform a later extension. Sections 3–7 (approach safety, arm control, post-capture, learning) apply unchanged. For a prepared handle, the relevant precedents are grapple fixtures (ISS FRGF for snare end-effectors, handrails for grippers) and Astroscale's docking plate ([grapple fixture overview](https://en.wikipedia.org/wiki/Grapple_fixture)).

**Goal:** a servicing robot that attaches to a Starlink-like satellite and then moves it to a commanded attitude or position.

**Where we are:**
- **P1:** attitude control.
- **P2:** filtered approach with glideslope guidance and a safety shield.
- **P3A:** approach to a pre-grasp point on a box-shaped client.

This note collects what flight programmes and the literature say about each remaining step, picks methods, and revises the plan. Sources are at the end. Items marked *(abstract only)* were read from abstracts or listings, not full papers.

## 1. What a Starlink is, as a client

- **v1.5:** about 295–306 kg. Bus about 2.8 × 1.3 m. One solar array about 2.8 × 8.1 m.
- **V2 Mini:** about 740–800 kg. Bus about 4.1 × 2.7 m. Two wings, about 30 m span in total.
- (Figures from FCC-filing summaries and launch lists.)
- **Our 3A model** (300 kg, 2.8 × 1.4 m, no array) matches v1.5. A V2 Mini variant is the natural next client: heavier than our servicer, which matters after capture.
- **No servicing interface.** I found no evidence that Starlinks carry a docking plate or grapple fixture, so any capture is of an **unprepared** client.
- **Why anyone would service one.** Failed Starlinks at their low altitude re-enter within about 5 years, and studies argue that replacing LEO satellites is usually cheaper than servicing them. A realistic purpose is therefore **removal or repositioning of a dead satellite**, for collision risk or a controlled re-entry, rather than refuelling.
- **Implication for us:** the most realistic client is dead (uncontrolled, possibly tumbling), unprepared, and as heavy as or heavier than the servicer.

## 2. How real missions attach to a client

| Approach | Who | Fits a Starlink? |
|---|---|---|
| Grapple the **launch-vehicle interface** (adapter ring, separation-bolt holes): the strongest structure on any satellite | NRL/DARPA RSGS arms on Northrop's MRV (launched Jul 2026); MEV-1 drew IS-901 onto stanchions against its launch ring | **Yes, in principle.** Starlinks launch stacked, so the stack interface points or bus edge are the analogue. Our "capture feature on the end face" stands in for this. |
| Probe into the **apogee-engine nozzle** | MEV-1/2 docking; MRV fitting Mission Extension Pods | No. Starlinks use electric (Hall/argon) thrusters, with no large nozzle. |
| **Caging** (arms close around the body before touching) | ClearSpace-1: tentacles, later claws, for PROBA-1 tumbling up to 3°/s | Plausible for a small flat bus. It tolerates pose error and avoids pre-contact pushes. |
| **Prepared plate** (magnetic or mechanical) | Astroscale docking plate, flown on ELSA-d (2021) | Only if the client had been fitted with one. Not Starlink today. |
| **Gecko (dry) adhesive** on flat surfaces | Stanford/JPL gripper tested on Astrobee on the ISS (2021) | Attractive for a flat bus face: no fixture needed. Strong only in shear, needs a gentle normal approach. Not yet demonstrated on a satellite in orbit. |
| Nets, harpoons, tethers | RemoveDEBRIS, Envisat studies | Low precision, not suited to "then reorient it". |

**Choice for 3B, as a staged plan:**
1. A **gripper grasping a structural edge** at the capture feature: a stiff rim on the bus end, standing in for the launch-interface structure, which is what real arms target.
2. As an alternative, an **adhesion pad** on the flat face, using MuJoCo's `adhesion` actuator. That is still real contact physics, not an invisible weld.
3. **Caging** as a robustness extension for the tumbling case (3E).

## 3. Approach and proximity operations (beyond 3A)

- **ADRAS-J (Astroscale, 2024).** The first close approach to real non-cooperative debris, an H-IIA upper stage of about 11 × 4 m and 3 t. It flew to about 15 m below the planned capture point, the payload attach fitting.
  - Two autonomous **aborts were triggered by unexpected relative-attitude anomalies**.
  - Lesson: estimating the client's attitude, and a monitor that aborts safely, are as important as guidance. Our shield idea carries over, now driven by attitude as well as position.
- **Orbital Express (2007).** Fully autonomous free-flight robotic capture, plus an autonomous fly-around and capture.
  - Its navigation sensor was used only within 120 m.
  - An anomaly caused a month-long pause.
  - Lesson: design for re-acquisition and repeated attempts. ClearSpace plans at least 3 capture attempts.
- **Guidance for rotating or tumbling targets:**
  - MPC with rotating keep-out constraints, e.g. linear time-varying "rotating hyperplanes";
  - tube MPC for robustness to tumble uncertainty (Albee et al.);
  - MPC-for-tracking with stability and safety guarantees (Rebollo et al.);
  - dynamic keep-out shapes instead of conservative spheres.
  - *(abstract only)* These are the classical baselines for 3E. Our 3A waypoint and corridor router is the simple version.
- **Pose estimation.** Real systems estimate the client's pose with cameras or lidar. Monocular deep pose estimation is benchmarked on SPEED/SPEED+ (ESA/Stanford competitions), typically as detection, then landmarks, then PnP, and the sim-to-real lighting gap is the main issue.
  - mjorbit has **no camera or lidar models**, but MuJoCo can render RGB/depth.
  - Practical path: first add a **pose-error model** (noise and bias on the client's attitude and position). Only later, optionally, a learned estimator on rendered images.

## 4. Arm control for capture

- **Free-flying vs free-floating.** With base actuators off during the grasp (free-floating), arm motion moves and rotates the servicer. The **generalised Jacobian** (Umetani and Yoshida) maps joint rates to end-effector motion in that case.
- **Reaction Null-Space (RNS) control.** Arm motions that cause no base rotation; validated in flight on ETS-VII.
  - Variants: a redundant 7-DoF arm for zero reaction; dual-arm, with one arm capturing and one compensating; adaptive RNS for unknown target inertia.
  - It constrains base rotation only; translation still drifts.
- **Impedance control for contact.** The end-effector behaves like a mass-spring-damper, so impacts don't push the client away or damage the arm. Variants:
  - single-axis impedance (a compliant wrist) during final approach;
  - multi-arm impedance.
  - Caution from the literature *(abstract only)*: some impedance schemes trade approach-to-grasp performance against post-grasp stabilisation, so unified designs exist.
- **Motion synchronisation** with a tumbling target before closing the gripper. PPO has been used for a 7-DoF arm (ESA ASTRA 2023, randomised spin and grasp point).
- **Contact simulation pitfalls** (DLR OOS-SIM hardware-in-the-loop): time delay and integration error in contact make results depend on the timestep and contact parameters. We already saw timestep sensitivity in P2 (1 s vs 0.1 s), so 3B must use small timesteps and run sensitivity checks.

## 5. After capture: stabilise, identify, move

- **Detumble the stack** with impedance or compliant coordination, so grasp loads stay low.
- **Identify the combined inertia online.**
  - Coarse: angular momentum conservation with the arm fixed.
  - Fine: recursive least squares, or optimisation with arm motion.
  - Adaptive controllers can also identify while tracking.
- **Momentum management:** absorb rotation in reaction wheels, then dump it with thrusters or magnetorquers.
- **Moving the stack (translation, re-entry):**
  - the thrust line must pass close to the **combined centre of mass**, which has shifted, so off-axis thrust makes torque;
  - it is a key reason to identify the centre of mass first;
  - MEV-style missions dock rigidly, then control the stack with the servicer's own thrusters.
- **Relocation scale.** Moving a LEO satellite to a disposal or re-entry orbit takes a Δv of tens of m/s, from hours to days of thrusting. That is a different regime from our 20-minute episodes.
  - mjorbit's orbit layer handles long horizons, at coarser timesteps.
  - Our "commanded position" step should start with a relative repositioning (metres to hundreds of metres), then a re-entry burn as an extension.

## 6. Learning: where it has helped, and tools

- **mjorbit's own capture result.** PPO on mjorbit-warp with 1024 parallel worlds: an Astrobee detumbles, approaches a drifting cargo module, grasps its handle and holds, with **88% grasp-and-hold success**, sim-to-sim only.
  - It is in our repo (`examples/ppo/astrobee_grasp.xml`) and is the natural learned reference for 3B.
- **Space-robot DRL** *(abstract only)*:
  - TD3 for free-floating capture trajectories, with base-pose disturbance in the reward (2025);
  - hierarchical DRL with curriculum and tactile force regulation for debris capture (PyBullet, 2025 thesis);
  - PPO motion synchronisation (2023);
  - learning with prior-policy guidance for dual-arm planning (MuJoCo).
  - The common pattern: **learning for trajectory and motion planning; classical or force control at contact.**
- **On-orbit RL flight.** NRL flew RL control of an Astrobee on the ISS (trained in Isaac Lab), the first RL robot control in space. Sim-to-real is feasible with domain randomisation.
- **Tooling:**
  - MuJoCo Playground (MJX, GPU, Google DeepMind, 2025) has manipulation environments and on-device rendering for pixel policies; useful designs to copy.
  - LeRobot imitation (ACT, Diffusion Policy) fits the plan "classical expert, then demonstrations, then a policy" for the arm.
- **Our own lessons apply directly:**
  - a strong classical base first;
  - learned residuals on top;
  - randomise sensor noise and other parameters, since r2 failed with perfect sensors;
  - a shield around learned commands;
  - large Monte Carlos and a test set used once.
- **No standard benchmark exists** for dual-arm or space capture. Papers build their own MuJoCo or PyBullet scenes; one open student project uses PyBullet with URDFs. A clean, documented mjorbit capture benchmark would itself be a contribution.

## 7. Promising ideas, ranked for us

1. **Edge grasp with impedance control on a free-flying servicer (3B baseline).** The servicer keeps station with its thrusters and attitude control while a 6–7 DoF arm reaches a rim at the capture feature: Jacobian or inverse-kinematics reach, impedance on contact, close. Then switch to **free-floating** (base actuators off at contact) with RNS-style arm motion. Metrics:
   - grasp success;
   - client velocity imparted (push-away);
   - peak contact force;
   - base attitude disturbance.
2. **Client pose-error model, plus an attitude-anomaly abort** (ADRAS-J lesson) in the 3A/3B shield.
3. **Momentum-based inertia identification then attitude control** for the stack (3C/3D). P1's controller with an identified inertia, plus centre-of-mass-aware thrust allocation.
4. **Learning where the literature finds value:**
   - PPO residual on the classical reach for timing and motion synchronisation with a rotating client (3E);
   - imitation (ACT or Diffusion Policy) from classical demonstrations for the arm;
   - always with domain randomisation over inertia, friction, pose error and noise.
5. **Adhesion pad** on the flat bus face as a second capture mode. It is simple in MuJoCo and a genuinely different approach.
6. **Caging and tube MPC for a tumbling client up to 3°/s** (ClearSpace-1's spec) as the hardest stage.
7. **V2 Mini-class client** (about 800 kg, heavier than the servicer) to stress post-capture control.

## Sources

- Starlink sizes: [Wikipedia launch list](https://en.wikipedia.org/wiki/List_of_Starlink_and_Starshield_launches), [McDowell, Starlink summary](https://planet4589.org/sci/starsim/index.html), [Spaceflight Now, V2 Mini](https://spaceflightnow.com/2023/02/26/spacex-unveils-first-batch-of-larger-upgraded-starlink-satellites/)
- Constellation removal and economics: [ESA SDC9 paper 222](https://conference.sdo.esoc.esa.int/proceedings/sdc9/paper/222/SDC9-paper222.pdf), [ESA SDC8 paper 74](https://conference.sdo.esoc.esa.int/proceedings/sdc8/paper/74/SDC8-paper74.pdf), [ISPreview on OneWeb removal](https://www.ispreview.co.uk/index.php/2021/11/oneweb-may-test-removal-service-on-failed-broadband-satellite.html)
- Docking plate: [Astroscale docking plate](https://astroscale.com/docking-plate), [SatNews 2021](https://satnews.com/2021/11/16/18790/)
- MRV and RSGS: [Spaceflight Now, MRV launch](https://spaceflightnow.com/2026/07/21/live-coverage-spacex-to-launch-novel-geosynchronous-robotic-servicing-satellite-on-decade-long-mission/), [NRL space robotics](https://www.dvidshub.net/news/printable/570075), [Aerospace America on MEV](https://epizodyspace.ru/bibl/inostr-yazyki/aerospace-america/2023/4/Kwon_Normalizing_satellite_servicing_Aerospace_America_61_no_04_(2023).pdf), [MRV fact sheet](https://cdn.northropgrumman.com/-/media/wp-content/uploads/Mission-Robotic-Vehicle-MRV-fact-sheet.pdf)
- ClearSpace-1: [mission page](https://clearspace.today/missions/clearspace-1), [SDC8 paper 320](https://conference.sdo.esoc.esa.int/proceedings/sdc8/paper/320/SDC8-paper320.pdf), [SpaceOps 2025](https://publications.spaceops.org/2025/download_by_id.php?id=0396)
- ADRAS-J: [15 m approach](https://satnews.com/2024/12/12/astroscales-adras-j-achieves-historic-15-meter-approach-to-space-debris/), [fly-around](https://satnews.com/2024/07/30/astroscales-adras-j-demos-space-debris-observations/), [deorbit](https://astroscale.com/en/news/astroscales-adras-j-mission-completes-operations-begins-deorbit)
- Orbital Express: [Boeing release](https://boeing.mediaroom.com/2007-05-10-Boeing-Orbital-Express-Completes-First-Autonomous-Free-Flight-and-Capture), [NASA NTRS](https://ntrs.nasa.gov/api/citations/20090001151/downloads/20090001151.pdf), [Spaceflight Now](https://spaceflightnow.com/news/n0707/23oe)
- Capture survey: [Frontiers 2021, robotic manipulation and capture in space](https://www.frontiersin.org/articles/10.3389/frobt.2021.686723)
- Impedance control: [multi-arm capture](https://scholar.hit.edu.cn/en/publications/impedance-control-of-multi-arm-space-robot-for-the-capture-of-non/), [single-axis impedance](https://doaj.org/article/455f695d29c841d59f795d2e9cbfdf08), [unified approach-grasp control, DLR](https://elib.dlr.de/216816/1/root_copyright.pdf)
- Reaction null space: [reactionless path planning](https://typeset.io/pdf/reactionless-path-planning-strategies-for-capture-of-54za9mzpte.pdf), [Cocuzza et al., IAC 2016](https://strathprints.strath.ac.uk/66175/1/Cocuzza_etal_IAC_2016_Soft_and_energy_efficient_robotic_capture_of_tumbling.pdf), [adaptive dual-arm RNS](https://bohrium.dp.tech/paper/arxiv/5a0f9fe49ed5dbea8e77c7d8)
- Post-capture identification: [integrated control after capture](https://portal.fis.tum.de/en/publications/an-integrated-control-scheme-for-space-robot-after-capturing-non-/), [two-step inertia identification](https://emeraldinsight.com/doi/abs/10.1108/AEAT-04-2018-0128?af=R), [adaptive identification for a combined spacecraft](https://scholar.hit.edu.cn/en/publications/integrated-design-of-trajectory-tracking-and-inertia-property-ide/), [spinning-base capture and stabilisation](https://arxiv.org/pdf/2402.01959)
- Tumbling-target MPC: [rotating-hyperplane MPC](https://www3.diism.unisi.it/~anto/public/papers/JGCD22.pdf), [arXiv 2403.10986](https://arxiv.org/pdf/2403.10986.pdf), [dynamic keep-out](https://arxiv.org/html/2512.21882v1), [stable and safe MPC](https://pure.nwpu.edu.cn/en/publications/a-novel-stable-and-safe-model-predictive-control-framework-for-au/)
- Pose estimation: [SPN, SPEED](https://arxiv.org/pdf/1906.09868), [SPNv3, SPEED+](https://arxiv.org/html/2409.11661v2)
- DRL capture: [TD3 trajectory planning, Robotica 2025](https://resolve.cambridge.org/core/journals/robotica/article/trajectory-planning-of-freefloating-space-robot-for-noncooperative-tumbling-target-capture-based-on-deep-reinforcement-learning/465B066C4811D176A929F58022B3161B), [York thesis 2025](https://yorkspace.library.yorku.ca/items/48e7e710-cab4-451f-9a32-6f1350b6d222), [ESA ASTRA 2023, PPO](https://robotics.estec.esa.int/ASTRA/Astra2023/2023%20PAPERS/95_Capra_Deep%20reinforcement%20learning%20for%20reactive%20IOS%20space%20manipulator%20operations.pdf), [dual-arm with prior policy, MuJoCo](https://arxiv.org/pdf/2209.01434)
- Simulation: [mjorbit paper](https://arxiv.org/pdf/2609.08010), [NRL Astrobee RL in space](https://govciomedia.com/nrl-demonstrates-first-reinforcement-learning-robot-control-in-space/), [MuJoCo Playground](https://playground.mujoco.org/), [DLR OOS-SIM](https://www.dlr.de/en/rm/research/robotic-systems/multi-arm/oos-sim), [De Stefano et al., T-RO](https://elib.dlr.de/134108/1/de_stefano_tro.pdf), [PyBullet dual-arm project](https://github.com/Mingrui-Yu/space_robot), [MuJoCo non-rigid detumbling](https://arxiv.org/pdf/2407.17617)
- Gecko gripper: [Stanford news](https://news.stanford.edu/2021/05/20/gecko-gripper-tested-aboard-iss/), [arXiv 2009.09151](https://arxiv.org/pdf/2009.09151.pdf), [Stanford BDML](https://bdml.stanford.edu/pmwiki/pmwiki.php/Main/AdhesionOuterSpace)

## Addendum: moving the docked stack (3D, 8 Nov)

- **MEV-1** took over the propulsion and attitude control of the docked Intelsat 901 stack, with momentum wheels for attitude and thrusters for station keeping, steering, pointing and relocation ([Wikipedia](https://en.wikipedia.org/wiki/Mission_Extension_Vehicle), [eoPortal](https://www.eoportal.org/satellite-missions/mev-1)). The search found no public detail on how it keeps thrust through the combined centre of mass.
- **Slews:** rate-limited eigenaxis quaternion feedback is the standard law, with slew-rate limits set by actuator capability ([NASA NTRS](https://ntrs.nasa.gov/api/citations/20040105588/downloads/20040105588.pdf)).
- **Unloading:** wheel momentum is typically unloaded by reaction-control thrusters in a separate mode ([NASA ISSFD 2007](https://www.issfd.org/ISSFD_2007/2-4.pdf), [NPS thesis](https://calhoun.nps.edu/entities/publication/edcbc042-55d5-4f25-b3c0-c71b717e2567)).
- **Our 3D result:** with thrusters only at the servicer's centre, momentum cannot be unloaded without Δv. This is the open design item for 3E.

## E2 addendum (10 Nov 2026): safety monitors and aborts

- **Passive safety first, active abort second.** Rendezvous designs shape the approach so a loss of thrust does not lead to collision, and keep a powered abort for segments that are not passively safe. Fault detection and isolation triggers the abort, as in the Cygnus ISS fault management. Sources: [Goggin & McClain](https://spacefoundation.org/wp-content/uploads/2019/07/Paper-Goggin-McClain-Rendezvous-Mission-Risk-Reduction-Through-Passively-Safe-Orbital-Rendezvous-_0.pdf), [arXiv 2608.03060](https://arxiv.org/pdf/2608.03060), [MERL TR2021-054](https://merl.com/publications/docs/TR2021-054.pdf), [UT fault management](https://sites.utexas.edu/near/files/2022/05/Fault_Management_20110307.pdf).
- **ADRAS-J (2024)** aborted autonomously twice on an unexpected relative attitude anomaly and backed away safely ([Astroscale, Dec 2024](https://astroscale.com/en/news/astroscales-adras-j-achieves-historic-15-meter-approach-to-space-debris)). That is the model for our `rate_anom` check.
- **Post-grasp:** limit the end-effector interaction forces and keep the stack configuration safe during stabilisation ([TU Wien](https://repositum.tuwien.at/handle/20.500.12708/223615), [DLR](https://elib.dlr.de/216816/1/root_copyright.pdf)). Runtime slip detection and release logic were not found in the literature searched, so ours is our own design (SPEC_3 §12).

## Project 4 research (12 Nov 2026): camera-based imitation-learned grasp

Searched: imitation learning for free-floating capture, LeRobot's current state, spacecraft pose estimation across the sim-to-real gap, safety filters for learned visuomotor policies, GPU batch rendering in MuJoCo, privileged-teacher distillation, and policies on moving targets. **Not researched:** event cameras, and flight heritage of learned grasping (the search found none). **Lidar (added 12 Nov, at the user's question):** NASA Raven, the relative-navigation testbed for OSAM-1, flew a visible camera, a long-wave IR camera and a 256×256 flash lidar ([NASA](https://www.nasa.gov/isam/raven), [NTRS](https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20150002731.pdf)). That three-sensor suite is the norm for servicers, so P4 adds a simulated bus flash lidar. MEV's and ADRAS-J's full suites were not confirmed. Most findings come from abstracts, search snippets and repo pages. The two space-capture works were read on their pages; their papers were not read in full.

**1. The closest precedents both use LeRobot-style ACT.**
- **SpacePolicy (IROS 2026)** ([repo](https://github.com/hallyx/space-policy)): imitation learning for free-floating capture. Arm motion drifts the base, so a policy that assumes a fixed base misses.
  - Fix: forecast the base trajectory and condition the action on it (`reaction_aware_act`).
  - LeRobot ACT plugin (targets LeRobot 0.5.1). Observation: RGB cameras plus 8-D arm/gripper state, no target pose. Action: 15-D joint position, velocity and gripper. Chunks of 32, executing 8.
  - Demonstrations by hindsight relabelling: replay a motion and re-render the target where the end effector ended up.
  - Evaluated on Aura, Dawn and Hubble models in simulation. No success rates on the repo page; no datasets, assets or weights released.
- **AstroBench** ([arXiv 2610.31064](https://arxiv.org/html/2610.31064v1)): a free-floating object-capture benchmark in Isaac Sim / Isaac Lab, with a fixed-base Panda arm and free objects (static, drifting, tumbling).
  - A frozen generalist VLA (π0.5 DROID) captured **6/400 (1.5%)**.
  - A task-specific ACT trained from scratch on 400 demonstrations (58,925 frames at 15 Hz, 30-action chunks with temporal ensembling) captured **14/20 (8/10 static, 6/10 moving)**.
  - The authors suspect π0.5's 1 s open-loop chunks, against ACT's per-step replanning. They say it is not a controlled comparison.
- **Implications:**
  - Train a task-specific policy (ACT first); a pretrained VLA is not worth trying here (this agrees with the 10 Nov decision).
  - Moving targets cost success even at slow speeds, so replan often.
  - Our free-flying bus holds its pose relative to the client, which removes most of the base-drift problem SpacePolicy solves. A free-floating variant would need its base forecast.

**2. Moving targets: action chunks must stay closed-loop.**
- Lack of reactivity "drastically" reduces success on moving objects ([DCDP, arXiv 2603.01953](https://arxiv.org/html/2603.01953v2)), and diffusion policies are slowed by inference latency ([GraspLDP, arXiv 2602.22862](https://arxiv.org/html/2602.22862v1)).
- Fixes in the literature: temporal ensembling (ACT), asynchronous inference ([arXiv 2602.23901](https://arxiv.org/html/2602.23901)), training-free chunk correction (DCDP).
- Our case is slow: the bar moves at ≤ 2°/s × 1 m ≈ 3.5 cm/s at the pre-grasp point, and approach is 1 cm/s. A 10 Hz policy with short executed chunks should be enough.
- Option: express the actions in the client's rotating frame, so the policy outputs a slow, smooth reference while the existing feed-forward handles the spin. The policy would then need the client rate. The classical system gets it from the client-rate estimate; the learned one would need an estimate of its own or would drop the option.

**3. Distil a privileged expert: behaviour cloning, then DAgger.**
- The standard recipe trains or writes a state-based teacher with privileged information, then distils it into an image-based student with DAgger. The teacher labels the states the student actually visits, with a teacher-mixing share decaying over iterations.
- Examples: [Dex-X](https://arxiv.org/pdf/2609.07747), [EXPERTGEN](https://arxiv.org/pdf/2603.15956), [TacSL](https://arxiv.org/pdf/2408.06506), [Local Policies](https://arxiv.org/pdf/2410.22332) (DAgger with a replay buffer), [Real-to-Sim-to-Real](https://arxiv.org/pdf/2403.03949), [NVIDIA ACGD](https://research.nvidia.com/publication/2025-10_acgd-visual-multitask-policy-learning-asymmetric-critic-guided-distillation.md).
- **We already have the teacher.** The classical controller can label any state, so DAgger costs only re-simulation. Plain behaviour cloning on expert runs never sees recoveries, and that compounding error is what DAgger fixes.

**4. Shields for learned visuomotor policies: switching filters cost task success.**
- Least-restrictive switching filters keep the system safe but can stop the task: the policy never finishes ([LatentCBF, arXiv 2511.18606](https://arxiv.org/pdf/2511.18606), reported about 2× safe-task success with a smooth filter instead).
- Alternatives:
  - safe sets built from demonstration data plus a recovery controller ([arXiv 2605.01201](https://arxiv.org/html/2605.01201));
  - Q-function safe sets ([TAIL-Safe, arXiv 2605.01195](https://www.emergentmind.com/papers/2605.01195)): vanilla flow-matching policies reached only 20–25% success under perturbations.
- **Implications:**
  - Our E2 monitors are a switching filter with an abort, so expect learned-policy aborts to cost success.
  - Report the intervention (abort) rate alongside success.
  - A tracking-error-type monitor needs a reference, and the learned policy has none. Redefine it as deviation from the classical expert's suggestion, or drop it for the learned run.

**5. Perception: the real gap is lighting, and we can only claim sim-only results.**
- On SPEED+, synthetic-to-hardware-in-the-loop transfer is the hard part: SPNv2 gave about 20 cm and 6.5° (lightbox) and 11° (sunlamp) ([SPEED+](https://arxiv.org/pdf/2110.03101v2), [SPNv2](https://arxiv.org/pdf/2203.04275v4)).
- Supervised domain adaptation with 5% labelled real data matches oracle performance ([arXiv 2509.13792](https://arxiv.org/abs/2509.13792v1)).
- **Implications:**
  - Randomise sun direction and intensity, eclipse, specular metal and MLI-like textures, camera noise and exposure.
  - State plainly that results are sim-to-sim.
  - A close-range wrist camera on a known bar is easier than whole-spacecraft pose estimation, but specular glare is the main risk.

**6. Tools.**
- **LeRobot:**
  - ICLR 2026 paper ([notes](https://en.papernotes.org/ICLR2026/robotics/lerobot_an_open-source_library_for_end-to-end_robot_learning/)); releases 0.4.4 (27 Feb 2026) and 0.5.0 (9 Mar 2026), per a third-party guide, unverified. SpacePolicy targets 0.5.1.
  - Dataset: Parquet (state, action) plus MP4 (cameras) on the Hugging Face Hub format.
  - One CLI, `lerobot-train --policy act|diffusion|smolvla`, with sizes of about 52M (ACT), 263M (Diffusion) and 450M (SmolVLA) parameters ([summary](https://www.emergentmind.com/papers/2602.22818)).
- **Rendering:**
  - MuJoCo Warp now has a native GPU batch ray-tracing renderer: many worlds, a different resolution and field of view per camera, textures and shadows ([repo](https://github.com/google-deepmind/mujoco_warp)). It replaces the deprecated Madrona bridge ([madrona_mjx](https://github.gg22.workers.dev/shacklettbp/madrona_mjx)). The world count is fixed when the render context is created. Fidelity against RTX renderers is an open question ([discussion #1337](https://github.com/google-deepmind/mujoco_warp/discussions/1337)).
  - **Checked 12 Nov (`probe_render.py`):** our installed mujoco-warp 3.7.0.1 exposes `create_render_context` and `render`.
  - CPU path: `mujoco.Renderer` on the plain-MuJoCo copy with `MUJOCO_GL=egl` ([TorchRL notes](https://docs.pytorch.org/rl/main/reference/generated/knowledge_base/MUJOCO_INSTALLATION.html)). **Works under WSL2 (checked 12 Nov)** through the Mesa EGL vendor: 28 frames/s at 224×224 with 20 CPU workers busy. Harmless EGL errors appear at teardown. The offscreen buffer size is set in `<visual><global offwidth offheight>`.

**Promising ideas, ranked for us:**
1. ACT on the wrist and bus cameras plus proprioception, with Cartesian grip-point actions at about 10 Hz. Contact (impedance and close) stays classical; the policy also decides when to close. Trained on classical-expert demonstrations, then DAgger.
2. The fair baseline: the classical controller on an image-estimated bar pose (a small keypoint or pose network on the same images). It separates "learning a policy" from "learning perception".
3. Diffusion Policy as a second learner. Check its latency, and whether short chunks keep it reactive at 2°/s.
4. Later: the free-floating base (SpacePolicy's base forecast), smooth safety filters, and SmolVLA only if language commands are wanted.

## Arm realism research (13 Nov 2026): what real space arms look like

Asked before making the ESPA arm more realistic (Levels 1+2: joint modules, mass layout, actuator model; same link lengths). Public specs for small (about 1 m) flight arms are scarce: Motiv xLink, GITAI S2 and the Northrop/NRL FREND-heritage arms publish no joint-level numbers that we could read, and several PDFs refused automated fetches. Numbers below are what we could confirm; anything marked *estimate* is ours.

**Arms with published numbers:**

| Arm | DoF / layout | Length | Mass | Joint torque | Joint speed |
|---|---|---|---|---|---|
| DLR CAESAR (GEO servicing) | 7, roll-pitch alternating, impedance-controlled, torque sensors in all joints | 2.4 m (up to 5 m) | about 60 kg (3 m version) | 80 N·m max per joint | up to 10°/s |
| ESA e.Deorbit study arm | 7 | not read | not read | ±80 N·m nominal (Harmonic Drive limit), 176 N·m momentary peak | 10°/s, set for safety |
| JEM Small Fine Arm (ISS) | 6 | 2.2 m | 190 kg | not found | not found |
| GITAI S2 (ISS demo) | not read | 1.5 m | not found | not found | not found |
| OSAM-1 / Restore-L servicing arm | 7: 3-axis shoulder, elbow pitch, 3-axis spherical wrist; 6-axis F/T sensor at the tip; FREND heritage | not confirmed | not found | not found | not found |
| ETS-VII (first satellite arm, 1997) | 6 | about 2 m | not found | not found | not found |
| Orbital Express OEDMS (MDA) | 6 | not found | not found | not found | not found |
| DLR LWR III (terrestrial, basis of ROKVISS and CAESAR joints) | 7 | 1.2 m | load-to-weight about 1:1 | not read | not read |

**Joint modules (DLR drive train, Wedler et al. 2012, read in full):**
- Each joint is a frameless brushless motor (DLR ILM / RoboDrive) on a Harmonic Drive gear, with motor-side position, link-side absolute position and a link torque sensor between gear and output; power, control and communication electronics sit in the module. Brakes are fitted (failsafe, set when power is off, per a patent on the same layout).
- Gear ratios: LWR III uses 1:160 (joint 5: 1:100). An HFUC8 at 1:100 gives 2.4 N·m nominal, 4.8 N·m repeatable and 9 N·m momentary peak. ROKVISS used an ILM70 motor on an HFUC25 gear.
- ROKVISS (2-joint arm on the ISS, 2005–2010, returned to Earth): joint friction rose about **50% in orbit** compared with the ground (20 °C, air) for joint 1, under 20% for joint 2, then stayed constant; temperature dependence was within identification uncertainty. Measured **joint stiffness about 4900 N·m/rad** in space (flex spline plus torque sensor), joint 2.
- Older NASA concepts: 200:1 harmonic drives, 0.25–0.7 rad/s maximum joint rate; the harmonic drive's compliance dominates the manipulator's, more than the links. Shuttle RMS joints used about 738:1 and 1842:1.
- An ESA TITAN modular-arm presentation lists a joint module of about 5 kg (444 × 168 × 189 mm) and a gripper of about 8 kg (unconfirmed: slide text only).
- On-orbit friction identification (Tiangong-2 arm): viscous plus Coulomb friction predicted friction well enough for position control.

**What this means for our arm (1.18 m reach, 6 joints, ESPA servicer):**
- Our current arm is light (9.2 kg including the gripper) and ideal: capsules, 0.05 kg·m² armature, no friction, 20/15/5 N·m. Real joints are self-contained modules with gear, brake, sensors and electronics, so most of the arm mass sits at the joints, and the motor's inertia seen through a 100–160:1 gear dominates the joint inertia.
- Real space arms move slowly: 10°/s joint speed is the common safety limit (CAESAR, e.Deorbit).
- Space joints use torque sensors to fight friction; uncompensated harmonic-drive friction is large and grows in orbit.
- Joint stiffness (about 5000 N·m/rad) is the main flexibility; it is Level 3 (not modelled now).

**Sources:** [DLR CAESAR](https://www.dlr.de/en/rm/research/robotic-systems/arms/caesar); [CAESAR, IAC 2018](https://elib.dlr.de/122123/1/IAC-2018_IAC-18,D1,6,3,x42634_Final.pdf); [Wedler et al. 2012, DLR's Dynamic Actuator Modules for Robotic Space Applications](https://esmats.eu/amspapers/pastpapers/pdfs/2012/wedler.pdf); [e.Deorbit robotic subsystem](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC7805711/) (numbers via search summary; page blocked automated reading); [ESA TITAN presentation](https://indico.esa.int/event/416/contributions/7407/attachments/4906/7533/CSID22_3_Modular%20Robotic%20System%20for%20Future%20Servicing%20and%20Deorbiting%20Missions_presentation_update5.pdf); [JAXA Kibo RMS](https://iss.jaxa.jp/en/about/kibo/rms); [OSAM-1 (eoPortal)](https://www.eoportal.org/satellite-missions/osam-1); [GITAI S2 on the ISS](https://www.therobotreport.com/gitai-dual-armed-robot-heads-to-the-iss-after-successful-launch/); [Motiv xLink](https://www.roboticstomorrow.com/news/2020/05/22/motiv-space-systems-launches-xlink-the-highly-modular-cost-efficient-space-rated-robotic-arm-system/15277); [NASA harmonic-drive space arm study (1990)](https://ntrs.nasa.gov/api/citations/19900019741/downloads/19900019741.pdf); [NASA manipulator concept (1989)](https://ntrs.nasa.gov/api/citations/19890010503/downloads/19890010503.pdf); [Tiangong-2 arm friction identification](https://ieeexplore.ieee.org/document/8484274).

## P4.4–P4.7 research (13 Nov 2026): ACT, DAgger, shield, second learner

Searched: the current LeRobot release, its ACT defaults and dataset API, ACT data needs and CPU latency, DAgger with chunked policies and scripted experts, runtime monitors for learned policies, action frames for moving targets, and new space-capture imitation work. Read in full: the LeRobot v0.5 and v0.6 release posts, ACT's `configuration_act.py` on main, and the dataset-v3 doc page. The rest comes from search snippets and abstracts. Also checked our own data (`demos_v3/ep_8000000.npz`).

**1. LeRobot now.**
- **Releases:**
  - 0.5.0 (9 Mar 2026): Python ≥ 3.12; Real-Time Chunking for flow and diffusion policies ([v0.5](https://huggingface.co/blog/lerobot-release-v050)).
  - 0.6.0 (7 Jul 2026): torch 2.7–2.11; depth video in datasets; `lerobot-rollout` with a DAgger strategy; about 2× faster video data loading ([v0.6](https://huggingface.co/blog/lerobot-release-v060)).
  - 0.7.0 (about 5 Nov 2026): a long list of breaking changes. Source is a third-party write-up ([explainx](https://explainx.ai/blog/lerobot-v0-7-0-interactive-rollouts-disk-less-streaming-remote-policy-server-cosmos-3-yam-unitree-g1-2026)); the official notes were not read.
  - **Pin one version** for both Colab and WSL. 0.6.x is the settled choice; take 0.7.0 only if one of its features is needed.
- **DAgger in LeRobot:** `lerobot-rollout --strategy.type=dagger` records human take-overs from a leader arm, with an intervention flag per frame. A scripted expert is not mentioned, so our DAgger loop stays our own code; only its output goes into a LeRobot dataset.
- **Dataset v3** ([docs](https://huggingface.co/docs/lerobot/en/lerobot-dataset-v3)):
  - The sequence is `LeRobotDataset.create(...)`, then `add_frame` per step, then `save_episode`.
  - **`finalize()` is required**, or the Parquet files are left incomplete.
  - A local `root=` works and pushing to the Hub is not needed, which keeps the rule of no data in git or on the Hub.
  - Read the exact `create` and `add_frame` signatures from the installed version.

**2. LeRobot ACT defaults and limits** ([configuration_act.py](https://github.com/huggingface/lerobot/blob/main/src/lerobot/policies/act/configuration_act.py)).
- **Defaults (about 52M parameters):**
  - network: ResNet-18 (ImageNet weights) per camera; a 4-layer encoder and a 1-layer decoder at width 512; CVAE latent 32;
  - training: `kl_weight` 10, dropout 0.1, AdamW lr 1e-5 (backbone 1e-5);
  - chunking: `chunk_size` 100 and `n_action_steps` 100. Temporal ensembling is off; its usual coefficient is 0.01, and with it on, `n_action_steps` must be 1.
- **All cameras must have the same image shape.** Ours are wrist 128, bus 192 and lidar 128, so either resize the bus camera to 128 (or render it at 128) or upsample the others.
- **The lidar needs 3 channels.** Images go through a 3-channel ResNet, so the lidar depth image has to be packed into 3 channels, for example range, normalised range and a return mask.
- **Non-image inputs:** `observation.state` (proprioception) and `observation.environment_state` (any other vector).
  - The P4.3 pose estimate could go in `environment_state`.
  - That gives a "modular" variant to compare with the pure-image one.

**3. Our data does not fit ACT as recorded.** `demos_v3` stores images at 4 Hz (every 0.25 s) and actions at 10 Hz (every 0.1 s), but a LeRobot dataset has one fps for both. Options:
- **Re-record with images at 10 Hz (`--img-steps 10`).** 125 episodes would be about 4.7 GB and take about 15 min on 8 workers. This is the cleanest: same seeds, same physics.
- **Build a 4 Hz dataset** with the action interpolated to the image times. The targets are smooth, but the close flag is a step. Cheaper, but a 4 Hz policy reacts more slowly to a 2°/s spin.
- **Recommended:** re-record at 10 Hz, and record the DART variant (§5) in the same pass.

**4. Data size, chunk size, latency.**
- **Data size:**
  - ALOHA-style ACT needs about 50 demonstrations per fixed task (secondary sources).
  - AstroBench used 400 demonstrations for 14/20 captures.
  - Our task varies spin rate and axis, so 125 may be thin. Plan a learning curve (125, then 250) on the 25 validation seeds.
- **Chunk size:**
  - Guides tie the chunk to one sub-motion.
  - The default 100 steps at 10 Hz is 10 s, longer than our approach phase.
  - Start with a chunk of 20 (2 s) and execute 5, or use temporal ensembling with `n_action_steps` 1.
- **CPU latency:** about 180 ms per ACT forward pass on an M1 CPU, 5 ms on an RTX 4090 ([LeRobot paper](https://arxiv.org/pdf/2602.22818)).
  - The simulation does not need to run in real time.
  - A CPU-only evaluation at 10 Hz is about 470 forward passes (about 1 min) per episode per worker, so 25 episodes on 8 workers take a few minutes.
  - Temporal ensembling (a forward pass every step) is affordable.

**5. Before DAgger: DART noise injection.**
- **What it is:** DART injects noise into the expert's executed actions while recording, but labels each frame with the expert's clean action, so the demonstrations show recoveries.
- **Results:** with a scripted supervisor in MuJoCo it was up to 3× cheaper than DAgger, and 62% better than plain behaviour cloning in a grasping task ([DART](https://arxiv.org/pdf/1703.09327), [BAIR blog](https://bair.berkeley.edu/blog/2017/10/26/dart)).
- **For us:** a few lines in the recorder. Perturb the commanded grip target (a few mm, ramped down near close) and save the clean target. It needs no expert query on the policy's states.
- **Related work:**
  - Recent work combines the two: noise set from the policy's own error, plus HG-DAgger-style gated corrections ([Policy-Calibrated DAgger, arXiv 2609.30462](https://arxiv.org/html/2609.30462v1)).
  - No paper was found that runs DAgger on ACT with a scripted expert.
- **DAgger needs an expert "suggest" function:** given any sim state, the target the classical expert would command now.
  - **Revised after reading `grasp_sim.py`:** the expert's reach and approach targets depend only on the bar pose and the time in the phase, not on the arm's state. So the suggest function is small (about 50 lines); see SPEC_4 §4.4.
  - The same targets also mean DART's main gain for us is broader state coverage with correct labels; the recovery itself is done by the low-level tracking.
  - The P4.6 `track` monitor needs the same function, so build it once.

**6. Shield and failure detection.** Detecting failure is well covered; recovery is usually a rollback or a corrected action, not a hand-over to a classical controller.
- **Rewind-IL:** online failure detection plus state respawning for chunked policies ([arXiv 2604.16683](https://arxiv.org/pdf/2604.16683)).
- **FIPER (NeurIPS 2025):** failure prediction without failure data, from out-of-distribution signals in the policy's embedding space and from action uncertainty.
- **FAIL-Detect:** extracts scalar signals from the policy and calibrates a threshold ([arXiv 2503.08558](https://arxiv.org/html/2503.08558v1)).
- **TAIL-Safe:** a Q-function safe set with gradient-based recovery; success under perturbation rose from 20% to 100% ([summary](https://www.emergentmind.com/papers/2605.01195)).
- **For us:**
  - Keep the E2 monitors and abort as the shield, the hard safety layer.
  - Add a **hand-over to the classical expert** when the policy drifts, for example when its target strays from the expert's suggestion beyond a bound before contact. It is a cheap middle step that we can do and most papers can't, because our expert can take over from any state.
  - Report success, safe outcomes, aborts and hand-overs.

**7. Moving target and action frame.**
- **Literature:**
  - Space-capture imitation, where the grasp point drifts between steps, needed temporal consistency (a 2026 paper using an inter-frame correlation network; snippet only).
  - Object-centric and relative actions help when the robot–object relation keeps changing (SPOT [arXiv 2411.00965](https://ar5iv.labs.arxiv.org/html/2411.00965), X-Imitator [arXiv 2605.12162](https://arxiv.org/html/2605.12162)).
  - There is no consensus on absolute versus relative actions ([arXiv 2602.23408](https://arxiv.org/pdf/2602.23408)).
- **For us:**
  - Start with absolute grip targets in the bus frame, which is what we record.
  - Try targets relative to the current grip point as the first variant.
  - Bar-frame targets would need the bar pose, which means the modular variant.

**8. Second learner.**
- **Real-Time Chunking:** in LeRobot it works for flow and diffusion policies (the Pi0 family, SmolVLA, Diffusion), not ACT.
- **Diffusion Policy:** about 263M parameters, so training belongs on Colab. Evaluation on CPU will be slow (many denoising steps per call), so use short episodes and few seeds.
- **Space capture:** no new 2026 visuomotor space-capture result was found beyond SpacePolicy and AstroBench.

**Recommended order** (replaces the SPEC_4 P4.4–P4.7 details where they differ):
1. **Environment:** a separate `lerobot` env in WSL with a pinned version, and the same version on Colab.
   - CPU torch is enough for conversion and evaluation.
   - Include mujoco and mjorbit so the evaluation runs our GraspSim directly.
2. **Data:**
   - Re-record the demonstrations with images at 10 Hz: 125 clean episodes plus 125 DART episodes, about 25 min of recording, one run at a time.
   - Convert them to LeRobot v3: bus camera at 128, lidar in 3 channels, 29-D state, 10-D action.
3. **ACT behaviour cloning on Colab:**
   - Chunk 20; train on the clean data first, then clean plus DART.
   - Evaluate on 25 validation seeds against the oracle and P4.3, with the E2 shield on.
4. **Expert suggest function:** use it for the `track` monitor, the hand-over shield and DAgger (2–3 rounds of 25 episodes).
5. **Variants, if time:**
   - modular ACT (the pose estimate as `environment_state`);
   - relative actions;
   - sensor ablation;
   - 250 episodes.
   - Diffusion Policy comes last.

## P4.8 research (13 Nov 2026, night): a learned correction for the last mile

Short pass from search summaries (abstracts only, full papers not read). Question: how should a learned policy sit on top of a classical approach when the grasp point moves?

**Residual (correction) policies on a classical controller:**
- Residual RL (Johannink et al., Siemens and UC Berkeley, arXiv 1812.03201): a hand-written feedback controller plus a learned correction, for contact-rich block insertion; the hand controller alone managed 2 of 20 with randomised block orientation.
- Residual Feedback Learning (Ranjbar et al., arXiv 2106.04306): a plain residual is limited by the controller's own feedback, which fights the correction; they also let the policy change the feedback signal. Relevant to us: our impedance loop will resist a correction added to its output, so the correction should go on the **target**, not on the torque.
- Sim-trained residuals for tight insertion (below 1 mm) on a potential-field controller, sparse rewards, zero-shot to hardware (NSF PAR 10599386, 10674874).
- Hybrid residual RL for book insertion (arXiv 2609.19962): phase-structured; the residual also decides **when to release**, which matches our close decision.
- Compliant Residual DAgger (arXiv 2506.16685): a residual with a force input that outputs a motion correction and a target force, learned from corrections to a running base policy. The closest match to "correction policy with force sensing, trained by imitation".
- FACTR 2 (arXiv 2606.12406): force or torque as one more observation token for an ACT-style policy.

**Capturing a spinning or tumbling target with an arm:**
- The standard requirement is near-zero relative velocity between the end effector and the grapple point at contact (Coordination Control of Free-Flyer Manipulators, arXiv 2303.10812; IAC-15 A2.4, "Dynamic balance of a space manipulator in the capture of a non-cooperative target", which notes that capture fails on excessive relative velocity).
- The classical recipe: estimate the target's motion with a Kalman filter, predict the grasp point ahead, and plan an intercept with zero relative velocity inside acceleration limits (pre-grasp optimal planning papers; Acta Astronautica 2017 on long-term grasp-point prediction; IAC-13 D3.2.8).
- No source gave a numeric relative-velocity limit; the full IAC 2015 and Acta Astronautica 2017 papers would.

**What this means for P4.8:**
1. Keep the predicted bar motion classical (filter plus feed-forward); the learned part corrects the target and decides when to close.
2. Put the correction on the impedance target (and bound it), so zero output is the classical approach.
3. Force or contact inputs are the natural addition for the contact phase.
4. Training options: residual RL with a sparse capture reward (the literature's usual choice), or imitation of a better expert. We have no better expert above the classical limit, which argues for RL or for an expert with privileged information (true pose and rate) teaching a policy that has only sensors.

Sources: https://arxiv.org/abs/1812.03201, https://arxiv.org/abs/2106.04306, https://arxiv.org/pdf/2609.19962, https://arxiv.org/pdf/2506.16685v2, https://arxiv.org/pdf/2606.12406, https://arxiv.org/abs/2303.10812, https://iafastro.directory/iac/archive/browse/IAC-15/A2/4/30115/, https://par.nsf.gov//servlets/purl/10599386
