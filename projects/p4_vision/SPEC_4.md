# Project 4: camera-based, imitation-learned grasp (draft, 12 Nov 2026)

Status (13 Nov): P4.0–P4.3 built and done, on the ESPA arm and on the realistic arm (arm2). **P4.4 onward replanned in §4.4** after a second research pass ([RESEARCH_SERVICING.md](../../learning/RESEARCH_SERVICING.md), "P4.4–P4.7 research"); §4.4 overrides steps 5–8 below where they differ. Original status: draft for review. Research: [RESEARCH_SERVICING.md](../../learning/RESEARCH_SERVICING.md), "Project 4 research (12 Nov 2026)". Agreed direction (10 Nov): learning goes into perception and the grasp policy, contact stays classical, and the E2 monitors shield the learned policy.

## 1. Question and claim

**Question:** can a policy that sees only camera images and its own servicer's state grasp the bar of a spinning client (≤ 2°/s) as reliably and as safely as the classical grasp, which reads the client's true pose from the simulator?

**Claim we can make:** sim-to-sim only. Same simulator, randomised rendering, held-out seeds. No real-image claim: research §5 shows the lighting gap is the hard part, and we have no real data.

**Success = the 3B/E1 criteria, plus a push limit:**
- both pads on the bar at the end of the close and the hold, the grip point within 2 cm of the bar axis, and bar slip < 2 mm/s (GraspSim's `success`);
- no unintended contact and no abort;
- **client push ≤ 2 mm/s.** 3B only reported the push and set no limit. This one was fixed on 12 Nov, before any estimator result: the ESPA expert's maximum is 0.9 mm/s, and a smoke test with an untrained estimator showed the grasp can still close 3 cm off the bar while pushing the client at 5.4 mm/s. Bar shift in the grip is reported, not gated.

**Safe = the E2 definition:** a monitor abort that reaches the safe state counts as safe, not as success.

## 2. Scope

- **Servicer and scene:** the ESPA servicer (`p3e_espa_scene.xml`) in free-flying mode. The bus holds its pose relative to the client, which removes the base-drift problem that SpacePolicy solves. A free-floating version is a later extension.
- **Phase learned:** reach, approach and the close decision, starting from the pre-grasp point after the classical fly-in (E1b). Fly-in, 3C and 3D stay classical.
- **The learned policy replaces the grasp target only.** The classical grasp gets its target from `grasp_target()`, which reads the client's true pose. The policy instead outputs the grip-point target, through the existing Cartesian impedance and jaw servo, which stay classical. This is the Cartesian end-effector action agreed on 10 Nov, so the policy is not tied to one arm.
- **Bus station keeping during the grasp** still uses the client's relative pose. This is a stated simplification: the bus could keep station on a filtered relative-navigation estimate, as in Project 2, but that is a separate perception problem. Make it explicit in results, and remove it as an extension (P4-ext-1).

## 3. Observations and actions

**Observation** (no client pose, no client rate):
- **Wrist camera:** 128×128 RGB above the jaws, looking along the approach axis.
- **Bus flash lidar:** 64×64 range image (added 12 Nov).
- **Bus camera:** 128×128 RGB.
- **Proprioception:** arm joint positions and speeds, jaw positions, and the grip-point pose and velocity in the bus frame.

**Action** at 10 Hz, ACT chunks of about 20, re-planned every step with temporal ensembling (research §1–2):
- the grip-point target as a change of pose (position and rotation vector) in the bus frame;
- a close flag (one bit, latched once set).

**Low level:** unchanged. The 100 Hz impedance tracks the interpolated target, and the jaws close on the flag.

## 4. Steps

1. **P4.0 Rendering** (checked on 12 Nov, `probe_render.py`):
   - `mujoco.Renderer` works under WSL2 with `MUJOCO_GL=egl` through Mesa: 28 frames/s at 224×224 while 20 CPU workers ran.
   - mujoco-warp 3.7.0.1 has `create_render_context` and `render`, the GPU batch renderer.
   - To do: measure the CPU renderer on an idle machine; try the GPU renderer for data generation; add the cameras to the ESPA scene, with `offwidth` and `offheight` set.
2. **P4.1 Domain randomisation:**
   - sun direction and intensity, an eclipse fraction, specular and texture variation on the client bus and bar;
   - camera noise, exposure, a small mounting error;
   - the client's mass properties, as in E3.
3. **P4.2 Demonstrations:**
   - Run the classical grasp (E1b fly-in, then grasp, monitors on) on seeds 8,000,000+ (a new demonstration split). Spin 0–2°/s about a random axis, with E3's errors.
   - Log at 10 Hz in LeRobot dataset format (Parquet plus MP4), successful episodes only.
   - Target: 500 episodes first (AstroBench's ACT used 400), then 2000 if the learning curve is still rising.
   - Record the expert's action at every step, so DAgger can reuse the code.
4. **P4.3 Fair baseline: classical on an estimated pose.**
   - A small keypoint or pose network on the same images estimates the bar pose; the classical grasp runs on that estimate.
   - This separates "learning perception" from "learning a policy" (research, ranked idea 2).
   - Without it, a learned-policy failure can't be put down to the policy or to perception.
5. **P4.4 ACT** (replanned 13 Nov, see §4.4; original text kept below) (LeRobot, 0.5.x as SpacePolicy uses):
   - Behaviour cloning on P4.2, trained locally on the RTX 4060 (ACT is about 52M parameters).
   - Pick the checkpoint on the validation split.
6. **P4.5 DAgger** (research §3):
   - Run the policy, have the classical expert label the states it visits, aggregate and retrain.
   - 3–5 rounds, with the expert's share decaying.
7. **P4.6 Shielded evaluation:**
   - The same E2 monitors and abort run on the learned policy.
   - The `track` monitor compares against the expert's suggested target instead of a planned path, because the policy has no path (research §4).
   - Report the abort rate.
8. **P4.7 Diffusion Policy:** the second learner, only after ACT is solid. Check its latency at 10 Hz, and whether short chunks keep it reactive at 2°/s.

### 4.1 Built (12 Nov): P4.0–P4.2

Files are in `projects/p4_vision/`; run them with `bash run_p4.sh <script> ...` (rl env, `MUJOCO_GL=egl`).

- **`make_scene.py`** generates `p3_capture/p4_espa_cam_scene.xml` from the frozen ESPA scene. It adds:
  - **wrist_cam:** on the gripper body, above the open upper jaw, looking between the jaws, fov 75°. The first mount, behind the jaw, was blocked by it.
  - **bus_cam:** on the +x face, above and beside the shoulder, aimed at the nominal bar, fov 40°.
  - a directional sun with shadows, the headlight off, and materials whose shine can be randomised.

  Nothing physical changes. **Check (`check_cam_physics.py`):** 4/4 episodes re-run on the plain ESPA scene without the recorder came out identical in outcome, push, bar shift and grasp time.
- **`cams.py` (`CamRig`)** renders 128×128 from GraspSim's synced plain-MuJoCo copy. Sites are hidden, because the green grip-point marker showed in the first images.
  - **Randomised per episode:**
    - sun direction, intensity and specular; ambient light;
    - client, bar and servicer colour, and material shine;
    - each camera's mount (±1 cm, ±1°);
    - exposure gain and pixel noise.
  - **Sun range:** the direction to the sun is kept within 70° of the grasp-face normal at the start. Uniform sampling left half the episodes with the face in full shadow, and real approaches are planned with the face lit.
  - **Held-out lighting:** a 15° cone, 50° from the face normal towards the orbit normal, is never used in training (`--holdout-sun` for evaluation).
- **Flash lidar (added 12 Nov, at the user's request):** real servicers fly visible and infrared cameras plus a flash lidar (NASA Raven: a 256×256 flash lidar).
  - `bus_lidar` sits 6 cm below the bus camera, fov 45°, and records a 64×64 range image in millimetres (0 = no return), range 0.3–20 m.
  - **Noise model:** a range-noise floor of 0.5–2 cm per episode plus 0.5% of range; random dropout of 1–5% per episode; 25% dropout on depth-edge pixels; more dropout at grazing incidence. Lighting has no effect.
  - An earlier version dropped every edge pixel, which erased the 3-pixel-tall bar.
  - **Check:** with the arm stowed, range at the bar-centre pixel minus the true distance = −2.3 to −2.8 cm (median per episode) = the bar's near face, 2.5 cm in front of its centre.
  - **Why:** the lidar is independent of lighting, where the cameras are weakest. A dim episode shows the bar clearly in range while it's hard to make out in the images.
  - Storage is about 8 KB per frame.
  - **Not modelled:** the thermal-IR camera (MuJoCo has no temperature model).
- **`grasp_sim.py`** got a `step_hook` (None by default), called once per control step with the loop's local variables. The no-op path is re-checked against p3b on 20 E3 missions after the ESPA validation run (`run_after_val.sh`).
- **`gen_demos.py`** records the classical expert at 10 Hz:
  - start: synchronised at the pre-grasp point, spin 0–2°/s about a random axis, E1 start errors;
  - monitors on; hold 5 s;
  - only successful episodes are saved, one `.npz` per episode on the WSL disk (`/home/shrey31/p4_data/<set>/`).
  - **Contents:**
    - images (T, 2, 128, 128, 3);
    - state (29);
    - action (10): the expert's grip-point target in the bus frame, plus a close flag. The target is the forward kinematics of the joint reference while reaching, then the impedance target.
    - lidar (T, 64, 64) uint16 range, mm;
    - privileged labels (39): bar pose in the bus frame and in each camera and lidar frame, client rate.
  - **Smoke test (4 episodes):** 4/4 succeeded, 463 frames each (46 s), about 17.5 MB per episode. The grip point tracks the logged target to within 4 mm median while reaching and 2 mm while approaching.
  - The LeRobot conversion comes with P4.4, in its own environment: LeRobot, pyarrow and ffmpeg are not in the rl env, and installing LeRobot there could change its torch.
- **`contact_sheet.py` / `run_sheets.sh`** make image sheets per episode (`sheets/`).
- **Next:** `demos_v1`, 500 episodes (`run_after_val.sh`, after the ESPA validation run). Then P4.3 (pose-estimator baseline) and P4.4 (LeRobot conversion, ACT).

### 4.2 P4.3 design (12 Nov)

**demos_v1** (500/500 expert episodes succeeded, 231,500 frames, 9.7 GB, 22 min on 20 workers).

**What the estimator must achieve (`data_stats.py`):**
- The bar moves in the bus frame during an episode by 12 cm median, up to 30 cm: the bus is still converging on its station.
- A "predict the mean pose" guess is off by 48 mm median (p95 108 mm) and 0.4° (p95 2.8°).
- The close trigger needs the grip point within 1 cm of the bar, so the estimator must reach about 5 mm or better while approaching.

**Estimator:**
- **Input:** wrist and bus images, lidar range, and the arm joint angles (the wrist camera moves with the arm, so the network needs to know where it is).
- **Output:** bar position (3) and rotation (6) in the bus frame. The client's shape is known (a cooperative target), so the client pose follows from the bar pose.
- **Network:** a small CNN per sensor, fused with the joint angles in an MLP. Trained on the GPU from a disk memmap (the uncompressed data is about 23 GB, more than the 15 GB of WSL RAM).
- **Split:** demos_v1 episodes 0–449 train, 450–499 select the checkpoint. Closed-loop evaluation uses fresh seeds.

**Classical grasp on the estimate (closed loop):**
- GraspSim gets a `pose_est` hook (None = truth, as now; re-checked bit-identical on p3b).
- Where the arm controller reads the client's pose (IK goal, approach target, close check, velocity feed-forward, close and hold references), it reads the estimate instead. Client velocity and spin come from a filter on the estimates.
- **Still on truth, as stated limitations:** the bus station keeping and the E2 monitors.
- **Report:** estimator error per phase (validation episodes), then closed-loop success, push, bar shift and aborts on the same seeds as the oracle.

### 4.3 P4.3 result (13 Nov): the estimate is as good as the truth for the grasp

**v1 (Colab T4, 20 epochs, mounts randomised ±1 cm, ±1°):** best epoch 5, 13.8 mm median overall, 11.7 mm while approaching; it overfit after that. `mount_floor.py` showed why: an unknown per-episode mount error of that size puts a 16–17 mm floor under any single-sensor estimate, and no image reveals it.

**v2 (what changed):**
- **Sensors:** mounts calibrated to ±1 mm, ±0.1° (floor 1.4 mm); bus camera 192 px, lidar 128 px; images every 0.25 s with labels at the image time.
- **Training:** photometric augmentation only (no image shifts: where the bar sits in the image is the signal).
- **Data:** `demos_v2`, 500/500 episodes, 7.4 GB; train on episodes < 450 (83,695 frames), select on the rest (4,649).
- **Training run:** Colab T4, 30 epochs, 234 min. Best epoch 23 (`best_v2_1.pt`); validation flat from epoch 19 while training loss kept falling.

**Estimator error (held-out episodes, median / p95):** all frames 2.2 / 6.1 mm and 0.17 / 0.55°; approach 2.0 / 5.5 mm; close 1.7 / 3.8 mm.

**Closed loop (classical grasp on the estimate, `eval_est.py`, same seeds as the oracle):**

| Run | Oracle (true pose) | Estimate |
|---|---|---|
| Validation, 200 seeds | 200/200, max push 0.99 mm/s | **200/200** (CI 98.1–100%), max push 1.02 mm/s, max bar shift 1.94 mm |
| Held-out sun, 200 validation seeds | – | **200/200**, max push 1.06 mm/s |
| **Test, 300 seeds, run once** | 300/300, max push 1.06 mm/s | **300/300** (CI 98.7–100%), max push 1.52 mm/s, max bar shift 1.94 mm |

- 0 collisions, 0 aborts and 0 pushes over 2 mm/s in every run.
- Estimate error at close: validation 2.1 mm median, 5.3 mm p95, 7.0 mm max; test 2.2 / 4.4 / 9.4 mm. The worst test seed for push (7000082, 1.52 mm/s against the oracle's 0.77) had a 7.2 mm close error.
- **Held-out lighting costs nothing:** error at close 2.0 / 4.4 / 6.0 mm.
- **Spin barely matters** (`spin_breakdown.py`): every 0.5°/s bin succeeds 100%; median error at close 1.8–2.3 mm across bins; correlation of spin with that error +0.12. Bar shift grows with spin in the oracle too (0.46 → 1.81 mm), so it comes from the classical grasp.
- **Open:** the median push is lower on the estimate (about 0.45 mm/s) than on the truth (about 0.8 mm/s), with the same maximum; probably the estimate's smoothing or its latch at close. Not investigated.
- **Limits, as designed:** the bus station keeping, the fly-in and the E2 monitors still use the truth; only the arm controller sees the estimate.

**Realistic arm (arm2, 13 Nov, after P4.3):** the servicer arm was given joint modules, gear inertia, torque limits and friction (`p3e_espa_arm2_scene.xml`, SPEC_3 and `learning/RESEARCH_SERVICING.md`). It changes how the arm looks on camera, so P4 continues with an arm2 camera scene and new demonstrations before P4.4.
- **Done 13 Nov:**
  - arm2 camera scene (`make_scene.py --arm arm2`);
  - `demos_v3`: 125 episodes;
  - v3 fine-tune of the pose network: 1.6/3.5 mm;
  - estimate grasp on arm2: 25/25, as the oracle.

### 4.4 Plan from P4.4 on (13 Nov, replanned)

All of this runs on **arm2** with the v3 network as the P4.3 baseline. Runs are about 25 seeds (§5). One sim job at a time; GPU training on Colab only.

**What the grasp code fixes for the policy** (`grasp_sim.py`, read 13 Nov):
- **Reach is joint-space:** a minimum-jerk path from stow to an IK solution of the pre-grasp pose. **Approach is Cartesian impedance** toward a target that slides along the bar normal.
- **Both expert targets depend only on the bar pose and the time in the phase, not on the arm's state.** So the expert can relabel any state the policy visits, given the true bar pose. A "suggest" function is about 50 lines, not a large job.
- **The policy has no clock.** It must read progress from the images and the arm state. This is normal for ACT.
- **The approach feed-forward `v_ref` uses the client's velocity and spin, which the policy does not have.** The policy's `v_ref` comes from the difference of its own target chunk instead.
- **Close and hold freeze the grip relative to the client.** With the policy they use the v3 pose estimate latched at close, as in P4.3, so contact stays classical but runs on estimated perception.
- **Bus station keeping stays on the true relative pose,** a stated simplification (§2).

**Steps:**
1. **P4.4a Policy hook in `grasp_sim.py`** (then re-freeze as v1.3).
   - `self.policy = None` is a no-op, checked bit-identical on p3b, ESPA and arm2.
   - When set, it is called at 10 Hz in reach and approach, and returns the grip target (bus frame), the rotation and the close flag.
   - Reach: target → one warm-started IK step → joint PD. Approach: impedance.
   - The policy's close flag replaces the expert's distance test; the approach timeout still applies.
   - The same hook carries DART noise (step 2) and later the ACT policy, the hand-over shield and DAgger. Also add `expert_suggest(sim)`: the expert's target for the current state (above).
2. **P4.4b Demonstrations at 10 Hz, clean and DART** (`gen_demos.py --img-steps 10`).
   - The images must be at the action rate: `demos_v3` has images at 4 Hz and actions at 10 Hz, and a LeRobot dataset has one rate.
   - `demos_v4`: 125 clean episodes, plus 125 DART episodes. DART runs the expert's target with noise (a few mm and about 0.5°, an Ornstein–Uhlenbeck process, ramped to 0 within 3 cm of the bar) but labels each frame with the clean expert target.
   - About 9.5 GB and 30 min, as two runs one after the other (disk: 935 GB free).
   - Check: the clean set reproduces the `demos_v3` outcomes seed for seed.
3. **P4.4c LeRobot env and conversion.**
   - The `rl` env has torch 2.13 and LeRobot 0.6 supports torch 2.7–2.11, so LeRobot gets its own env: Python 3.12, LeRobot **0.6.x pinned** (0.7.0 is a week old, with breaking changes), CPU torch, and the same version on Colab.
   - Conversion (`to_lerobot.py`, run in that env):
     - `observation.images.wrist`, `.bus` (192 → 128, since ACT needs one image size) and `.lidar` (range packed into 3 channels: range, inverse range, return mask);
     - `observation.state` 29-D;
     - `action` 10-D (target position, rot6, close flag);
     - 10 fps, local `root`, no Hub push, `finalize()`.
   - Check: decoded frames equal the npz arrays within video compression (report the PSNR).
4. **P4.4d ACT on Colab.**
   - Settings: `chunk_size` 20 (2 s), temporal ensembling 0.01 with `n_action_steps` 1, ResNet-18 ImageNet, `kl_weight` 10.
   - Training: clean data first, then clean plus DART. Episodes 110–124 (and the matching DART episodes) are validation.
   - Pick the checkpoint by validation action error (all frames and approach frames), as P4.3 did by position error.
5. **P4.4e Closed-loop evaluation.**
   - Same 25 validation seeds as P4.3, E2 monitors on, rendering in the loop at 10 Hz.
   - Compared: oracle, P4.3 estimate, ACT clean, ACT clean plus DART.
   - ACT runs on CPU (about 0.1–0.2 s per call): either in the LeRobot env, if mujoco and mjorbit install there, or as a small policy server in the LeRobot env that the sim in the `rl` env calls through a local socket. Smoke-test the first and fall back to the second.
6. **P4.5 Hand-over shield and DAgger.**
   - **Hand-over:** before contact, if the policy's target strays from `expert_suggest` by more than a bound (start at 2 cm or 2°), the classical expert takes over for the rest of the grasp. Reported separately: success, safe, aborts, hand-overs.
   - **DAgger:** 2–3 rounds of 25 episodes on new demonstration seeds. The policy runs with a decaying expert share; every frame is labelled with `expert_suggest`; aggregate and fine-tune on Colab.
   - Correction (13 Nov, from `safety.py`): the E2 `track` monitor checks the **bus** against its station-keeping reference, not the arm. It needs no change for the policy; the arm-side check is the hand-over bound above.
7. **P4.6 Shielded evaluation and conditions.** The best ACT, with the E2 monitors (and separately with the hand-over), is run on these conditions:
   - nominal at 0, 1 and 2°/s;
   - the held-out sun;
   - a client kick;
   - sensor ablations: no lidar, and lidar plus wrist only.
   - Then the test split once (25 seeds, §5).
8. **P4.7 Variants, as time allows, each against the same 25 seeds:**
   - modular ACT (the v3 pose estimate as `observation.environment_state`);
   - targets relative to the current grip point;
   - 250 clean episodes (learning curve);
   - Diffusion Policy last: about 263M parameters, Colab training, slow on CPU, so few seeds.

**Decisions made in this replan:**
- **DART before DAgger:** the cheapest fix for compounding error, with a scripted expert.
- **A hand-over to the classical expert as a middle layer under the E2 abort,** because our expert can take over from any state.
- **ACT's own temporal ensembling** instead of long chunks: our targets move with the spin.
- **Contact and hold stay on the P4.3 estimate.**

**P4.4a done (13 Nov):**
- `grasp_sim.py` has `policy` (None = expert), `expert_suggest()` and `_policy_step()`.
  - Reach: warm-started IK (50 iterations) to a joint reference, its rate by finite difference.
  - Approach: impedance, with the feed-forward from the target's own rate.
  - The close flag acts one step later. The approach timeout is kept.
- **No-change checks (policy None):**
  - p3b, 20 E3 missions (`run_e3_wiring.sh`): **20/20 identical**;
  - arm2, 25 E3 missions (`run_e3_arm2.sh validation 25`, against a saved copy): **25/25 identical**.
- **Frozen as v1.3** (`e3_espa_freeze.sha256`, all hashes OK; v1.2 kept).
- `p4_vision/policies.py`: `ExpertPolicy` (the expert through the hook) and `DartPolicy`.
  - DART noise: Ornstein–Uhlenbeck, τ 0.5 s, 2 mm and 0.3° per axis, the norm capped at 2.5σ, ramped to zero between 6 and 3 cm from the bar.
- **`policy_smoke.py`, 8 validation seeds, arm2, monitors on:**

  | Mode | Success | Push and bar shift vs the expert | Grasp time | Approach tracking error max |
  |---|---|---|---|---|
  | Expert | 8/8 | — | — | 11.5 mm |
  | Expert through the hook | 8/8 | equal | +0.01 s | 7.1 mm |
  | DART | 8/8 | equal | — | 9.2 mm |

  - The hook's tracking is tighter because the finite-difference feed-forward includes the blend.
  - A first DART try at 4 mm per axis, uncapped (peaks of 17 mm), gave 2/8 `clear_pred` safe aborts.

**Decided by the user (13 Nov):**
- **ACT starts on images only:** wrist, bus and lidar, plus the 29-D proprioceptive state. The v3 pose estimate stays out of the input and is used only for the close/hold reference. The modular variant stays in P4.7.
- **Bus camera 128 px for ACT.** Demonstrations still record it at 192 (the v3 network and the latched hold estimate need 192), and the conversion resizes it.
- **Lidar included** as a third image (3-channel packing above). If ACT cannot use it well, the no-lidar ablation in P4.6 shows that.

### 4.5 P4.8: the learned last mile as a correction policy (13 Nov, draft)

**Why the plan changed.** End-to-end ACT (§4.4) fails closed loop: 3/25 with temporal ensembling, 5/25 with a classical reach, and 0/25 for both when only the newest prediction is executed (the raw output is noisy by tens of mm; one collision). Offline, its approach error stays at 5–7 mm from 16k to 40k steps while the reach error keeps falling, so the approach is limited by perception and the action form, not by training time. The P4.3 estimator gives 1.6 mm from the same sensors. Scope agreed with the user: classical does everything up to the pre-grasp point and after capture; learning goes into the last 12 cm.

**Split:**
- **Classical:** spin estimate, fly-in, co-rotation, the reach to the pre-grasp point, the predicted bar motion fed forward to the arm, braking, detumble, identification, stack moves.
- **Learned:** a bounded correction added to the classical approach **target** (not the torque: the impedance loop would fight it), and the close decision. Zero output is the classical approach.
- **Inputs to try:** the P4.3 estimate and its rate, the grip-point pose relative to that estimate, wrist force/torque, jaw contact, a short-range distance sensor between the jaws; images only if these are not enough.

**Where classical stops (spin sweep, `run_spin_sweep.sh`, 25 validation seeds per row, arm2, monitors on, episodes start synchronised):**

| Spin | True pose | P4.3 estimate | What goes wrong |
|---|---|---|---|
| 2°/s | 25/25 | 25/25 | nothing; push ≤ 1.1 mm/s |
| 3°/s | 19/25 | 13/25 | captures 24 and 21, but the client push exceeds 2 mm/s in 5 and 9 (up to 8.5 mm/s); estimate error at close 8.4 mm (2.6 at 2°/s) |
| 4, 5, 6°/s | 11/25 | 11/25 | 14 `track` aborts during reach: the bus cannot hold station |

- By spin axis (true pose): about the approach axis (axis 0, the bar barely moves) 11/11 at every rate up to 6°/s; about the two axes that sweep the bar, 8/14 at 3°/s and 0/14 from 4°/s.
- **Correction after tracing two 3°/s episodes (`dbg_push.py`, seeds 6000000 and 6000005, true pose): the 3°/s limit is thrust too, not the arm.**
  - Co-rotation at 3°/s asks 0.95–1.1 N per axis against the 0.75 N limit. The bus is already 75–200 mm off station during approach (and over 500 mm in hold); the arm makes up the difference, and in one case stalls 12 mm short for 5 s near its reach limit.
  - The client push is not a contact fault. Before the jaws close it is 0.26–0.35 mm/s. It rises to 2.4–2.9 mm/s during the 3 s close and to about 6.5 mm/s in hold, because once the grip is made the client supplies the centripetal force the thrusters cannot (about 0.3 N on 300 kg is 1 mm/s², which matches).
  - Demand scales with the square of the rate, so the thrust limit sits near 2.6°/s for this standoff.
- So **no arm-side correction can fix the 3°/s rows**: the fault is a servicer that is not co-rotating when it attaches. The spin-rate task as first drafted is withdrawn.

**What a learned last mile can still be for (to choose):**
1. **Camera calibration error.** P4.3's 25/25 needs mounts known to ±1 mm; at ±1 cm the estimate had a 16 mm floor. The wrist camera sees the jaws and the bar together, so a policy that servoes on that view does not depend on the mount calibration. This is the clearest gap the classical-on-estimate grasp has at ≤ 2°/s.
2. **A wobbling (nutating) client,** where the bar does not follow the predicted circle.
3. **Spin above 2.6°/s,** only after a classical mode exists for it (a closer standoff, or partial co-rotation with the arm following the bar, plus a planned momentum exchange at capture). The learned part would then be the arm's tracking of a bar that moves at cm/s.

**Task chosen by the user (13 Nov): 1, camera calibration error.** Baseline, classical grasp on the v3 estimate, 25 validation seeds, spin 0–2°/s (`run_mount_sweep.sh`):

| Mount error (per axis, per sensor) | Success | Captured | Push over 2 mm/s | Estimate error at close, median / max |
|---|---|---|---|---|
| ±1 mm, 0.1° (as trained) | 25/25 | 25 | 0 | 2.2 / 6.2 mm |
| ±3 mm, 0.3° | 25/25 | 25 | 0 | 4.7 / 12.4 mm |
| ±5 mm, 0.5° | 24/25 | 25 | 1 | 7.6 / 17.3 mm |
| ±10 mm, 1° | 16/25 | 24 | 8 | 16.1 / 30.4 mm |

No collisions or aborts in any row: the grasp still closes on the bar, but off-centre, and pushes the client. **Test condition: ±10 mm, 1°. Target: back to at least 24/25.**

**Proposed design (draft, not yet agreed):**
- A small network reads the wrist image and outputs the bar's position relative to the grip point (the fingers and the bar are both in that image, so the camera mounts drop out). Labels come from the simulator.
- Over the last 12 cm the classical approach servoes on that offset, and closes when it is inside the 1 cm gate. The v3 estimate stays as the coarse pose for the reach.
- Data: new demonstrations recorded with ±10 mm, 1° mount error. Training on Colab.
- Reinforcement learning is kept for afterwards, only if contact or close timing still fails with a good offset.

**First build and result (13 Nov; details in CONTEXT.md):**
- Built: `palm_cam` between the jaws (the old `wrist_cam` never sees the pads, `probe_wristcam.py`); `policies.BiasPolicy` and `gen_palm.py` (150 off-centre approach episodes, 32,827 frames, mounts ±10 mm, 1°); `palm_net.py`, `train_palm.py` (CPU, 20 min); `pose_est.PalmPoseEst`; `eval_est.py --mode palm`; runner `run_palm_build.sh`.
- Network, validation median error: across the jaws 1.9 mm, depth 6.1 mm, along the bar 11.6 mm (unobservable on a uniform bar, and the grasp ignores it; the v3 value is kept), rotation not learned (training tilts were only ±2°; the v3 rotation is kept). It overfits at 128 training episodes.
- Ungated, the palm reading caused collisions (7 to 9 in 25): out of view or in the dark it still answers "centred". **Coarse-to-fine gate:** palm is used only where v3 puts the bar in the palm view and the two agree within 4 cm.

| Mount error | v3 alone | v3 + gated palm | Typical push, v3 / palm |
|---|---|---|---|
| ±1 mm, 0.1° | 25/25 | 23/25 | 0.50 / 0.57 mm/s |
| ±5 mm, 0.5° | 24/25 | 23/25 | 0.93 / 0.56 mm/s |
| ±10 mm, 1° | 16/25 | **22/25** | 1.70 / 0.75 mm/s |

- 0 collisions and 0 aborts in every gated run. The palm correction makes the result nearly independent of calibration, but has its own floor of about 2 hard pushes in 25, so it does not reach the 24/25 target and is slightly worse than v3 at 1 mm.

**Second build (13 Nov): palm light and more data.** Tracing the hard pushes (`dbg_palm.py`) showed the palm image going black near the bar (the gripper shadows it) and the dark pads vanishing, so the network called the bar centred while it was 6–15 mm off. Added `palm_light`, a spot beside the palm camera that is on only while `palm_cam` renders (other images and the physics unchanged; oracle 25/25 identical). Re-recorded 300 episodes (`palm_v2`, 65,706 frames) and retrained (`palm_net_v2`, 17 min on CPU): validation median error across the jaws 0.95 mm, depth 2.3 mm.

| Mount error | v3 alone | v3 + gated palm v2 | Push with palm v2, median / max |
|---|---|---|---|
| ±1 mm, 0.1° | 25/25 | 25/25 | 0.31 / 0.62 mm/s |
| ±5 mm, 0.5° | 24/25 | 25/25 | 0.41 / 0.75 mm/s |
| ±10 mm, 1° | 16/25 | 25/25 | 0.47 / 0.99 mm/s |
| ±10 mm, 1°, fresh seeds 6000100–124 | 16/25 | 25/25 | 0.50 / 1.75 mm/s |

- 0 collisions, 0 aborts and 0 over the push limit in every palm v2 run. The fresh-seed row is the fair one: the darkness problem was found by tracing three of the first 25 seeds.
- **Target met** (at least 24/25 at ±10 mm, 1°). At ±10 mm over both seed sets: 32/50 without the palm correction, 50/50 with it.
- **Limits:** rotation and the position along the bar still come from v3; spin ≤ 2°/s; the arm's own kinematics are taken as calibrated; validation seeds only (the test split is unused).

**Open choices (user):**
- What next: the full demo (30 m start, capture, align, release, retreat), palm rotation, or the test split.
- (Earlier) How far to take the palm network (trace the hard pushes, more data, larger tilts, a bar-in-view output). The user chose reinforcement learning as the training method (13 Nov) for the spin-rate task; whether it is still the right method depends on the task (task 1 could be plain supervised learning of the jaw-to-bar offset from the wrist image).
- Which extra sensors to add to the scene first (force/torque and jaw contact are the cheapest: the simulator already has the contact forces).

## 5. Evaluation

- **Splits:** demonstrations 8,000,000+; validation 6,000,000+ (E3 validation seeds, grasp phase only, after the fly-in); test 7,000,000+, used once.
- **Sizes:**
  - P4.3 used validation 200 and test 300 (CI half-width about ±3% near 95%; an unsafe rate of 0 bounded at about 1%).
  - **From 13 Nov, about 25 per run** (the user's call: a demonstration project, quick experiments). The confidence intervals are wide and are reported as such.
- **Compared, on the same seeds:**
  1. classical with true pose (oracle);
  2. classical on estimated pose (P4.3);
  3. ACT with behaviour cloning;
  4. ACT with DAgger;
  5. later, Diffusion Policy.
- **Conditions:**
  - nominal at 0, 1 and 2°/s;
  - a client-kick fault (does the shield still catch it?);
  - a rendering shift held out from training: a sun direction outside the training range;
  - sensor ablation: cameras only, cameras plus lidar, lidar plus wrist camera. Does the lidar make the policy robust to lighting?
- **Metrics:**
  - success, safe and unsafe (with Wilson 95% confidence intervals);
  - client push, bar shift, peak contact force;
  - grasp time, abort rate and reason;
  - policy latency.

## 6. Risks and how we'll know

- **Specular glare and eclipse wash out the bar.** Measure success against sun angle, and keep a held-out lighting condition.
- **A spinning target needs reactivity.** Compare success at 0, 1 and 2°/s; if 2°/s drops, try client-rotating-frame actions (research §2).
- **Over-abort by the shield.** Report it. A smooth safety filter is a later extension.
- **Dataset size and disk:** 500 episodes × about 60 s × 10 Hz × 2 cameras is about 600k frames, and MP4 keeps that manageable. Rendering throughput decides whether the GPU renderer is needed.
- **mjorbit physics vs plain-MuJoCo rendering:** render from the synced plain-MuJoCo copy (`GraspSim.kd`), which already mirrors qpos every control step.

## 7. Extensions (later)

1. **P4-ext-1:** the bus keeps station on a camera-based relative-pose estimate.
2. **P4-ext-2:** a free-floating base, conditioning on a forecast of the base's motion (SpacePolicy).
3. **P4-ext-3:** fly-in from images.
4. **P4-ext-4:** SmolVLA, only if language commands are wanted.
