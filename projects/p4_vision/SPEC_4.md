# Project 4: camera-based, imitation-learned grasp (draft, 12 Nov 2026)

Status: **draft for review. Nothing is built yet.** Research: [RESEARCH_SERVICING.md](../../learning/RESEARCH_SERVICING.md), "Project 4 research (12 Nov 2026)". Agreed direction (10 Nov): learning goes into perception and the grasp policy, contact stays classical, and the E2 monitors shield the learned policy.

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
5. **P4.4 ACT** (LeRobot, 0.5.x as SpacePolicy uses):
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
