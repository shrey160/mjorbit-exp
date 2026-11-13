# mjorbit project context

Last updated: 13 November 2026.

This file records the user's goals, work completed, verified setup, and lessons from this conversation. Read it when continuing the project. Runtime details such as whether the viewer is still running must be checked again.

## Resume here (next session)

**PLAN FOR THE NEXT SESSION (written 13 Nov, night, at the user's request). Start here.**

*Where things stand:* end-to-end ACT is dropped (it fails closed loop; details in the "13 Nov, night" block below). The learned last mile is now a **palm-camera correction** on top of the classical grasp: at ±10 mm, 1° camera-mount error the grasp is 25/25 with it and 16/25 without, on fresh validation seeds. A **full demo** (30 m start, camera-guided grasp, move and turn, release, retreat) runs and is published. **Nothing from 13 Nov is committed.** One sim job at a time; runs of about 25; CPU only for training.

**Step 1. Housekeeping (ask before each destructive or outward action).**
1. Commit in three parts, only when the user says so: (a) ACT pipeline and its negative result (`act_policy.py`, `eval_act.py`, `run_act_eval.sh`, `val_act.py`, `to_lerobot.py`, `policies.py`, the P4.4 scripts and notebook, `grasp_sim.py` policy hook and freeze v1.3); (b) palm-camera correction (`make_scene.py`, `cams.py`, `pose_est.py`, `eval_est.py`, `gen_palm.py`, `palm_net.py`, `train_palm.py`, `run_palm_*.sh`, the `dbg_*`/`probe_*` scripts, sweeps); (c) demo (`demo_stage_a.py`, `export_demo.py`, `run_export_demo.sh`) plus CONTEXT, SPEC_4, RESEARCH. Result JSONs: follow what earlier commits did (check `git ls-files projects/p4_vision | grep json`). `replay/` and `*.pdf` are gitignored.
2. Stray temporary scenes in `projects/p3_capture` (`.case_*.xml`, `.demo_release.xml`): list them, confirm with the user, then delete; add `.demo_release.xml` to `.gitignore`.
3. Rebuild the learner's guide (`learning/make_p44_guide.py`): it still presents ACT and a retrain as the plan. Rewrite sections 5–7 around: why ACT failed, the thrust limit found in the spin sweep, the palm camera and light, the demo.

**Step 2. Make the palm result solid (about 1 hour of runs).**
1. Test split, once: seeds 7,000,000–24 at ±10 mm, 1°, v3 alone and v3 + palm v2 (`eval_est.py --split test`; add `--split` pass-through to `run_palm_build.sh eval`). Report with Wilson intervals. Do not tune on it afterwards.
2. Robustness on validation seeds 6000100–124: held-out sun (`--holdout-sun`), and the result by spin rate (0–2°/s bins, from the per-episode `spin`).
3. Palm rotation: new data with larger tilts (`BiasPolicy(rot_deg=6)` or so, 300 episodes, `PV=v3`), retrain, then compare `--palm-no-rot` against palm rotation at ±10 mm. Keep the gate. Watch for the rotation feedback runaway seen with v1.
4. Write SPEC_4 §4.5 final table and one LEARNINGS entry (measure before defining a learning task: the spin sweep and `dbg_push.py` showed thrust, not the arm, was the limit).

**Step 3. Plan (do not build yet) "the servicer body's own eyes" as P5.** Research pass first (RESEARCH_SERVICING.md rule), then a short spec for the user to review:
- Goal: replace the true client relative pose, velocity and spin in bus station keeping, the fly-in and the clearance/rate monitors with an estimate from the bus camera and lidar.
- Pieces: a client-pose network on the bus sensors at 8 m to 1.25 m (the v3 network only covers the bar at grasp range); a filter giving smooth relative velocity and the spin axis and rate (extend the Project 2 nine-state filter with attitude states); monitors on the estimate with their thresholds re-tuned on a tune split.
- Measure first: how much pose noise and delay the station keeping and fly-in tolerate (inject noise into the true pose and sweep it), so the estimator has a target.

**Step 4. Later, in this order unless the user says otherwise:** test the release on a batch of missions with a wheel model; faster spin (needs a classical partial-co-rotation mode; thrust limit is near 2.6°/s); wrist force and jaw contact inputs; RL only for gentler contact.

**Answered by the user on 13 Nov, night ("yeah, go ahead"):**
- **Commit: done** on branch `p4.8-palm-camera-and-demo` (three commits: ACT pipeline and negative result; palm-camera correction; demo and docs). Not pushed and not merged into `main`: ask.
- **Stray files:** there are 276 `.case_*.xml` (11–13 Nov) plus `.demo_release.xml` in `projects/p3_capture`, not the "about ten" I had told the user, so I asked again before deleting. All are gitignored, regenerated on every run, and no job was running. Check the user's answer before deleting.
- **Next project:** taken as "own eyes" (P5, Step 3 below), the first option and my recommendation; the user did not pick explicitly, so confirm at the start.
- Step 1.1 is therefore done; Step 1.2 waits on the answer; Step 1.3 (guide) is open.

**13 Nov, evening (superseded by the plan above and the "13 Nov, night" block below): ACT closed-loop results.**
- **State:** P4.4 ACT closed loop works mechanically but grasps poorly. 20k checkpoint: **3/25** temporal ensembling, **2/25** chunk execution (re-plan every 20 frames), **5/25 hybrid** (classical reach + ACT approach and close). The oracle and P4.3 are 25/25. 0 collisions; E2 aborted every bad episode. The shadow run proves the wrapper is correct.
- **Diagnosis:** the policy has no clock. The expert is time-indexed (20 s min-jerk reach, then a 1 cm/s approach); ACT learned "target ≈ gripper + δ" (copycat), so a lagging arm stays lagging. Details are in the P4.4e bullets further down this file (search "P4.4e").
- **The user said "Yes, prepare retrain", then interrupted before any work.** Ask before starting. Plan:
  1. add time-in-phase (or phase progress) to the state, and consider dropping the gripper pose and velocity;
  2. re-convert with `to_lerobot.py` (needs t and phase from the npz: `phase` and `t` exist; reach and approach start times follow from the phase changes);
  3. a new Colab upload and notebook run (dataset name v5);
  4. `val_act.py` and `eval_act.py` must build the same state (`act_policy._obs`).
  - Then DAgger (P4.5).
- **Colab 40k resume:** started 13 Nov 13:54 UTC (about 37 min). The user should re-run notebook cells 6–7 and paste the table; not seen yet. It will not fix the clock problem.
- **Learner's guide for the user:** `learning/P4_4_ACT_guide.pdf` (5 pages: pipeline, offline and closed-loop results, diagnosis, next steps, how to help, reading list, glossary). Rebuild with `python learning/make_p44_guide.py C:/Shrey_Projs/robotics/mjorbit` (Windows python with reportlab; `*.pdf` is gitignored).
- **New uncommitted files this session:**
  - `p4_vision/act_policy.py`, `eval_act.py`, `run_act_eval.sh` (flags `--ens temporal|none`, `--replan k`, `--shadow`, `--expert-reach`);
  - `learning/make_p44_guide.py`;
  - plus everything listed earlier as uncommitted.
  - Results are in `p4_vision/p44_val_act_{act20k_te,act20k_k20,act20k_hyb_te,shadow,smoke*}.json` and the `act_eval_*.log` files.
  - The 20k model is in WSL at `/home/shrey31/p4_data/act_v4/run1/020000/pretrained_model`; the zip is in `colob_ops/check_20000.zip`.
- **Running ACT eval:** `bash run_act_eval.sh <ckpt> <tag> [N] [W] [args]` in the LeRobot env, which imports `mjorbit` from `simulator/src` (it works). Write .py files with LF endings: Windows Python's `open(..., 'w')` wrote CRLF once; fixed with `sed -i 's/\r$//'`.

**13 Nov, night: review of P4.4 and two no-retrain checks (user: "Run both checks").**
- **40k log pasted (training only, to 38k):** l1 0.057 → 0.042, 8.5 steps/s, `grdn:nan` once per epoch at the epoch boundary (loss stays finite; probably one AMP-skipped step). No validation table yet; the 40k checkpoint is not on this PC.
- **40k offline val table (pasted later; median mm all / reach / approach, close accuracy 1.0 throughout):** 20k 8.7 / 18.2 / 5.3; 28k 7.2 / 11.5 / 5.9; 36k 6.6 / 10.8 / 5.9; **40k 5.9 / 10.7 / 5.2**. Reach kept improving (18 → 11 mm); **approach has been flat at 5–7 mm since about 16k**, so the approach error is not from undertraining (more likely perception: 128 px, 4×4 feature grid, absolute targets). No reason to evaluate 40k closed loop under the new scope.
- **Review findings (code and saved results read):**
  - Undertrained: 20k steps × batch 32 is about 6 passes (ACT recipes use hundreds); settle error 22 mm on a constant label.
  - DART noise (2 mm, cap 5 mm) is far below the closed-loop deviations (30–950 mm). DART and DAgger labels are time-indexed, not corrective: the expert target does not depend on the arm state.
  - 15 of 17 temporal-ensembling timeouts never left stow: the min-jerk start moves about 1 mm in the first second, and the 20 settle frames per episode teach "stay".
  - Suspected: near-uniform temporal ensembling (m 0.01) averages stale "gripper + δ" predictions, so it brakes. Never ran `--ens none --replan 1` on 25 seeds.
  - Close decision: 14 of 19 hybrid closes failed the expert's 1 cm test, several at 8–15 s (expert about 16.6 s); offline close accuracy 1.000 was misleading.
  - Absolute bus-frame action (1–2% of the workspace is 5–10 mm); a grip-relative target should be in the retrain. 128 px gives a 4×4 ResNet grid per camera.
  - `eval_act.py` never sets `pose_est`: close and hold use the true client pose (the spec says the v3 estimate). The P4.5 hand-over bound (2 cm) is below ACT's reach error.
  - P4.3's 25/25 rests on mounts calibrated to ±1 mm, ±0.1°.
- **Checks, 20k checkpoint, 25 validation seeds, 8 workers, one at a time:**
  1. `run_act_eval.sh <ckpt> act20k_n1 25 8 --ens none --replan 1` → `p44_val_act_act20k_n1.json`;
  2. the same with `--expert-reach`, tag `act20k_hyb_n1`.
  - **Check 1 result (`p44_val_act_act20k_n1.json`, 5.3 min): 0/25.** 19 failed (timeout), 6 safe aborts, 0 collisions, never closed. The arm stayed at stow in every episode (approach target error median 952 mm). So the "ensembling brakes" idea was wrong: the newest first action at stow is "stay", and the later chunk steps were what moved the arm in the earlier runs.
  - **Check 2 result (`p44_val_act_act20k_hyb_n1.json`, 1.0 min): 0/25.** 24 safe aborts (clear_pred 21, thr_resid 2, track 1) and **1 collision** (seed 6000008, abort reason rate_anom, push 2.2 mm/s): the first unsafe outcome in any P4.4 run. Most aborts came 0.1–7 s after the hand-over. The un-averaged target sits 16–60 mm from the gripper (5 mm with ensembling), so frame-to-frame prediction noise is tens of mm and the arm jerks. The policy closed once.
  - **Conclusions:** the 20k model's raw output is too noisy to execute directly; temporal ensembling was hiding that. The model needs to be much more precise (longer training, grip-relative action, better perception input) before any control-side change matters. The E2 monitors are not a complete shield for a jerky policy near the client (1 collision in 25).
- **Direction from the user (13 Nov, night): learn the "last mile"**: the close approach to the client, the attach (close decision and contact), and acting on it afterwards. Reach stays classical (it needs only a rough bar pose, P4.3). Open: whether "act upon it" means learning post-capture too (3C/3D are classical today) or only handing over to them; assumed the second until the user says otherwise. Ideas discussed: no radar; add wrist force/torque and jaw contact inputs, a short-range distance sensor between the jaws, markers on the bar; feed the P4.3 estimate to the policy; grip-relative targets; state-dependent labels.
- **New scope agreed (13 Nov, night; user: "do it that way"), call it P4.8, the learned last mile as a correction policy:**
  - Classical keeps: spin estimate, fly-in, co-rotation, the coarse reach to the pre-grasp point (12 cm), the predicted bar motion fed to the arm, and everything after capture.
  - Learned: a small correction on top of the classical approach target over the last 12 cm, plus the close decision and contact. Zero output = the classical approach. Inputs to consider: the P4.3 estimate, wrist force/torque, jaw contact, a short-range distance sensor.
  - The task must be one the classical grasp cannot already do (it is 25/25 at ≤ 2°/s): most naturally **spin above 2°/s**, where the bus cannot fully co-rotate (centripetal force about 0.23 N at 2°/s and 0.9 N at 4°/s against 0.75 N per axis). Alternatives: a wobbling client, worse camera calibration.
  - **Step 1, RUNNING: spin sweep of the classical grasp** to find where it breaks. `eval_est.py --spin S` (new flag: fixed rate, same axis and rendering draws) and `run_spin_sweep.sh ["2 3 4 5 6"] [25]`: oracle and v3 estimate at each rate, 25 validation seeds, arm2 camera scene → `p48_spin<S>_{oracle,est}.json`, log `spin_sweep.log`. Caveat: episodes start already synchronised, which is optimistic above the thrust limit.
  - **Sweep DONE (10 min, `p48_spin{2..6}_{oracle,est}.json`):** oracle / estimate success: 2°/s 25/25, 25/25; 3°/s 19/25, 13/25 (captures 24 and 21, push over the limit in 5 and 9, up to 8.5 mm/s; estimate error at close 8.4 mm); 4, 5, 6°/s 11/25 both (14 `track` aborts in reach, identical at each rate). By axis: spin about the approach axis (axis 0) 11/11 at every rate; the two sweeping axes 8/14 at 3°/s and 0/14 from 4°/s. 0 collisions.
  - **Reading:** two limits. At 3°/s the last mile is the problem (push, estimate), which is the correction policy's task. From 4°/s the bus cannot hold station (thrust); that needs a classical partial-co-rotation mode first.
  - **Research pass done:** RESEARCH_SERVICING.md, "P4.8 research". **Task definition written: SPEC_4 §4.5** (draft). Task: 2–3.5°/s about the sweeping axes, correction on the approach target plus the close decision.
  - **User chose reinforcement learning (13 Nov, night).** Plan: (1) diagnose why 3°/s pushes too hard (`dbg_push.py`, single seeds, traces around close); (2) design the RL task in SPEC_4 §4.5; (3) build the correction-policy env on the `GraspSim.policy` hook, CPU workers, small MLP, PPO on CPU (no GPU: thermal); (4) smoke: zero correction = classical; (5) measure throughput. No long training run without the user's go-ahead.
  - **Diagnosis DONE, and it withdraws the spin-rate task (`dbg_push.py 3 6000000 6000005`, run with `P4_SCENE=... bash run_p4.sh dbg_push.py <spin> <seeds>`):** at 3°/s the thrust demand is 0.95–1.1 N against 0.75 N, the bus is 75–200 mm off station in approach (over 500 mm in hold), and the push appears only after the jaws close (0.26–0.35 mm/s before; 2.4–2.9 after the 3 s close; about 6.5 in hold): the client supplies the missing centripetal force (0.3 N / 300 kg = 1 mm/s²). The thrust limit is near 2.6°/s. **An arm correction cannot fix this, so steps 2–5 of the RL plan were NOT started.** SPEC_4 §4.5 corrected, with three candidate tasks: (1) camera calibration error (wrist-view servoing; my recommendation), (2) a nutating client, (3) faster spin after a new classical bus mode. **Waiting on the user's choice.**
  - **User chose task 1, camera calibration error (13 Nov, night).** Step 1 RUNNING: `run_mount_sweep.sh ["3 5 10"] [25]`: classical grasp on the v3 estimate with mount error ±M mm and ±M/10° per axis per sensor (`eval_est.py --mount-m --mount-deg`, new flags; the network was trained at 1 mm, 0.1°), spin 0–2°/s, 25 validation seeds → `p48_mount<M>_est.json`, log `mount_sweep.log`. Baseline at 1 mm: 25/25 (`p43_val_est_arm2v3.json`). Then: look at the wrist view near the bar (`sheets/arm2_ep_*.png`) and propose the wrist-view correction design.
  - **Mount sweep DONE (8 min; v3 estimate, 25 validation seeds, spin 0–2°/s):**

    | Mount error (per axis, per sensor) | Success | Captured | Push over 2 mm/s | Estimate error at close, median / max |
    |---|---|---|---|---|
    | ±1 mm, 0.1° (trained) | 25/25 | 25 | 0 | 2.2 / 6.2 mm |
    | ±3 mm, 0.3° | 25/25 | 25 | 0 | 4.7 / 12.4 mm |
    | ±5 mm, 0.5° | 24/25 | 25 | 1 | 7.6 / 17.3 mm |
    | ±10 mm, 1° | **16/25** | 24 | 8 | 16.1 / 30.4 mm |

    0 collisions and 0 aborts in every row. So the gap is real and graded; ±10 mm, 1° is the test condition (baseline 16/25).
  - **Wrist view checked (`sheets/arm2_ep_8000000.png`):** both fingers are in the wrist image all episode and the bar shows between them in approach, so the jaw-to-bar offset is readable from one image.
  - **Proposed design (waiting for the user's go-ahead):** a small supervised network, wrist image → bar position relative to the grip point (labels from the simulator); the classical approach servoes on that offset over the last 12 cm and closes on it; v3 stays as the coarse estimate. Needs new demos with ±10 mm mount error (about 125 episodes, 6 min local CPU) and a short Colab training. RL only afterwards if contact or close timing still fails. Demo idea from the user (later): start 30 m out, capture, align, release and move back; missing pieces are recording stage A into the replay and a planned release-and-retreat phase.
  - **Wrist camera view checked at the user's prompt (`probe_wristcam.py <seed> <out.png>`, object-coloured renders, sheet `sheets/wristcam_probe.png`, seed 6000002):** the current `wrist_cam` (above the upper jaw) has the gripper structure across the middle of the bar; the bar is 2–11% of the image as slivers at the sides and **the pads are never visible (0%)**. My earlier "the bar shows between the fingers" reading of the small sheet was too generous. Two side mounts I tried are blocked by the gripper body. **A palm camera between the jaws, wrist3 frame (0.03, 0, 0) looking along +x, fovy 90:** bar 9–22% and pads 18–24% of the image through approach and close, with the bar between the pads visible while closing. **Recommendation: add it as a second camera (`palm_cam`) in `make_scene.py`** and keep `wrist_cam` so the v3 estimator stays valid; cameras do not change physics. Not built yet.
  - **User: "Yes, build it" (13 Nov, night). BUILDING the palm-camera correction, steps in order, one run at a time:**
    1. `make_scene.py`: `palm_cam` on wrist3 at (0.03, 0, 0) looking along +x, fovy 90 (second camera; `wrist_cam` kept). `cams.py`: `CamRig.palm_id`, `render_palm()`, palm mount error and noise drawn last so the other sensors render as before. Regenerate with `bash run_p4.sh make_scene.py --arm arm2`; check: oracle 25 seeds identical to `p43_val_oracle_arm2.json`.
    2. `policies.BiasPolicy` (expert approach with a per-episode target offset: ±10/30/20 mm in x/y/z of the target frame and ±2°, shrunk near the bar; closes on time) and `gen_palm.py` (records palm image + label = bar pose in the grip-point frame at 10 Hz in approach/close/first second of hold; all outcomes saved; mounts ±10 mm, 1°). Data: `/home/shrey31/p4_data/palm_v1`, demo seeds 8,001,000+, about 150 episodes.
    3. `palm_net.py`, `train_palm.py`: small CNN, palm image → bar position and rot6 in the grip frame. **CPU training, 8 threads** (no GPU: thermal rule).
    4. `pose_est.PalmPoseEst`: v3 estimate in settle and reach; in approach the bar pose = the arm's own grip-point pose (kinematics, assumed calibrated) composed with the palm offset. `eval_est.py --mode palm --palm-ckpt`.
    5. Test: ±10 mm, 1°, 25 validation seeds, against the 16/25 baseline (`p48_mount10_est.json`).
    - Runner: `run_palm_build.sh <step>`. Progress and results are added below as they come.
    - **Step 1 done:** scene regenerated; oracle 25/25 identical to `p43_val_oracle_arm2.json` (`cmp_runs.py`, `p48_oracle_palmscene.json`).
    - **Step 2 done:** smoke 4/4, sheet `sheets/palm_smoke.png` (bar as a band across the middle, a pad above and below; nearly black once the jaws close). **Data `/home/shrey31/p4_data/palm_v1`: 150 episodes, 32,827 frames, 607 MB, 8.2 min**; 132 captured, 18 safe aborts (clear_pred 12, track 6), 0 collisions.
    - **Step 3 done (20 epochs, 1 min each on CPU):** 23,168 train and 3,982 validation frames. Validation median error: x (depth) 6.1 mm, **y (along the bar) 11.6 mm, z (closing direction) 1.9 mm**; in the last 6 cm z p95 is 6.5 mm; rotation 1.6°. It overfits (train loss 0.49 → 0.004, validation flat from epoch 4; best.pt = epoch 4). y is unobservable (a uniform bar looks the same along its length) and does not matter for the grasp, so `PalmPoseEst` keeps the v3 value on that axis. More episodes should help x and rotation (only 128 mount draws in training).
    - **Step 5 RUNNING:** `run_palm_build.sh eval 10 25` (palm rotation) then `TAG=_norot ... eval 10 25 --palm-no-rot` (rotation from v3) → `p48_mount10_palm.json`, `p48_mount10_palm_norot.json`, log `palm_eval.log`.
    - **Step 5 first results (25 seeds, ±10 mm, 1°; baseline 16/25, 0 collisions): palm with its own rotation 4/25, 9 collisions; palm position with v3 rotation 16/25, 7 collisions.** Both worse or no better, and unsafe.
      - Rotation: the network did not learn it (training offsets were only ±2°, so 1.6° validation error is just the mean); in closed loop the error integrates to tens of degrees. Rotation stays with v3.
      - Position (`dbg_palm.py <seed>`): with the bar out of view or the image dark, the palm network still answers "centred, 5–8 cm ahead" (seed 6000013: true offset 440 mm ahead and 620 mm across, palm says 75, 2), and the approach runs away. Where the bar is in view it converges (seed 6000011: across-jaw error 2–4 mm, captured, push 0.34 mm/s).
      - **Fix applied:** coarse-to-fine gate in `PalmPoseEst`: palm is used only where v3 puts the bar inside the palm view (0–20 cm ahead, within 6 cm across) and the two agree within 4 cm; otherwise v3. **RUNNING:** `TAG=_gate run_palm_build.sh eval 10 25 --palm-no-rot` → `p48_mount10_palm_gate.json`, log `palm_eval_gate.log`.
    - **Gated result (`p48_mount10_palm_gate.json`, 25 seeds, ±10 mm, 1°): 22/25, 0 collisions, 0 aborts** (baseline v3 alone 16/25). Captured 24, push over the limit 2 (was 8), 1 failed; push median 0.75 mm/s (was 1.70), max 4.14. Palm used in 3,631 updates, rejected by the gate in 515. Failures: seeds 6000005 (push 4.14), 6000019 (push 3.17), 6000022 (failed, also failed with v3 alone). Short of the 24/25 target. The total error at close (13 mm median) is mostly along the bar, which nothing observes and the grasp ignores.
    - **RUNNING next:** the same gated estimator at ±1 mm and ±5 mm (`TAG=_gate run_palm_build.sh eval 1|5 25 --palm-no-rot` → `p48_mount{1,5}_palm_gate.json`, log `palm_eval_gate2.log`), to check it does no harm where v3 already works (baselines 25/25 and 24/25).
    - **Result at smaller mount errors (gated palm vs v3 alone, 25 seeds):** ±1 mm 23/25 vs 25/25; ±5 mm 23/25 vs 24/25; ±10 mm 22/25 vs 16/25. All 0 collisions, 0 aborts, 25/25 or 24/25 captured; the palm losses are 2 hard pushes each (max 3.0 mm/s at 1 and 5 mm). **Reading:** the palm correction makes the grasp almost independent of calibration (22–23/25 across 1–10 mm, typical push 0.56–0.75 mm/s), but it has its own failure floor of about 2 in 25, so it is slightly worse than v3 where v3 is well calibrated. **Not finished: the next job is that floor** (trace the hard-push seeds, more data, larger tilts, a bar-in-view output). Waiting on the user.
    - **User: "Yes, improve it" (13 Nov, night).** Order: trace the hard-push seeds with `dbg_palm.py <seed> <mount_mm>`; fix the cause; more data and retrain if needed (`run_palm_build.sh data N`, `train`); re-test at 1, 5, 10 mm. Progress below.
    - **Cause of the hard pushes found (`dbg_palm.py 6000005 10`, `6000019 10`):** the palm image darkens as the gripper nears the bar (image mean 22 → 2 → 0.7: the gripper shadows it), and the dark pads vanish against the dark scene. From about mean 14 down the network reports the bar centred across the jaws while it is 6–15 mm off, and its depth is 20–50 mm wrong; the v3 value is off by the mount error, so the gate (4 cm) passes it. Hard-push seeds: 6000013 and 6000014 at 1 mm; 6000005 and 6000013 at 5 mm; 6000005 and 6000019 at 10 mm.
    - **Fix in progress:** `palm_light`, a spot beside the palm camera on wrist3 (`make_scene.py`), off by default and switched on only while `palm_cam` renders (`cams.render_palm`), so other images and the physics are unchanged. Then: regenerate the scene and re-check the oracle (`run_palm_build.sh scene`), smoke sheet, re-record as `palm_v2` (300 episodes for more mount draws), retrain as `palm_net_v2` (10 epochs: v1 peaked at epoch 4), re-test at 1, 5, 10 mm. `run_palm_build.sh` takes `PV=v2` for the data and network names.
    - **Palm v2 DONE (`run_palm_v2.sh`, log `palm_v2.log`; palm light on, sheet `sheets/palm_smoke.png`):** scene regenerated, oracle 25/25 identical. Data `/home/shrey31/p4_data/palm_v2`: 300 episodes, 65,706 frames, 14 min (266 captured, 34 safe aborts, 0 collisions). Network `/home/shrey31/p4_data/palm_net_v2/best.pt` (epoch 8 of 10, 1.7 min per epoch on CPU): validation median error x 2.3 mm, y 6.1 mm, **z 0.95 mm** (z p95 in the last 6 cm 3.2 mm; v1: 6.1 / 11.6 / 1.9, p95 6.5).
    - **Gated test with v2 (25 validation seeds, rotation and along-bar value from v3):**

      | Mount error | v3 alone | palm v1 | **palm v2 (light, 300 episodes)** | push median / max, v2 |
      |---|---|---|---|---|
      | ±1 mm, 0.1° | 25/25 | 23/25 | **25/25** | 0.31 / 0.62 mm/s |
      | ±5 mm, 0.5° | 24/25 | 23/25 | **25/25** | 0.41 / 0.75 mm/s |
      | ±10 mm, 1° | 16/25 | 22/25 | **25/25** | 0.47 / 0.99 mm/s |

      0 collisions, 0 aborts, 0 over the push limit; the gate rejects only 63–70 of about 4,150 palm readings (v1: about 500). Files `p48_mount{1,5,10}_palm_v2.json`.
    - **Caveat:** the fix was found by tracing validation seeds 6000005/13/19, so these 25 seeds are no longer unseen. **RUNNING: confirmation on fresh validation seeds 6000100–124 at ±10 mm** (v3 alone and palm v2) → `p48_mount10_est_fresh.json`, `p48_mount10_palm_v2_fresh.json`, log `palm_fresh.log`. The test split (7,000,000+) is untouched.
    - **Confirmation on fresh validation seeds 6000100–124, ±10 mm, 1° (DONE):** v3 alone **16/25** (24 captured, 8 over the push limit, 1 failed; push median 1.40, max 4.87 mm/s); **gated palm v2 25/25** (0 collisions, 0 aborts, push median 0.50, max 1.75 mm/s; gate rejected 106 readings). So the result holds on seeds that were never traced. Over both seed sets at ±10 mm: 32/50 against 50/50.
    - **State: P4.8 calibration task works.** Open items: rotation and the along-bar position still come from v3 (palm rotation needs larger tilts in the data); spin ≤ 2°/s only; the test split (7,000,000+) is unused; SPEC_4 §4.5 updated; **nothing committed** (new files: `gen_palm.py`, `palm_net.py`, `train_palm.py`, `palm_sheet.py`, `cmp_runs.py`, `probe_wristcam.py`, `dbg_palm.py`, `dbg_push.py`, `run_palm_build.sh`, `run_palm_v2.sh`, `run_palm_fresh.sh`, `run_spin_sweep.sh`, `run_mount_sweep.sh`; changed: `make_scene.py`, `cams.py`, `policies.py`, `pose_est.py`, `eval_est.py`). User's demo idea (30 m start, capture, align, release, move back) is the natural next piece.
    - **User: "Build the demo" (13 Nov, night): one mission from about 30 m out: far approach, fly-in, vision grasp (v3 + palm v2, mounts ±10 mm), brakes, detumble, identification, move and turn, then release and retreat.** Plan (new files only; the frozen p3_capture code is not edited):
      1. `p3_capture/demo_stage_a.py` (rl env, GPU for a few seconds): stage A for one case (seed 6000000, start scaled to 30 m), records the path (LVLH position and client rotation per epoch) and the hand-off error → `replay/demo_stage_a.json`.
      2. `p3_capture/export_demo.py` (from `export_mission.py`): mission on the camera scene with `PalmPoseEst` in the grasp, hand-off from step 1, then a new **release and retreat**: rebuild the unlocked model, open the jaws, pull the gripper back 0.25 m, back the bus off 3 m while stowing the arm; reports the client's residual rate, clearance and any contact. Far-approach frames are mapped into the grasp world so the join is continuous → `replay/demo_full.json`.
      3. `replay/index.html`: new run "Full demo", phase colours for far approach, release and retreat, and a note on what is sensed and what is still read from the simulator (bus station keeping, client spin, monitors use the true client state; checked in `safety.py`). Then republish the replay artifact (https://claude.ai/artifact/WxRmRjcM5Zu4DoBkTQhKw8).
      - Answer given to the user on sensing: grasp = own cameras; far approach = filtered noisy nav measurements; post-capture = noisy gyro/accelerometer; **bus station keeping, spin axis/rate and the clearance/rate monitors read the true client state.**
      - **Demo progress:** `demo_stage_a.py 6000000 30` → `replay/demo_stage_a.json`: 30.0 m to the 8 m hold in 440 s, arrival error 2.3 cm, 1.75 mm/s (the env needs 2 worlds; world 0 recorded). `export_demo.py` first run (`run_export_demo.sh`, log `p3_capture/export_demo.log`, 8 min): **whole mission succeeded with the vision grasp** (mounts ±10 mm: captured, push 0.55 mm/s, bar shift 0.53 mm, palm used 166 times, 0 rejected; mass error 0.12%; stack moved 13.7 m and turned 58°, final error 3.0 cm, 0.07°; 19.5 g propellant). Release: no contact, servicer from 2.1 m to 5.1 m, arm stowed, client rate change 0.001°/s, **but the client was pushed 2.09 mm/s** (jaws opened in one step) and the far-approach join had a 0.4 m gap. **Second run RUNNING** with the jaws eased open over 6 s, no thrust while they open, and the join gap blended over the far approach. `replay/index.html` edited (demo run, colours far/release/retreat, "Whole approach" camera, sensing note; backup of the old page in the temp folder); not yet checked in the browser or republished.
      - **DEMO DONE AND PUBLISHED (13 Nov, night).** Third run of `run_export_demo.sh` (8 min) → `replay/demo_full.json` (3,098 frames, 2,143 s), checkpoint `replay/demo_ckpt.pkl` (`export_demo.py <stage_a> <out> --release-only [--print]` re-runs only the release, about 1 min).
        - Far: 30.0 m → 8 m hold, 440 s, 2.3 cm. Fly-in 381 s. Grasp on cameras (mounts ±10 mm, 1°): captured, push 0.55 mm/s, bar shift 0.53 mm, palm used 166 times, 0 rejected. Mass error 0.12%, inertia 0.24%. Moved 13.7 m, turned 58°, final 3.0 cm, 0.07°, 19.5 g.
        - Release (new, demo only, in `export_demo.release`): jaws eased open over 6 s with no thrust and the attitude held fixed in LVLH, gripper back 0.20 m in 6 s, bus back 3 m in 76 s (thrust at 60% of the limit, never saturated) while the arm stows. **No contact; client disturbed by 0.013 mm/s and 0.001°/s beyond free orbital motion**; servicer from 2.1 m to 5.1 m; stow error 0.9°.
        - Two measurement lessons: (1) the client's raw velocity change over 100 s (2.1 mm/s) is orbital drift at 14 m from the reference point (−n²(r − 3(r·e)e)), not a push; the metric now subtracts it. (2) Holding an inertial attitude while still gripping braked the stack's LVLH rate through the grip (client rate changed 0.027°/s); fixed with an LVLH-fixed reference.
        - Display-only fix: the far-approach hold point and the grasp simulation's start differ by 0.395 m; blended linearly over the far approach.
        - Page: `replay/index.html` (run "Full demo from 30 m" is the default; phases far / release / retreat; "Whole approach" camera; sensing note in "What to watch"). Checked locally (launch config "replay"; no console errors; far, approach and retreat views looked right). **Republished to https://claude.ai/artifact/WxRmRjcM5Zu4DoBkTQhKw8 (version 4).** The release phase uses ideal wheel torques (no wheel-momentum model) and is not part of E3 or any freeze.
        - New files: `p3_capture/demo_stage_a.py`, `export_demo.py`, `run_export_demo.sh`; `replay/` is gitignored. Nothing committed.
      - **Bug reported by the user (13 Nov, night):** in the demo the pair looked paused in orbit during the far approach and only moved with the orbit from the fly-in. Cause: far frames used a constant radial direction and treated LVLH as inertial. Fix in `export_demo.finish()`: far frames are rotated about the client centre by the orbit rotation back from the hold time (axis and rate measured from the recorded radial vectors), positions, attitudes and the radial vector together; then `--release-only` regenerates `demo_full.json` from the checkpoint and the page is republished.
      - **Fixed and republished (version 5):** `turn_far()` in `export_demo.py`; measured orbit rate 0.0648°/s about world z, 28.5° over the 440 s far approach; the radial vector and the servicer path are continuous at the join (0.0° and 0.000 m); release numbers unchanged. Checked locally with the "Earth below" camera: the Earth view changes between 10 s and 430 s.
    - **Ideas to close the gap (not started):** more episodes (more mount draws; the network overfits at 128), larger tilt offsets so rotation is learned, a "bar in view" output, and looking at the 3 failures with `dbg_palm.py`.
    - (Step 3 command, for the record:) **Step 3 RUNNING:** `run_palm_build.sh train 20` → `/home/shrey31/p4_data/palm_net_v1/{best,last}.pt`, log `p4_vision/palm_train.log` (CPU, 8 threads, approach frames only, last 15% of episodes as validation; resumable from last.pt).
  - From the sweep files: the push at 3°/s is systematic per axis (axis 1 about 2.4 mm/s in 4 of 6 seeds, axis 2 1.1–2.9; axis 0 0.2–0.5), bar shift about 2.8 mm on both sweeping axes. With the estimate, the error at close on axis 2 is 19–35 mm (2–3 mm on axis 0): the estimator was trained at ≤ 2°/s.
- **Retrain proposal afterwards (superseded by P4.8 above; the full-ACT retrain is not planned):** time-in-phase, drop settle frames, grip-relative target, 100k+ steps; open question: a state-feedback expert label so DAgger is corrective.

**Earlier: 13 Nov, session closed here.**
- **v3 fine-tune done on Colab** (T4, 3.1 min; train 20,570 frames from episodes 0–109, val 1,403 frames from 110–124). Validation median / p95 position error:

  | | all | settle | reach | approach | close |
  |---|---|---|---|---|---|
  | epoch 0 = v2 weights on arm2 images | 3.5 / 21.1 mm | 0.6 / 1.4 | 2.0 / **33.1** | 3.8 / 7.1 | 3.9 / 7.5 |
  | **epoch 3 (best, `pose_v3/best.pt`)** | **1.6 / 3.5 mm** | 1.8 / 3.4 | 1.6 / 3.7 | **1.4 / 3.6** | 1.7 / 2.9 |
  | epoch 10 | 1.6 / 3.4 mm | 1.6 / 3.6 | 1.6 / 3.5 | 1.6 / 3.6 | 1.7 / 3.1 |

  Rotation about 0.2° median. The v2 network was already usable on the new arm (median fine), but its reach-phase tail was large (p95 33 mm, the moving arm housings in view); 1 epoch of fine-tuning fixed it, flat after epoch 3. Better than v2 on its own data (2.2 / 6.1 mm).
- **Estimate grasp on arm2 done (13 Nov, 25 validation seeds, 2.5 min; `p43_val_oracle_arm2.json`, `p43_val_est_arm2v3.json`, log `p43_eval_arm2v3.log`):** oracle 25/25, max push 0.61 mm/s, bar shift 1.81 mm; **v3 estimate 25/25** (CI 87–100%), 0 collisions, 0 aborts, 0 over the push limit, max push 0.97 mm/s, bar shift 1.81 mm. Estimate error medians: settle 3.4, reach 2.5, approach 2.7 mm; at close 2.2 median / 4.4 p95 / 6.2 max mm (old arm with v2: 2.1 / 5.3 / 7.0). Weights: `colob_ops/best_v3.pt` = `/home/shrey31/p4_data/pose_v3/best.pt`. `run_p43_eval.sh` now takes `ORACLE=<file>` (default the old-arm file); command: `P4_SCENE=p4_espa_arm2_cam_scene.xml ORACLE=p43_val_oracle_arm2.json bash run_p43_eval.sh <ckpt> arm2v3 25`.
- **P4.4+ replanned (13 Nov, later):**
  - Research is in `learning/RESEARCH_SERVICING.md`, "P4.4–P4.7 research"; the plan is in **SPEC_4 §4.4**, which overrides the old steps 5–8.
  - Order:
    1. policy hook and `expert_suggest` in `grasp_sim.py`, then re-freeze as v1.3;
    2. `demos_v4` at 10 Hz images, 125 clean and 125 DART episodes;
    3. LeRobot 0.6.x in its own env (the `rl` env's torch 2.13 is outside LeRobot's 2.7–2.11), then convert (bus camera to 128, lidar in 3 channels);
    4. ACT on Colab: chunk 20, temporal ensembling;
    5. 25-seed closed loop with E2 on, against the oracle and the P4.3 estimate;
    6. hand-over shield and DAgger;
    7. conditions and the test split;
    8. variants.
  - **Decided by the user:**
    - ACT starts on images only: wrist, bus (resized 192 → 128 in conversion) and lidar (3-channel), plus state;
    - the v3 estimate is only the close/hold reference.
- **P4.4a done (13 Nov), uncommitted:**
  - policy hook and `expert_suggest` in `grasp_sim.py`;
  - identity checks with no policy set: p3b 20/20, arm2 25/25;
  - freeze v1.3;
  - `p4_vision/policies.py` (ExpertPolicy, DartPolicy) and `policy_smoke.py`: expert 8/8, expert through the hook 8/8, DART (2 mm per axis, capped at 5 mm) 8/8. Details in SPEC_4 §4.4.
- **P4.4b done (13 Nov), `run_demos_v4.sh`:**
  - `gen_demos.py --dart`: DartPolicy executes the target; the label is the clean expert target.
  - Smoke: the clean recording reproduces `demos_v3` state and action exactly, with 467 images per episode at 10 Hz.
  - `/home/shrey31/p4_data/demos_v4_clean`: 125/125, seeds 8000000–124, 58,431 frames, 4.6 GB.
  - `/home/shrey31/p4_data/demos_v4_dart`: 125/125, seeds 8000125–249, 58,447 frames, 4.6 GB.
  - Recording took 32 min on 8 workers.
- **P4.4c in progress (13 Nov).** The user chose: LeRobot setup, then a **short throttled local ACT test** with a temperature watch. Full training stays on Colab.
  - **Env:** `p4_vision/setup_lerobot_env.sh` built a pixi env at `/home/shrey31/lerobot_px`: Python 3.12, **lerobot 0.6.1**, torch 2.11+cu130 (CUDA OK), mujoco 3.7.0, ffmpeg. Run commands with `bash p4_vision/lrrun.sh python <ABSOLUTE path>`, because lrrun cds into the env.
    - `[core-scripts]` was dropped: its evdev wheel fails to build.
    - The system Python has no venv module (it needs sudo apt).
  - **Converter:** `p4_vision/to_lerobot.py` builds one dataset, clean first and then DART, plus a sidecar `<out>_episodes.json` (seed, dart, train/val with the last 15 of each source as val).
    - Images: bus 192 → 128 (INTER_AREA). Lidar is **grey**: 0.3–1.8 m mapped to levels 1–255, 0 = no return, the same value in all 3 channels. A first packing with different data per channel lost 17.6 mm to yuv420p chroma subsampling.
    - Encoding: h264 at **CRF 10**, GOP 2, streaming. Arrays are read once per episode; the slowness was NpzFile decompressing on every access, not the codec. Now 4 episodes take 0.2 min (they took 6.6).
    - **`check_lerobot.py`** (smoke, 4 episodes): state and action exact; wrist and bus PSNR 43 dB; lidar 44 dB, range error 5.9 mm median (one level) and 17.7 mm p95. CRF 18 lost 11.8 mm.
    - torchcodec fails to load in the env (`libtorchcodec_core4.so`), so LeRobot falls back to pyav decoding. Watch the data-loading speed in training.
    - **Done:** `/home/shrey31/p4_data/lerobot/demos_v4`, 250 episodes (220 train; val = clean episodes 110–124 and DART episodes 235–249), 116,878 frames, 1.7 GB, 16 min.
      - Check: state and action exact; PSNR wrist 42, bus 41, lidar 44.5 dB; lidar 5.9 mm median.
      - **One bad frame:** the last wrist frame of episode 11 (seed 8000011) decodes black; every other episode's last 3 frames match (`check_lerobot_ends.py` and an inline check). It is probably a video-file boundary. Left as is: 1 of 116,878 frames, in the hold phase.
      - Wrist frames at episode start are genuinely black: the stowed wrist camera looks at space.
    - **ACT test 1 (13 Nov):** `lerobot-train` works on demos_v4.
      - 220 train episodes, 102,851 frames; ACT 52M parameters; chunk 20, AMP, batch 8.
      - **About 10 steps/s (84 samples/s)**; data loading 0.004 s per step with pyav; 0.95 GB GPU memory.
      - Loss 33 at step 25, 7.4 at step 50.
      - **The watchdog stopped it at step 61:** the GPU went 74 → 85 °C in about 6 s at 51 W. The GPU idles at 73 °C (laptop warm).
    - **ACT test 2, done (13 Nov):** `run_act_test.sh 600 8 87 80 76`, with a duty-cycle throttle (SIGSTOP the trainer at 80 °C, SIGCONT at 76 °C, hard kill at 87 °C). Log in `act_test.log`; temperatures in `act_test_temp.log`.
      - **600/600 steps in 5.5 min**, paused 30 times for about 236 s (about 70% of the time).
      - Effective about 1.8 steps/s; the GPU peaked at 85 °C (it overshoots after a pause starts) and was mostly 79–84 °C.
      - Loss fell from 33 to 2.36; l1 0.68 → 0.27; kld 3.3 → 0.21.
      - Checkpoint in `/home/shrey31/p4_data/act_test/checkpoints/last/pretrained_model` (safetensors plus the pre/post-processor normalisers).
      - **Conclusion:** local training works but is thermally limited to about 1.8 steps/s, so 50–100k steps would take 8–15 h. **Real ACT training goes on Colab.** Local use: smoke tests and CPU evaluation.
    - **Colab set up (13 Nov):** `p4_vision/colab_act_v4.ipynb`, uploaded to Drive `mjorbit_p4/code_v4/` with `val_act.py`.
      - Drive ids: code_v4 1o1VLweP8Br7K-dniJOyqqgqD-eEtmMC3; notebook 1neHOr8BXzn97BYnNRV1L6avH1T8tN3R8 (https://colab.research.google.com/drive/1neHOr8BXzn97BYnNRV1L6avH1T8tN3R8; fresh-runtime version: asserts Python >= 3.12, installs lerobot, uninstalls torchaudio, then runs an import check in a subprocess (torch CUDA, transformers AutoProcessor, ACTPolicy) and stops if it fails; L4 preferred, data workers = min(8, vCPUs - 1). It installs only lerobot[training]==0.6.1 on Colab's own CUDA 13 stack: the first run pinned a cu128 torch, which clashed with Colab's torchaudio 2.11.0+cu130 (pip saw 2.11.0 as already satisfied and did not replace it) and broke the transformers AutoProcessor import. Fix in that session: pip uninstall -y torchaudio); also installs torchaudio 2.11 cu128, because Colab's CUDA 13 torchaudio broke the transformers import on the first run; older copy 1kgSu... still on Drive, the first upload trashed); data_v4 1ZD2DCLi0CgBGk3O4qbw2EZe_RNnhA3-Z (empty).
      - **The user uploads** `tmp/colab_upload_v4/data_v4/demos_v4_lerobot.tar` (1.76 GB) and `SHA256SUMS` into `data_v4/`. The tar comes from `make_colab_upload_v4.sh`; sha 5d1e682d…ba62.
      - **The notebook:**
        - installs torch 2.11 from the cu128 index (the default wheel is CUDA 13 and needs a newer driver) and then lerobot 0.6.1;
        - copies and checks the tar, and unpacks it to `/content/data`;
        - trains ACT: chunk 20, AMP, batch 32, 20k steps (about 6 passes), num_workers 2, checkpoints every 2000 steps to `mjorbit_p4/act_v4/run1/`, resumable from Drive;
        - runs `val_act.py` on every checkpoint (`val_<step>.json`) and prints a summary.
      - Untested on Colab: the Python version (needs 3.12), the T4 speed, and the data loading with 2 vCPUs.
    - **`val_act.py`:** offline action error per phase on the validation episodes. Labels come from the dataset, and the phase from the sidecar index (`add_phase_index.py` added it). Checked locally on the 600-step test model: 25 mm median.
    - **Colab run1 DONE (13 Nov, L4, 20k steps).** Offline val (30 val episodes, 2820 frames), position median / p95, rot median, by step:
      - all: 2k 28.7/67.3 mm 1.86°; 10k 13.5/44.2 0.94°; 18k 9.4/28.7 0.62°; **20k 8.7/33.5 mm 0.70°**.
      - 20k per phase: settle 22.3/26.8; reach 18.2/43.0 (2.11°); **approach 5.3/16.8 (0.20°)**; close 3.1/6.8; hold 3.6/11.4. Close-flag accuracy 1.000 at every step except 14k (0.993) and 16k (0.999).
      - 18k is better on settle and reach, and has the lower p95 overall (28.7 mm); 20k is better on approach, close and hold, which are the phases that matter for the grasp. **Use 20k.** The error was still falling at 20k (not plateaued): resuming to about 40k is the obvious next run if closed-loop results fall short.
      - Caveat: this is one-step open-loop error against the clean expert label. Approach p95 16.8 mm is above the expert's 10 mm close gate, so the closed-loop test decides. Compare: the P4.3 estimator was 1.6/3.5 mm.
      - **20k checkpoint on this PC:** `colob_ops/check_20000.zip` (564 MB, gitignored; includes the optimizer state). Its `pretrained_model/` is unpacked to WSL `/home/shrey31/p4_data/act_v4/run1/020000/pretrained_model`. The Drive connector lost permission on 13 Nov.
      - **Resume to 40k (given to the user, 13 Nov):** an extra Colab cell runs after cells 1–4 on a fresh runtime: `lerobot-train --config_path=<run1>/checkpoints/<highest>/pretrained_model/train_config.json --resume=true --steps=40000 --num_workers=NW`. Checked in LeRobot 0.6.1: resume restores step, optimizer and RNG; the loop is `range(step, cfg.steps)`, so the CLI `--steps` override extends it. There is no LR scheduler (constant AdamW lr 1e-5), so extending is clean. Notebook cell 5 alone would no-op (no `--steps`). Afterwards, cells 6–7 validate the new checkpoints.
    - **Colab resume to 40k RUNNING (started 13 Nov 13:54 UTC, about 37 min at 8.9 steps/s; data-bound: data_s 0.42 vs updt_s 0.13; kld_loss 0.000, i.e. the CVAE latent is unused, harmless since inference uses z = 0).** The user re-runs cells 6–7 afterwards and pastes the table.
    - **P4.4e IN PROGRESS (13 Nov, user said go ahead): ACT closed loop on the 20k checkpoint, local, CPU sim.** Plan: `act_policy.py` (wrapper for `GraspSim.policy`: renders wrist/bus/lidar at 10 Hz like the demos, builds the 29-D state, predicts a chunk, converts the bus-frame target to world at the current bus pose; chunk execution or temporal ensembling) and `eval_act.py` (eval_est.py layout, 25 validation seeds, arm2 camera scene, E2 on, compared with `p43_val_oracle_arm2.json` and `p43_val_est_arm2v3.json`). Problem: the sim needs the compiled `mjorbit` (rl env) and ACT needs the LeRobot env (do not install LeRobot into rl). Both are Python 3.12 + mujoco 3.7.0, so first try importing `mjorbit` from `simulator/src` inside the LeRobot env.
      - **Works:** `mjorbit` from `simulator/src` (in-tree `build/cpp` bindings) loads in the LeRobot env, and EGL rendering works. Written: `act_policy.py` (ACTGraspPolicy: `ens` temporal|none, `m`, `replan` frames, linear/slerp interpolation between 10 Hz targets from the last commanded one, close = flag > 0.5; logs target vs expert_suggest) and `eval_act.py` (KEEP/wilson from eval_est, per-episode 0.5 s trace, `.partial.jsonl`), plus `run_act_eval.sh <ckpt> <tag> [N] [W] [args]` → `p44_val_act_<tag>.json`.
      - **Smoke, seed 6000000, 20k checkpoint, 1 worker:**
        - temporal ensembling every frame: captured, push 0.42 mm/s, 0.9 min;
        - chunk execution, re-plan every 10 frames: FAILED, push 4.4 mm/s;
        - chunk execution, re-plan every 20 frames: captured, push 0.99 mm/s, 0.2 min.
      - **Finding (copycat lag):** the ACT target stays only 1–7 mm ahead of the gripper, so the arm lags the expert's time-based reach path by up to 420 mm with temporal ensembling (274 mm re-planning every 10 frames, 170 mm every 20) and catches up near the end of reach. In approach it trails the expert's target by about 30 mm. Capture still works, because approach blends from wherever the arm is and the close decision is visual. Likely cause: the gripper pose in the state ("target ≈ gripper + δ"). Fixes to consider: drop or noise the gripper-pose and joint state, or condition on the phase or time.
      - **25-seed results, 20k checkpoint (13 Nov), against oracle 25/25 and P4.3 estimate 25/25:**
        - **Temporal ensembling (`p44_val_act_act20k_te.json`): 3/25** (CI 4–30%). 17 failed (approach timeout; the arm stalled near stow, ACT target up to 0.95 m from the expert's), 5 safe aborts (track 3, clear_pred 2), push up to 9.6 mm/s, 0 collisions, 7.8 min.
        - **Chunk execution, re-plan every 20 frames (`p44_val_act_act20k_k20.json`): 2/25.** 8 captured but 18 over the 2 mm/s push limit, 10 safe aborts (track 7, clear_pred 3), approach target error median 45 mm, 0 collisions, 1.2 min.
      - **Shadow diagnostic (`--shadow`: the expert drives, ACT predicts live on 2 seeds): reach 13–16 mm and approach 4.6–4.9 mm median error, rotation 0.2°**, matching offline val. So the wrapper is correct, and the failure is closed-loop covariate shift. The expert's reach and approach are time-indexed (min-jerk over 20 s, then 1 cm/s), but ACT has no clock: its target is roughly gripper + δ, and a 13–20 mm error exceeds the expert's 5–7 mm lead, so a lagging arm stays lagging (a fixed point near stow).
      - **Options (user to choose):**
        - (a) hybrid diagnostic, no retraining: the classical reach, then ACT from approach on;
        - (b) retrain with time-in-phase (or phase progress) in the state and/or the gripper pose and velocity removed (copycat);
        - (c) DAgger (P4.5): the expert labels any state through expert_suggest, but it needs (b)'s time input, or the labels contradict each other.
        - The 40k checkpoint alone is unlikely to fix this.
      - **User chose (a), the hybrid test (13 Nov). RUNNING:** `--expert-reach` (act_policy `expert_reach`: the reach is `expert_suggest` on the true pose, as the oracle; ACT drives approach and decides when to close), temporal ensembling, 20k checkpoint, 25 validation seeds, 8 workers → `p44_val_act_act20k_hyb_te.json`.
      - **Hybrid result: 5/25** (CI 9–39%), against 3/25 fully ACT, and 25/25 for the oracle and P4.3.
        - Outcomes: 9 captured (4 of them over the 2 mm/s push limit), 12 safe aborts (track 9 with push 7–10 mm/s, so the jaws push the bar and the client; clear_pred 3), 4 failed (timeout, approach target 120–940 mm off). 0 collisions, 3.1 min.
        - The policy closed in 19/25 episodes, but the expert's 1 cm condition held at only 5 of those closes (target about 20 mm off at close).
      - **Conclusion:** the approach itself is the weak part, not only the reach. ACT trails the expert's time-indexed approach (median 36 mm) and closes early or off-axis. The fix is (b), time in phase plus less proprioceptive copying, then DAgger.
    - **After Colab:** copy the best checkpoint's `pretrained_model/` back, then run the closed-loop ACT evaluation (P4.4e): an ACT policy wrapper for `GraspSim.policy`, with rendering in the loop, on 25 validation seeds.
  - Then a short ACT test with `lerobot-train` (a few hundred steps, small batch), watching `nvidia-smi` temperature and stopping it if it goes too hot. The GPU read 75–79 °C idle while the CPU recording ran.
  - Wrapper commands inside `wsl bash -c "..."` lose `$VAR` assignments in this setup. Use scripts or full paths.
- **Earlier next steps (done):** 3. Commit the uncommitted camera-scene / v3 work (make_scene `--arm`, gen_demos `P4_SCENE`, check_cam_physics scene arg, train_pose `--init`, colab_p43_v3.ipynb, make_colab_upload_v3.sh, run_arm2_cam_smoke.sh, arm2 finger recolour + freeze hash, CONTEXT). `p3_capture/p4_espa_arm2_cam_scene.xml` is generated: add it to the "generated scenes" section of `.gitignore` instead of committing it.
  4. Then P4.4 (LeRobot conversion, ACT) on demos_v3 (record more episodes if ACT needs them).

**13 Nov, latest before that: v3 data and fine-tune on the realistic arm.**
- Camera scene: `make_scene.py --arm arm2` → `p4_espa_arm2_cam_scene.xml`; `P4_SCENE=p4_espa_arm2_cam_scene.xml` switches gen_demos / eval_est / export_p4_replay to it. Smoke (`run_arm2_cam_smoke.sh`): 4/4, physics identical to the plain arm2 scene, sheets in `p4_vision/sheets/arm2_*.png`. Finger visuals recoloured grey (not bar orange).
- `demos_v3`: 125/125 episodes succeeded, 58,431 frames, 1.9 GB, 5.9 min on 8 workers, `/home/shrey31/p4_data/demos_v3`. Shards (5 × 25 episodes) in `tmp/colab_upload_v3/data_v3/`; **waiting for the user to upload them to Drive `data_v3/` and run the notebook.** The Drive connector cannot see Colab-made files, so whether `pose_v2/best.pt` is on Drive is unchecked; if missing, upload `colob_ops/best_v2_1.pt` there as `best.pt`.
- `train_pose.py --init <weights>` fine-tunes (keeps the position normalisation, scores the start weights as epoch 0).
- Colab: `colab_p43_v3.ipynb`; Drive `mjorbit_p4/code_v3/` (filled by Claude) and empty `data_v3/` (user uploads the shards from `tmp/colab_upload_v3/data_v3/`, made by `make_colab_upload_v3.sh`). Starts from Drive `pose_v2/best.pt`, 10 epochs, lr 3e-4, episodes 0–109 train; results to `pose_v3/`.

**13 Nov, later: P4.3 wrapped up.** Write-ups in SPEC_4 §4.3, SPEC_3 §14.4 (arm2) and LEARNINGS; freeze v1.2 (`e3_espa_freeze.sha256`, all 36 hashes OK; v1.1 kept); Vision Grasp Replay re-exported with the epoch-23 network (`run_p4_replay.sh`; close errors 1.5 and 2.2 mm) and republished. Everything committed on branch `p4.3-and-arm2` (commit 0245e5b), pushed, and fast-forward merged into `main` (13 Nov). **Next:** arm2 camera scene, small `demos_v3`, retrain on Colab, re-check the estimate grasp (about 25 missions); then P4.4 (LeRobot, ACT).

**13 Nov, earlier (stopped here first). Start with this block.**
- **Colab v2 training finished** (30 epochs, 234 min on a T4). Best epoch 23; validation median / 95th percentile:
  - all frames 2.2/6.1 mm, 0.17/0.55°;
  - approach 2.0/5.5 mm, 0.11/0.26°;
  - close 1.7/3.8 mm.
  - From epoch 19 on, validation stayed flat at about 2.2–2.3 mm while training loss kept falling (to 0.0006). 20 epochs would have been enough.
- **Checkpoints on disk:**
  - final best: `colob_ops/best_v2_1.pt`, copied to `/home/shrey31/p4_data/pose_v2/best_final.pt`;
  - last epoch: `colob_ops/last_v2.pt`;
  - epoch 4: `colob_ops/best_v2.pt`, copied to `best_ep4.pt`.
- **Oracle closed loop done:** `p4_vision/p43_val_oracle.json`, 200/200 successes, max push 0.99 mm/s, 3 min.
- **Estimate closed loop done (13 Nov, `p4_vision/p43_val_est_final.json`, 14 min on 8 CPU workers):**
  - 200/200 successes (95% CI 98.1–100%), 0 collisions, 0 aborts, 0 over the push limit;
  - max push 1.02 mm/s (oracle 0.99), max bar shift 1.94 mm (oracle 1.81);
  - estimate error medians: settle 3.5 mm, reach 2.7 mm, approach 2.2 mm; at close 2.1 mm median, 5.3 mm p95, 7.0 mm max.
  - The rerun command: `bash projects/p4_vision/run_p43_eval.sh <ckpt> <tag> 200`.
- **Held-out sun done (13 Nov, `p43_val_est_final_holdout.json`, `run_p43_holdout.sh`, 12 min):** same 200 validation seeds, sun inside the 15° held-out cone (never in training).
  - 200/200 (95% CI 98.1–100%), 0 collisions, 0 aborts, 0 over the push limit; max push 1.06 mm/s, max bar shift 1.99 mm.
  - Estimate error medians: settle 3.1 mm, reach 2.5 mm, approach 2.2 mm; at close 2.0 mm median, 4.4 mm p95, 6.0 mm max. No loss against normal lighting.
- **Test split done, run once (13 Nov, seeds 7,000,000–299, `run_p43_test.sh`, `p43_test_oracle.json`, `p43_test_est_final.json`):**
  - Oracle 300/300 (95% CI 98.7–100%), max push 1.06 mm/s, 4.4 min.
  - **Estimate 300/300 (98.7–100%)**, 0 collisions, 0 aborts, 0 over the push limit; max push 1.52 mm/s (seed 7000082, close error 7.2 mm; oracle 0.77 there), max bar shift 1.94 mm, 19.7 min.
  - Estimate error medians: settle 3.6 mm, reach 2.5 mm, approach 2.2 mm; at close 2.2 mm median, 4.4 mm p95, 9.4 mm max (seed 7000208, push still 0.81 mm/s).
- **Spin-rate breakdown done (13 Nov, `spin_breakdown.py`, no new simulation):** bins 0–0.5 / 0.5–1 / 1–1.5 / 1.5–2°/s (46/48/45/61 seeds), all bins 100% in all three runs.
  - Estimate error at close (median / p95): 1.8/4.7, 2.3/5.5, 2.2/5.2, 2.2/5.3 mm; approach 1.9–2.4 mm median. Correlation of spin with the error at close is +0.12 (both lighting sets), so spin barely matters.
  - Max bar shift grows with spin in the oracle too (0.46 → 1.81 mm), so it comes from the classical grasp, not the estimate.
  - Odd but harmless: median client push is lower with the estimate (about 0.45 mm/s) than with the oracle (about 0.8 mm/s); max push is the same (about 1 mm/s). Not explained yet; maybe the EMA smoothing or the latch at close.
- **Then:**
  - held-out sun;
  - spin-rate breakdown;
  - test split of 300, run once;
  - re-export the browser replay with `best_final.pt` and republish the same artifact (https://claude.ai/artifact/4kfsXQvUnRcTjoMvFB2RKA, currently uses epoch 4);
  - write up SPEC_4, LEARNINGS and CONTEXT;
  - re-freeze `grasp_sim.py`;
  - commit and push. `p4/replay/index.html` is not tracked by git because of the `**/replay/` rule in `.gitignore`.

**13 Nov: realistic arm "arm2" (Levels 1+2), first checks.**
- Research first: `learning/RESEARCH_SERVICING.md`, "Arm realism research" (CAESAR, e.Deorbit, ROKVISS/DLR joint modules, JEM SFA; small 1 m flight arms publish almost nothing).
- Scene `projects/p3_capture/p3e_espa_arm2_scene.xml` (the ESPA scene stays as the reference). Same kinematics, ranges, gripper, bus size and 150.1 kg total.
  - Level 1: joint housings carry the mass (shoulder 3 + 3 kg, elbow 2.5, wrist 3 × 1.5, carbon tubes 0.6 + 0.45, gripper 2.5); arm 16.55 kg (was 10.1), bus 133.55 kg; arm geoms now collide with the client (class `arm_col`).
  - Level 2: armature 0.8/0.8/0.3/0.05/0.05/0.05 kg·m² (160:1 and 100:1 gearing), torque 60/60/40/12/12/12 N·m, residual friction 0.3/0.3/0.2/0.1/0.1/0.1 N·m; 10°/s joint speed limit recorded, not enforced.
- Static check (`check_arm2.py`): stowed 0.651 × 0.600 × 0.900 m (fits ESPA), radius 0.596 m, reach unchanged; shoulder inertia in the ready pose 6.9 kg·m² (was 3.9).
- Grasp smoke (`arm2_smoke.py`, `run_arm2_smoke.sh`, 3B grasp, 8 seeds, still and 2°/s, no controller retuning):
  - arm2 8/8 and 8/8, no arm-client contact, push median 0.6–0.65 mm/s, max 0.84 (old arm 0.8, 0.93).
  - Joint speed peak 9.2°/s in reach, 1.9°/s in approach: within 10°/s. The old arm hit 26°/s at the wrist and asked 1.5× its wrist torque limit (wrist jitter in approach); arm2 uses at most 8% of any torque limit.
  - Approach tracking error max 11.6 mm (old 8.2): the heavier arm lags the impedance target more.
- **3C grip problem found and fixed (13 Nov):** in the first E3 attempt on arm2 nearly every mission ended `grip_reclamped` (pad_chatter in 3C identification). Single-mission debug (`dbg_arm2_3c.py`, `run_dbg_arm2.sh`, seeds 6000001–2): removing all friction or only the **wrist** friction fixes it; removing shoulder/elbow friction, gear inertia or arm collisions does not; dropping friction on braked joints in `stack_sim.lock_arm` did not help (reverted). Cause: 0.1 N·m wrist friction against the 30 N·m/rad grip rotation stiffness kept the wrist from complying while the jaws closed, so the pads seated unevenly. Wrist friction now 0.03 N·m (0.02 and 0.05 both pass). Gripper visuals swapped into `p3e_espa_arm2_scene.xml` at the same time (physics bit-identical).
- **Run sizes (user, 13 Nov):** demonstration project, keep runs small (about 25). `run_e3_arm2.sh <split> [N=25] [workers=8]` runs the first N missions; P4.3 eval scripts default to 25.
- **Wrist stiction removed (13 Nov):** seed 6000015 (spin_axis_err, 1°/s) still re-clamped with 0.03 N·m wrist friction (pad drop 4 s into detumble); removing wrist friction or the shoulder/elbow gear inertia fixed it, removing shoulder/elbow friction or arm collisions did not. The arm2 wrist now has only viscous damping (no Coulomb stiction), as a stand-in for a torque-controlled or passively compliant wrist; shoulder/elbow keep 0.3/0.3/0.2 N·m.
- **E3 on arm2, 25 missions, final (13 Nov, same seeds):** **24/25 success (96%, CI 80–99%), 25/25 safe, 0 unsafe**, nominal 22/22, all three faulted missions as on the old arm (thruster_stuck seed 6000016 safe abort, the others success). Every mission's outcome matches the old ESPA arm. Final error ≤ 6.6 cm and 0.12°, mass error ≤ 0.39%, fly-in clearance ≥ 0.29 m, RCS median 17.4 g, 8.6 min.
- **Full-mission replay (13 Nov):** `export_mission.py <out.json> <seed> [scene]` records a whole E3 mission (fly-in from the hold point, grasp, brakes, detumble, id, 3D move) by wrapping GraspSim.run and StackRun._step; 4 fps to capture, 1 fps after. Seed 6000000 on arm2 → `replay/mission_arm2.json` (1601 s, 13.7 m move, 58° turn, 3.0 cm / 0.08° final). The replay page (https://claude.ai/artifact/WxRmRjcM5Zu4DoBkTQhKw8, version 2) now opens on "Full mission, new arm"; the camera follows the stack, new phase colours, 300× speed. Local check: `.claude/launch.json` "replay" serves `projects/p3_capture/replay` (the browser pane must be visible, or animation frames stop). Version 3 adds Earth: the exporter records the radial direction per frame (LVLH x in the world frame; world is client-centred and inertial, so it turns about 104° over the mission); the page draws a procedural Earth with atmosphere and stars as a background layer in km, 400 km below, plus an "Earth below" camera. The sim has no Sun model: the page puts the sun at (0.45, 0.8, 0.4) so this arc stays in daylight.
- *Superseded (wrist 0.03 N·m):* **E3 on arm2, 25 missions done (13 Nov, validation 6000000–24, 9 min on 8 workers, `results_e3_arm2_validation_n25.json`):** 23/25 success (92%, CI 75–98%), 25/25 safe, 0 unsafe; nominal 22/22. Faulted: thruster_weak success, thruster_stuck safe abort (same as the old arm), spin_axis_err seed 6000015 grip re-clamp (old arm: success; one pad drop, worth a look). Final error ≤ 6.4 cm and 0.13°, mass error ≤ 0.36%, fly-in clearance ≥ 0.29 m, RCS median 17.5 g. Old arm on the same 25 seeds: 24 success, 1 safe abort.
- *Superseded:* **E3 validation on arm2 started 13 Nov about 04:30 UTC** (`run_e3_arm2.sh validation 12`, log `p3_capture/e3_arm2_val.log`, 499 missions, 12 workers, about 4 h). Reuses the ESPA stage-A hand-off (stage A only uses mass and bounding radius, unchanged). If interrupted, rerun the same command: it resumes from `results_e3_arm2_validation.json.partial.jsonl`. Compare with ESPA validation: 84.8% success, 99.8% safe, 0 unsafe, nominal 395/399.
- **Gripper visuals (13 Nov), waiting in `p3e_espa_arm2_scene.next.xml`:** jaw drive motor, guide rails with end caps, finger carriages and backing plates, F/T ring, wrist camera on a mast; class `vis` (density 0, no contact). `check_vis_identical.py`: masses, inertias, colliding geoms and a 2000-step simulation are bit-identical. **Swap it over `p3e_espa_arm2_scene.xml` once the E3 run ends** (E3 reloads the scene per mission). Renders: `output/arm2/arm_compare.png`, `gripper_compare.png` (`render_arm2.py`, `render_gripper.py`).
- **Not yet checked on arm2:** fly-in, 3C/3D (brakes, detumble with the heavier arm), E2/E3, P4 camera scene and images.

**12 Nov: the ESPA servicer port is complete (SPEC_3 §14).**
- `robot_spec` is wired in; p3b gives bit-identical results (20/20 E3 missions, re-checked after `step_hook`).
- Bus gains are scaled by mass and inertia. ESPA `pregrasp_d` changed from 1.35 to 1.25 m (the arm was at 96% of its reach).
- **ESPA E3, no tuning:**

  | Run | Missions | Success | Safe | Unsafe | Nominal success |
  |---|---|---|---|---|---|
  | Tune | 200 | 89.0% | 99.5% | 0 | 167/169 |
  | Validation | 500 | 84.8% | 99.8% | 0 | 395/399 |
  | Test (seeds 7000000–299, as p3b) | 300 | 84.7% | 100% | 0 | 242/244 |

  The one difference from p3b is more `clear_pred` safe aborts, about 1% of nominal missions.
- Freeze `e3_espa_freeze.sha256` v1.1 (only the `grasp_sim.py` hash changed, for the no-op `step_hook`; v1 is in `e3_espa_freeze_v1.sha256`).
- `demos_v1` recorded: 500/500 episodes, 231,500 frames, 9.7 GB at `/home/shrey31/p4_data/demos_v1`.
- **Next: P4.3**, the pose-estimator baseline (SPEC_4 §4.2). See the next block.

**P4.3 in progress (12 Nov, stopped by three laptop shutdowns, two of them from overheating):**
- **Built** (all in `projects/p4_vision/`):
  - `data_stats.py`: in the bus frame the bar moves 12 cm median during an episode (up to 30 cm); a mean-pose guess is off by 48 mm median.
  - `pack_data.py`: packs a demo set into memmaps. **v1 data deleted 12 Nov** (`demos_v1`, `demos_v1_pack`, `tmp/colab_upload`, about 43 GB), at the user's request; v1 results stay in this file and SPEC_4.
  - `pose_net.py` (2.7M parameters: a CNN per sensor plus the arm joint angles → bar position and rot6 in the bus frame).
  - `train_pose.py`: resumable; saves `state.pt` every epoch.
  - `pose_est.py` (`NetPoseEst`: 10 Hz estimate, EMA smoothing, latched at close, velocity feed-forward from the bus's own motion).
  - `eval_est.py`: closed loop, oracle vs estimate, same seeds as gen_demos.
- **`grasp_sim.py`:** new `pose_est` hook (None = truth). Re-checked 20/20 identical on p3b. The ESPA freeze hash for `grasp_sim.py` is out of date again; re-freeze when P4.3 is done.
- **P4 push limit fixed before any result:** success also needs client push ≤ 2 mm/s (SPEC_4 §1).
- **Training state:** epoch 1 of 20 is saved at `/home/shrey31/p4_data/pose_v1/state.pt`. Validation after epoch 1: 14 mm median overall, 13 mm while approaching (target about 5 mm).
- **Resume:** `wsl -d Ubuntu -- bash /mnt/c/Shrey_Projs/robotics/mjorbit/projects/p4_vision/run_p43.sh` (log `p4_vision/p43.log`; the identity check can be removed from it, since it has passed). It continues training from `state.pt`, then evaluates 200 validation seeds, oracle vs estimate.
- **Plan: train on Colab** (free T4), chosen 12 Nov. Drive folder `MyDrive/mjorbit_p4/` (https://drive.google.com/drive/folders/1G7ea8RtyuczMO86P68WA3P8SX7-6N1w5) was created by Claude through the Drive connector, with `code/` already filled (3 scripts and the notebook, byte-identical). The user still uploads `tmp/colab_upload/data/*` (10 shards + SHA256SUMS) into `data/` and `tmp/colab_upload/pose_v1/state.pt` into `pose_v1/`. Then open `code/colab_p43.ipynb` in Colab and run all cells; it resumes from Drive after any disconnect. `train_pose.py` picks fp16 on a T4 (no bf16), untested on Colab so far. Afterwards copy `pose_v1/best.pt` back to `/home/shrey31/p4_data/pose_v1/` and run only the evaluation steps of `run_p43.sh` locally (CPU only).
- **v1 result (Colab T4, 20 epochs):** best epoch 5, 13.8 mm median overall, 11.7 mm while approaching. The network overfit after that.
  - **Cause, from `mount_floor.py`:** the ±1 cm, ±1° mount randomisation alone puts a 16–17 mm floor under any single-sensor estimate. It differs per episode and no image reveals it.
- **v2 (12 Nov, recorded):**
  - **Sensors:** mounts calibrated to ±1 mm and ±0.1° (the floor is now 1.4 mm); bus camera 192, lidar 128; images every 0.25 s with labels at the image time (rows stay at 10 Hz).
  - **Training:** photometric augmentation only (no shifts: the bar's place in the image is the pose signal).
  - **Data:** `demos_v2`, 500/500 episodes, 7.4 GB at `/home/shrey31/p4_data/demos_v2`. The upload bundle is `tmp/colab_upload_v2/`.
  - **Drive:**
    - `code_v2/` has the scripts and `colab_p43_v2.ipynb` (uploaded by Claude);
    - the user uploads the shards into `data_v2/`;
    - results go to `pose_v2/`;
    - delete v1's `data/` and `pose_v1/` from Drive first, to stay under 15 GB.
  - **Later:** a learning curve with 125, 250 and 500 episodes, to check whether 500 is enough.
- **Thermal limit (12 Nov):**
  - **Shutdowns:** sustained GPU training shut the laptop down twice, once with a 20-worker simulation alongside it, and once alone with 4 data loaders, after about 4 minutes.
  - **What ran fine before:** CPU-only runs (E3, 20 workers for 2+ h) and the demo recording (20 workers with EGL CPU rendering).
  - **Before training again, pick one:**
    - add a duty-cycle throttle to `train_pose.py` (pause between steps to cap GPU load);
    - train at 64×64 (about 4× less compute);
    - cap GPU power in the vendor tool (the user's choice; Claude doesn't change system settings);
    - use a cooling pad;
    - train in the cloud.

**Project 4 prepared (12 Nov):**
- Research: [RESEARCH_SERVICING.md](learning/RESEARCH_SERVICING.md), "Project 4 research". Closest precedents: SpacePolicy (LeRobot ACT, free-floating capture) and AstroBench (task ACT 14/20 against a generalist VLA at 1.5%).
- Draft spec, awaiting the user's review: [SPEC_4.md](projects/p4_vision/SPEC_4.md). ACT on wrist and bus cameras outputs grip-point targets; contact stays classical; the classical expert gives the demonstrations and DAgger labels; the E2 shield stays on; the fair baseline is the classical grasp on an image-estimated pose.
- Rendering checked: EGL works under WSL2 (28 frames/s at 224×224 while busy), and mujoco-warp 3.7.0.1 has a batch renderer.
- **P4.0–P4.2 built (12 Nov, SPEC_4 §4.1):** camera scene, randomised camera rig, `step_hook` in GraspSim, demo recorder. Smoke tests passed: 4/4 episodes; the camera scene's physics is identical to the plain ESPA scene.
  - `gen_demos.py` skips episodes already saved, so re-running it resumes.

**Earlier (11 Nov): E3 is complete on the p3b servicer.** Details are in [SPEC_3.md](projects/p3_capture/SPEC_3.md) §13 (code frozen as v2 in `e3_freeze.sha256`).

**Then:**
1. **Project 4:** P4.3 pose-estimator baseline, then P4.4 LeRobot conversion and ACT in a separate env.
2. **Optional:** E2 per-fault rows on ESPA (`evaluate_p3e_safety.py --set scene=p3e_espa_scene.xml`); re-run the E2 kick and nominal validation rows with the new `rate_anom`.

**This session (11 Nov):**
- Ran the E3 tuning pass (clean, freeze v1), the 1000-mission validation, two fixes (freeze v2), a regression check and the test split. Results are below.
- Wrote the 32-page guide `output/pdf/mjorbit_project_guide.pdf`. The source is `learning/PROJECT_GUIDE.md`; build it with `python tmp/pdfs/build_guide.py` (Windows Python with reportlab; render checks with pypdfium2).
- **Runs:** long runs need the laptop awake, plugged in with the lid open; a power-off killed the first validation run. The app's keep-awake ends about 5 minutes after the session goes idle. Launch WSL runs from PowerShell (`wsl -d Ubuntu -- bash /mnt/c/.../script.sh`). Git Bash rewrites `/mnt/...` paths, and `$VAR` set inside `bash -lc` did not reach `rlrun.sh`. Put commands in a script file.

**E3 results (freeze v2 unless noted):**

| Run | Missions | Success | Safe | Unsafe | Nominal success |
|---|---|---|---|---|---|
| Tuning (v1) | 200 | 89.5% | 99.5% | 0 | 168/169 |
| Validation (v1) | 1000 | 85.0% | 99.6% | 0 | 773/776 |
| Regression (v2, validation seeds 6000000–199) | 200 | 88.0% | 100% | 0 | 163/163 |
| **Test (v2)** | **300** | **85.7% (81.3–89.2%)** | **100% (98.7–100%)** | **0** | **244/244** |

- **Test details:** faulted missions 56/56 safe. Final error ≤ 9.1 cm and 0.33°; mass error ≤ 0.6%; fly-in clearance ≥ 0.53 m; RCS median 26 g. 21/300 fly-ins were slowed by the online path-rate scaling.
- **Why the test was cut to 300:** stage B was cut from 1000 at 22:25 with 16 done, before any outcome was looked at. This servicer is a stand-in for the ESPA one, and 300 bounds the unsafe rate at about 1%.
- **Fixes after validation (v1 to v2):**
  - `pad_chatter`: any pad-contact drop trips the grip monitor. It caught the barely loose grips (about 0.07 mm) that `pad_loss_s` missed.
  - Fly-in feedforward of the client's angular acceleration (ẇ × r), from a filtered derivative of the measured rate.
  - Online fly-in path-rate scaling (k from 1 down to 0.25) so the feedforward stays within 0.65 N at the measured client rate. A nutating client's rate vector wanders off the planning axis and made the axial leg thrust-infeasible.
- **Stage B is resumable:** it writes `<out>.partial.jsonl` per mission and resumes from it (a power-off killed the first validation run). `run_e3B.sh <split>` runs stage B only.

**Decisions this session (also saved in memory):**
- **The current servicer and client are a test run;** the user's final models will differ. So, after E3:
  1. Build the **robot description layer** (`robot_spec.py`, already written, not yet wired in).
  2. Port the code to the new **ESPA-class servicer** (`p3e_espa_scene.xml`): 150 kg, 0.70 × 0.60 × 0.90 m stowed (fits the 61 × 71 × 97 cm ESPA port), one 6-joint arm (0.55 + 0.45 m) folded flat for launch, same gripper, thrusters and wheels. The user wants to keep it simple.
  3. Re-run E2 and E3 on it, with the current servicer as the reference.
- **Project 4:** camera-based, imitation-learned grasp with LeRobot (ACT or Diffusion Policy first; SmolVLA later only if language commands are wanted).
  - The classical controller provides the demonstrations, and the E2 monitors act as the shield.
  - Use a Cartesian end-effector action space so the learned policy is not tied to one arm.

**E3 design (`evaluate_e3.py`, SPEC_3 §13):**
- **Stage A (GPU, batched):** 3A with the 9-state filter under the u1 uncertainty preset flies from 20–50 m to a **hold point 8 m out on the client's angular-momentum axis**.
  - Why the momentum axis: the body spin axis nutates, because the spin plus the LVLH rate is not about a principal axis. A hold point on it swings about 0.3 m and 40/200 approaches timed out.
  - `CaptureEnv(goal_mode="hold")` and `set_spin()` implement this.
- **Hand-off:** the arrival error, in the client body frame, goes to `GraspSim.reset(hold_dp_b, hold_dv_b)`.
- **Stage B (CPU pool):** `Stack3D` with the fly-in, the monitors and the fault, passed as `grasp_kw`, then grasp → 3C (grip monitor, re-clamp) → 3D.
  - **Per case:** spin 0–2°/s about a random axis; random client mass properties; client yaw.
  - **Faults:** with probability 0.2, one of client kick, weak thruster, stuck thruster, spin-axis error, or loose jaw brake.
  - **Options:** `--no-act` (monitors record only) and `--seeds` (run selected missions).
- **Outcome classes:**
  - **safe:** success, safe abort, grip re-clamped;
  - **unsafe:** collision, an abort that never reached safety, a lost client;
  - **neutral failures:** approach failed, grasp failed, 3D failed.
- **Success criteria (fixed beforehand):** 3D ends within 0.2 m, 2 mm/s, 0.5° and 0.01°/s with both pads on the bar.
- **Smoke test (20 nominal missions, before the tuning pass):** 19 succeeded and 1 aborted. That abort is now removed: the saturation trigger is record-only, and with no abort the mission succeeds. Final errors were ≤ 7.7 cm and ≤ 0.18°; RCS use was about 25 g; median wall time is about 3 minutes per mission per core.

**Monitor changes made during E3, from tuning data:**
- `rate_anom` is now a **jump** in the client's inertial angular velocity within 2 s, above 0.1°/s, and is not checked during close.
  - Why: a client spinning about a non-principal axis drifts slowly with no fault (only its angular momentum is constant). Comparing against the rate at the start false-aborted 6/20 nominal missions.
  - Re-check on the E2 tuning seeds: 40/40 kicks detected instantly; 0 false alarms in 40 nominal runs.
- `thrust_sat` is now **record-only** (`MonitorCfg.sat_abort=False`). A 1.9°/s nutating client tripped it; with no abort the mission succeeded and kept ≥ 0.58 m clearance.
- **Consequence for E2:** the §12 E2 validation numbers were measured with the old `rate_anom` and with `thrust_sat` able to abort. Re-run the E2 validation kick and nominal rows (`run_p3e_val.sh`, or just those two faults) before quoting them as final.

**Replay page:** [Servicer Capture Replay](https://claude.ai/artifact/WxRmRjcM5Zu4DoBkTQhKw8) (private artifact). It is a three.js playback of a nominal fly-in and grasp, and of a spin kick leading to an abort, with an inertial or client-rotating view. Data comes from `export_replay.py` (`run_export_replay.sh <out.json> <seed> <spin> [fault] [fps]`) into `replay/`, and the page is `replay/index.html`. To show a full mission, export one after E3 and republish the same file.

**Where Project 3 stands.** The servicer (150 kg, 6-joint arm, parallel-jaw gripper) and the client (a rectangular satellite with a 5 × 5 cm **square** grapple bar; mass properties randomised and unknown to the controller) work end to end on validation cases:

| Step | What | Result (validation) |
|---|---|---|
| 3A | approach to the pre-grasp point: box keep-out, 15° corridor, 9-state filter | 100%, 0 collisions (u1 and none) |
| 3B | inverse-kinematics reach, impedance approach, freeze references, close | 100% free-flying and free-floating, 200 cases |
| 3B load | grip strength (elliptic cones, no-slip, condim 4, square bar, 17.6 N clamp) | ≥ 8 N and ≥ 3.6 N·m every direction |
| 3C | brakes, detumble, momentum-form identification | mass 0.1%, inertia 0.3–0.5%, centre of mass 2.5 mm |
| 3D | move 5–15 m (thrust aligned through the centre of mass) and rotate 30–120° | 100% of 40, about 3 cm and 0.1° |
| 3D momentum | saturating wheels, gyroscopic term, RCS unloading (default) | 100%, no saturation, about 30 g of propellant |
| 3E E1 | spinning client ≤ 2°/s: synchronised grasp, detumble, identification | 100%; thrust limits it to about 2°/s |
| 3E E1b | fly-in from a spin-axis hold (rotating-frame spiral) then grasp | 100% at 0 / 0.5 / 1 / 2°/s, clearance ≥ 0.6 m |
| 3E E2 | monitors and aborts with injected faults (SPEC_3 §12) | **0 collisions with monitors** (5 kick / 3 weak-thruster collisions without); 0 false alarms; safe in ≤ 40 s; grip re-clamp keeps 50/50 (8/50 with no action) |
| 3E E3 | whole mission, 3A → fly-in → grasp → 3C → 3D, faults at rate 0.2 (SPEC_3 §13) | test 300: **0 unsafe**, 100% safe, 85.7% success, nominal 244/244 |

**E2 in brief (`safety.py`, `GraspCfg(mon=True, fault_kind=...)`, `StackCfg(grip_mon=True, grip_response="reclamp")`):**
- **Monitors:**
  - predicted coasting clearance;
  - client-rate anomaly (the ADRAS-J trigger);
  - thruster FDI from the accelerometer residual (under 1 s);
  - tracking and thrust saturation, near the client only;
  - unexpected contact;
  - post-capture grip slip.
- **Abort:**
  1. open the jaws;
  2. retract the gripper inertially 0.25 m;
  3. stop co-rotating and back off along the radial line to 6 m with a Hermite path inside the thrust budget;
  4. stow the arm;
  5. on a thruster alarm, re-point the bus so the bad axis lies out of the back-off plane.
- **Grip:** slip leads to releasing the brake and re-squeezing with the jaw drive. Safing alone saves only half.
- **Validation hygiene:** thresholds were set on the tune split (3,000,000+). The weak-thruster re-pointing was added after validation failures, so both thruster faults were re-run on the fresh val2 split (4,000,000+).
- **Open policy question:** monitors abort stuck-thruster cases that could have been captured (95 aborts against 98 captures with no monitor). Fault-tolerant continuation (estimate the bias, feed it forward, resume) is a possible extension.

**Pending, smaller:**
- re-run 3D after the shared wheel-model refactor (`run_p3d_unload.sh`);
- re-run the E2 validation kick and nominal rows with the new `rate_anom`;
- E3 open question: the thruster-weak missions that aborted (10/11 on test) might be capturable with fault-tolerant continuation; the replay page could show an E3 mission (`export_replay.py`).
- nutating tumbles beyond the slight wobble;
- RCS minimum-impulse model;
- E2 extensions: nav-fault monitors (need a pose estimator), wheel/RCS faults in 3D, fault-tolerant continuation;
- the replay page could show a whole mission, including 3C and 3D.

**How to run (from PowerShell; scripts take the simulator root as working directory):**
```
wsl -d Ubuntu -- bash -c "cd /mnt/c/Shrey_Projs/robotics/mjorbit/simulator && bash ../projects/p3_capture/<script>.sh"
```
- **Scripts:**
  - `run_p3b_eval.sh`, `run_p3b_load.sh`;
  - `run_p3c_eval.sh <tag> --n N --set k=v`;
  - `run_p3d_unload.sh`;
  - `run_p3e_grasp.sh <spins>`, `run_p3e_stack.sh`, `run_p3e_flyin.sh`;
  - E2: `run_p3e_safety.sh <tag> [--fault K] [--no-act] [--split tune|validation|val2]`, `run_p3e_tune.sh`, `run_p3e_val.sh`, `run_p3e_val2.sh`, `run_p3e_grip.sh <tag> --loose L|-1 [--no-act] [--set grip_response=reclamp]`, `run_p3e_grip_val.sh`.
- **Single cases:**
  - `run_p3c_one.sh <seed>`;
  - `run_p3d_one.sh <seed> <align|fixed> <id|prior|truth> [none]`;
  - `run_flyin_test.sh <spin> <seed>`;
  - `run_dbg_e2.sh <seed> <fault> [rec]`, `run_dbg_e2_multi.sh <fault> <seeds...>` (prints outcome and first unintended contact);
  - E3: `run_e3.sh <split> <n> [workers]` (stage A then B), `run_e3_a.sh --n N --split S --out F`, `run_e3_b.sh --handoff F --workers W --out G [--no-act] [--seeds ...]`; `run_e3B.sh <split> [workers]` (stage B only, resumes from `<out>.partial.jsonl`); `run_e3_regress.sh`, `run_e3_test300.sh` (the exact runs of 11 Nov); diagnosis replays: `dbg_e3.py <seeds>` (post-capture pad-contact gaps), `dbg_track.py <seeds>` (fly-in feedforward, thrust and tracking error), run through `rlrun.sh`;
  - ESPA: `run_check_espa.sh` (mass, stowed envelope, reach), `run_robot_spec.sh [scene.xml ...]` (prints the robot description).
- **Gotchas:**
  - put loops in `.sh` files (PowerShell blanks `$var` and breaks inline Python), and strip CR with `sed -i 's/\r$//'`;
  - mjorbit compiles through a temporary XML next to the scene, so parallel workers retry on a race; delete stray `.case_*.xml` files;
  - the mjorbit CPU data exposes only qpos/qvel/xpos/xmat/ctrl, so `GraspSim` keeps a synced plain-MuJoCo copy for Jacobians and contacts;
  - the Windows `python` has no numpy (use WSL for analysis, or the stdlib);
  - from Git Bash, set `MSYS_NO_PATHCONV=1` before passing `/mnt/...` paths to `wsl`, and keep shell variables inside `.sh` files;
  - `pkill` from a `bash -c` that contains the pattern can kill itself first, so check `ps` afterwards (on 10 Nov two validation loops ran at once and mixed code versions).

**Earlier history on 3A** (kept for reference): the client changed on 8 Nov from a Starlink-like bus to the rectangular satellite. 3A re-run: 100% under u1 and none (99.5% for an inertial client under u1); raw measurements only 68.5%, so the filter is required at grasp precision.

**Research for the end goal is done (8 Nov):** [learning/RESEARCH_SERVICING.md](learning/RESEARCH_SERVICING.md), with sources, covers missions (MRV/RSGS, MEV, ClearSpace-1, ADRAS-J, Orbital Express, ELSA-d), capture interfaces, arm control, post-capture identification, learning results and tools. The roadmap in SPEC_3 §1 was revised from it. Key points:
- (Starlink findings, kept for a later unprepared-client extension) a real Starlink is **unprepared**, likely dead, possibly heavier than the servicer. Real arms grab **launch-interface structure**. Alternatives: an adhesion pad (gecko-like) and caging (ClearSpace-1). The current client sidesteps this with a **grapple bar**.
- Arm control: generalised Jacobian, **reaction null space** (flight-proven on ETS-VII) for free-floating capture, **impedance control** at contact.
- Post-capture: momentum-based, then RLS **identification of the inertia and centre of mass**; thrust through the combined centre of mass; momentum dumping.
- Safety: ADRAS-J aborted on relative-attitude anomalies, so add a client pose-error model and an attitude-anomaly abort.
- Learning: valuable for motion planning and synchronisation; contact stays classical. mjorbit's PPO Astrobee grasp (88%, `examples/ppo/astrobee_grasp.xml`) is the learned reference. Use domain randomisation and timestep/contact sensitivity checks.

**3B classical grasp done (8 Nov, SPEC_3 §5):**
- Code: `p3b_scene.xml` (servicer, 6-joint arm, parallel-jaw gripper, client with bar, contact on), `grasp_sim.py` (mjorbit CPU physics plus a synced plain-MuJoCo copy for Jacobians and contacts; inverse-kinematics reach, Cartesian impedance approach, freeze-and-close, hold), `evaluate_p3b.py`, `run_p3b_eval.sh`, `trace_grasp.py`.
- **100% on 200 validation cases** for both free-flying and free-floating, at dt 0.001 / 0.002 / 0.005 s. No unintended contact, client push ≤ 0.75 mm/s, peak contact force about 7.7 N.
- Free-floating disturbs the base up to 6.7°, against 1.5° free-flying, which motivates reaction-null-space motion.
- **Grip load test done (SPEC_3 §6):**
  - MuJoCo's default pyramidal friction let the grip creep at about 7 N whatever the clamp. The scene now uses **elliptic cones with impratio 10**, and the squeeze is **0.03 (17.6 N clamp)**.
  - The grip holds ≥ 8.3 N and ≥ 3.7 N·m in every direction, against an estimated 3C need of about 0.5 N and 0.4 N·m (requirement 2 N / 1.5 N·m).
  - 3B re-run with these defaults: still 100% (`*_v2.json`). Scripts: `loadtest_p3b.py`, `run_p3b_load.sh`.
**3C done (8 Nov, SPEC_3 §7):** `stack_sim.py`, `evaluate_p3c.py`, `run_p3c_eval.sh <tag> [--n --set k=v]`, `run_p3c_final.sh`; debugging aids `debug_p3c.py` and `check_p3c_physics.py`.
- Randomised unknown client (200–400 kg, centre of mass ±10 cm, inertia ±20%), grasped, braked, given a tumble of up to 2°/s.
- The servicer detumbles in ≤ 7 s, then identifies the stack by momentum-form least squares from wheel pulses and thrust pulses (4 s windows).
- **100 cases:** mass 0.11%, inertia 0.28%, centre of mass 2.5 mm (medians). The counter-torque from the estimate cuts the attitude error during off-axis thrust from 10.3° to 0.02°. With 10× sensor noise: about 1–2% and 17 mm.
- **Model changes needed to make the stack rigid** (now defaults):
  - arm and jaw brakes (pinned joint ranges after capture);
  - `noslip_iterations` 20;
  - condim 4;
  - a **square-section grapple bar** (round version in `p3b_scene_round.xml`).

  3B re-run on the square bar: 100% in both modes.
- **Wheel momentum reached 84 N·m·s,** mostly from holding attitude against off-axis thrust. 3D must thrust through the stack centre of mass or dump momentum.
**3D done (8 Nov, SPEC_3 §8):** `maneuver3d.py` (`Stack3D`, `Man3DCfg`), `evaluate_p3d.py`, `run_p3d_eval.sh`, `run_p3d_one.sh <seed> <align|fixed> <id|prior|truth>`.
- After 3C, translate the stack centre of mass 5–15 m (CW epochs) and reorient 30–120° (rate-limited eigenaxis slew with the identified inertia), then hold.
- **id/align: 100% of 40 cases,** final error 3 cm and 0.11° (median); identical to using the truth.
- **Without identification** (nominal prior): 80% success, 4× position error, 10× transfer attitude error.
- **Fixed attitude** (wheels compensate the off-centre-of-mass thrust): 97.5%, about 25% more wheel momentum, one 10° excursion.
- **Momentum management done (SPEC_3 §9):**
  - wheels now saturate (30 N·m·s per axis) and produce the gyroscopic torque −ω × h;
  - 3D starts with 3C's leftover momentum;
  - the attitude law compensates ω × (Îω + h);
  - optional RCS couple unloading (0.5 N·m, k 0.05/s), now the **default**.
- **Results (40 cases):**
  - align + unloading: **100%**, no saturation, wheels end below 1 N·m·s, about 30 g of propellant (max 45 g);
  - no unloading: 95% (2 saturate and lose attitude);
  - slower slews: 65% (too slow, still saturating);
  - fixed + unloading: 97.5%.
  - Script: `run_p3d_unload.sh`. The evaluator takes variants as `id/align/unload=true,w_max_deg=0.2`.
- The end-state protocol was revised twice for measurement reasons (documented).
**3E step E1 done (8 Nov, SPEC_3 §10): spinning client.**
- `GraspSim.reset(spin_deg_s, spin_axis)`: the servicer starts synchronised at the pre-grasp point. Synchronised station keeping uses ω × r velocity and centripetal feed-forward.
- **One wheel model** (`StackRun.actuate`) now covers 3C and 3D: saturation, gyroscopic reaction, RCS unloading.
- **Grasp:** 100% at 1 and 2°/s (100 cases each); the bar shifts up to 11 mm at 2°/s.
- **3C on the spinning stack:** detumble ≤ 9 s, no saturation, about 25 g of RCS propellant at 2°/s, identification as accurate as for a still client.
- **Thrust limits synchronisation to about 2°/s** (0.5 of 0.75 N).
- **Brake engagement** (model rebuild) makes pad contacts resettle for ≤ 0.03 s, so the grip check has a 0.05 s grace window.
- Scripts: `run_p3e_grasp.sh <spins>`, `run_p3e_stack.sh`.
**E1b fly-in done (SPEC_3 §11):** `flyin.py` and `GraspCfg(flyin=True)`.
- From a hold point 8 m out on the spin axis, the servicer flies a rotating-frame path: axial, radial spiral, axial; minimum-jerk, ≤ 5 cm/s, Coriolis and centripetal feed-forward.
- **100% (100 cases each) at 0 / 0.5 / 1 / 2°/s**, fly-in 200–400 s, clearance ≥ 0.6 m, no thrust saturation, arrival error ≤ 2 cm / 0.8 mm/s / 0.15°.
- Script: `run_p3e_flyin.sh`.
- **Next in 3E:**
  1. E2 safety monitors and aborts (clearance, relative-attitude anomaly, grip slip, wheel saturation);
  2. E3 whole-mission Monte Carlo (chain 3A to the spin-axis hold → fly-in → grasp → 3C → 3D per case), then the test set once.
- **Also pending:** re-run 3D after the wheel-model refactor.

**Original 3B plan** (SPEC_3 §4):
- 6–7 joint arm with a two-finger gripper closing around the **grapple bar**; contact on, timestep ≤ 0.01 s;
- classical baseline: inverse-kinematics or resolved-rate reach plus impedance control;
- free-flying first, then free-floating with reaction-null-space arm motion.
- 3A+ (pose-error model, servicer attitude, abort) can run in parallel or just before.

Code is in `projects/p3_capture/`: `p3a_scene.xml`, `capture_env.py` (`CaptureEnv` on P2's maneuver env through the new geometry hooks in `approach_env.ApproachEnv`), `capture_guidance.py`, `evaluate_p3a.py` (`--unc --yaw --client-rot --gkw seg_margin=4 r_safe=12`), and the scripts `run_p3a_tune.sh` and `run_p3a_cases.sh`.

**Project 2, final state:**

**Final test-set result (1000 cases, u1):** r1/599 + shield got 99.2% success (Wilson 98.4–99.6%) with 0 collisions and Δv 0.303. The shielded CW + filter benchmark used Δv 0.322 at 99.5%, so the residual was −0.0185 m/s on paired cases and lower in 89%. The fuel and collision gates are met. The reliability gate is not: the same CW base behind the shield got 100%. Under no uncertainty, every controller got 100%.

**Main findings:**
- **The shield matters.** It is needed at 1000 cases: unshielded CW collides 1–2 times per 1000, and no shielded run ever collided.
- **CW's fixed 480 s arrival is the weak point:**
  - it arrives late and fast, about 20 s before the settle deadline;
  - with a 0.1 s physics step, CW-based success drops to 91–97%;
  - `--cw-arrive 440` fixes that.
- **Direct PPO (u1/2099)** is the most robust: 100% in every validation run, and it never needs the shield. It uses 9% more fuel than r1.

**Glideslope follow-up (SPEC_2 §19, validation for tuning, fresh split `--split val2`, seed 2000, for confirmation):**
- `guidance.py`, `GlideslopeEpoch`, a hybrid: bias-compensated CW to the hold point (arrival 440 s) until 3 m out, then glideslope with closing speed 0.03·ρ, clipped to [0.003, 0.1] m/s.
- On val2, shielded: 100% success under u1, none and dt 0.1; Δv 0.280 (u1) / 0.249 (none); 0 interventions; closest approach 2.95 m. It beats CW (0.322), r1 (0.304) and direct PPO (0.332). **This is the best controller in the 20 s action space.**
- **Residual r2/599 on the glideslope base** (`logs/r2`, `run_r2.sh`): u1 0.260 (−0.019 paired), but only 60% success under none and 56% under `u1dyn`.
  - The cause: with zero nav noise the covariance inputs are out of distribution.
  - Fix: randomise nav noise in training, or drop or normalise the covariance inputs. Not done.
- New flags: `evaluate.py --glideslope K,VMAX,VMIN[,RSWITCH,TARRIVE] --no-cw --base gs --split val2`, `train.py --base gs`. New presets `u1nav` and `u1dyn`.

**Next options:**
- Retrain r2 with nav-noise randomisation, and confirm on val2 under u1, none and dt 0.1.
- Project 3 (arm capture, via LeRobot).
- A shield for the 1 Hz controllers; residual and shield support in replay.
- Rebuild the PDF: `python tmp/pdfs/build_curriculum.py`.

**Starting point:**
- Code (all in `projects/p2_approach/`):
  - `nav_filter.py`: 9-state bias filter (numpy `BiasKalman`, GPU `BiasKalmanTorch`), `NavController`;
  - `residual_env.py` (`--env residual`): CW epoch + nav9 plus 0.02·a;
  - `shield.py`: B5 shield (`Shield`, `NavTracker`), hooked into `ManeuverEnv.step_world_dv` via `env.shield`;
  - `evaluate.py` flags: `--shield` (plain and shielded), `--shield-only`, `--epoch-only` (skip the 1 Hz runs), `--raw-epoch`, `--shield-drift` (free-drift backup), `--shield-sigma`, `--shield-horizon`;
  - checks: `check_nav.py`, `profile_residual.py`, `diag_shield.py` (when and where the shield acts);
  - scripts: `run_b6.sh` (1000-case Monte Carlo and the dt 0.1 re-run), `run_test.sh` (test set: already used, do not re-run for selection), `run_b5.sh <unc> <n> <tag>` (residual and nav9 CW with and without the shield, then direct PPO with the shield), `run_b5_all.sh`, `run_r1.sh`, `run_eval_r1.sh <its>`.
- **B5 design:** for each 20 s Δv candidate, predict the burn, a coast to the next epoch, then the backup "stop and station-keep" for 300 s. It must stay at least 2 m + 3σ (line of sight) from the target.
  - Candidates, in order: nominal, CW to hold, stop, retreat.
  - The spec's free-drift version is infeasible: the u1 disturbance drifts the vehicle about 0.9 m in 300 s, while the hold point is 1 m from the keep-out zone. All controllers fell to 0% success with it.
- **B5 results, u1 validation, 200 cases:**
  - r1/599 + S: 100%, 0 collisions, Δv 0.325, 24% of episodes intervened;
  - benchmark CW epoch + nav9 + S: 100%, Δv 0.349, 29%;
  - direct PPO u1/2099 + S: 100%, Δv 0.344, never intervened (min dist 2.83 m);
  - the shield raised the minimum distance from about 2.1 m to 2.38–2.40 m;
  - the 2B gate passes on validation for r1 + S (and direct PPO + S).
- **Why the shield acts:** every intervention falls at 460–480 s, because single-arc CW arrives at the 3 m hold point closing at 7–9 cm/s. The braking distance of about 0.56 m doesn't fit inside the margin, so there are about 40 "no certified option" epochs per 200 episodes. The real fix is a closing-speed limit in guidance (glideslope); this is an optional follow-up.
- **Selected policy, provisional:** `logs/r1/model_599.pt` with the shield. Under `none` it no longer beats the benchmark (+0.003), so its advantage is u1-specific.
- **Open items:**
  - r1 vs direct PPO: decide in B6 at 1000 cases;
  - no shield yet for the 1 Hz controllers;
  - at 1 Hz, bias compensation raises Δv (cause unverified);
  - `record_rollout.py` and `replay_p2.py` don't support `--env residual` or the shield;
  - the learning PDF hasn't been rebuilt since B1 (`python tmp/pdfs/build_curriculum.py`; LEARNINGS.md is current).
- The test set (seed 1000) was used once on 8 Nov (SPEC_2 §18). Use fresh validation seeds for any further selection.

**How to run:**
- From PowerShell: `wsl -d Ubuntu -- bash /mnt/c/Shrey_Projs/robotics/mjorbit/tools/rlrun.sh python ../projects/p2_approach/<script> ...`. The working directory is the simulator root.
- B5/B6 evaluation: `bash /mnt/c/.../projects/p2_approach/run_b5.sh u1 1000 mc` from the simulator root. 200 cases take about 75 s per script call; the run writes `results_b5_<tag>_<unc>.json` and `eval_b5_<tag>_<unc>.log`.
- Residual training: `bash projects/p2_approach/run_r1.sh` (about 37 min for 600 iterations). Maneuver training example: `train.py --env maneuver --unc u1 --stage a3 --num-envs 2048 --steps-per-env 16 --gamma 0.99 --lam 0.99 --w-shape 0.1 --w-fuel 4 --std-min 0.02 --resume <ckpt>`.
- Shell gotchas:
  - Call `wsl` from PowerShell, not Git Bash, which rewrites `/mnt/...` paths.
  - Don't put `$var` loops inside `wsl -- bash -c "..."` from PowerShell; write a `.sh` file and strip CR with `sed -i 's/
$//'`.
  - `sed -n '/x/,$p'` breaks under PowerShell quoting; use `grep -A`.
- Replay: `record_rollout.py --env maneuver --checkpoint <ckpt> --out <npz>`, then `replay_p2.py --traj <npz> --port 8422` and open http://localhost:8422.
- Nothing is left running.

## User goals and learning preferences

The user wants to understand space robotics and train their own control policies in mjorbit on their laptop.

**End goal (stated 8 Nov 2026):** a servicing robot that attaches itself to a satellite and then moves it to a commanded attitude or position. Projects 1–3 are steps toward this. **Client, as changed by the user on 8 Nov:** a generic rectangular satellite (300 kg class) with something to grasp (a grapple bar), not a Starlink. Unprepared or Starlink-like clients are a later extension. Control of the combined stack after capture is part of the goal.

The agreed project progression is:

1. **Stabilize:** stop a randomly spinning spacecraft, then extend to pointing toward a requested orientation and holding it.
2. **Approach:** direct the spacecraft toward a nearby co-orbiting satellite, brake, match relative motion, and maintain a holding position without contact.
3. **Capture:** add a gripper arm, reach a prepared handle on the target satellite, grasp it, retain it, and stabilize the combined system.

The user is comfortable with Python, knows AI, can spend 10+ hours per week, and expects to move quickly. They want conceptual understanding while using AI coding tools for implementation and debugging. Do not force a beginner AI course or a fixed twelve-week schedule. Establish familiarity with specific control/RL topics as needed rather than assuming general AI experience covers them.

Use a milestone-based learning loop: **predict the motion, implement a small experiment with AI help, inspect physical measurements, explain the result, then increase difficulty.** Retain meaningful baseline comparisons and evaluation even if the user moves quickly.

The initial request was code-free research and curriculum preparation. The user then requested cloning, installation, and opening a simulation (done 6 November), and installing the GPU training environment (done 7 November). Policy training started 7 November: Project 1 (detumble, pointing) is complete and Project 2 is in progress (sections below).

## Workspace and artifacts

Windows workspace: `C:\Shrey_Projs\robotics\mjorbit`

| Location | Contents |
|---|---|
| [README.md](README.md) | Original curriculum overview |
| [learning/CURRICULUM.md](learning/CURRICULUM.md) | Lessons, exercises, and understanding checks |
| [learning/PROJECTS.md](learning/PROJECTS.md) | Three project briefs, observations/actions, baselines, and training stages |
| [learning/EVALUATION.md](learning/EVALUATION.md) | Proposed success gates, experiment records, and hardware measurement plan |
| [learning/AI_WORKFLOW.md](learning/AI_WORKFLOW.md) | How to use AI coding assistance while learning |
| [learning/SOURCES.md](learning/SOURCES.md) | Primary references and reading sequence |
| [learning/RESEARCH_SERVICING.md](learning/RESEARCH_SERVICING.md) | Research toward the servicing end goal (8 Nov): missions, capture methods, control, learning, sources |
| [projects/p3_capture/SPEC_3.md](projects/p3_capture/SPEC_3.md) | Project 3 spec, roadmap and 3A results |
| [output/pdf/mjorbit_learning_curriculum.pdf](output/pdf/mjorbit_learning_curriculum.pdf) | Combined 22-page guide; chapter 07 "What we learned so far" (from [learning/LEARNINGS.md](learning/LEARNINGS.md), added 7 Nov) records actual results. Rebuild with `python tmp/pdfs/build_curriculum.py` (Windows Python, reportlab); previous version kept in `tmp/pdfs/` |
| [LOCAL_SETUP.md](LOCAL_SETUP.md) | Actual installation, verification, restart command, and camera workaround |
| [simulator/](simulator/) | Cloned upstream repository |
| [mjorbit_paper.pdf](mjorbit_paper.pdf) | Local copy of the paper |

Important: the curriculum, overview, and PDF were written before installation and before the user clarified their AI experience. Their statements that nothing is installed and their twelve-week beginner pacing are historical. For current status use this file and `LOCAL_SETUP.md`. The underlying project specifications remain useful. Numerical gates are educational proposals, not measured results or flight requirements.

## Paper and project understanding

Primary sources: [paper](https://arxiv.org/pdf/2609.08010), [project page](https://johnzhang3.github.io/mjorbit/), [repository](https://github.com/johnzhang3/mjorbit), [documentation](https://johnzhang3.github.io/mjorbit/docs/).

mjorbit combines MuJoCo articulated/contact dynamics with orbital dynamics and spacecraft-specific forces, actuators, and sensors. Its orbit-following local frame and cancellation-resistant differential-gravity calculation make small robot motions numerically manageable alongside planetary-scale orbits. Integrator choice still affects the accuracy/speed tradeoff.

The paper's reorientation, docking, and capture/nadir-pointing demonstrations use MPPI planning. Its Astrobee capture demonstration uses learned control. An impressive simulator/planner demo is not automatically an existing RL task or trained policy.

For learning, start with numerical simulator state, simple geometry, bounded control authority, and small policies. Defer camera-based perception, arbitrary distant-orbit transfers, and complex tumbling-target capture.

## Verified device and WSL environment

- Host: Windows; PowerShell is the working shell.
- CPU: AMD Ryzen AI 9 HX 370.
- GPU: NVIDIA GeForce RTX 4060 Laptop GPU, about 8 GB VRAM (`8188 MiB` reported).
- NVIDIA driver reported during setup: `610.74`.
- Existing Linux distribution: `Ubuntu`, WSL version 2, user `shrey31`.
- WSL reported about 15 GiB memory and 4 GiB swap at setup. This is the guest's available memory, not a verified host-RAM specification.
- GPU visibility inside Ubuntu was verified with `nvidia-smi`.

Both the **CPU simulator/browser viewer** (`default`) and the **GPU training environment** (`rl`: PyTorch 2.13 cu130, Warp 1.15, mujoco-warp 3.7.0.1, rsl-rl 5.4.1) are installed and verified as of 7 November 2026. See `LOCAL_SETUP.md` for versions, tests, and measured throughput.

## Repository and installation details

- Clone path: `C:\Shrey_Projs\robotics\mjorbit\simulator`
- Ubuntu path: `/mnt/c/Shrey_Projs/robotics/mjorbit/simulator`
- Origin: `https://github.com/johnzhang3/mjorbit.git`
- Checked revision: `a09090671fe6299b1085d235623e4b2d00b4e675`
- Repository working tree was clean at the last check. No upstream source or lockfile modifications were needed.
- Read `simulator/AGENTS.md` before modifying upstream code.

Pixi was installed inside Ubuntu at `/home/shrey31/.pixi/bin/pixi`. Its installer was run without updating the user's shell PATH, so use the full executable path unless PATH is deliberately configured later.

The default environment provides Python **3.12.13**, MuJoCo **3.7.0**, a C++ compiler, CMake, Ninja, and viewer dependencies. The C++ plugin and Python bindings compiled successfully.

### Installation lessons

1. Installing the environment on the Windows-mounted filesystem was very slow. The attempt was stopped, and the incomplete environment was subsequently removed.
2. Project-local Pixi configuration at `simulator/.pixi/config.toml` sets `detached-environments` to `/home/shrey31/.cache/mjorbit-envs`. Source remains in the Windows workspace; environments live on Ubuntu's native filesystem. Installation then completed quickly.
3. Default CMake configuration initially chose Unix Makefiles, but Make was unavailable. Explicitly configuring **Ninja**, supplied by the pinned environment, resolved this. Use `CMAKE_GENERATOR=Ninja` for subsequent fresh builds.
4. The committed lockfile has an older format. Keep it unchanged and use `--locked` unless an intentional dependency update is requested.
5. Detached environments survive separately from the workspace; moving/removing the project can require separate environment cleanup.

## Verification already completed

- Minimal example: advanced to `Time: 1.00 s`.
- Public model/data API tests: **8 passed**.
- Free-drift analytical comparison: 60 simulated seconds, 6,000 steps at 0.01 s; final position error approximately `7.83e-6 m` against its CW reference; example passed.
- Viewer returned HTTP 200, connected in Chrome, and visibly rendered Earth, the spacecraft, and simulation controls.

These checks establish a working local CPU setup for those scenarios. They do not establish broad physics fidelity or learned-policy performance.

GPU/RL checks (7 November 2026):

- `python -m pytest tests/mjorbit_warp -q` in the `rl` env: 41 passed, 3 skipped. (Plain `pytest` fails collection with `No module named 'tests'`.)
- The bundled PPO truss example (`examples/ppo/train.py`, full orbital backend) trains end to end.
- Measured throughput on that example: ~15.5k env steps/s at 1024 worlds, ~26k at 2048, ~41k at 4096; peak total GPU memory ~1.1 GiB at 4096. Memory is not the constraint at these sizes. These are 10-iteration measurements on one contact-rich task, not a capacity limit.

The user's viewer demos (`capture_stabilize_mppi`, `arm_reach_mppi`) are MPPI planners, not learned policies. The repo also has `examples/ppo/astrobee_*`: a learned detumble, approach, and grasp task that overlaps all three user projects and is a useful reference for task/reward design.

## Viewer operation and confirmed camera issue

Viewer address: [http://localhost:8080/](http://localhost:8080/).

The viewer is not running (started and stopped again on 13 Nov). Check before starting another server on the same port. Check before starting another server on the same port.

The default `free_drift` task shows a dual-arm spacecraft with solar wings, slow tumble, and periodic arm motion. Default altitude is 500 km. The user adjusted the altitude control to 770 km during exploration; do not reset user settings merely to match defaults. Some parameters require Reset to take effect.

Other registered tasks: `arm_reach_mppi` and `capture_stabilize_mppi`. They use planning and can need more computation.

**Confirmed usability lesson:** the user reported camera jitter when dragging the view. `Track spacecraft` was enabled. The tracking implementation in `simulator/src/viewer/framing.py` sends camera-position/look-at updates every frame, which can compete with browser mouse input. Turning the checkbox off resolved the reported issue; the user explicitly confirmed it worked.

- Keep **Track spacecraft unchecked** for manual navigation.
- Use **Reframe view** to recenter.
- The restart instructions now include `--no-track`.
- This was a settings workaround, not a source-code fix to the camera tracker.
- Pause stops simulation playback; it does not shut down the server.

### Restart from PowerShell

Keep the terminal open while using the viewer, then open the local address in a browser:

```powershell
wsl -d Ubuntu -- bash -lc 'cd /mnt/c/Shrey_Projs/robotics/mjorbit/simulator && export CMAKE_GENERATOR=Ninja && /home/shrey31/.pixi/bin/pixi run --locked viewer --task free_drift --no-track --host 127.0.0.1 --port 8080'
```

Press Ctrl+C in that terminal to stop a manually launched viewer. The server binds to loopback for local access.

## Technical conventions to preserve

These distinctions matter when defining policies and debugging motion:

- Reference orbit position/velocity use **km and km/s** in absolute ECI coordinates.
- Robot world coordinates use **metres and m/s**, relative to the moving chief/reference origin, with axes parallel to ECI.
- That world frame is not LVLH. LVLH rotates; velocity conversion needs the frame-rotation contribution.
- Free-body angular velocity is expressed in the body frame.
- `xmat` maps body-frame vectors to world coordinates; `ximat` represents principal-inertia orientation and can differ.
- Applied body wrench buffers use world-frame forces/torques; transform body commands appropriately.
- Use the existing `MjoModel`, `model.make_data(...)`, `mjo_forward`, and `mjo_step` API.
- Distinguish physics timestep from policy/control interval and report both.

## Agreed project boundaries and evaluation lessons

For Project 1, detumbling and pointing are separate tasks. Start with a damping baseline and an explicitly declared ideal torque model. Later introduce physical wheel/thruster constraints. Reaction wheels store angular momentum; they cannot absorb arbitrary spin indefinitely.

For Project 2, initially start metres away in the same orbital neighbourhood. Success requires position, velocity, clearance, and hold duration. A goal fixed in a rotating frame needs a consistent desired velocity. Use a baseline attitude controller before jointly learning translation and rotation.

For Project 3, start with a reachable prepared handle and low relative motion. Verify scripted reach/contact before learning. A closed gripper or brief contact does not establish capture: require retained grasp and post-capture stabilization. A two-joint arm needs restricted geometry or spacecraft repositioning; arbitrary 3D capture needs a reachability assessment.

Compare against conventional controllers using matched limits and initial conditions. Keep held-out cases, failure counts, and physical metrics. Report force effort as a proxy unless a propellant model actually exists. Reward alone and selected animations are insufficient evidence.

## Next useful work

1. Project 1A spec is written: [projects/p1_detumble/SPEC.md](projects/p1_detumble/SPEC.md), with model [p1_spacecraft.xml](projects/p1_detumble/p1_spacecraft.xml) and checks in [measure.py](projects/p1_detumble/measure.py). The design is a 10 kg box with I = 0.108/0.242/0.283 kg·m², ±0.01 N·m body torques, 10 Hz control, 30 s episodes, and observation = body ω plus previous action. Measured: torques act in the body frame with positive sign; coast conserves momentum (body ω wanders through gyroscopic coupling); gravity gradient is ~2.6e-7 N·m (negligible); the saturated damping baseline (K = 2/s) gets 100% success up to ±0.3 rad/s per axis, settling within 7.7 s, with effort near the ‖H₀‖ floor. So PPO can at best match it. Open choice: a 0.002 N·m "hard" variant would make the deadline bind.
2. Done 7 November 2026: `detumble_env.py`, `check_env.py`, `train.py`, `evaluate.py`, `tb_summary.py` in `projects/p1_detumble/` (results in SPEC §11).
   - The GPU env matches the CPU model closed-loop (≤ 2.9e-4 rad/s). Open-loop multi-axis tumbling shows an O(dt) backend integration difference, not a bug.
   - Curriculum S1 → S2 → S3 took about 9 minutes in total at about 30k steps/s.
   - Test set (seed 1000, 200 cases): policy 100% success, settle median 5.15 s, max 8.3 s; damping baseline 100%, 5.2 / 8.4 s. The policy uses 7% more effort and leaves a ~1e-4 rad/s residual. It matches the baseline and does not beat it, as predicted.
   - Gotcha: rsl-rl `--resume` continues the iteration count, so S2 ends at `model_298.pt` and S3 at `model_547.pt`.
3. Hard variant (0.002 N·m) done 7 November 2026, SPEC §11 (`--tau-max 0.002` on train/evaluate; run `logs/hard`, chosen checkpoint `model_750.pt`). Test: baseline 48%, PPO 47% (overlap: both 93, baseline-only 3, policy-only 1). 31/200 test cases are provably infeasible (‖H₀‖/(√3·τ_max) > 25 s). Damping gain K = 2 vs 20 makes no difference: saturated damping already applies the greedy box-corner torque. The policy ties; it found no non-greedy advantage.
4. Remaining for 1A: training seeds 1–2, inertia ±20% and observation-noise stress tests.
5. Project 1B (pointing) done 7 November 2026: [projects/p1_detumble/SPEC_1B.md](projects/p1_detumble/SPEC_1B.md).
   - Files: `pointing_env.py` (`PointingEnv` subclasses `DetumbleEnv` via `_task_terms`/`_on_reset` hooks); obs = error-quaternion vector (sign-fixed) + ω/0.3 + previous action; 40 s episodes. `train.py --stage p1/p2/p3` (≤ 30°/90°/180°); `run_pointing_curriculum.sh`; `evaluate_pointing.py`.
   - Baseline: rate-limited PD (ω_ref = axis·min(Kp·θ, √(2βα_min θ))). Tuned on validation: Kp 0.6, Kd 2, β 0.8.
   - First test (v1): both 100% success. Baseline settles in a median 11.85 s (max 18.0 s); PPO in 15.8 s (max 26.1 s) and stops about 2.8° off target in every case.
   - Investigation (SPEC_1B §9–14), all choices on validation:
     - The 2.8° offset was **not** reward satisficing. It was the network's fixed zero-torque point, bias ÷ gain, because near-target error inputs are tiny (shown with `diag_offset.py`).
     - Fix: obs v2 appends clip(e/0.02, ±1).
     - Reward v2 (graded bonuses, rate penalty only near the target) plus obs v2 gave about 0.01° precision but slow cruising on large turns (92–94% success); γ = 0.997 helped only a little (`diag_failures.py`).
     - Reward v1 + obs v2 was best.
   - Chosen `logs/p3rv1ov2/model_797.pt` (`--obs v2`). Test: 100% success; settle 12.9 / 18.4 / 21.1 s vs baseline 11.85 / 16.8 / 18.0 s; final error 1.17° (baseline 0.001°); 11% more effort. Still does not beat the baseline.
   - Reward v3 (v1 + 0.5·exp(−(θ°/0.5)²) fine bonus) + obs v2 (`logs/p3rv3ov2`), validation only (SPEC_1B §16): 100% success, worst final error 0.028° (vs 1.17°), but settling is slower (median 15.8 s, max 27.8 s vs 12.6 / 21.2 s) and effort is slightly lower. Neither dominates; the user chooses between speed (rv1ov2) and precision (rv3ov2). The test set has been used once (rv1ov2); testing v3 would be a second use, to be reported as such.
   - **Final 1B choice (SPEC_1B §17): v3 + zoom, `logs/p3rv3ov2/model_797.pt`.** Test (seed 1000, second test-set use): 100% success [98.1, 100], settle median/max 16.25/26.8 s (baseline 11.85/18.0), worst final error 0.028°, effort 0.145 vs 0.154 N·m·s. Project 1 complete; next candidates: robustness stress tests (inertia error, sensor noise), extra training seeds, or Project 2 (approach).
   - Remaining options: an odd-symmetric policy; seeds 1–2.
   - Flags: `train.py --reward v1|v2 --obs v1|v2 --gamma`; `run_pointing_curriculum.sh <reward> <obs>` (GAMMA env var); `evaluate_pointing.py --obs`.

The `rl` environment is installed and benchmarked (done 7 November 2026). The `default` and `rl` environments share `simulator/build/cpp`, so build them one at a time and run `sync-package` in the environment you are about to use.

Full bundled truss run done 7 November 2026 (run `truss440`, 18.4 min at 1024 worlds): eval mean pointing error 3.3°, final-quarter 2.6° (repo reports ~1° final-quarter on a 3080). Details and replay instructions are in `LOCAL_SETUP.md`. The user watched it through [tools/replay_truss.py](tools/replay_truss.py), a live viser replay on port 8421, because the repo's mp4 recorder needs Chrome and ffmpeg in Ubuntu (not installed; needs sudo). The user's first own policy (Project 1A detumble) now exists: `projects/p1_detumble/logs/s3/model_547.pt`.

## Project 2: approach (projects/p2_approach), status 2026-11-07

Full record in [projects/p2_approach/SPEC_2.md](projects/p2_approach/SPEC_2.md) (§1–14).

**Task.**
- Scene `p2_scene.xml`: a 10 kg chaser with ±0.05 N world-frame thrusters (attitude assumed held) and an inert 100 kg, 1 m target at the chief, in a 400 km / 51.6° orbit. Physics dt is 1.0 s, which gives 1.2 cm drift error over 600 s.
- Goal: reach the hold point (0, −3, 0) m LVLH, on V-bar, which is a natural equilibrium. Entering the 2 m keep-out sphere is a failure.
- Success: within 0.25 m and 0.01 m/s for the final 60 s of a 600 s episode. Starts are 20–50 m away, inside a 60° cone around −V-bar.
- Splits: validation seed 100, test seed 1000. The test set is still unused.

**What was learned, in order.**
1. The physics matches CW to 0.1 mm. A hand-written PD baseline hits 100% success, so fuel (Δv) became the learning target.
2. Direct 1 Hz thrust PPO (`approach_env.py`):
   - The fuel cost on Gaussian exploration collapsed the policy to "never thrust".
   - Strong potential-based shaping made it learn, but biased it toward fast arrival.
   - Best reliable result: Δv 0.58.
3. The PD baseline was a weak benchmark. **CW impulsive targeting is the real one:** Δv 0.231 at 1 Hz and 0.292 with 20 s epochs, both 100% success.
4. **Maneuver-epoch actions** (`maneuver_env.py`: one 20 s Δv decision, 30 per episode):
   - Training is tractable.
   - Weakening the shaping, setting λ 0.99 and putting a floor under the exploration noise brings Δv down steadily.
   - Ideal-setting pick: `logs/m3/model_1500.pt`, 100% success, Δv 0.320.
   - That beats PD (0.348) but not CW.
5. **Uncertainty u1** (`--unc u1`):
   - Errors: thrust gain ±10%, misalignment ≤3°, constant disturbance ±2e-5 m/s², navigation noise 5 cm and 1 mm/s.
   - On raw measurements, the 1 Hz classical controllers chase noise and burn 1.5–2.4 m/s.
   - With the CW Kalman filter, CW 1 Hz is again best: 100%, Δv 0.272.
   - Epoch CW reaches only 77–83%, even with the filter.
   - **PPO `logs/u1/model_2099.pt`** (fine-tuned under u1): **100%, 0 collisions, Δv 0.344.** It is the only fully reliable controller in the 20 s action space, but it uses 26% more fuel than filtered 1 Hz CW.

**Next: 2B, GNC-structured learning** (SPEC_2 §14):
- B1: a 9-state filter that also estimates the disturbance bias.
- B2: optional accelerometer cutoff for burns.
- B3: the strongest classical benchmark, using that filter.
- B4: PPO on the filter estimate, a residual PPO on top of CW, and a recurrent policy as a stretch goal.
- B5: an explicit-simplex safety shield applied to all controllers.
- B6: a 1000-case Monte Carlo, then the test set once.
- Gate: in the 20 s action space under u1, match classical reliability and beat epoch CW on Δv.

**Tools.**
- Browser replay: `replay_p2.py` (viser, port 8422), with recordings from `record_rollout.py --env maneuver`.
- Evaluation: `evaluate.py --env maneuver --unc u1 --kf`.
- Notes:
  - Edit project files with UTF-8 I/O. A cp1252 write once emptied `evaluate.py`.
  - In PowerShell, filter output with simple `Select-String` patterns.

**After Project 2.** The user wants to explore LeRobot for Project 3: ACT and Diffusion Policy locally, SmolVLA (~450M), and π0.5 only on a rented GPU. The plan is to distil a state-based expert into an image-based policy.
