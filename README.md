# Learn spacecraft control with mjorbit

## Repository (mjorbit-exp)

The code, specs and notes of a space-robotics learning programme on the [mjorbit](https://github.com/johnzhang3/mjorbit) simulator, built up to an arm-equipped servicer that captures a spinning client satellite by a grapple bar.

- **Layout:**
  - `projects/`: one folder per project, each with its spec (`SPEC*.md`):
    - `p1_detumble`;
    - `p2_approach`;
    - `p3_capture` (grasp, stack control, safety monitors, whole-mission E3);
    - `p4_vision` (cameras, lidar and a learned pose estimator).
  - `learning/`: curriculum, research notes and lessons learned.
  - `tools/`: helper scripts.
  - `CONTEXT.md`: the running project log.
- **Not in this repo:**
  - the simulator: clone [mjorbit](https://github.com/johnzhang3/mjorbit) into `simulator/` (see `LOCAL_SETUP.md`);
  - generated data (results, logs, demonstrations, model weights, images, PDFs): see `.gitignore`;
  - `p3_capture/p4_espa_cam_scene.xml`: generate it with `projects/p4_vision/make_scene.py`.

The original plan from 6 November follows; [CONTEXT.md](CONTEXT.md) has the current state.

---

Prepared 6 November 2026 as research and project preparation; since then the projects above have been built (see CONTEXT.md).

This is a concept-first learning programme for someone comfortable with Python, new to robotics and reinforcement learning, studying at least 10 hours per week and using AI coding tools for implementation help.

The mission progression is:

1. Stop a spacecraft spinning, then hold a requested orientation.
2. Approach a nearby satellite in the same orbital neighbourhood and hold a relative position.
3. Reach a prepared capture handle with a robot arm, grasp it, and stabilize the coupled bodies.

This is a strong progression because each project introduces one major challenge: rotation, relative translation, then articulated contact. The third project includes post-capture control: touching the handle alone does not complete the mission.

## Start here

| Document | Purpose |
|---|---|
| [Curriculum](learning/CURRICULUM.md) | Twelve-week first pass, lessons, experiments, and understanding checks |
| [Project briefs](learning/PROJECTS.md) | Scope, observations, actions, training stages, and baseline controllers |
| [Evaluation](learning/EVALUATION.md) | Proposed success gates, experiments, and laptop resource plan |
| [Working with AI](learning/AI_WORKFLOW.md) | Use coding assistance while retaining ownership of the concepts |
| [Research and reading](learning/SOURCES.md) | Primary sources, reading assignments, and unresolved questions |

## Working assumptions

- Hardware: RTX 4060 Laptop GPU with 8 GB VRAM and AMD Ryzen AI 9 HX 370, confirmed in this session. System RAM and WSL availability still need verification.
- Learning software: mjorbit CPU for inspection/reference, mjorbit-warp for batched simulation, and the project's PPO/rsl-rl route for training. The documented GPU/RL workflow targets Linux/NVIDIA; native Windows is not an established release target. Investigate WSL2 before installing. [Platform documentation](https://johnzhang3.github.io/mjorbit/docs/installation.html)
- Start with numerical simulator state, simple collision geometry, a circular approximately 400 km Earth orbit, and a passive target with a prepared handle.
- Initially use bounded ideal forces/torques to isolate control concepts. Explicitly introduce physical thruster allocation and wheel momentum constraints later.
- Project 2 starts with local proximity operations, not a transfer between distant orbits. Distant rendezvous is a separate orbital-guidance extension.
- Project 3 starts with a slowly drifting, non-spinning target. Tumbling-target capture, vision, flexible structures, and arbitrary surface grasping are extensions.
- Twelve weeks is a planning estimate at roughly 10-14 study hours/week, not a guaranteed completion or training-time estimate. Move on when the learning and evaluation gates pass.

All numerical gates and task ranges in these documents are proposed educational design choices. They are not flight requirements, paper results, or measured laptop performance. Finalize them after verifying actuator authority and geometry, before training or inspecting held-out results.

## First session

Read the first two curriculum lessons. Draw the Earth, reference orbit, spacecraft body axes, and a velocity arrow. Explain why a spacecraft can be almost weightless while accelerating toward Earth, and why switching off a thruster does not stop motion. Then specify Project 1's inputs, commands, and success condition in plain language.
