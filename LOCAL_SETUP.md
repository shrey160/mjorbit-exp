# Local mjorbit setup

Installed and verified on 6 November 2026.

## Open the running simulation

[Open the orbit viewer](http://localhost:8080/).

The `free_drift` scene shows a dual-arm spacecraft and solar wings in a 500 km circular orbit. The spacecraft tumbles and its arms slew periodically. This is an orbital dynamics demonstration, not a trained policy.

Use **Pause**, **Reset**, **Speed**, **Reframe view**, and the **Frame** selector. Initial orbit and tumble settings apply on Reset. Other tasks are available in the Task menu; planning tasks can require more computation.

For manual mouse navigation, leave **Track spacecraft** unchecked. The tracker sends camera-position updates each frame, which can compete with mouse movement. Use **Reframe view** to recenter as needed. The restart command below starts with tracking disabled; you can enable it in the sidebar when you want automatic following.

## What was installed

- Repository: `C:\Shrey_Projs\robotics\mjorbit\simulator`
- Origin: https://github.com/johnzhang3/mjorbit
- Revision: `a09090671fe6299b1085d235623e4b2d00b4e675`
- Linux environment: the existing `Ubuntu` WSL2 distribution, user `shrey31`.
- Pixi executable: `/home/shrey31/.pixi/bin/pixi`.
- Python 3.12.13, MuJoCo 3.7.0, and the compiled mjorbit CPU backend.
- Project-local Pixi configuration: `simulator/.pixi/config.toml` keeps environments under `/home/shrey31/.cache/mjorbit-envs`, on Ubuntu's native filesystem. This avoids slow package installation on the Windows mount. Pixi manages the mapping to the project.
- CMake was configured with Ninja, which is supplied by the pinned environment. No upstream source or lockfile changes were needed.
- NVIDIA access was verified inside WSL2: RTX 4060 Laptop GPU, about 8 GB VRAM.

The CPU/browser viewer environment (`default`) was installed first. The GPU training environment (`rl`) was added on 7 November 2026; see the next section.

## GPU training environment (`rl`)

Installed 7 November 2026 with `pixi install -e rl --locked` (about 18 minutes, mostly the PyTorch/CUDA download). It shares the same detached environment folder and was built with `pixi run -e rl --locked sync-package`.

| Component | Version |
|---|---|
| PyTorch | 2.13.0+cu130, `torch.cuda.is_available()` = True |
| NVIDIA Warp | 1.15.0 (CUDA toolkit 12.9, driver 13.3), sees `cuda:0` RTX 4060 Laptop, 8 GiB, sm_89 |
| MuJoCo / mujoco-warp | 3.7.0 / 3.7.0.1 |
| rsl-rl-lib | 5.4.1 |

Checks:

- GPU backend tests: **41 passed, 3 skipped** (`python -m pytest tests/mjorbit_warp -q`). Plain `pytest tests/mjorbit_warp` (the `test-warp` task) fails at collection with `No module named 'tests'`; `python -m pytest` puts the repo root on the import path.
- PPO smoke test on the bundled truss hug-and-point example (`examples/ppo/train.py`, full orbital `mjorbit` backend): trained without errors and mean reward rose over 5 iterations.

Measured training throughput, truss example, 10 iterations each, 32 steps per world per iteration:

| Parallel worlds | Env steps/s | Iteration time | Peak GPU memory |
|---|---|---|---|
| 256 | ~4.6k | 1.8 s | ~0.6 GiB |
| 1024 | ~15.5k | 2.1 s | ~0.7 GiB |
| 2048 | ~26k | 2.5 s | ~0.8 GiB |
| 4096 | ~41k | 3.2 s | ~1.1 GiB |

Peak memory is total GPU usage sampled every 0.5 s (about 0.35 GiB was in use before training). Memory is not the limit at these sizes; throughput was still rising at 4096. These numbers are for this one contact-rich task, and short runs; your own tasks will differ. The README's 440-iteration truss run at 1024 worlds would take roughly 15 minutes here. The first run of a session also spends some seconds compiling Warp kernels (cached afterwards in `~/.cache/warp`).

Run training from PowerShell (Ubuntu paths), for example:

```powershell
wsl -d Ubuntu -- bash -lc 'cd /mnt/c/Shrey_Projs/robotics/mjorbit/simulator && export CMAKE_GENERATOR=Ninja && /home/shrey31/.pixi/bin/pixi run -e rl --locked python examples/ppo/train.py --num-envs 1024 --max-iterations 440'
```

Logs and checkpoints go to `simulator/examples/ppo/logs/<run>` (git-ignored). The short benchmark runs above left small logs there (`smoke256`, `bench1024`, `bench2048`, `bench4096`) that can be deleted.

## First full training run (truss example)

Run 7 November 2026: `examples/ppo/train.py --num-envs 1024 --max-iterations 440 --run-name truss440`.

- Wall time **18.4 minutes** (~13.6k env steps/s sustained), peak GPU memory ~0.7 GiB total.
- Mean reward rose from about −1,400 (iteration 50) to about +4,000 (iteration 439). Episodes ran close to the full 1,200 steps (120 s) and the truss stayed hugged about 99% of the time.
- Evaluation (`play.py`, 64 worlds, seed 7, 120 s): mean pointing error **3.3°**, final-quarter mean **2.6°**, final-quarter 90th percentile 3.0°. The repo reports about 2–3° mean and about 1° final-quarter on an RTX 3080, so this run is close but slightly less precise at the end. That is one seed and one run.
- Checkpoints: `simulator/examples/ppo/logs/truss440/model_*.pt`. Training log copy: [output/runs/truss440_train.log](output/runs/truss440_train.log).

### Watching a trained policy

The repo's video recorder (`scripts/record/`) needs Google Chrome and ffmpeg installed inside Ubuntu, which they are not (installing them needs `sudo`). Instead, [tools/replay_truss.py](tools/replay_truss.py) replays a saved trajectory live in the browser, with play/pause, speed, a time scrubber, and the current and initial pointing error.

Save a trajectory, then serve it (run from PowerShell; keep the terminal open):

```powershell
wsl -d Ubuntu -- bash -lc 'cd /mnt/c/Shrey_Projs/robotics/mjorbit/simulator && /home/shrey31/.pixi/bin/pixi run -e rl --locked python examples/ppo/play.py --checkpoint examples/ppo/logs/truss440/model_439.pt --num-envs 64 --steps 1200 --seed 7 --save-traj ../output/runs/truss440_world43.npz'
```

```powershell
wsl -d Ubuntu -- bash -lc 'cd /mnt/c/Shrey_Projs/robotics/mjorbit/simulator && /home/shrey31/.pixi/bin/pixi run -e rl --locked python ../tools/replay_truss.py --traj ../output/runs/truss440_world43.npz --port 8421'
```

Then open [http://localhost:8421/](http://localhost:8421/). The saved trajectory [output/runs/truss440_world43.npz](output/runs/truss440_world43.npz) is the world closest to a 55° start, so the first command is only needed for a new checkpoint.

The `default` and `rl` environments share the native build directory `simulator/build/cpp`. Run `sync-package` in the environment you are about to use, and do not build both at the same time.

## Restart from PowerShell

Run this command and keep that terminal open while using the viewer:

```powershell
wsl -d Ubuntu -- bash -lc 'cd /mnt/c/Shrey_Projs/robotics/mjorbit/simulator && export CMAKE_GENERATOR=Ninja && /home/shrey31/.pixi/bin/pixi run --locked viewer --task free_drift --no-track --host 127.0.0.1 --port 8080'
```

Then open [http://localhost:8080/](http://localhost:8080/). Press Ctrl+C in the terminal to stop a manually started viewer. The current viewer was started by the assistant and is left running for this session; ask the assistant to stop it if needed. Pause stops simulation playback without shutting down the server.

The viewer is bound to loopback. It is intended for local use. If port 8080 is occupied, identify the existing service or choose another port and open the corresponding address.

## Checks completed

- Minimal example advanced to `Time: 1.00 s`.
- Public model/data API: **8 tests passed**.
- A 60 s free-drift example matched its CW reference with final position error about `7.83e-6 m` and passed the example's relative-error check.
- The browser connected and visibly rendered Earth, the spacecraft, and active simulation controls.

The analytical comparison checks a particular short, simple scenario; it does not establish general spacecraft-model fidelity or training performance.

## Notes

- Pixi reports that the committed lockfile uses an older format. It was kept unchanged and installations/runs use `--locked`.
- Windows learning materials and the combined curriculum PDF remain alongside the cloned repository.
- If the workspace is moved or removed later, the detached Linux environments are stored separately and may need explicit cleanup.
