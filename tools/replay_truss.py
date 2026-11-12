"""Live browser replay of a saved truss trajectory (from examples/ppo/play.py --save-traj).

Run from the simulator repo root in the rl env:
    python replay_truss.py --traj <file.npz> --port 8421
then open http://localhost:8421/ . The camera is set once per browser connection, so
mouse navigation is not fought by per-frame camera updates.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

_ROOT = Path.cwd()
for p in (str(_ROOT / "src"), str(_ROOT / "scripts" / "record")):
    if p not in sys.path:
        sys.path.insert(0, p)

import viser  # noqa: E402
from scene import SingleScene, forward_to  # noqa: E402

from mjorbit import MjoModel  # noqa: E402


def align_deg(data, truss_bid: int) -> float:
    """Angle between the truss long axis (body x) and nadir; either tip counts."""
    axis = np.asarray(data.xmat[truss_bid], dtype=float).reshape(3, 3)[:, 0]
    r = np.asarray(data.orbit.R_eci, dtype=float)
    nadir = -r / np.linalg.norm(r)
    return float(np.degrees(np.arccos(min(1.0, abs(float(axis @ nadir))))))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--traj", required=True)
    ap.add_argument("--port", type=int, default=8421)
    ap.add_argument("--mag", type=float, default=45000.0)
    ap.add_argument("--distance", type=float, default=100.0)
    args = ap.parse_args()

    d = np.load(args.traj, allow_pickle=True)
    qpos, r_eci, v_eci = d["qpos"], d["R_eci"], d["V_eci"]
    dt = float(d["dt"])
    n = len(qpos)

    model = MjoModel.from_xml_path(str(d["xml_path"]))
    data = model.make_data()
    bus, truss = model.body_id("bus"), model.body_id("truss")

    server = viser.ViserServer(host="127.0.0.1", port=args.port)
    forward_to(model, data, qpos[0], R_eci=r_eci[0], V_eci=v_eci)
    scene = SingleScene(server, model, data, mag=args.mag, track_body_id=bus)

    playing = server.gui.add_checkbox("Play", initial_value=True)
    speed = server.gui.add_slider("Speed (x real time)", min=1, max=60, step=1, initial_value=10)
    t_slider = server.gui.add_slider("Sim time (s)", min=0.0, max=(n - 1) * dt, step=dt,
                                     initial_value=0.0)
    err_text = server.gui.add_text("Pointing error (deg)", initial_value="", disabled=True)
    start_err = align_deg(data, truss)
    server.gui.add_text("Initial error (deg)", initial_value=f"{start_err:.1f}", disabled=True)

    target = scene.cluster_center_render([bus, truss])
    pos, look = scene.camera_pose(distance=args.distance, target=target,
                                  offset_rsw=np.array([0.42, 0.28, 0.86]))
    server.initial_camera.position = tuple(float(x) for x in pos)
    server.initial_camera.look_at = tuple(float(x) for x in look)
    server.initial_camera.up = (0.0, 0.0, 1.0)
    server.initial_camera.fov = float(np.radians(48.0))
    print(f"camera pos={np.round(pos, 2)} look_at={np.round(look, 2)}", flush=True)

    print(f"replaying {n} steps ({n * dt:.0f} s sim) at http://localhost:{args.port}/", flush=True)
    frame_dt = 1.0 / 30.0
    sim_t = 0.0
    while True:
        tick = time.perf_counter()
        if playing.value:
            sim_t += frame_dt * float(speed.value)
            if sim_t > (n - 1) * dt:
                sim_t = 0.0
            t_slider.value = round(sim_t / dt) * dt
        else:
            sim_t = float(t_slider.value)
        k = min(n - 1, int(round(sim_t / dt)))
        forward_to(model, data, qpos[k], R_eci=r_eci[k], V_eci=v_eci)
        scene.update()
        err_text.value = f"{align_deg(data, truss):.1f}"
        time.sleep(max(0.0, frame_dt - (time.perf_counter() - tick)))


if __name__ == "__main__":
    main()
