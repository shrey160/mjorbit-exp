"""Browser replay of recorded Project 2A rollouts (viser), in the LVLH frame.

    pixi run -e rl --locked python ../projects/p2_approach/replay_p2.py --traj <file.npz> [--port 8422]

Scene (metres, LVLH; camera "up" = radial, away from Earth):
  grey cube      target (1 m), at the origin
  red sphere     2 m keep-out zone
  green dot      hold point, 3 m behind the target along-track (-y)
  gold / blue / white boxes   PPO policy / planned-400 s PD baseline / CW targeting
                 (drawn 3x real size so they are visible from 50 m), with their trails
Thrust arrows show each chaser's current LVLH force (scaled).
"""

from __future__ import annotations

import argparse
import time

import numpy as np
import viser

CTRL = {"policy": ((230, 180, 40), "PPO policy"),
        "planned": ((70, 130, 230), "Baseline (planned 400 s)"),
        "cw": ((235, 235, 235), "CW targeting (480 s)")}
VIS_SCALE = 3.0


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--traj", required=True)
    ap.add_argument("--port", type=int, default=8422)
    args = ap.parse_args()
    d = np.load(args.traj, allow_pickle=True)
    cases = list(d["cases"])
    hold = d["hold"]
    T = d["policy_F"].shape[0]
    ctrl = {k: v for k, v in CTRL.items() if f"{k}_p" in d.files}

    server = viser.ViserServer(host="127.0.0.1", port=args.port)
    server.scene.set_up_direction((1.0, 0.0, 0.0))
    server.initial_camera.position = (25.0, -55.0, 35.0)
    server.initial_camera.look_at = (0.0, -15.0, 0.0)
    server.initial_camera.up = (1.0, 0.0, 0.0)

    sc = server.scene
    sc.add_box("/target", dimensions=(1.0, 1.0, 1.0), color=(150, 160, 180))
    sc.add_icosphere("/keepout", radius=2.0, color=(220, 60, 60), opacity=0.18)
    sc.add_icosphere("/hold", radius=0.15, color=(60, 220, 90), position=tuple(hold))
    sc.add_frame("/lvlh", axes_length=5.0, axes_radius=0.04)
    sc.add_label("/lvlh/xlab", "x radial (up)", position=(5.5, 0.0, 0.0))
    sc.add_label("/lvlh/ylab", "y along-track", position=(0.0, 5.5, 0.0))
    sc.add_label("/lvlh/zlab", "z orbit normal", position=(0.0, 0.0, 5.5))
    sc.add_label("/holdlab", "hold point", position=tuple(hold + np.array([0.6, 0, 0])))

    gui = server.gui
    case_dd = gui.add_dropdown("Start case", options=[str(c) for c in cases],
                               initial_value=str(cases[0]))
    playing = gui.add_checkbox("Play", initial_value=True)
    speed = gui.add_slider("Speed (x real time)", min=1, max=60, step=1, initial_value=20)
    t_slider = gui.add_slider("Time (s)", min=0, max=T, step=1, initial_value=0)
    shown = {k: gui.add_checkbox(f"Show {lab}", initial_value=True) for k, (_, lab) in ctrl.items()}
    readout = {k: gui.add_text(lab, initial_value="", disabled=True) for k, (_, lab) in ctrl.items()}

    handles: dict = {}

    def build_case(ci: int) -> None:
        for h in handles.values():
            h.remove()
        handles.clear()
        for k, (col, _) in ctrl.items():
            P = d[f"{k}_p"][:, ci]
            handles[f"{k}_trail"] = sc.add_spline_catmull_rom(
                f"/{k}/trail", positions=P[::3], color=col, line_width=2.0)
            handles[f"{k}_body"] = sc.add_box(
                f"/{k}/body", dimensions=tuple(VIS_SCALE * np.array([0.5, 0.3, 0.2])), color=col)
            handles[f"{k}_start"] = sc.add_icosphere(
                f"/{k}/start", radius=0.25, color=col, position=tuple(P[0]))

    state = {"case": None, "t": 0.0}

    def draw(t: int) -> None:
        ci = cases.index(int(case_dd.value))
        for k in ctrl:
            vis = shown[k].value
            for part in ("trail", "body", "start"):
                handles[f"{k}_{part}"].visible = vis
            P, V = d[f"{k}_p"][:, ci], d[f"{k}_v"][:, ci]
            F, dv = d[f"{k}_F"][:, ci], d[f"{k}_dv"][:, ci]
            handles[f"{k}_body"].position = tuple(P[t])
            name = f"/{k}/thrust"
            f = F[min(t, T - 1)]
            if vis and np.linalg.norm(f) > 1e-4:
                tip = P[t] - f / 0.05 * 3.0  # exhaust direction, 3 m at full thrust
                sc.add_spline_catmull_rom(name, positions=np.stack([P[t], tip]),
                                          color=(255, 120, 30), line_width=4.0)
            else:
                try:
                    sc.add_spline_catmull_rom(name, positions=np.stack([P[t], P[t]]),
                                              color=(255, 120, 30), line_width=0.1)
                except Exception:
                    pass
            e = np.linalg.norm(P[t] - hold)
            readout[k].value = (f"to hold {e:6.2f} m | speed {np.linalg.norm(V[t]):.3f} m/s | "
                                f"dv used {dv[min(max(t - 1, 0), T - 1)] if t else 0.0:.3f} m/s")

    @case_dd.on_update
    def _(_e) -> None:
        state["t"] = 0.0
        t_slider.value = 0

    last = time.perf_counter()
    while True:
        now = time.perf_counter()
        dt, last = now - last, now
        ci = cases.index(int(case_dd.value))
        if state["case"] != ci:
            build_case(ci)
            state["case"] = ci
        if playing.value:
            state["t"] = (state["t"] + dt * speed.value) % (T + 1)
            t_slider.value = int(state["t"])
        else:
            state["t"] = float(t_slider.value)
        draw(int(t_slider.value))
        time.sleep(1 / 30)


if __name__ == "__main__":
    main()
