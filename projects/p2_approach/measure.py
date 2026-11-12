"""Project 2 pre-spec measurements on the CPU backend.

Run from the simulator repo root:
    pixi run -e rl --locked python ../projects/p2_approach/measure.py

Checks, in order:
  1. Compiled masses; the target placed at the chief stays there.
  2. Thrust pulse: sign, frame and size of the world-frame thrusters.
  3. Free drift vs the Clohessy-Wiltshire solution, and physics-timestep accuracy.
  4. Hold-point behaviour: a V-bar point is a natural equilibrium, a radial offset is not.
  5. Size of the relative-gravity accelerations vs the thruster limit.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from mjorbit import MjoModel, MjoSpec, OrbitInit, mjo_forward, mjo_step
from mjorbit.constants import GM_EARTH, R_EARTH

XML = Path(__file__).with_name("p2_scene.xml")
ALT_KM = 400.0
INC = np.radians(51.6)
A_KM = R_EARTH + ALT_KM
N = np.sqrt(GM_EARTH / A_KM**3)  # mean motion, rad/s
F_MAX = 0.05  # N per axis
M_CHASER = 10.0


def orbit() -> OrbitInit:
    v = np.sqrt(GM_EARTH / A_KM)
    return OrbitInit(R_eci=[A_KM, 0.0, 0.0], V_eci=[0.0, v * np.cos(INC), v * np.sin(INC)])


def compile_model(dt: float) -> MjoModel:
    spec = MjoSpec.from_xml_path(str(XML))
    return spec.compile(mj_timestep=dt)


def fresh(model, p_lvlh, v_lvlh):
    data = model.make_data(orbit=orbit())
    data.qpos[:] = 0.0
    data.qpos[3] = 1.0
    data.qpos[10] = 1.0
    data.qvel[:] = 0.0
    p_lvlh = np.asarray(p_lvlh, float)
    data.qpos[0:3] = data.world_position_from_lvlh(p_lvlh)
    data.qvel[0:3] = data.world_velocity_from_lvlh(p_lvlh, np.asarray(v_lvlh, float))
    data.ctrl[:] = 0.0
    mjo_forward(model, data)
    return data


def lvlh_state(data, adr=0):
    p = data.lvlh_position_from_world(np.asarray(data.qpos[adr:adr + 3]))
    v = data.lvlh_velocity_from_world(np.asarray(data.qpos[adr:adr + 3]),
                                      np.asarray(data.qvel[adr // 7 * 6:adr // 7 * 6 + 3]))
    return p, v


def cw(p0, v0, t):
    x0, y0, z0 = p0
    vx0, vy0, vz0 = v0
    nt = N * t
    c, s = np.cos(nt), np.sin(nt)
    x = (4 - 3 * c) * x0 + s / N * vx0 + 2 / N * (1 - c) * vy0
    y = 6 * (s - nt) * x0 + y0 - 2 / N * (1 - c) * vx0 + (4 * s - 3 * nt) / N * vy0
    z = z0 * c + vz0 / N * s
    return np.array([x, y, z])


def drift(dt, p0, v0, t_total):
    model = compile_model(dt)
    data = fresh(model, p0, v0)
    for _ in range(int(round(t_total / dt))):
        mjo_step(model, data)
    p, _ = lvlh_state(data)
    tp = np.asarray(data.qpos[7:10]).copy()
    return p, tp


def main() -> None:
    print(f"orbit: {ALT_KM:.0f} km circular, inc {np.degrees(INC):.1f} deg, "
          f"n = {N:.4e} rad/s, period {2 * np.pi / N / 60:.1f} min")

    # 1. masses
    model = compile_model(0.05)
    for name in ("chaser", "target"):
        bid = model.body_id(name)
        print(f"[1] {name}: mass {float(np.asarray(model.body_mass)[bid]):.2f} kg")

    # 2. thrust pulse: +F_MAX on world x for 10 s from rest at the chief offset
    data = fresh(model, [0.0, -20.0, 0.0], [0.0, 0.0, 0.0])
    v_before = np.asarray(data.qvel[0:3]).copy()
    data.ctrl[:] = [F_MAX, 0.0, 0.0]
    for _ in range(200):
        mjo_step(model, data)
    dv = np.asarray(data.qvel[0:3]) - v_before
    print(f"[2] 10 s of +{F_MAX} N on world x: world dv = {np.round(dv, 6)} m/s "
          f"(expected x {F_MAX / M_CHASER * 10:.4f}; gravity adds ~1e-5)")

    # 3. free drift vs CW, and timestep accuracy, 600 s
    p0, v0, T = [5.0, -30.0, 2.0], [0.01, 0.02, -0.01], 600.0
    ref = cw(p0, v0, T)
    p_fine, tgt = drift(0.01, p0, v0, T)
    print(f"[3] drift 600 s from {p0}, v {v0}: CW {np.round(ref, 3)}")
    print(f"    dt 0.01 sim {np.round(p_fine, 3)}  |sim - CW| = "
          f"{np.linalg.norm(p_fine - ref):.4f} m; target moved {np.linalg.norm(tgt):.2e} m")
    for dt in (0.05, 0.1, 0.2, 0.5, 1.0):
        p, _ = drift(dt, p0, v0, T)
        print(f"    dt {dt:<4} |sim - dt0.01| = {np.linalg.norm(p - p_fine):.2e} m")

    # 4. hold points (zero LVLH velocity), 600 s
    for hp in ([0.0, -3.0, 0.0], [0.5, -3.0, 0.0], [0.0, -3.0, 0.5]):
        p, _ = drift(0.05, hp, [0.0, 0.0, 0.0], T)
        print(f"[4] hold {hp} with zero LVLH velocity -> after 600 s {np.round(p, 3)} "
              f"(moved {np.linalg.norm(p - np.array(hp)):.3f} m)")

    # 5. relative-gravity acceleration scale (CW terms) at 50 m vs thrust
    a_rad = 3 * N**2 * 50.0
    a_thr = F_MAX / M_CHASER
    print(f"[5] CW accel at 50 m radial: {a_rad:.2e} m/s^2; coriolis at 0.1 m/s: "
          f"{2 * N * 0.1:.2e}; thrust limit {a_thr:.2e} m/s^2 ({a_thr / a_rad:.0f}x)")


if __name__ == "__main__":
    main()
