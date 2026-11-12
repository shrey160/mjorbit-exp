"""Project 1 pre-spec measurements on the CPU backend.

Run from the simulator repo root (any pixi env with mjorbit built), e.g.:
    pixi run -e rl --locked python ../projects/p1_detumble/measure.py

Checks, in order:
  1. Compiled mass/inertia vs the hand calculation.
  2. Torque pulses: sign, axis, and body-vs-world frame of the motor actuators.
  3. Zero-torque coast: angular momentum conservation and gyroscopic coupling.
  4. Gravity-gradient torque size at 400 km (vs the 0.01 N*m actuator limit).
  5. Saturated rate-damping baseline over random initial spins (the gate test).
"""

from __future__ import annotations

import time
from pathlib import Path

import numpy as np

from mjorbit import MjoModel, OrbitInit, mjo_forward, mjo_step

XML = Path(__file__).with_name("p1_spacecraft.xml")
MU = 398600.4418  # km^3/s^2
R0 = 6378.137 + 400.0  # km, 400 km circular
ORBIT = OrbitInit(R_eci=[R0, 0.0, 0.0], V_eci=[0.0, float(np.sqrt(MU / R0)), 0.0])
I_HAND = np.array([10 / 12 * (0.3**2 + 0.2**2), 10 / 12 * (0.5**2 + 0.2**2),
                   10 / 12 * (0.5**2 + 0.3**2)])
TAU_MAX = 0.01
DT = 0.01
DECIM = 10  # control at 10 Hz


def quat_to_mat(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def fresh(model, w_body=(0.0, 0.0, 0.0), quat=(1.0, 0.0, 0.0, 0.0)):
    data = model.make_data(orbit=ORBIT)
    data.qpos[3:7] = quat
    data.qvel[:] = 0.0
    data.qvel[3:6] = w_body
    data.ctrl[:] = 0.0
    mjo_forward(model, data)
    return data


def h_world(data, bid) -> np.ndarray:
    r = np.asarray(data.xmat[bid]).reshape(3, 3)
    return r @ (I_HAND * np.asarray(data.qvel[3:6]))


def main() -> None:
    model = MjoModel.from_xml_path(str(XML))
    bid = model.body_id("sc")
    print(f"timestep {DT} s, control every {DECIM} steps ({DECIM * DT:.1f} s)")

    # 1. Inertia ---------------------------------------------------------------
    print("\n[1] inertia")
    print("  hand-computed I (kg m^2):", np.round(I_HAND, 4))
    try:
        print("  compiled body_mass:", float(np.asarray(model.body_mass)[bid]),
              " body_inertia:", np.round(np.asarray(model.body_inertia)[bid], 4))
    except AttributeError as exc:
        print("  compiled inertia not exposed on wrapper:", exc)

    # 2. Torque pulses ---------------------------------------------------------
    print("\n[2] torque pulses: +0.01 N*m for 1.0 s from rest")
    for axis in range(3):
        for label, quat in (("identity attitude", (1, 0, 0, 0)),
                            ("rotated 90deg about world z",
                             (np.cos(np.pi / 4), 0, 0, np.sin(np.pi / 4)))):
            d = fresh(model, quat=quat)
            d.ctrl[:] = 0.0
            d.ctrl[axis] = TAU_MAX
            for _ in range(100):
                mjo_step(model, d)
            w = np.asarray(d.qvel[3:6]).copy()
            expect = TAU_MAX / I_HAND[axis]
            print(f"  axis {'xyz'[axis]} ({label}): body w = {np.round(w, 5)}"
                  f"  expected w_{'xyz'[axis]} = +{expect:.5f}")

    # 3. Coast -----------------------------------------------------------------
    print("\n[3] zero-torque coast, 60 s, w0 = (0.10, 0.05, -0.08) rad/s body")
    d = fresh(model, w_body=(0.10, 0.05, -0.08))
    h0 = h_world(d, bid)
    e0 = 0.5 * float(np.sum(I_HAND * np.asarray(d.qvel[3:6]) ** 2))
    wmin, wmax = np.full(3, np.inf), np.full(3, -np.inf)
    for _ in range(6000):
        mjo_step(model, d)
        w = np.asarray(d.qvel[3:6])
        wmin, wmax = np.minimum(wmin, w), np.maximum(wmax, w)
    h1 = h_world(d, bid)
    e1 = 0.5 * float(np.sum(I_HAND * np.asarray(d.qvel[3:6]) ** 2))
    print(f"  |H| world: {np.linalg.norm(h0):.6f} -> {np.linalg.norm(h1):.6f} kg m^2/s;"
          f"  direction change {np.degrees(np.arccos(np.clip(h0 @ h1 / np.linalg.norm(h0) / np.linalg.norm(h1), -1, 1))):.3f} deg")
    print(f"  rotational KE: {e0:.6e} -> {e1:.6e} J (rel change {(e1 - e0) / e0:+.2e})")
    print(f"  body w ranges during coast: x [{wmin[0]:+.3f},{wmax[0]:+.3f}]"
          f" y [{wmin[1]:+.3f},{wmax[1]:+.3f}] z [{wmin[2]:+.3f},{wmax[2]:+.3f}]")

    # 4. Gravity gradient ------------------------------------------------------
    print("\n[4] gravity-gradient: start at rest, body rotated 45deg about world z, 60 s")
    d = fresh(model, quat=(np.cos(np.pi / 8), 0, 0, np.sin(np.pi / 8)))
    for _ in range(6000):
        mjo_step(model, d)
    w = np.asarray(d.qvel[3:6])
    tau_est = np.linalg.norm(I_HAND * w) / 60.0
    n = np.sqrt(MU / R0**3)
    tau_theory = 1.5 * n**2 * (I_HAND[1] - I_HAND[0])  # 3 n^2 dI sin(2*45)/2
    print(f"  |w| after 60 s: {np.linalg.norm(w):.2e} rad/s;"
          f" mean |torque| ~ {tau_est:.2e} N m (theory peak ~{tau_theory:.2e});"
          f" that is {tau_est / TAU_MAX:.1e} of the actuator limit")

    # 5. Damping baseline ------------------------------------------------------
    print("\n[5] saturated rate damping tau = clip(-K * I * w, +-0.01), 10 Hz, 30 s")
    rng = np.random.default_rng(0)
    for wmax0 in (0.1, 0.3):
        for k in (0.5, 1.0, 2.0):
            res = run_baseline(model, rng, wmax0, k, n_eps=50)
            print(f"  |w0_i|<={wmax0:.1f}, K={k:.1f}/s: success {res['succ']:.0%};"
                  f" settle median {res['settle_med']:.1f} s, max {res['settle_max']:.1f} s;"
                  f" final |w| max {res['final_max']:.1e}; effort {res['effort']:.3f} N m s;"
                  f" saturated {res['sat']:.0%} of control steps")


def run_baseline(model, rng, wmax0: float, k: float, n_eps: int) -> dict:
    succ, settle, finals, effort, sat = [], [], [], [], []
    t0 = time.perf_counter()
    for _ in range(n_eps):
        w0 = rng.uniform(-wmax0, wmax0, 3)
        q = rng.normal(size=4)
        d = fresh(model, w_body=w0, quat=q / np.linalg.norm(q))
        n_ctrl = int(30.0 / (DT * DECIM))
        speeds, eff, nsat = [], 0.0, 0
        for _ in range(n_ctrl):
            w = np.asarray(d.qvel[3:6])
            raw = -k * I_HAND * w
            tau = np.clip(raw, -TAU_MAX, TAU_MAX)
            nsat += int(np.any(np.abs(raw) > TAU_MAX))
            d.ctrl[:] = tau
            for _ in range(DECIM):
                mjo_step(model, d)
            eff += float(np.sum(np.abs(tau))) * DT * DECIM
            speeds.append(float(np.linalg.norm(d.qvel[3:6])))
        sp = np.asarray(speeds)
        below = sp < 0.01
        last5 = below[-int(5.0 / (DT * DECIM)):]
        succ.append(bool(last5.all()))
        # settle time = start of the final unbroken run below threshold
        idx = len(below)
        while idx > 0 and below[idx - 1]:
            idx -= 1
        settle.append(idx * DT * DECIM if below[-1] else np.inf)
        finals.append(sp[-1])
        effort.append(eff)
        sat.append(nsat / n_ctrl)
    st = np.asarray(settle)
    finite = st[np.isfinite(st)]
    return {
        "succ": float(np.mean(succ)),
        "settle_med": float(np.median(finite)) if finite.size else np.inf,
        "settle_max": float(finite.max()) if finite.size else np.inf,
        "final_max": float(np.max(finals)),
        "effort": float(np.mean(effort)),
        "sat": float(np.mean(sat)),
        "wall": time.perf_counter() - t0,
    }


if __name__ == "__main__":
    main()
