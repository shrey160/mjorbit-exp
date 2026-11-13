"""Demo (13 Nov): record the far approach (E3 stage A, SPEC_3 section 13) for ONE case, for the
full-mission replay. Same environment, guidance and nav filter as evaluate_e3.py stage A
(CaptureEnv, hold-point goal, uncertainty preset u1), one world; the start is scaled to a
chosen range. Stops once the servicer has sat at the hold point for `dwell` epochs (stage A
itself flies the whole episode). Writes the path and the hand-off error for export_demo.py:

  t [s], p (servicer relative to the client centre, LVLH) [m], R (client body -> LVLH),
  g (hold point relative to the client centre, LVLH), dp_b / dv_b (arrival error, client frame)

  bash tools/rlrun.sh python ../projects/p3_capture/demo_stage_a.py <seed> <range_m> <out.json> [scene]
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "p2_approach"))

import numpy as np  # noqa: E402

import mjorbit  # noqa: E402,F401
from approach_env import UNCERTAINTY, sample_starts  # noqa: E402
from capture_env import CaptureEnv, CaptureEnvCfg  # noqa: E402
from capture_guidance import CaptureGuidance  # noqa: E402
from evaluate_e3 import case_draws  # noqa: E402
from nav_filter import NavController  # noqa: E402
from robot_spec import load_spec  # noqa: E402

seed, rng_m, out = int(sys.argv[1]), float(sys.argv[2]), sys.argv[3]
scene = sys.argv[4] if len(sys.argv) > 4 else "p3e_espa_arm2_scene.xml"
c = case_draws(seed, 0.0)  # nominal mission: no fault
sp = load_spec(Path(__file__).with_name(scene))
cfg = CaptureEnvCfg(num_envs=2,  # the env needs at least 2 worlds: the case is flown twice, world 0 is recorded
                     auto_reset=False, device="cuda", yaw_mode="random", client_rot="lvlh",
                    goal_mode="hold", episode_length_s=3600.0, **UNCERTAINTY["u1"])
cfg.servicer_radius = sp.servicer_radius
env = CaptureEnv(cfg)
env.set_spin([c["spin"]] * 2, [c["axis"]] * 2)
rng = np.random.default_rng(seed)
p0, v0 = sample_starts(rng, 1, cfg.r_min, cfg.r_max, cfg.cone_deg, cfg.v0_max)
p0 = np.repeat(p0 / np.linalg.norm(p0, axis=1, keepdims=True) * rng_m, 2, axis=0)
v0 = np.repeat(v0, 2, axis=0)
env.set_initial_conditions(p0, v0, unc_seed=seed + 5000, yaw=np.array([c["yaw"]] * 2))
holder = {}


def make(view):
    holder["g"] = CaptureGuidance(view)
    return holder["g"]


ctrl = NavController(env, make, mode="dv")
dt = cfg.control_dt
rows, ok, dwell = [], 0, 3


def snap(t):
    _, _, p, v = env.lvlh_state()
    geo = env.client_geometry()
    e, ev = p[0] - geo["g"][0], v[0] - geo["gd"][0]
    rows.append(dict(t=t, p=(p[0] - geo["c"][0]).tolist(), R=geo["R"][0].tolist(), g=(geo["g"][0] - geo["c"][0]).tolist(),
                     err=float(np.linalg.norm(e)), speed=float(np.linalg.norm(ev)),
                     dp_b=(geo["R"][0].T @ e).tolist(), dv_b=(geo["R"][0].T @ ev).tolist()))
    return rows[-1]


snap(0.0)
for k in range(env.max_episode_length):
    env.step_world_dv(ctrl())
    r = snap((k + 1) * dt)
    ok = ok + 1 if (r["err"] < 0.05 and r["speed"] < 0.003) else 0
    if ok >= dwell or env.last_done_info is not None:
        break
last = rows[-1]
dist = [float(np.linalg.norm(r["p"])) for r in rows]
print(f"seed {seed}: spin {c['spin']:.2f} deg/s axis {c['axis']}, start {dist[0]:.1f} m, {len(rows)} frames, dt {dt} s, "
      f"arrived t {last['t']:.0f} s, err {last['err'] * 100:.1f} cm, speed {last['speed'] * 1000:.2f} mm/s, "
      f"closest {min(dist):.2f} m, settled {ok >= dwell}")
Path(out).write_text(json.dumps(dict(seed=seed, case=c, scene=scene, dt=dt, range_m=rng_m, arrived=bool(ok >= dwell),
                                     dp_b=last["dp_b"], dv_b=last["dv_b"], rows=rows)))
