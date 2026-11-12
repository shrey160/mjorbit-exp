"""3E step E3: whole-mission Monte Carlo (SPEC_3 section 13).

Each case is one mission:
  A  3A approach (GPU env, 9-state nav filter, uncertainty preset u1) from 20-50 m to the
     hold point 8 m out on the client's spin axis (the face normal if not spinning)
  B  fly-in from the hold point (the 3A arrival error, in the client frame, is handed
     over) -> synchronised grasp -> 3C brakes, detumble, identification -> 3D move 5-15 m
     and rotate 30-120 deg; E2 monitors on throughout (pre-contact aborts, grip re-clamp)
Per case (seeded): client spin uniform 0-2 deg/s about a random body axis, client yaw,
random client mass properties (3C), start state, and with probability fault_rate one
injected fault: client_kick | thruster_weak | thruster_stuck | spin_axis_err (during
fly-in / reach / approach) or grip_loose (jaw brakes 0.05-0.3 mm loose at 3C).

Mission success (criteria fixed before the runs): 3A arrives; grasp captured with no
unintended contact; 3C keeps the client; 3D ends within 0.2 m, 2 mm/s, 0.5 deg and
0.01 deg/s of the command with both pads on the bar. Safe outcomes: success, a
pre-contact abort that reached its safe state, or a grip alarm whose re-clamp kept the
client. Unsafe: any collision, an abort that did not reach safety, a lost client.

    stage A:  python evaluate_e3.py A --n N --split S --out handoff.json
    stage B:  python evaluate_e3.py B --handoff handoff.json --workers W --out results.json
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "p2_approach"))

import numpy as np  # noqa: E402

SPLITS = {"tune": 5000, "validation": 6000, "test": 7000}
FAULTS = ["client_kick", "thruster_weak", "thruster_stuck", "spin_axis_err", "grip_loose"]


def case_draws(seed, fault_rate):
    r = np.random.default_rng([seed, 13])
    spin = float(r.uniform(0.0, 2.0))
    axis = int(r.integers(3))
    yaw = float(r.uniform(-np.pi, np.pi))
    fault = FAULTS[int(r.integers(len(FAULTS)))] if r.uniform() < fault_rate else ""
    loose = float(r.uniform(0.05, 0.3))
    return dict(spin=spin, axis=axis, yaw=yaw, fault=fault, loose=loose)


# ------------------------------------------------------------------------- stage A
def stage_a(args):
    import mjorbit  # noqa: F401
    from approach_env import UNCERTAINTY, sample_starts
    from capture_env import CaptureEnv, CaptureEnvCfg
    from capture_guidance import CaptureGuidance
    from nav_filter import NavController
    base = SPLITS[args.split] * 1000
    n = args.n
    cases = [case_draws(base + i, args.fault_rate) for i in range(n)]
    # servicer bounding radius from the stage-B scene's robot description (3A flies
    # the 150 kg point-mass servicer of p3a_scene.xml with a keep-out sphere; the arm is stowed)
    from robot_spec import load_spec
    sp = load_spec(Path(__file__).with_name(args.scene))
    cfg = CaptureEnvCfg(num_envs=n, auto_reset=False, device=args.device, yaw_mode="random",
                        client_rot="lvlh", goal_mode="hold", episode_length_s=args.episode_s,
                        **UNCERTAINTY[args.unc])
    if args.scene != "p3b_scene.xml":  # p3b keeps the frozen 3A settings exactly
        cfg.servicer_radius = sp.servicer_radius
    print(f"stage A: scene {args.scene}, mass {cfg.mass:.1f} kg, servicer radius {cfg.servicer_radius} m")
    env = CaptureEnv(cfg)
    env.set_spin([c["spin"] for c in cases], [c["axis"] for c in cases])
    rng = np.random.default_rng(base)
    p0, v0 = sample_starts(rng, n, cfg.r_min, cfg.r_max, cfg.cone_deg, cfg.v0_max)
    env.set_initial_conditions(p0, v0, unc_seed=base + 5000, yaw=np.array([c["yaw"] for c in cases]))
    holder = {}

    def make(view):
        holder["g"] = CaptureGuidance(view)
        return holder["g"]
    ctrl = NavController(env, make, mode="dv")
    keys = ("success", "collided", "dv", "min_dist", "final_err", "final_speed", "settle")
    res = {k: np.full(n, np.nan) for k in keys}
    dp_b = np.full((n, 3), np.nan)
    dv_b = np.full((n, 3), np.nan)
    done = np.zeros(n, bool)
    t0 = time.time()
    for _ in range(env.max_episode_length):
        env.step_world_dv(ctrl())
        info = env.last_done_info
        if info is not None:
            new = ~done[info["idx"]]
            ids = info["idx"][new]
            if ids.size:
                _, _, p, v = env.lvlh_state()
                geo = env.client_geometry()
                e = p - geo["g"]
                ev = v - geo["gd"]
                dp_b[ids] = np.einsum("nji,nj->ni", geo["R"][ids], e[ids])
                dv_b[ids] = np.einsum("nji,nj->ni", geo["R"][ids], ev[ids])
                for k in keys:
                    res[k][ids] = info[k][new]
                done[ids] = True
        if done.all():
            break
    out = []
    for i, c in enumerate(cases):
        out.append(dict(seed=base + i, **c, a_success=bool(res["success"][i] == 1),
                        a_collided=bool(res["collided"][i] == 1), a_dv=float(res["dv"][i]),
                        a_min_clear=float(res["min_dist"][i]), a_err_m=float(res["final_err"][i]),
                        a_speed=float(res["final_speed"][i]), a_settle_s=float(res["settle"][i]),
                        dp_b=dp_b[i].tolist(), dv_b=dv_b[i].tolist()))
    ok = sum(o["a_success"] for o in out)
    print(f"stage A: {ok}/{n} arrived at the hold point, collisions {sum(o['a_collided'] for o in out)}, "
          f"{time.time() - t0:.0f} s")
    Path(args.out).write_text(json.dumps(dict(split=args.split, unc=args.unc, scene=args.scene, cases=out)))


# ------------------------------------------------------------------------- stage B
def _mission(c, mon_act=True, scene="p3b_scene.xml"):
    import os
    from maneuver3d import Man3DCfg, Stack3D
    from stack_sim import GripAbort, StackCfg
    out = dict(seed=c["seed"], spin=c["spin"], axis=c["axis"], fault=c["fault"])
    gfault = c["fault"] if c["fault"] in ("client_kick", "thruster_weak", "thruster_stuck",
                                          "spin_axis_err") else ""
    scfg = StackCfg(spin_deg_s=c["spin"], spin_axis=c["axis"], impose_tumble=False, grip_mon=True,
                    grip_response="reclamp", grip_mon_act=mon_act,
                    grip_loose_mm=c["loose"] if c["fault"] == "grip_loose" else 0.0, scene=scene)
    gk = dict(flyin=True, mon=True, mon_act=mon_act, fault_kind=gfault)  # fly-in, monitors, pre-contact fault
    for k in range(10):
        try:
            sr = Stack3D(c["seed"], scfg, Man3DCfg(), grasp_kw=gk)
            break
        except ValueError as e:
            if "parse" not in str(e).lower() and "empty" not in str(e).lower():
                raise
            time.sleep(0.3 + 0.3 * k + 0.01 * (os.getpid() % 10))
    sr.grasp_reset_kw = dict(hold_dp_b=c["dp_b"], hold_dv_b=c["dv_b"])
    t0 = time.time()
    try:
        r = sr.run3d()
        out["stage"] = "3D" if "final_pos_err_m" in r else ("3C" if r.get("grasped") else "grasp")
    except GripAbort as e:  # a grip alarm during 3D: re-clamp and stop
        r = {}
        sr.safe_coast()
        r.update(grip_abort=str(e), grip_abort_phase="3D", **sr.grip_outcome())
        g = getattr(sr, "grasp_out", {})
        r.update({f"g_{k}": g.get(k) for k in ("outcome", "aborted", "collision", "min_clear", "t_fault",
                                                "detect_reason", "fault_phase")})
        r["grasped"] = True
        out["stage"] = "3D"
    out["wall_s"] = time.time() - t0
    keep = ("grasped", "g_outcome", "g_aborted", "g_abort_reason", "g_abort_phase", "g_safe_reached",
            "g_t_to_safe", "g_collision", "g_hit_force_max", "g_min_clear", "g_detect_reason", "g_t_detect",
            "g_t_fault", "g_fault_phase", "g_fault_mag", "g_flyin_s", "g_flyin_min_clear", "g_flyin_end_err_m",
            "g_bar_shift_mm", "g_flyin_k_min", "gm_pad_drops", "grip_abort", "grip_abort_phase", "retained", "safed", "grip_detect",
            "mass_err_pct", "I_err_rel_pct", "d_err_mm", "detumble_s", "success", "final_pos_err_m",
            "final_vel_mm_s", "final_att_err_deg", "final_rate_deg_s", "dv_used", "rcs_prop_g",
            "sat_s", "pads_lost_3d", "t_3d_s", "dr_m", "att_cmd_deg")
    for k in keep:
        if k in r:
            v = r[k]
            out[k] = float(v) if isinstance(v, (np.floating,)) else (bool(v) if isinstance(v, np.bool_) else v)
    return out


def classify(c, b):
    """Mission outcome from the stage A row c and stage B row b (None if not run)."""
    if not c["a_success"]:
        return "A_collision" if c["a_collided"] else "A_fail"
    if b is None:
        return "not_run"
    if b.get("g_collision"):
        return "collision"
    if b.get("g_aborted"):
        return "safe_abort" if b.get("g_safe_reached") else "abort_unsafe"
    if not b.get("grasped"):
        return "grasp_fail"
    if b.get("grip_abort"):
        return "grip_reclamped" if b.get("retained") else "client_lost"
    if b.get("success"):
        return "success"
    return "3D_fail"


SAFE = {"success", "safe_abort", "grip_reclamped"}


def wilson(k, n, z=1.96):
    if n == 0:
        return (float("nan"), float("nan"))
    p = k / n
    den = 1 + z * z / n
    c = (p + z * z / (2 * n)) / den
    h = z * np.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return (round(c - h, 4), round(c + h, 4))


def stage_b(args):
    from multiprocessing import Pool
    h = json.loads(Path(args.handoff).read_text())
    cases = h["cases"]
    if h.get("scene", "p3b_scene.xml") != args.scene:
        print(f"note: hand-off made for {h.get('scene', 'p3b_scene.xml')}, stage B runs {args.scene}")
    jobs = [c for c in cases if c["a_success"] and (not args.seeds or c["seed"] in args.seeds)]
    import functools
    t0 = time.time()
    # checkpoint: one JSON line per finished mission, so a crash/power loss loses nothing
    ck = Path(args.out + ".partial.jsonl")
    rows = [json.loads(ln) for ln in ck.read_text().splitlines() if ln.strip()] if ck.exists() else []
    done = {r["seed"] for r in rows}
    todo = [c for c in jobs if c["seed"] not in done]
    print(f"stage B: {len(jobs)} missions, {len(done)} from checkpoint, {len(todo)} to run", flush=True)
    with Pool(args.workers) as pool, ck.open("a") as fh:
        for i, r in enumerate(pool.imap_unordered(functools.partial(_mission, mon_act=not args.no_act, scene=args.scene),
                                                  todo, chunksize=1), 1):
            fh.write(json.dumps(r, default=str) + "\n")
            fh.flush()
            rows.append(r)
            if i % 25 == 0 or i == len(todo):
                print(f"  {len(rows)}/{len(jobs)} done, {(time.time() - t0) / 60:.1f} min", flush=True)
    byseed = {r["seed"]: r for r in rows}
    missions = []
    if args.seeds:
        cases = [c for c in cases if c["seed"] in args.seeds]
    for c in cases:
        b = byseed.get(c["seed"])
        m = dict(c)
        m.pop("dp_b", None), m.pop("dv_b", None)
        if b:
            m.update(b)
        m["outcome"] = classify(c, b)
        missions.append(m)
    n = len(missions)
    oc = Counter(m["outcome"] for m in missions)
    nom = [m for m in missions if not m["fault"]]
    flt = [m for m in missions if m["fault"]]
    ok = lambda ms: sum(m["outcome"] == "success" for m in ms)  # noqa: E731
    safe = lambda ms: sum(m["outcome"] in SAFE for m in ms)  # noqa: E731
    f = lambda k, ms: np.array([m[k] for m in ms if isinstance(m.get(k), (int, float)) and m.get(k) == m.get(k)], float)  # noqa: E731
    succ = [m for m in missions if m["outcome"] == "success"]
    summ = dict(
        split=h["split"], scene=args.scene, n=n, wall_min=round((time.time() - t0) / 60, 1), outcomes=dict(oc),
        success=ok(missions) / n, success_ci95=wilson(ok(missions), n),
        safe=safe(missions) / n, safe_ci95=wilson(safe(missions), n),
        unsafe=n - safe(missions) - sum(m["outcome"] in ("A_fail", "grasp_fail", "3D_fail", "not_run") for m in missions),
        nominal=dict(n=len(nom), success=ok(nom), success_ci95=wilson(ok(nom), len(nom)),
                     outcomes=dict(Counter(m["outcome"] for m in nom))),
        faulted=dict(n=len(flt), success=ok(flt), safe=safe(flt), safe_ci95=wilson(safe(flt), len(flt)),
                     by_fault={k: dict(Counter(m["outcome"] for m in flt if m["fault"] == k)) for k in FAULTS}),
        a_dv_median=float(np.median(f("a_dv", missions))) if missions else None,
        a_settle_s_median=float(np.median(f("a_settle_s", [m for m in missions if m["a_success"]]))),
        final_pos_err_m_max=float(f("final_pos_err_m", succ).max()) if succ else None,
        final_att_err_deg_max=float(f("final_att_err_deg", succ).max()) if succ else None,
        mass_err_pct_abs_max=float(np.abs(f("mass_err_pct", succ)).max()) if succ else None,
        flyin_min_clear_min=float(f("g_flyin_min_clear", missions).min()) if f("g_flyin_min_clear", missions).size else None,
        rcs_prop_g_median=float(np.median(f("rcs_prop_g", succ))) if succ else None,
        mission_sim_wall_s_median=float(np.median(f("wall_s", missions))) if f("wall_s", missions).size else None,
    )
    for k, v in summ.items():
        print(f"{k:26s} {v}")
    bad = [m for m in missions if m["outcome"] not in SAFE and m["outcome"] != "A_fail"]
    for m in bad[:30]:
        print("  ", {k: m.get(k) for k in ("seed", "outcome", "fault", "spin", "axis", "g_outcome",
                                            "g_abort_reason", "grip_abort", "success", "final_pos_err_m",
                                            "final_att_err_deg", "final_rate_deg_s", "final_vel_mm_s")})
    Path(args.out).write_text(json.dumps(dict(summary=summ, missions=missions), indent=1, default=str))
    print("wrote", args.out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("stage", choices=["A", "B"])
    ap.add_argument("--n", type=int, default=100)
    ap.add_argument("--split", choices=list(SPLITS), default="tune")
    ap.add_argument("--fault-rate", type=float, default=0.2)
    ap.add_argument("--unc", default="u1")
    ap.add_argument("--episode-s", type=float, default=3600.0)
    ap.add_argument("--device", default="cuda")
    ap.add_argument("--handoff", default=None)
    ap.add_argument("--workers", type=int, default=20)
    ap.add_argument("--no-act", action="store_true", help="monitors record only (no aborts)")
    ap.add_argument("--seeds", type=int, nargs="*", default=[])
    ap.add_argument("--scene", default="p3b_scene.xml", help="servicer scene (e.g. p3e_espa_scene.xml)")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    (stage_a if args.stage == "A" else stage_b)(args)


if __name__ == "__main__":
    main()
