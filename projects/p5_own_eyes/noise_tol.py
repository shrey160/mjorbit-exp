"""P5 step 1 (14 Nov): how much error and delay in the client state can the servicer body tolerate?

Today the bus station keeping, the fly-in and the E2 monitors read the simulator's true client
pose, velocity and spin. Before building an estimator, this script corrupts that client state
and measures what happens, so the estimator has a number to meet.

  - Fly-in from the 8 m hold point plus the classical grasp (GraspSim, flyin=True, monitors on).
  - The ARM stays on the true bar pose: only the effect on the BUS (and, with --mon-noisy, on the
    monitors) is measured.
  - Corrupted client state, world frame, every control step (100 Hz):
        position + e_p,  attitude rotated by e_a,  velocity + e_v,  angular rate + e_w
    each error a first-order Gauss-Markov process per axis (1-sigma given, correlation time
    --tau), as a filter's output error would be. --delay: the client state is `delay` seconds
    old and not propagated (worst case; a filter would propagate the rotation).
  - grasp_sim.py is not edited: TolSim overrides bus_ctrl/_freeze_refs and swaps the client
    state in the kinematic copy for the duration of the call.

Conditions run one after another in one worker pool:
  python noise_tol.py --conds base:0,0,0,0,0 p30:30,0,0,0,0 --n 25 --workers 8 --out p5_tol.json
  each condition is name:pos_mm,att_deg,vel_mm_s,rate_deg_s,delay_s
"""
from __future__ import annotations

import argparse
import contextlib
import json
import sys
import time
from collections import Counter, deque
from multiprocessing import Pool
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "p3_capture"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "p2_approach"))
import safety  # noqa: E402
from evaluate_e3 import case_draws  # noqa: E402
from grasp_sim import GraspCfg, GraspSim, _rotvec  # noqa: E402

TUNE = 5_000_000


class Corrupt:
    """Client state as an estimator would report it: delayed truth plus correlated errors."""

    def __init__(self, sp, sa, sv, sw, delay, tau, dt, rng):
        self.s = np.repeat([sp, sa, sv, sw], 3)
        self.a = float(np.exp(-dt / tau))
        self.rng = rng
        self.e = rng.normal(size=12) * self.s
        self.buf = deque(maxlen=int(round(delay / dt)) + 1)
        self.out = None

    def step(self, pc, Rc, vc, wc):
        self.e = self.a * self.e + np.sqrt(1 - self.a**2) * self.s * self.rng.normal(size=12)
        self.buf.append((pc.copy(), Rc.copy(), vc.copy(), wc.copy()))
        pc, Rc, vc, wc = self.buf[0]
        e = self.e
        self.out = (pc + e[0:3], Rotation.from_rotvec(e[3:6]).as_matrix() @ Rc, vc + e[6:9], wc + e[9:12])


class TolSim(GraspSim):
    cond = None  # (pos m, att rad, vel m/s, rate rad/s, delay s) or None = truth
    tau = 2.0
    mon_noisy = False
    noise_seed = 0
    est = None

    def reset(self, rng=None, **kw):
        self.est = None
        super().reset(rng, **kw)
        if self.cond is not None and any(self.cond):
            self.est = Corrupt(*self.cond, self.tau, self.cfg.ctrl_dt, np.random.default_rng([self.noise_seed, 77]))
        self._ph = "flyin"
        self.m = {g: dict(n=0, e2=0.0, emax=0.0, amax=0.0, dv=0.0, sat=0, v2=0.0) for g in ("flyin", "prox")}
        self.step_hook = self._hook

    def _hook(self, loc):
        self._ph = loc["phase"]

    def _truth(self):
        kd, d = self.kd, self.data
        Rc = kd.xmat[self.bid_target].reshape(3, 3).copy()
        return kd.xpos[self.bid_target].copy(), Rc, d.qvel[14:17].copy(), Rc @ d.qvel[17:20]

    def sync(self):
        super().sync()
        if self.est is not None:
            self.est.step(*self._truth())

    @contextlib.contextmanager
    def swapped(self):
        """The client state in the kinematic copy (and its velocity) replaced by the estimate."""
        if self.est is None or self.est.out is None:
            yield
            return
        kd, d, b = self.kd, self.data, self.bid_target
        keep = (kd.xpos[b].copy(), kd.xmat[b].copy(), np.array(d.qvel[14:20]))
        pe, Re, ve, we = self.est.out
        kd.xpos[b], kd.xmat[b] = pe, Re.reshape(-1)
        d.qvel[14:17], d.qvel[17:20] = ve, Re.T @ we
        try:
            yield
        finally:
            kd.xpos[b], kd.xmat[b] = keep[0], keep[1]
            d.qvel[14:20] = keep[2]

    def _freeze_refs(self):
        super()._freeze_refs()  # the arm's hold pose stays relative to the true client
        if self.est is not None:  # the bus freezes relative to the client as it sees it
            kd = self.kd
            pe, Re, _, _ = self.est.out
            self.rel_pos_ref = Re.T @ (kd.xpos[self.bid_chaser] - pe)
            self.rel_att_ref = Re.T @ kd.xmat[self.bid_chaser].reshape(3, 3)
            self._frozen_true = self._true_rel()

    def _true_rel(self):
        kd = self.kd
        pc, Rc, _, _ = self._truth()
        return Rc.T @ (kd.xpos[self.bid_chaser] - pc), Rc.T @ kd.xmat[self.bid_chaser].reshape(3, 3)

    def bus_ctrl(self):
        with self.swapped():
            F, T = super().bus_ctrl()
        # true tracking error: against the reference built from the true client state
        kd, d = self.kd, self.data
        pc, Rc, vc, wc = self._truth()
        ps, vs = kd.xpos[self.bid_chaser], d.qvel[0:3]
        Rs = kd.xmat[self.bid_chaser].reshape(3, 3)
        fly = getattr(self, "flyin_path", None) is not None and self._flyin_t is not None
        if fly:
            p_ref, v_ref, _, R_ref, _ = self.flyin_path.reference(self._flyin_t, pc, vc, Rc, wc, self.R_rel0,
                                                                   k=self._flyin_k)
        else:
            rp, ra = self.rel_pos_ref, self.rel_att_ref
            if self._ph in ("close", "hold") and self.est is not None:
                rp, ra = self._frozen_true  # the bus should hold where it was at the close command
            p_ref = pc + Rc @ rp
            v_ref = vc + np.cross(wc, p_ref - pc)
            R_ref = Rc @ ra
        m = self.m["flyin" if fly else "prox"]
        e = float(np.linalg.norm(ps - p_ref))
        m["n"] += 1
        m["e2"] += e * e
        m["emax"] = max(m["emax"], e)
        m["amax"] = max(m["amax"], float(np.degrees(np.linalg.norm(_rotvec(R_ref @ Rs.T)))))
        m["dv"] += float(np.abs(F).sum()) * self.cfg.ctrl_dt / self.m_serv
        m["sat"] += int(np.any(np.abs(F) >= self.f_max - 1e-4))
        m["v2"] += float(np.sum((vs - v_ref) ** 2))
        return F, T


_orig_check = safety.Monitor.check


def _check(self, sim, *a, **k):
    if getattr(sim, "mon_noisy", False):
        with sim.swapped():
            return _orig_check(self, sim, *a, **k)
    return _orig_check(self, sim, *a, **k)


safety.Monitor.check = _check

_W = {}
KEEP = ("success", "grasped", "outcome", "aborted", "abort_reason", "abort_phase", "collision", "push_dv",
        "min_clear", "flyin_min_clear", "flyin_k_min", "flyin_s", "flyin_end_err_m", "detect_reason", "phase_end")


def _init(scene, tau, mon_noisy):
    import os
    for k in range(20):
        try:
            sim = TolSim(GraspCfg(scene=scene, flyin=True, hold_s=5.0, mon=True, mon_act=True))
            break
        except ValueError as e:
            if "parse" not in str(e).lower() and "empty" not in str(e).lower():
                raise
            time.sleep(0.3 + 0.3 * k + 0.05 * (os.getpid() % 10))
    sim.tau, sim.mon_noisy = tau, mon_noisy
    _W["sim"] = sim


def _episode(job):
    name, cond, seed = job
    sim = _W["sim"]
    c = case_draws(seed, 0.0)
    sp, sa, sv, sw, dl = cond
    sim.cond = (sp * 1e-3, np.radians(sa), sv * 1e-3, np.radians(sw), dl)
    sim.noise_seed = seed
    t0 = time.time()
    r = sim.run(np.random.default_rng(seed), spin_deg_s=c["spin"], spin_axis=c["axis"])
    out = dict(cond=name, seed=seed, spin=c["spin"], axis=c["axis"], wall_s=round(time.time() - t0, 1),
               **{k: r.get(k) for k in KEEP})
    for g, m in sim.m.items():
        n = max(m["n"], 1)
        out.update({f"{g}_err_rms_mm": 1000 * np.sqrt(m["e2"] / n), f"{g}_err_max_mm": 1000 * m["emax"],
                    f"{g}_att_max_deg": m["amax"], f"{g}_dv_mm_s": 1000 * m["dv"], f"{g}_sat_frac": m["sat"] / n,
                    f"{g}_vrms_mm_s": 1000 * np.sqrt(m["v2"] / n), f"{g}_s": m["n"] * sim.cfg.ctrl_dt})
    return out


def summarise(name, cond, rows):
    n = len(rows)
    med = lambda k: float(np.median([r[k] for r in rows]))  # noqa: E731
    mx = lambda k: float(np.max([r[k] for r in rows]))  # noqa: E731
    ok = sum(bool(r["success"]) and not r["collision"] and r["push_dv"] <= 0.002 for r in rows)
    return dict(cond=name, pos_mm=cond[0], att_deg=cond[1], vel_mm_s=cond[2], rate_deg_s=cond[3], delay_s=cond[4],
                n=n, success=ok, outcomes=dict(Counter(r["outcome"] for r in rows)),
                aborts=dict(Counter(f'{r["abort_reason"]}@{r["abort_phase"]}' for r in rows if r["aborted"])),
                collisions=sum(bool(r["collision"]) for r in rows),
                push_max_mm_s=1000 * mx("push_dv"), min_clear_m=float(np.min([r["min_clear"] for r in rows])),
                flyin_err_max_mm=dict(median=med("flyin_err_max_mm"), max=mx("flyin_err_max_mm")),
                prox_err_rms_mm=dict(median=med("prox_err_rms_mm"), max=mx("prox_err_rms_mm")),
                prox_err_max_mm=dict(median=med("prox_err_max_mm"), max=mx("prox_err_max_mm")),
                prox_vrms_mm_s=med("prox_vrms_mm_s"), prox_att_max_deg=mx("prox_att_max_deg"),
                flyin_dv_mm_s=med("flyin_dv_mm_s"), prox_dv_mm_s=med("prox_dv_mm_s"),
                flyin_sat_frac=med("flyin_sat_frac"), prox_sat_frac=med("prox_sat_frac"), flyin_s=med("flyin_s"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--conds", nargs="+", required=True, help="name:pos_mm,att_deg,vel_mm_s,rate_deg_s,delay_s")
    ap.add_argument("--n", type=int, default=25)
    ap.add_argument("--start", type=int, default=0)
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--tau", type=float, default=2.0, help="error correlation time [s]")
    ap.add_argument("--mon-noisy", action="store_true", help="the E2 monitors also read the corrupted state")
    ap.add_argument("--scene", default="p3e_espa_arm2_scene.xml")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    conds = []
    for s in a.conds:
        name, v = s.split(":")
        conds.append((name, tuple(float(x) for x in v.split(","))))
    jobs = [(name, cond, TUNE + a.start + i) for name, cond in conds for i in range(a.n)]
    ck = Path(a.out + ".partial.jsonl")
    rows = [json.loads(ln) for ln in ck.read_text().splitlines() if ln.strip()] if ck.exists() else []
    done = {(r["cond"], r["seed"]) for r in rows}
    todo = [j for j in jobs if (j[0], j[2]) not in done]
    print(f"{len(jobs)} episodes, {len(done)} from checkpoint, {len(todo)} to run", flush=True)
    t0 = time.time()
    with Pool(a.workers, initializer=_init, initargs=(a.scene, a.tau, a.mon_noisy)) as pool, ck.open("a") as fh:
        for i, r in enumerate(pool.imap(_episode, todo, chunksize=1), 1):
            fh.write(json.dumps(r, default=float) + "\n")
            fh.flush()
            rows.append(r)
            if i % 25 == 0 or i == len(todo):
                print(f"  {i}/{len(todo)}, {(time.time() - t0) / 60:.1f} min", flush=True)
    summ = [summarise(name, cond, sorted((r for r in rows if r["cond"] == name), key=lambda r: r["seed"]))
            for name, cond in conds]
    Path(a.out).write_text(json.dumps(dict(tau=a.tau, mon_noisy=a.mon_noisy, scene=a.scene, n=a.n, summary=summ,
                                           episodes=rows), indent=1, default=float))
    print(f"{'cond':10s} {'ok':>5s} {'abort':>5s} {'coll':>4s} {'fly max':>8s} {'prox rms':>8s} {'prox max':>8s} "
          f"{'v rms':>6s} {'push':>5s} {'dv fly':>6s} {'dv prox':>7s} {'sat':>5s}  aborts")
    for s in summ:
        print(f"{s['cond']:10s} {s['success']:>2d}/{s['n']:<2d} {sum(s['aborts'].values()):>5d} {s['collisions']:>4d} "
              f"{s['flyin_err_max_mm']['max']:>8.0f} {s['prox_err_rms_mm']['median']:>8.1f} "
              f"{s['prox_err_max_mm']['max']:>8.0f} {s['prox_vrms_mm_s']:>6.2f} {s['push_max_mm_s']:>5.2f} "
              f"{s['flyin_dv_mm_s']:>6.0f} {s['prox_dv_mm_s']:>7.0f} {s['prox_sat_frac']:>5.2f}  {s['aborts']}")
    print("wrote", a.out)


if __name__ == "__main__":
    main()
