"""P5.1: the relative-motion filter (rel_filter.RelFilter) on synthetic pose measurements.

Measurements stand in for the future client-pose network: the true client pose relative to the
servicer, every 0.1 s, `--latency` old, with an error per axis of
    position  mp [mm] x (range / 2.7 m)     attitude  ma [deg]
of which a fraction --slow (variance) is a slow drift (correlation time 5 s; a network's error
changes with the view, not from frame to frame) and the rest is white.

  --loop open    the bus flies on the truth; the filter runs beside it and its error is logged
  --loop closed  the bus station keeping and the fly-in read the filter. The servicer first
                 waits --preroll seconds at the hold point, then plans the fly-in from the
                 ESTIMATED client attitude and spin axis (no true spin axis). The arm and the
                 E2 monitors stay on the truth (monitors are P5.2).

python filt_eval.py --loop closed --conds m10:10,0.5 m20:20,1 --n 25 --workers 8 --out p5_filt.json
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from collections import Counter
from multiprocessing import Pool
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parent))
from noise_tol import KEEP, TUNE, TolSim  # noqa: E402  (also sets the p3_capture path)
from evaluate_e3 import case_draws  # noqa: E402
from grasp_sim import GraspCfg, GraspSim  # noqa: E402
from rel_filter import RelFilter  # noqa: E402
import safety  # noqa: E402
from safety import CLIENT_RADIUS, box_clearance  # noqa: E402

GROUPS = ("preroll", "flyin", "prox", "post")


class FiltEst:
    def __init__(self, sim, mp, ma, slow, latency, rng, every=10, tau_b=5.0, r_ref=2.7, **fkw):
        self.dt = sim.cfg.ctrl_dt
        self.f = RelFilter(self.dt, **fkw)
        self.mp, self.ma, self.slow, self.rng = mp, ma, slow, rng
        self.every, self.lat, self.r_ref = every, int(round(latency / self.dt)), r_ref
        self.ab = float(np.exp(-every * self.dt / tau_b))
        self.b = rng.normal(size=6)  # slow error, unit variance
        self.k, self.q, self.out = 0, [], None
        self.err = {g: dict(n=0, p2=0.0, v2=0.0, a2=0.0, w2=0.0, pm=0.0, vm=0.0, am=0.0, wm=0.0) for g in GROUPS}
        self.nis = []
        self.last_upd = 0  # control step of the last measurement update

    def age(self):
        """Age [s] of the newest measurement the filter has used."""
        return (self.k - self.last_upd) * self.dt + self.lat * self.dt

    def _measure(self, rho, Rc):
        sp = self.mp * max(np.linalg.norm(rho) / self.r_ref, 1.0)
        self.b = self.ab * self.b + np.sqrt(1 - self.ab**2) * self.rng.normal(size=6)
        e = np.sqrt(self.slow) * self.b + np.sqrt(1 - self.slow) * self.rng.normal(size=6)
        return rho + sp * e[0:3], Rotation.from_rotvec(self.ma * e[3:6]).as_matrix() @ Rc, sp

    def step(self, sim, group, drop=False):
        kd, d = sim.kd, sim.data
        pc, Rc, vc, wc = sim._truth()
        # relative to the servicer's centre of mass (from its own joint angles): before contact
        # its acceleration is exactly thrust / mass, whatever the arm does
        mujoco.mj_subtreeVel(sim.km, kd)
        ps, vs = kd.subtree_com[sim.bid_chaser].copy(), kd.subtree_linvel[sim.bid_chaser].copy()
        rho, v = pc - ps, vc - vs
        f = self.f
        if not f.ready:  # the filter is already running at hand-over: start from one measurement
            z, Rm, sp = self._measure(rho, Rc)
            f.init(z, Rm, sp, self.ma, v0=v + 1e-3 * self.rng.normal(size=3))  # far-approach nav: 1 mm/s
        else:
            f.push(-np.asarray(d.ctrl[0:3], float) / sim.m_serv)  # thrust commanded over the last step
            if self.k % self.every == 0 and not drop:  # drop: the link is down, nothing arrives
                self.q.append((self.k + self.lat, *self._measure(rho, Rc)))
            while self.q and self.q[0][0] <= self.k:
                _, z, Rm, sp = self.q.pop(0)
                f.update(z, Rm, sp, self.ma, self.lat)
                self.nis.append((f.nis_p, f.nis_a))
                self.last_upd = self.k
        self.k += 1
        r_e, v_e, R_e, w_e = f.now()
        self.out = (ps + r_e, R_e, vs + v_e, w_e)
        m = self.err[group]
        ep, ev = float(np.linalg.norm(r_e - rho)), float(np.linalg.norm(v_e - v))
        ea = float(np.linalg.norm(Rotation.from_matrix(R_e @ Rc.T).as_rotvec()))
        ew = float(np.linalg.norm(w_e - wc))
        m["n"] += 1
        for key, val in (("p", ep), ("v", ev), ("a", ea), ("w", ew)):
            m[key + "2"] += val * val
            m[key + "m"] = max(m[key + "m"], val)


class FiltSim(TolSim):
    closed = True
    preroll = 30.0
    meas = (0.010, np.radians(0.5))
    slow = 0.5
    latency = 0.15
    fkw: dict = {}
    filt_est = None
    # P5.2: monitors on the estimate
    mon_est = False
    k_sig = 3.0  # allowance = k_sig x 1-sigma of the predicted clearance
    track_budget = 0.08  # m, predicted-clearance loss from bus tracking error (measured: 0.05-0.07)
    stale_s = 3.0  # abort when the newest client state is older than this
    drop = None  # (start s after the fly-in begins, duration s): link dropout
    horizon = 30.0

    def reset(self, rng=None, **kw):
        self.filt_est = None
        self.cond = None
        super().reset(rng, **kw)
        self.filt_est = FiltEst(self, *self.meas, self.slow, self.latency,
                                np.random.default_rng([self.noise_seed, 78]), **self.fkw)
        self.est = self.filt_est if self.closed else None
        self._pre = self.preroll if (self.closed and self.flyin_path is not None) else 0.0
        self.pre_out = {}
        self.mon_noisy = self.mon_est and self.closed
        self._cp_allow, self.allow_max, self.cp_min, self._fly_clock = 0.0, 0.0, np.inf, 0.0
        if self.closed and self.flyin_path is not None:
            # run() sizes its time limit from this path; the path is re-planned (and may be
            # slowed) after the wait, so leave room. It is only an upper bound.
            self.flyin_path.total += self.preroll + 600.0

    def sync(self):
        GraspSim.sync(self)
        if self.filt_est is not None:
            fly = getattr(self, "flyin_path", None) is not None and self._flyin_t is not None
            pre = self._pre > 0 if self.closed else self.filt_est.k * self.cfg.ctrl_dt < self.preroll
            post = getattr(self, "_ph", "") in ("close", "hold", "abort", "done")  # jaws on the bar: other dynamics
            dr = self.drop is not None and self.drop[0] <= self._fly_clock < self.drop[0] + self.drop[1]
            self.filt_est.step(self, "preroll" if pre else ("flyin" if fly else ("post" if post else "prox")), dr)

    def allowance(self, rng=None):
        """Uncertainty allowance on the predicted clearance [m]: k_sig x (position + horizon x
        velocity + client radius x (attitude + horizon x rate)), 1-sigma each. Position and
        attitude also carry the stated accuracy of the provided state, because its slow drift
        is not averaged out by the filter."""
        fe = self.filt_est
        sp, sv, sa, sw = fe.f.sigma()
        if rng is None:
            rng = float(np.linalg.norm(fe.out[0] - self.kd.xpos[self.bid_chaser]))
        sp = float(np.hypot(sp, fe.mp * max(rng / fe.r_ref, 1.0)))
        sa = float(np.hypot(sa, fe.ma))
        H = self.horizon
        return self.k_sig * (sp + H * sv + CLIENT_RADIUS * (sa + H * sw))

    def _leg_cp(self, fp, w_b, i):
        """Smallest predicted free-drift clearance along leg i of path fp, for a servicer
        exactly on the path and a client turning at w_b (body frame)."""
        t0 = float(sum(fp.T[:i]))
        best = np.inf
        for t in np.arange(t0, t0 + fp.T[i] + 1e-9, 1.0):
            r_b, rd_b, _ = fp.body_ref(t)
            v = np.cross(w_b, r_b) + rd_b
            for tau in np.arange(0.0, self.horizon + 1e-9, 2.0):
                x = Rotation.from_rotvec(w_b * tau).as_matrix().T @ (r_b + v * tau)
                best = min(best, box_clearance(x, self.boxes))
        return best - self.r_serv - self.cfg.clear_margin

    def _slow_path(self, fp, w_b, margin):
        """Lengthen each leg until the path alone keeps `margin` of predicted clearance."""
        fac = []
        for i in range(len(fp.T)):
            f = 1.0
            while self._leg_cp(fp, w_b, i) < margin and f < 4.0:
                fp.T[i] *= 1.15
                f *= 1.15
            fac.append(round(f, 2))
        fp.total, fp.leg1 = float(sum(fp.T)), fp.T[0]
        return fac

    def _replan(self):
        """Plan the fly-in from the estimated client attitude and spin axis."""
        from flyin import FlyIn
        cfg, kd = self.cfg, self.kd
        pe, Re, _, we = self.est.out
        w_rel = we - np.array([0.0, 0.0, self.n])
        wn = float(np.linalg.norm(w_rel))
        s_w = w_rel / wn if wn > np.radians(0.05) else Re @ np.array([-1.0, 0.0, 0.0])
        _, Rc, _, wc = self._truth()
        wt = wc - np.array([0.0, 0.0, self.n])
        cosang = abs(float(s_w @ wt)) / max(np.linalg.norm(wt), 1e-12)
        self.pre_out = dict(axis_err_deg=float(np.degrees(np.arccos(min(cosang, 1.0)))),
                            rate_err_deg_s=float(np.degrees(np.linalg.norm(we - wc))),
                            att_err_deg=float(np.degrees(np.linalg.norm(Rotation.from_matrix(Re @ Rc.T).as_rotvec()))))
        ps, Rs = kd.xpos[self.bid_chaser], kd.xmat[self.bid_chaser].reshape(3, 3)
        self.flyin_path = FlyIn(Re.T @ (ps - pe), self.rel_pos_ref.copy(), Re.T @ s_w,
                                h_safe=cfg.flyin_h_safe, v_max=cfg.flyin_v)
        self.R_rel0 = Re.T @ Rs
        self._flyin_t = 0.0
        if self.mon_est:
            # the planned path must leave the monitor its allowance plus a tracking budget
            # (the frozen plan leaves 0.03 m; the pre-grasp point itself has 0.30 m)
            margin = min(self.allowance(rng=2.5) + self.track_budget, 0.26)
            self.pre_out.update(path_margin=margin, path_slow=self._slow_path(self.flyin_path, Re.T @ we, margin),
                                path_s=self.flyin_path.total)

    def bus_ctrl(self):
        if self._pre > 0 and self._flyin_t is not None:
            self._flyin_t = 0.0  # wait at the hold point while the filter settles
            self._pre -= self.cfg.ctrl_dt
            if self._pre <= 0:
                self._replan()
        elif self._flyin_t is not None:
            self._fly_clock += self.cfg.ctrl_dt
        return super().bus_ctrl()


# ---- P5.2: the E2 monitors read the estimate (noise_tol's wrapper swaps it in), the predicted
# clearance loses the uncertainty allowance, and a stale client state is an abort trigger
_tol_check = safety.Monitor.check
_orig_pc = safety.Monitor.predicted_clearance


def _pc(self, sim):
    cp = _orig_pc(self, sim) - getattr(sim, "_cp_allow", 0.0)
    if hasattr(sim, "cp_min"):
        sim.cp_min = min(sim.cp_min, cp)
    return cp


def _check(self, sim, phase, t, F_raw, grasped, hits):
    # armed once the filter has converged (end of the wait at the hold point; in a mission the
    # filter is already running at hand-over). Until then the monitors read the truth.
    on = (getattr(sim, "mon_est", False) and getattr(sim, "closed", False)
          and getattr(sim, "filt_est", None) is not None and sim._pre <= 0)
    sim.mon_noisy = on
    sim._cp_allow = sim.allowance() if on and not grasped and phase != "hold" else 0.0
    if on:
        sim.allow_max = max(sim.allow_max, sim._cp_allow)
    why = _tol_check(self, sim, phase, t, F_raw, grasped, hits)
    if on and not why and not grasped and phase in ("flyin", "settle", "reach", "approach") \
            and sim.filt_est.age() > sim.stale_s:
        self.first.setdefault("stale_link", (t, phase))
        return "stale_link"
    return why


safety.Monitor.predicted_clearance = _pc
safety.Monitor.check = _check


_W = {}


def _init(scene, a):
    import os
    for k in range(20):
        try:
            sim = FiltSim(GraspCfg(scene=scene, flyin=True, hold_s=5.0, mon=True, mon_act=True))
            break
        except ValueError as e:
            if "parse" not in str(e).lower() and "empty" not in str(e).lower():
                raise
            time.sleep(0.3 + 0.3 * k + 0.05 * (os.getpid() % 10))
    sim.closed, sim.preroll, sim.slow, sim.latency = a["closed"], a["preroll"], a["slow"], a["latency"]
    sim.fkw = dict(sig_a=a["sig_a"], sig_w=a["sig_w"])
    sim.mon_est, sim.k_sig, sim.stale_s, sim.drop = a["mon"] == "est", a["k_sig"], a["stale"], a["drop"]
    _W["sim"] = sim


def _episode(job):
    name, (mp, ma), seed = job
    sim = _W["sim"]
    c = case_draws(seed, 0.0)
    sim.meas = (mp * 1e-3, np.radians(ma))
    sim.noise_seed = seed
    t0 = time.time()
    r = sim.run(np.random.default_rng(seed), spin_deg_s=c["spin"], spin_axis=c["axis"])
    out = dict(cond=name, seed=seed, spin=c["spin"], axis=c["axis"], wall_s=round(time.time() - t0, 1),
               **{k: r.get(k) for k in KEEP}, **{"pre_" + k: v for k, v in sim.pre_out.items()},
               allow_max=sim.allow_max, cp_min=sim.cp_min)
    for g, m in sim.m.items():
        n = max(m["n"], 1)
        out.update({f"{g}_err_rms_mm": 1000 * np.sqrt(m["e2"] / n), f"{g}_err_max_mm": 1000 * m["emax"],
                    f"{g}_dv_mm_s": 1000 * m["dv"], f"{g}_sat_frac": m["sat"] / n})
    sc = dict(p=1000.0, v=1000.0, a=np.degrees(1.0), w=np.degrees(1.0))
    for g, m in sim.filt_est.err.items():
        if m["n"]:
            for key in "pvaw":
                out[f"f_{g}_{key}_rms"] = sc[key] * np.sqrt(m[key + "2"] / m["n"])
                out[f"f_{g}_{key}_max"] = sc[key] * m[key + "m"]
    nis = np.array(sim.filt_est.nis)
    out["nis_p"], out["nis_a"] = float(nis[:, 0].mean()), float(nis[:, 1].mean())  # 3 if consistent
    return out


def summarise(name, cond, rows):
    med = lambda k: float(np.median([r[k] for r in rows if k in r]))  # noqa: E731
    mx = lambda k: float(np.max([r[k] for r in rows if k in r]))  # noqa: E731
    ok = sum(bool(r["success"]) and not r["collision"] and r["push_dv"] <= 0.002 for r in rows)
    s = dict(cond=name, meas_mm=cond[0], meas_deg=cond[1], n=len(rows), success=ok,
             outcomes=dict(Counter(r["outcome"] for r in rows)),
             aborts=dict(Counter(f'{r["abort_reason"]}@{r["abort_phase"]}' for r in rows if r["aborted"])),
             collisions=sum(bool(r["collision"]) for r in rows), push_max_mm_s=1000 * mx("push_dv"),
             flyin_dv_mm_s=med("flyin_dv_mm_s"), prox_err_rms_mm=med("prox_err_rms_mm"),
             flyin_err_max_mm=mx("flyin_err_max_mm"), nis_p=med("nis_p"), nis_a=med("nis_a"),
             flyin_s=med("flyin_s"), min_clear_m=float(np.min([r["min_clear"] for r in rows])),
             allow_max_m=mx("allow_max"), cp_min_m=float(np.min([r["cp_min"] for r in rows])))
    if any("pre_path_margin" in r for r in rows):
        s["path_margin_m"] = med("pre_path_margin")
        s["path_s"] = dict(median=med("pre_path_s"), max=mx("pre_path_s"))
    for g in GROUPS:
        for key in "pvaw":
            if any(f"f_{g}_{key}_rms" in r for r in rows):
                s[f"f_{g}_{key}"] = dict(rms=med(f"f_{g}_{key}_rms"), max=mx(f"f_{g}_{key}_max"))
    if any("pre_axis_err_deg" in r for r in rows):
        s["pre_axis_err_deg"] = dict(median=med("pre_axis_err_deg"), max=mx("pre_axis_err_deg"))
        s["pre_rate_err_deg_s"] = dict(median=med("pre_rate_err_deg_s"), max=mx("pre_rate_err_deg_s"))
    return s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--loop", choices=("open", "closed"), required=True)
    ap.add_argument("--conds", nargs="+", required=True, help="name:meas_pos_mm,meas_att_deg")
    ap.add_argument("--n", type=int, default=25)
    ap.add_argument("--start", type=int, default=0)
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--preroll", type=float, default=30.0)
    ap.add_argument("--latency", type=float, default=0.15)
    ap.add_argument("--slow", type=float, default=0.5, help="fraction of the error variance that drifts slowly")
    ap.add_argument("--sig-a", type=float, default=2e-5, help="filter: acceleration noise [m/s^2/sqrt(Hz)]")
    ap.add_argument("--sig-w", type=float, default=5e-6, help="filter: rate random walk [rad/s/sqrt(s)]")
    ap.add_argument("--mon", choices=("truth", "est"), default="truth", help="what the E2 monitors read")
    ap.add_argument("--k-sig", type=float, default=3.0, help="clearance allowance in sigmas (--mon est)")
    ap.add_argument("--stale", type=float, default=3.0, help="stale-link abort threshold [s] (--mon est)")
    ap.add_argument("--drop", default="", help="link dropout: start,duration [s after the fly-in starts]")
    ap.add_argument("--scene", default="p3e_espa_arm2_scene.xml")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    conds = [(s.split(":")[0], tuple(float(x) for x in s.split(":")[1].split(","))) for s in a.conds]
    jobs = [(name, cond, TUNE + a.start + i) for name, cond in conds for i in range(a.n)]
    cfg = dict(closed=a.loop == "closed", preroll=a.preroll, slow=a.slow, latency=a.latency, sig_a=a.sig_a,
               sig_w=a.sig_w, mon=a.mon, k_sig=a.k_sig, stale=a.stale,
               drop=tuple(float(x) for x in a.drop.split(",")) if a.drop else None)
    t0 = time.time()
    rows = []
    with Pool(a.workers, initializer=_init, initargs=(a.scene, cfg)) as pool:
        for i, r in enumerate(pool.imap(_episode, jobs, chunksize=1), 1):
            rows.append(r)
            if i % 25 == 0 or i == len(jobs):
                print(f"  {i}/{len(jobs)}, {(time.time() - t0) / 60:.1f} min", flush=True)
    summ = [summarise(name, cond, [r for r in rows if r["cond"] == name]) for name, cond in conds]
    Path(a.out).write_text(json.dumps(dict(cfg=cfg, scene=a.scene, n=a.n, summary=summ, episodes=rows), indent=1,
                                      default=float))
    for s in summ:
        print(f"\n{s['cond']} (meas {s['meas_mm']} mm x range/2.7, {s['meas_deg']} deg): {s['success']}/{s['n']} "
              f"{s['outcomes']} aborts {s['aborts']} push max {s['push_max_mm_s']:.2f} mm/s, fly-in dv "
              f"{s['flyin_dv_mm_s']:.0f} mm/s, prox station err {s['prox_err_rms_mm']:.1f} mm rms, "
              f"NIS pos {s['nis_p']:.1f} att {s['nis_a']:.1f}; fly-in {s['flyin_s']:.0f} s, min true clearance "
              f"{s['min_clear_m']:.3f} m, monitor cp min {s['cp_min_m']:.3f} m, allowance max {s['allow_max_m']:.3f} m"
              + (f", path margin {s['path_margin_m']:.3f} m, planned {s['path_s']['median']:.0f} s (max "
                 f"{s['path_s']['max']:.0f})" if "path_margin_m" in s else ""))
        if "pre_axis_err_deg" in s:
            print(f"  after pre-roll: spin axis err median {s['pre_axis_err_deg']['median']:.1f} max "
                  f"{s['pre_axis_err_deg']['max']:.1f} deg, rate err median {s['pre_rate_err_deg_s']['median']:.4f} "
                  f"max {s['pre_rate_err_deg_s']['max']:.4f} deg/s")
        print(f"  {'filter error':10s} {'pos mm':>14s} {'vel mm/s':>14s} {'att deg':>14s} {'rate deg/s':>16s}   (rms median / max)")
        for g in GROUPS:
            if f"f_{g}_p" in s:
                print(f"  {g:10s} " + " ".join(f"{s[f'f_{g}_{k}']['rms']:>6.{d}f} /{s[f'f_{g}_{k}']['max']:>6.{d}f}"
                                               for k, d in (("p", 1), ("v", 2), ("a", 2), ("w", 4))))
    print("wrote", a.out)


if __name__ == "__main__":
    main()
