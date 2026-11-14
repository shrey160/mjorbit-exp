"""P5.4: the whole chain on the servicer's own information.

  - client state from the link (provided-state model of filt_eval.py) through the relative-motion
    filter -> bus station keeping, fly-in, E2 monitors (with the P5.2 allowance and stale-link abort)
  - arm on the Project 4 estimators (v3 bar-pose network + palm camera), camera mounts +-10 mm, 1 deg
  - --cam-fix: close-range hand-over. The link position has a slow error the filter cannot
    remove (P5.3': 60 mm per axis leaves the bus up to 0.19 m off station). Two seconds after the
    cameras first see the bar (after the fly-in) the filter's position input switches from the
    link to the camera-derived client position (attitude stays on the link). The filter state is
    shifted at once; the opposite offset is added to its output and removed at --fix-rate, so the
    bus moves over slowly (the monitor predicts a 30 s coast). The servicer waits in settle until
    the offset is gone, then reaches. At the close command position updates stop.

python cam_eval.py --conds nom:60,0.05,1,0.3 --cam-fix --n 25 --workers 8 --out <abs path>.json
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from multiprocessing import Pool
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "p4_vision"))
import filt_eval  # noqa: E402
from filt_eval import GROUPS, FiltSim, summarise  # noqa: E402
from noise_tol import KEEP, TUNE  # noqa: E402
from evaluate_e3 import case_draws  # noqa: E402
from grasp_sim import GraspCfg  # noqa: E402
from safety import CLIENT_RADIUS  # noqa: E402

SCENE = "p4_espa_arm2_cam_scene.xml"
SETTLE_MAX = 150.0
CAM_SIG = 0.015  # m, 1-sigma of the camera-derived client position (v3 at a 10 mm mount error)


class CamSim(FiltSim):
    cam_fix = False
    fix_rate = 0.003  # m/s
    fix_tol = 0.005  # m: converged when the correction is this close to its target ...
    fix_dwell = 3.0  # ... for this long, and the bus is near its reference

    def reset(self, rng=None, **kw):
        self.cfg.settle_s = SETTLE_MAX if self.cam_fix else 2.0
        super().reset(rng, **kw)
        self.b, self._b_ok, self._b_frozen = None, 0.0, False
        self._rc, self._cam_t, self._meas_off = None, 0.0, False
        self._rp0 = self.rel_pos_ref.copy()
        self.fx = dict(b_mm=np.nan, settle_s=np.nan, bus_off_close_mm=np.nan, link_err_close_mm=np.nan,
                       used_err_close_mm=np.nan)
        self._t_settle = None
        self._link_p = None

    def _hook(self, loc):
        prev = self._ph
        super()._hook(loc)
        ph, t = loc["phase"], loc["t"]
        if ph == "settle" and prev != "settle":
            self._t_settle = t
        if ph == "reach" and prev == "settle" and self._t_settle is not None:
            self.fx["settle_s"] = t - self._t_settle
        if ph == "close" and prev != "close":
            self._b_frozen = True
            pc, Rc, _, _ = self._truth()
            ps = self.kd.xpos[self.bid_chaser]
            self.fx["bus_off_close_mm"] = 1e3 * float(np.linalg.norm(ps - (pc + Rc @ self._rp0)))
            self.fx["link_err_close_mm"] = 1e3 * float(np.linalg.norm(self._link_p - pc)) if self._link_p is not None else np.nan
            self.fx["used_err_close_mm"] = 1e3 * float(np.linalg.norm(self.filt_est.out[0] - pc))

    def _before_filter(self):
        """Hand the filter's position input over from the link to the cameras once they see the bar."""
        fe, pe, dt = self.filt_est, self.pose_est, self.cfg.ctrl_dt
        if fe.out is None:
            return
        if self._b_frozen:  # jaws closing: the camera estimate is latched and contact changes the
            self._meas_off, fe.z_override = True, None  # dynamics; coast on the thrust model
            return
        if not (self.cam_fix and pe is not None and pe.p_b is not None and self._ph in ("settle", "reach", "approach")):
            return
        # client centre from the camera's bar position and the LINK attitude (0.05 deg; the camera's
        # rotation error times the 0.84 m bar offset would add 15 mm), relative to the servicer's
        # centre of mass as the filter wants it
        rho = pe.bar(self)[0] - fe.out[1] @ pe.geo[0] - self.kd.subtree_com[self.bid_chaser]
        # the link-to-camera difference is averaged over 2 s (never the position itself: it turns
        # with the client, and smoothing it would lag by tens of mm)
        diff = rho - fe.f.now()[0]
        self._rc = diff.copy() if self._rc is None else self._rc + diff
        self._cam_t += dt
        if self.b is None and self._cam_t >= 2.0:
            # hand-over: move the filter's position to the camera value at once, and add the
            # opposite offset to its output, removed at fix_rate, so the bus moves over slowly
            delta = self._rc / round(self._cam_t / dt)
            fe.f.x[0:3] += delta
            self.b = -delta
            self.fx["b_mm"] = 1e3 * float(np.linalg.norm(delta))
        if self.b is not None:
            fe.z_override = (rho, CAM_SIG)

    def sync(self):
        super().sync()
        fe = self.filt_est
        if fe is None or fe.out is None:
            return
        self._raw_p = fe.out[0].copy()
        dt = self.cfg.ctrl_dt
        if self.b is not None:
            n = float(np.linalg.norm(self.b))
            if not self._b_frozen:
                self.b = self.b * max(0.0, 1.0 - self.fix_rate * dt / max(n, 1e-9))
            if self._ph == "settle":
                ref = getattr(self, "_bus_ref", None)
                near = ref is not None and np.linalg.norm(self.kd.xpos[self.bid_chaser] - ref[0]) < 0.02
                self._b_ok = self._b_ok + dt if (n < self.fix_tol and near) else 0.0
                if self._b_ok >= self.fix_dwell:
                    self.cfg.settle_s = 0.0  # converged: let the run move on to the reach
            if os.environ.get("P5_DBG") and fe.k % 500 == 0:
                pc, Rc, _, _ = self._truth()
                ps = self.kd.xpos[self.bid_chaser]
                rf = getattr(self, "_bus_ref", None)
                print(f"  dbg {self._ph:8s} k {fe.k} |b| {1e3 * n:5.1f} filter err {1e3 * np.linalg.norm(self._raw_p - pc):5.1f} "
                      f"bus-ref {1e3 * np.linalg.norm(ps - rf[0]) if rf else -1:5.1f} bus off "
                      f"{1e3 * np.linalg.norm(ps - (pc + Rc @ self._rp0)):5.1f} ok {self._b_ok:.1f}", flush=True)
            o = fe.out
            fe.out = (o[0] + self.b, o[1], o[2], o[3])

    def allowance(self, rng=None):
        if self.b is None:
            return super().allowance(rng)
        sp, sv, sa, sw = self.filt_est.f.sigma()  # position now tied to the cameras
        sp, sa = float(np.hypot(sp, CAM_SIG)), float(np.hypot(sa, self.filt_est.ma))
        H = self.horizon
        return self.k_sig * float(np.sqrt(sp**2 + (H * sv) ** 2 + (CLIENT_RADIUS * sa) ** 2 + (CLIENT_RADIUS * H * sw) ** 2))


_W = {}


def _init(a):
    import torch
    torch.set_num_threads(1)
    from cams import CamRig
    for k in range(20):
        try:
            sim = CamSim(GraspCfg(scene=SCENE, flyin=True, hold_s=5.0, mon=True, mon_act=True))
            break
        except ValueError as e:
            if "parse" not in str(e).lower() and "empty" not in str(e).lower():
                raise
            time.sleep(0.3 + 0.3 * k + 0.05 * (os.getpid() % 10))
    sim.closed, sim.preroll, sim.slow = True, 30.0, a["slow"]
    sim.ekw = dict(tau_b=a["tau_b"], r_ref=1e9)
    sim.mon_est, sim.cam_fix, sim.fix_rate, sim.k_sig = True, a["cam_fix"], a["fix_rate"], a["k_sig"]
    rig = CamRig(sim.km, mount_m=a["mount_mm"] * 1e-3, mount_deg=a["mount_mm"] / 10.0)
    est = None
    if a["arm"] == "cam":
        from pose_est import PalmPoseEst
        est = PalmPoseEst(a["ckpt"], a["palm_ckpt"], rig, use_rot=False)
    _W.update(sim=sim, rig=rig, est=est)


def _episode(job):
    name, cond, seed = job
    sim, rig, est = _W["sim"], _W["rig"], _W["est"]
    c = case_draws(seed, 0.0)
    rig.randomize(np.random.default_rng(seed + 12345))
    if est is not None:
        est.reset()
    sim.pose_est = est
    sim.meas, sim.noise_seed = (cond[0] * 1e-3, np.radians(cond[1])), seed
    sim.ekw = dict(sim.ekw, every=max(int(round(1.0 / (cond[2] * sim.cfg.ctrl_dt))), 1))
    sim.latency = cond[3]
    t0 = time.time()
    r = sim.run(np.random.default_rng(seed), spin_deg_s=c["spin"], spin_axis=c["axis"])
    out = dict(cond=name, seed=seed, spin=c["spin"], axis=c["axis"], wall_s=round(time.time() - t0, 1),
               **{k: r.get(k) for k in KEEP}, **{"pre_" + k: v for k, v in sim.pre_out.items()},
               allow_max=sim.allow_max, cp_min=sim.cp_min, bar_shift_mm=r.get("bar_shift_mm"), **sim.fx)
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
    out["nis_p"], out["nis_a"] = float(nis[:, 0].mean()), float(nis[:, 1].mean())
    if est is not None and est.log:
        out["est_close_mm"] = 1e3 * est.log[-1][2]
        out["n_palm"], out["n_palm_rej"] = est.n_palm, est.n_rej
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--conds", nargs="+", required=True, help="name:pos_mm,att_deg,rate_hz,latency_s")
    ap.add_argument("--n", type=int, default=25)
    ap.add_argument("--start", type=int, default=0)
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--cam-fix", action="store_true")
    ap.add_argument("--fix-rate", type=float, default=0.003, help="slew of the correction [m/s]")
    ap.add_argument("--k-sig", type=float, default=2.0, help="clearance allowance in sigmas (the monitor already has a 0.30 m margin)")
    ap.add_argument("--arm", choices=("cam", "truth"), default="cam")
    ap.add_argument("--mount-mm", type=float, default=10.0)
    ap.add_argument("--slow", type=float, default=0.9)
    ap.add_argument("--tau-b", type=float, default=60.0)
    ap.add_argument("--ckpt", default="/home/shrey31/p4_data/pose_v3/best.pt")
    ap.add_argument("--palm-ckpt", default="/home/shrey31/p4_data/palm_net_v2/best.pt")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    conds = [(s.split(":")[0], tuple(float(x) for x in s.split(":")[1].split(","))) for s in a.conds]
    jobs = [(name, cond, TUNE + a.start + i) for name, cond in conds for i in range(a.n)]
    cfg = {k: v for k, v in vars(a).items() if k not in ("conds", "out")}
    t0 = time.time()
    rows = []
    with Pool(a.workers, initializer=_init, initargs=(cfg,)) as pool:
        for i, r in enumerate(pool.imap(_episode, jobs, chunksize=1), 1):
            rows.append(r)
            if i % 25 == 0 or i == len(jobs):
                print(f"  {i}/{len(jobs)}, {(time.time() - t0) / 60:.1f} min", flush=True)
    summ = []
    med = lambda rs, k: float(np.nanmedian([r[k] for r in rs if r.get(k) is not None])) if rs else np.nan  # noqa: E731
    mx = lambda rs, k: float(np.nanmax([r[k] for r in rs if r.get(k) is not None])) if rs else np.nan  # noqa: E731
    for name, cond in conds:
        rs = [r for r in rows if r["cond"] == name]
        s = summarise(name, cond, rs)
        for k in ("bus_off_close_mm", "link_err_close_mm", "used_err_close_mm", "settle_s", "b_mm", "est_close_mm"):
            if any(r.get(k) is not None for r in rs):
                s[k] = dict(median=med(rs, k), max=mx(rs, k))
        s["failed"] = [(r["seed"], r["outcome"], r["abort_reason"], r["abort_phase"], round(1e3 * r["push_dv"], 2))
                       for r in rs if not (r["success"] and not r["collision"] and r["push_dv"] <= 0.002)]
        summ.append(s)
        print(f"\n{name} (link {cond[0]} mm, {cond[1]} deg, {cond[2]} Hz, {cond[3]} s late; cam-fix {a.cam_fix}, arm "
              f"{a.arm}): {s['success']}/{s['n']} {s['outcomes']} aborts {s['aborts']} collisions {s['collisions']} "
              f"push max {s['push_max_mm_s']:.2f} mm/s; min true clearance {s['min_clear_m']:.3f} m, monitor cp min "
              f"{s['cp_min_m']:.3f} m, allowance max {s['allow_max_m']:.3f} m")
        for k in ("link_err_close_mm", "used_err_close_mm", "bus_off_close_mm", "settle_s", "est_close_mm"):
            if k in s:
                print(f"  {k:20s} median {s[k]['median']:.1f} max {s[k]['max']:.1f}")
        print(f"  failed: {s['failed']}")
    Path(a.out).write_text(json.dumps(dict(cfg=cfg, scene=SCENE, summary=summ, episodes=rows), indent=1, default=float))
    print("wrote", a.out)


if __name__ == "__main__":
    main()
