"""P6.1: the Project 5 chain with a SILENT client: no link, client pose from a (future) network on
the bus camera and lidar, here replaced by a vision-like error model on the true pose. No images.

Measurement model (one message every 1/rate s, `latency` s old):
  - position error 1-sigma  mp x max(range / 2.7 m, 1), attitude 1-sigma ma, both x `face` when the
    grapple bar's face (client -x) is turned away (cos of the view angle below 0.26, 75 deg).
    Half the variance drifts slowly (5 s; a network's error follows the view), half is white.
  - symmetry flips: the client is a box 1.5 x 1 x 1 m, so a network can return the pose turned by
    one of its symmetries (90 deg steps about x, 180 deg about y or z). Without the bar in view
    any of the 7; with it only 180 deg about x (the bar itself is symmetric). A flip starts with
    probability p_flip per message and lasts 1 s on average (it follows the view, not the frame).
  - outliers: probability p_out per message; position off by N(0, 0.3 m) per axis, attitude random.
  - dropouts: bursts at drop_per_min, 1-6 s each (eclipse, sun in view, arm in the way).
The network is assumed to state its 1-sigma correctly (sp, sa) and to flag whether the bar is seen.

Filter (rel_filter.RelFilter, tuned noise, tidal term, no drift state) with
  - gating: position and attitude are each rejected when their innovation exceeds `gate` (NIS);
  - symmetry snapping (--sym on): before the attitude update the measurement is turned by the
    client symmetry (allowed for the bar flag) that brings it nearest to the prediction;
  - started from the first message; during the observation it is restarted when attitude has
    been rejected for t_reinit s (a wrong start, e.g. a flip while the bar was hidden).

Mission changes against Project 5:
  - observation at the hold point: the fly-in starts once the filter's rate sigma is below w_thr,
    after at least t_min s and with the bar seen since the last restart; after t_max s without
    that the run aborts safely at the hold point (`no_converge`);
  - lost track = `stale_link` of filt_eval (no accepted message for --stale s);
  - `nav_anom` = attitude rejected for t_anom s (replaces the single-message NIS abort of P5.5).
Arm on the true bar pose, scene without cameras, speed-limit fly-in, monitors on the estimate.

python vis_eval.py --conds nom:10,1,0.02,0.05,1 --n 25 --workers 8 --out <abs>/p6_vis.json
  each condition is name:pos_mm,att_deg,p_out,p_flip,drop_per_min
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from collections import Counter
from multiprocessing import Pool
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

P5 = Path(__file__).resolve().parents[1] / "p5_own_eyes"
sys.path.insert(0, str(P5))
import filt_eval  # noqa: E402,F401  (installs the P5.2 monitor wrapper)
from filt_eval import GROUPS, FiltEst, FiltSim, summarise  # noqa: E402
from fly_lim import LimMixin  # noqa: E402
from noise_tol import KEEP, TUNE  # noqa: E402
from evaluate_e3 import case_draws  # noqa: E402
from grasp_sim import GraspCfg  # noqa: E402
from rel_filter import RelFilter, _exp  # noqa: E402
import safety  # noqa: E402

SCENE = "p3e_espa_arm2_scene.xml"
BAR_N = np.array([-1.0, 0.0, 0.0])  # grapple-bar face normal, client body frame
COS_VIS = 0.26


def _rx(a):
    return Rotation.from_rotvec([a, 0.0, 0.0]).as_matrix()


_RY = Rotation.from_rotvec([0.0, np.pi, 0.0]).as_matrix()
SYM_ALL = [_rx(k * np.pi / 2) for k in range(4)] + [_RY @ _rx(k * np.pi / 2) for k in range(4)]
SYM_BAR = [SYM_ALL[0], SYM_ALL[2]]  # the bar in view: only 180 deg about x is left


def _ang(R):
    return float(np.linalg.norm(Rotation.from_matrix(R).as_rotvec()))


class GFilter(RelFilter):
    """RelFilter (6 states) with innovation gating and symmetry snapping."""

    def update_g(self, rho_m, R_m, sp, sa, age_steps, gate, sym):
        n = max(len(self.acc) - age_steps, 0)
        self._advance(n)
        y = rho_m - self.x[0:3]
        S = self.P[0:3, 0:3] + sp**2 * np.eye(3)
        nis_p = float(y @ np.linalg.solve(S, y))
        acc_p = nis_p <= gate
        if acc_p:
            K = self.P[:, 0:3] @ np.linalg.inv(S)
            self.x = self.x + K @ y
            self.P = self.P - K @ self.P[0:3, :]
        if sym is not None:  # turn the measurement by the symmetry nearest the prediction
            R_m = min((R_m @ Sk.T for Sk in sym), key=lambda Rk: _ang(Rk @ self.R.T))
        ya = Rotation.from_matrix(R_m @ self.R.T).as_rotvec()
        Sa = self.Pa[0:3, 0:3] + sa**2 * np.eye(3)
        nis_a = float(ya @ np.linalg.solve(Sa, ya))
        acc_a = nis_a <= gate
        if acc_a:
            Ka = self.Pa[:, 0:3] @ np.linalg.inv(Sa)
            d = Ka @ ya
            self.R = _exp(d[0:3]) @ self.R
            self.w = self.w + d[3:6]
            self.Pa = self.Pa - Ka @ self.Pa[0:3, :]
        self.nis_p, self.nis_a = nis_p, nis_a
        return acc_p, acc_a


class VisEst(FiltEst):
    def __init__(self, sim, vk, rng, every, latency, tide=True, **fkw):
        self.tide, self.dt = tide, sim.cfg.ctrl_dt
        self.f = GFilter(self.dt, **fkw)
        self.rng, self.every = rng, every
        self.lat = int(round(latency / self.dt))
        self.mp, self.ma, self.r_ref = vk["mp"], vk["ma"], 2.7  # read by FiltSim.allowance
        self.vk = vk
        self.slow = 0.5
        self.ab = float(np.exp(-every * self.dt / 5.0))
        self.b = rng.normal(size=6)
        self.k, self.q, self.out = 0, [], None
        self.err = {g: dict(n=0, p2=0.0, v2=0.0, a2=0.0, w2=0.0, pm=0.0, vm=0.0, am=0.0, wm=0.0) for g in GROUPS}
        self.nis = []
        self.z_override = None
        self.last_upd = 0
        self.flip, self.drop_left = None, 0
        self.rej_a_t = 0.0  # seconds of consecutive attitude rejections
        self.rej_a_max = 0.0  # longest such streak once the monitors are armed
        self.bar_seen = False
        self.resolved = False  # which end carries the bar is known (seen, or its absence seen)
        self.nobar_t = 0.0  # seconds the estimated bar end has faced the servicer without a bar
        self.st = Counter()

    def _measure(self, rho, Rc):
        vk, rng = self.vk, self.rng
        # cos between the bar face normal and the direction from the client to the servicer
        c = float((Rc.T @ -rho) @ BAR_N) / max(float(np.linalg.norm(rho)), 1e-9)
        vis = c > COS_VIS
        fac = 1.0 if vis else vk["face"]
        sp = vk["mp"] * max(np.linalg.norm(rho) / self.r_ref, 1.0) * fac
        sa = vk["ma"] * fac
        self.b = self.ab * self.b + np.sqrt(1 - self.ab**2) * rng.normal(size=6)
        e = np.sqrt(self.slow) * self.b + np.sqrt(1 - self.slow) * rng.normal(size=6)
        z, Rm = rho + sp * e[0:3], _exp(sa * e[3:6]) @ Rc
        dtm = self.every * self.dt
        if self.flip is not None and rng.random() > np.exp(-dtm / 1.0):
            self.flip = None
        if self.flip is not None and vis and not any(np.allclose(self.flip, Sk) for Sk in SYM_BAR):
            self.flip = None  # a flip begun with the bar hidden ends when the network sees the bar
        if self.flip is None and rng.random() < vk["p_flip"]:
            g = SYM_BAR if vis else SYM_ALL
            self.flip = g[1 + rng.integers(len(g) - 1)]
            self.st["flips"] += 1
        if self.flip is not None:
            Rm = Rm @ self.flip
            self.st["flip_msgs"] += 1
        if rng.random() < vk["p_out"]:
            z = rho + 0.3 * rng.normal(size=3)
            Rm = Rotation.random(random_state=int(rng.integers(2**31))).as_matrix()
            self.st["outliers"] += 1
        return z, Rm, sp, sa, vis

    def step(self, sim, group, drop=False):
        kd, d, vk = sim.kd, sim.data, self.vk
        pc, Rc, vc, wc = sim._truth()
        mujoco.mj_subtreeVel(sim.km, kd)
        ps, vs = kd.subtree_com[sim.bid_chaser].copy(), kd.subtree_linvel[sim.bid_chaser].copy()
        rho, v = pc - ps, vc - vs
        f, dt = self.f, self.dt
        observing = sim._pre > 0
        if self.drop_left > 0:
            self.drop_left -= 1
        elif self.rng.random() < vk["drop"] / 60.0 * dt:
            self.drop_left = int(self.rng.uniform(1.0, 6.0) / dt)
            self.st["dropouts"] += 1
        if f.ready:
            a_in = -np.asarray(d.ctrl[0:3], float) / sim.m_serv
            if self.tide:
                th = sim.n * float(kd.time)
                u = np.array([np.cos(th), np.sin(th), 0.0])
                r_e = f.now()[0]
                a_in = a_in - sim.n**2 * (r_e - 3.0 * (r_e @ u) * u)
            f.push(a_in)
        if self.k % self.every == 0 and not drop and self.drop_left == 0:
            self.q.append((self.k + self.lat, *self._measure(rho, Rc)))
        while self.q and self.q[0][0] <= self.k:
            _, z, Rm, sp, sa, vis = self.q.pop(0)
            if not f.ready:
                f.init(z, Rm, vk["r_inf"] * sp, vk["r_inf"] * sa, v0=v + 1e-3 * self.rng.normal(size=3))  # far-approach nav: 1 mm/s
                self.bar_seen, self.rej_a_t, self.last_upd = vis, 0.0, self.k
                self.resolved, self.nobar_t = vis, 0.0
                continue
            if observing and not self.resolved:
                # negative evidence (P6.1 inspection): the end the filter thinks carries the bar
                # faces the servicer, yet the network reports no bar -> the bar is on the other end
                r_e, _, R_e, _ = f.now()
                c_end = float((R_e.T @ -r_e) @ BAR_N) / max(float(np.linalg.norm(r_e)), 1e-9)
                self.nobar_t = self.nobar_t + self.every * dt if (c_end > 0.5 and not vis) else 0.0
                if self.nobar_t >= 3.0:
                    f.R = f.R @ _RY  # turn the estimate to the other end (a box symmetry)
                    self.resolved, self.nobar_t = True, 0.0
                    self.st["end_flips"] += 1
            sym = (SYM_BAR if vis else SYM_ALL) if vk["sym"] else None
            # r_inf: the filter treats each message as independent; the slow half of the error is
            # not, so its stated sigma is inflated or the filter is overconfident (smoke test)
            ri = vk["r_inf"]
            acc_p, acc_a = f.update_g(z, Rm, ri * sp, ri * sa, self.lat, vk["gate"], sym)
            self.nis.append((f.nis_p, f.nis_a))
            self.st["msgs"] += 1
            self.st["rej_p"] += not acc_p
            self.st["rej_a"] += not acc_a
            if acc_p or acc_a:
                self.last_upd = self.k
            if acc_a:
                self.rej_a_t = 0.0
                self.bar_seen = self.bar_seen or vis
                self.resolved = self.resolved or vis
            else:
                self.rej_a_t += self.every * dt
                if not observing:
                    self.rej_a_max = max(self.rej_a_max, self.rej_a_t)
            if observing and self.rej_a_t >= vk["t_reinit"]:
                f.ready = False  # wrong start: begin again from the next message
                self.st["restarts"] += 1
        self.k += 1
        if not f.ready:
            return  # restarting: hold the last output for a message or two (None before the first)
        r_e, v_e, R_e, w_e = f.now()
        self.out = (ps + r_e, R_e, vs + v_e, w_e)
        m = self.err[group]
        ep, ev = float(np.linalg.norm(r_e - rho)), float(np.linalg.norm(v_e - v))
        ea = min(_ang(R_e @ (Rc @ Sk).T) for Sk in SYM_BAR)  # the bar looks the same turned 180 deg about x
        ew = float(np.linalg.norm(w_e - wc))
        m["n"] += 1
        for key, val in (("p", ep), ("v", ev), ("a", ea), ("w", ew)):
            m[key + "2"] += val * val
            m[key + "m"] = max(m[key + "m"], val)


class Hop:
    """Minimum-jerk move between two points fixed in inertial axes relative to the (estimated)
    client centre, used as the bus reference in place of the fly-in path while it lasts. The
    attitude reference stays as at the hold point (synchronised with the client)."""
    total = 1e9  # GraspSim ends the fly-in when its clock passes this; the hop is not the fly-in

    def __init__(self, sim, r0, r1, v_max):
        self.sim, self.r0, self.d = sim, np.asarray(r0, float), np.asarray(r1, float) - r0
        self.T = max(1.875 * float(np.linalg.norm(self.d)) / v_max, 20.0)
        self.t0 = float(sim.data.time)

    def done(self):
        return float(self.sim.data.time) - self.t0 >= self.T

    def reference(self, t, pc, vc, Rc, w_c, R_rel0, k=1.0):
        from flyin import _mj
        p, v, a = _mj((float(self.sim.data.time) - self.t0) / self.T)
        return pc + self.r0 + p * self.d, vc + v / self.T * self.d, a / self.T**2 * self.d, Rc @ R_rel0, w_c


class VisSim(LimMixin, FiltSim):
    vk: dict = {}
    t_min, t_max, w_thr = 30.0, 1200.0, np.radians(0.01)
    t_anom = 2.0
    rate_hz, lat_s = 10.0, 0.15
    hop_on, hop_deg, w_hop = True, 45.0, np.radians(0.05)

    def reset(self, rng=None, **kw):
        self.preroll = self.t_max  # FiltSim waits at the hold point this long at most
        super().reset(rng, **kw)
        every = max(int(round(1.0 / (self.rate_hz * self.cfg.ctrl_dt))), 1)
        self.filt_est = VisEst(self, self.vk, np.random.default_rng([self.noise_seed, 79]), every, self.lat_s,
                               **self.fkw)
        self.est = self.filt_est
        self.obs_t, self.obs_fail, self.obs_ok = 0.0, False, False
        self.hop, self.hop_state, self.hop_info = None, None, {}

    def _replan(self):
        super()._replan()
        _, Rc, _, _ = self._truth()  # attitude error modulo the bar's own symmetry
        self.pre_out["att_err_deg"] = float(np.degrees(min(_ang(self.est.out[1] @ (Rc @ Sk).T) for Sk in SYM_BAR)))

    def _hop_out(self):
        """Inspection hop: from the spin axis to a point hop_deg off it at the same range, aimed
        where the estimated bar end will face when the servicer arrives. From there an end face
        turns into view (either the bar is seen, or its absence on that end is)."""
        f, kd = self.filt_est.f, self.kd
        r_e, _, R_e, w_e = f.now()
        r0 = -r_e  # servicer relative to the client centre
        D = float(np.linalg.norm(r0))
        w_rel = w_e - np.array([0.0, 0.0, self.n])
        s = w_rel / max(float(np.linalg.norm(w_rel)), 1e-12)
        s_side = s if s @ r0 >= 0 else -s
        e = -R_e[:, 0]  # estimated bar-face normal, world
        e = e - (e @ s) * s
        e /= max(float(np.linalg.norm(e)), 1e-9)
        b = np.radians(self.hop_deg)
        L = 2 * D * np.sin(b / 2)
        T = max(1.875 * L / self.cfg.flyin_v, 20.0)
        e_lead = _exp(w_rel * T) @ e  # the face turns with the client while the servicer moves
        r1 = D * (np.cos(b) * s_side + np.sin(b) * e_lead)
        self.hop = Hop(self, r0, r1, self.cfg.flyin_v)
        self.hop_state, self._r_axis = "out", D * s_side
        self.hop_info.update(t_out=self.obs_t, hop_s=self.hop.T)

    def bus_ctrl(self):
        if self._pre > 0 and self._flyin_t is not None:
            dt = self.cfg.ctrl_dt
            self.obs_t += dt
            fe = self.filt_est
            sw = fe.f.sigma()[3] if fe.f.ready else np.inf
            if self.hop_state == "out" and self.hop.done():
                self.hop_state = "look"
                self.hop_info["t_look"] = self.obs_t
            if self.hop_state == "look" and fe.resolved:
                self.hop_info["look_s"] = self.obs_t - self.hop_info["t_look"]
                pe = fe.f.now()[0]
                self.hop = Hop(self, -pe, self._r_axis, self.cfg.flyin_v)
                self.hop_state = "back"
            if self.hop_state == "back" and self.hop.done():
                self.hop_state, self.hop = "done", None
                self.hop_info["t_done"] = self.obs_t
            if (self.hop_on and self.hop_state is None and fe.f.ready and self.obs_t >= self.t_min
                    and not fe.resolved and sw < self.w_hop):
                self._hop_out()
            idle = self.hop_state in (None, "done")
            if fe.f.ready and self.obs_t >= self.t_min and fe.resolved and idle and sw < self.w_thr:
                self.obs_ok = True
                self._pre = 0.5 * dt  # FiltSim.bus_ctrl counts it down and plans the fly-in
            elif self._pre <= 1.0001 * dt:
                self.obs_fail = True
        if self.hop is not None and self._pre > 0:
            keep = self.flyin_path
            self.flyin_path = self.hop
            try:
                return super().bus_ctrl()
            finally:
                self.flyin_path = keep
        return super().bus_ctrl()


_p5_check = safety.Monitor.check


def _check6(self, sim, phase, t, F_raw, grasped, hits):
    why = _p5_check(self, sim, phase, t, F_raw, grasped, hits)
    if why or not isinstance(sim, VisSim) or sim._pre > 0:
        return why
    if sim.obs_fail:
        self.first.setdefault("no_converge", (t, phase))
        return "no_converge"
    if sim.mon_est and not grasped and phase in ("flyin", "settle", "reach", "approach") \
            and sim.filt_est.rej_a_t >= sim.t_anom:
        self.first.setdefault("nav_anom", (t, phase))
        return "nav_anom"
    return why


safety.Monitor.check = _check6

_W = {}


def _init(a):
    for k in range(20):
        try:
            sim = VisSim(GraspCfg(scene=SCENE, flyin=True, hold_s=5.0, mon=True, mon_act=True))
            break
        except ValueError as e:
            if "parse" not in str(e).lower() and "empty" not in str(e).lower():
                raise
            time.sleep(0.3 + 0.3 * k + 0.05 * (os.getpid() % 10))
    sim.closed, sim.slow = True, 0.5
    sim.fkw = dict(sig_a=2e-5, sig_w=a["sig_w"])
    sim.mon_est, sim.k_sig, sim.stale_s = True, a["k_sig"], a["stale"]
    sim.nis_abort, sim.abort_est = 0.0, True  # P5.5 single-message NIS abort off: gated messages instead
    sim.t_min, sim.t_max, sim.w_thr, sim.t_anom = a["t_min"], a["t_max"], np.radians(a["w_thr"]), a["t_anom"]
    sim.rate_hz, sim.lat_s = a["rate"], a["latency"]
    sim.hop_on = a.get("hop", "on") == "on"
    _W.update(sim=sim, a=a)


def _episode(job):
    name, cond, seed, fault = job
    sim, a = _W["sim"], _W["a"]
    sim.cfg.fault_kind = fault
    sim.vk = dict(mp=cond[0] * 1e-3, ma=np.radians(cond[1]), p_out=cond[2], p_flip=cond[3], drop=cond[4],
                  face=a["face"], gate=a["gate"], sym=a["sym"] == "on", t_reinit=a["t_reinit"], r_inf=a["r_inf"])
    c = case_draws(seed, 0.0)
    sim.noise_seed = seed
    t0 = time.time()
    r = sim.run(np.random.default_rng(seed), spin_deg_s=c["spin"], spin_axis=c["axis"])
    fe = sim.filt_est
    out = dict(cond=name, seed=seed, spin=c["spin"], axis=c["axis"], wall_s=round(time.time() - t0, 1),
               **{k: r.get(k) for k in KEEP}, **{"pre_" + k: v for k, v in sim.pre_out.items()},
               allow_max=sim.allow_max, cp_min=sim.cp_min, obs_s=sim.obs_t, obs_ok=sim.obs_ok, fault=fault,
               rej_a_max=fe.rej_a_max, **{"n_" + k: v for k, v in fe.st.items()},
               hop_used=sim.hop_state is not None, **{"hop_" + k: v for k, v in sim.hop_info.items()},
               **{k: r.get(k) for k in ("t_fault", "fault_phase", "fault_mag", "t_detect", "t_abort", "safe_reached",
                                        "t_to_safe")})
    for g, m in sim.m.items():
        n = max(m["n"], 1)
        out.update({f"{g}_err_rms_mm": 1000 * np.sqrt(m["e2"] / n), f"{g}_err_max_mm": 1000 * m["emax"],
                    f"{g}_dv_mm_s": 1000 * m["dv"], f"{g}_sat_frac": m["sat"] / n})
    sc = dict(p=1000.0, v=1000.0, a=np.degrees(1.0), w=np.degrees(1.0))
    for g, m in fe.err.items():
        if m["n"]:
            for key in "pvaw":
                out[f"f_{g}_{key}_rms"] = sc[key] * np.sqrt(m[key + "2"] / m["n"])
                out[f"f_{g}_{key}_max"] = sc[key] * m[key + "m"]
    nis = np.array(fe.nis) if fe.nis else np.zeros((1, 2))
    out["nis_p"], out["nis_a"] = float(np.median(nis[:, 0])), float(np.median(nis[:, 1]))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--conds", nargs="+", required=True, help="name:pos_mm,att_deg,p_out,p_flip,drop_per_min")
    ap.add_argument("--n", type=int, default=25)
    ap.add_argument("--start", type=int, default=0)
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--rate", type=float, default=10.0, help="network messages per second")
    ap.add_argument("--latency", type=float, default=0.15)
    ap.add_argument("--face", type=float, default=1.5, help="error factor with the bar face turned away")
    ap.add_argument("--gate", type=float, default=16.3, help="innovation gate (NIS, 3 dof; 16.3 = 99.9%%)")
    ap.add_argument("--sym", choices=("on", "off"), default="on")
    ap.add_argument("--t-reinit", type=float, default=2.0)
    ap.add_argument("--t-anom", type=float, default=2.0)
    ap.add_argument("--t-min", type=float, default=30.0)
    ap.add_argument("--t-max", type=float, default=1200.0)
    ap.add_argument("--hop", choices=("on", "off"), default="on", help="inspection hop when the bar is not seen")
    ap.add_argument("--w-thr", type=float, default=0.01, help="rate 1-sigma that ends the observation [deg/s]")
    ap.add_argument("--k-sig", type=float, default=2.0)
    ap.add_argument("--stale", type=float, default=10.0,
                    help="lost-track abort [s without an accepted message]; a backstop: the allowance grows while coasting")
    ap.add_argument("--r-inf", type=float, default=3.0,
                    help="factor on the stated sigma given to the filter (tuning series: 3)")
    ap.add_argument("--sig-w", type=float, default=2e-6, help="filter rate random walk [rad/s/sqrt(s)] (tuning: 2e-6)")
    ap.add_argument("--faults", nargs="*", default=[""])
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    conds = [(s.split(":")[0], tuple(float(x) for x in s.split(":")[1].split(","))) for s in a.conds]
    conds = [(name + ("+" + f if f else ""), cond, f) for name, cond in conds for f in a.faults]
    jobs = [(name, cond, TUNE + a.start + i, f) for name, cond, f in conds for i in range(a.n)]
    cfg = {k: v for k, v in vars(a).items() if k not in ("conds", "out")}
    t0 = time.time()
    rows = []
    with Pool(a.workers, initializer=_init, initargs=(cfg,)) as pool:
        for i, r in enumerate(pool.imap(_episode, jobs, chunksize=1), 1):
            rows.append(r)
            if i % 25 == 0 or i == len(jobs):
                print(f"  {i}/{len(jobs)}, {(time.time() - t0) / 60:.1f} min", flush=True)
    summ = []
    for name, cond, _ in conds:
        rs = [r for r in rows if r["cond"] == name]
        s = summarise(name, cond, rs)
        q = lambda k, f=np.median: float(f([r[k] for r in rs if r.get(k) is not None]))  # noqa: E731
        s.update(obs_s=dict(median=q("obs_s"), max=q("obs_s", np.max)), obs_ok=sum(r["obs_ok"] for r in rs),
                 rej_a_max_s=q("rej_a_max", np.max), hops=sum(r["hop_used"] for r in rs),
                 hop_look_s=sorted(round(r["hop_look_s"]) for r in rs if r.get("hop_look_s") is not None),
                 counts={k: int(sum(r.get(k, 0) for r in rs)) for k in ("n_msgs", "n_rej_p", "n_rej_a", "n_flips",
                                                                         "n_outliers", "n_dropouts", "n_restarts",
                                                                         "n_end_flips")},
                 failed=[(r["seed"], r["outcome"], r["abort_reason"], r["abort_phase"], round(1e3 * r["push_dv"], 2))
                         for r in rs if not (r["success"] and not r["collision"] and r["push_dv"] <= 0.002)])
        summ.append(s)
        print(f"\n{name} ({cond[0]} mm x range/2.7, {cond[1]} deg, outliers {cond[2]}, flips {cond[3]}, drops "
              f"{cond[4]}/min): {s['success']}/{s['n']} {s['outcomes']} aborts {s['aborts']} collisions "
              f"{s['collisions']} push max {s['push_max_mm_s']:.2f} mm/s; min true clearance {s['min_clear_m']:.3f} m, "
              f"monitor cp min {s['cp_min_m']:.3f} m, allowance max {s['allow_max_m']:.3f} m")
        print(f"  observation: converged {s['obs_ok']}/{s['n']}, {s['obs_s']['median']:.0f} s median, "
              f"{s['obs_s']['max']:.0f} s max; longest attitude rejection after it {s['rej_a_max_s']:.1f} s; "
              f"inspection hops {s['hops']}, look time at the hop point {s['hop_look_s']} s")
        if "pre_rate_err_deg_s" in s:
            print(f"  at fly-in start: rate err median {s['pre_rate_err_deg_s']['median']:.4f} max "
                  f"{s['pre_rate_err_deg_s']['max']:.4f} deg/s, axis err median {s['pre_axis_err_deg']['median']:.1f} "
                  f"max {s['pre_axis_err_deg']['max']:.1f} deg")
        print(f"  counts {s['counts']}")
        print(f"  {'filter error':10s} {'pos mm':>14s} {'vel mm/s':>14s} {'att deg':>14s} {'rate deg/s':>16s}   (rms median / max)")
        for g in GROUPS:
            if f"f_{g}_p" in s:
                print(f"  {g:10s} " + " ".join(f"{s[f'f_{g}_{k}']['rms']:>6.{d}f} /{s[f'f_{g}_{k}']['max']:>6.{d}f}"
                                               for k, d in (("p", 1), ("v", 2), ("a", 2), ("w", 4))))
        print(f"  failed: {s['failed']}")
    Path(a.out).write_text(json.dumps(dict(cfg=cfg, scene=SCENE, summary=summ, episodes=rows), indent=1, default=float))
    print("wrote", a.out)


if __name__ == "__main__":
    main()
