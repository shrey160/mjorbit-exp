"""P6.1 debug: one episode's observation phase, printed every 20 s (bar seen, filter rate sigma, true
rate error, spin rate and the bar face's best view angle so far).
python dbg_obs.py <seed> [t_max]"""
import sys

import numpy as np

import vis_eval as V
from evaluate_e3 import case_draws

seed = int(sys.argv[1])
t_max = float(sys.argv[2]) if len(sys.argv) > 2 else 300.0
a = dict(k_sig=2.0, stale=10.0, t_min=30.0, t_max=t_max, w_thr=0.01, t_anom=2.0, rate=10.0, latency=0.15, sig_w=2e-6,
         hop="on")
V._init(a)
sim = V._W["sim"]
sim.vk = dict(mp=0.01, ma=np.radians(1.0), p_out=0.02, p_flip=0.05, drop=1.0, face=1.5, gate=16.3, sym=True,
              t_reinit=2.0, r_inf=3.0)
sim.noise_seed = seed
c = case_draws(seed, 0.0)
print("spin", c["spin"], "axis", c["axis"])
orig = V.VisEst._measure
st = dict(cmax=-1.0, nvis=0, n=0)


def meas(self, rho, Rc):
    cc = float((Rc.T @ -rho) @ V.BAR_N) / max(float(np.linalg.norm(rho)), 1e-9)
    st["cmax"], st["n"], st["nvis"] = max(st["cmax"], cc), st["n"] + 1, st["nvis"] + (cc > V.COS_VIS)
    return orig(self, rho, Rc)


V.VisEst._measure = meas
ob = V.VisSim.bus_ctrl


def bus(self):
    k = self.filt_est.k
    if self._pre > 0 and k % 2000 == 0 and self.filt_est.f.ready:
        fe = self.filt_est
        _, _, _, wc = self._truth()
        pc, Rc, _, _ = self._truth()
        ps = self.kd.xpos[self.bid_chaser]
        print(f"t {k * self.cfg.ctrl_dt:5.0f} s  hop {self.hop_state}  range {np.linalg.norm(ps - pc):.2f} m  "
              f"resolved {fe.resolved} flips {fe.st['end_flips']}  bar_seen {fe.bar_seen}  sig_w {np.degrees(fe.f.sigma()[3]):.4f}  "
              f"rate err {np.degrees(np.linalg.norm(fe.f.w - wc)):.4f} deg/s  |w| {np.degrees(np.linalg.norm(wc)):.3f}  "
              f"cos max {st['cmax']:.2f}  vis {st['nvis']}/{st['n']}", flush=True)
    return ob(self)


V.VisSim.bus_ctrl = bus
r = sim.run(np.random.default_rng(seed), spin_deg_s=c["spin"], spin_axis=c["axis"])
print(r.get("outcome"), r.get("abort_reason"), "obs", sim.obs_t, sim.obs_ok, sim.hop_info, "push", r.get("push_dv"))
_, Rc, _, _ = sim._truth()
print("attitude error at end, deg", np.degrees(V._ang(sim.filt_est.f.R @ Rc.T)))
