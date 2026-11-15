"""P5.4b: trace of the fly-in with the bus and the monitors on the link filter (single process).

Link as in cam_eval.py (position error that does not grow with range, 60 s correlation), scene
without cameras (the arm is on the truth; only the fly-in is of interest). Every 5 s it prints
  clr     true clearance now          cp      the monitor's value (estimate, allowance taken off)
  allow   the allowance               cp_path the same prediction for a bus exactly on the planned
                                              path (estimated client rate), no allowance
  ep      bus error against the reference built from the estimate
  fp, fv  filter position / velocity error

python dbg_fly.py <pos_mm> <att_deg> <rate_hz> <latency_s> <seeds...> [--lim] [--k-sig 2]
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parent))
import filt_eval  # noqa: E402
from filt_eval import FiltSim  # noqa: E402
from evaluate_e3 import case_draws  # noqa: E402
from grasp_sim import GraspCfg  # noqa: E402
import safety  # noqa: E402
from safety import box_clearance  # noqa: E402

_pc0 = safety.Monitor.predicted_clearance


def _pc(self, sim):
    cp = _pc0(self, sim)  # called with the estimate swapped in when the monitors are armed
    sim._cp_last = cp
    fp, ft = getattr(sim, "flyin_path", None), getattr(sim, "_flyin_t", None)
    sim._cp_path = np.nan
    if fp is not None and ft is not None:
        w_b = np.array(sim.data.qvel[17:20])
        r_b, rd_b, _ = fp.body_ref(ft)
        v = np.cross(w_b, r_b) + sim._flyin_k * rd_b
        best = np.inf
        for tau in np.arange(0.0, 30.0 + 1e-9, 1.0):
            best = min(best, box_clearance(Rotation.from_rotvec(w_b * tau).as_matrix().T @ (r_b + v * tau), sim.boxes))
        sim._cp_path = best - sim.r_serv - sim.cfg.clear_margin
    return cp


safety.Monitor.predicted_clearance = _pc


def make(base):
    class Trace(base):
        rows: list = []

        def _hook(self, loc):
            super()._hook(loc)
            t, ph = loc["t"], loc["phase"]
            if ph != "flyin" or self._flyin_t is None or self._pre > 0 or int(round(t * 100)) % 500:
                return
            kd, d = self.kd, self.data
            pc, Rc, vc, wc = self._truth()
            ps = kd.xpos[self.bid_chaser].copy()
            fe = self.filt_est
            pe, Re, ve, we = fe.out
            p_ref, v_ref, _, _, _ = self.flyin_path.reference(self._flyin_t, pe, ve, Re, we, self.R_rel0, k=self._flyin_k)
            self.rows.append((t, self._flyin_t, self._flyin_k, np.linalg.norm(ps - pc), self.clearance_now(),
                              getattr(self, "_cp_last", np.nan), self._cp_allow, getattr(self, "_cp_path", np.nan),
                              1e3 * np.linalg.norm(ps - p_ref), 1e3 * np.linalg.norm(d.qvel[0:3] - v_ref),
                              1e3 * np.linalg.norm(pe - pc), 1e3 * np.linalg.norm(ve - vc),
                              np.degrees(np.linalg.norm(we - wc))))
    return Trace


def main():
    argv = sys.argv[1:]
    k_sig = float(argv[argv.index("--k-sig") + 1]) if "--k-sig" in argv else 2.0
    lim = "--lim" in argv
    a = [x for i, x in enumerate(argv) if not x.startswith("--") and (i == 0 or argv[i - 1] != "--k-sig")]
    mp, ma, hz, lat, seeds = float(a[0]), float(a[1]), float(a[2]), float(a[3]), [int(s) for s in a[4:]]
    base = FiltSim
    if lim:
        from fly_lim import LimSim
        base = LimSim
    sim = make(base)(GraspCfg(scene="p3e_espa_arm2_scene.xml", flyin=True, hold_s=5.0, mon=True, mon_act=True))
    sim.closed, sim.preroll, sim.slow, sim.latency = True, 30.0, 0.9, lat
    sim.ekw = dict(tau_b=60.0, r_ref=1e9, every=max(int(round(1.0 / (hz * sim.cfg.ctrl_dt))), 1))
    sim.mon_est, sim.k_sig = True, k_sig
    # --tuned: the filter noise levels of filt_eval.py (cam_eval.py ran on RelFilter's looser defaults)
    if "--tuned" in argv or "--bias" in argv:
        sim.fkw = dict(sig_a=2e-5, sig_w=5e-6)
    if "--bias" in argv:
        sim.fkw.update(bias_frac=0.9, bias_tau=60.0)
    if "--tide" in argv:
        sim.ekw["tide"] = True
    print("filter", sim.fkw, "tide", sim.ekw.get("tide", False), flush=True)
    hdr = (f"{'t':>6s} {'path':>6s} {'k':>4s} {'range':>6s} {'clr':>6s} {'cp':>6s} {'allow':>6s} {'cp_path':>7s} "
           f"{'ep mm':>6s} {'ev':>5s} {'fp mm':>6s} {'fv':>5s} {'fw':>6s}")
    for seed in seeds:
        c = case_draws(seed, 0.0)
        sim.meas, sim.noise_seed, sim.rows = (mp * 1e-3, np.radians(ma)), seed, []
        r = sim.run(np.random.default_rng(seed), spin_deg_s=c["spin"], spin_axis=c["axis"])
        fp = sim.flyin_path
        print(f"\n=== seed {seed} {'speed limit' if lim else 'lengthened legs'}: spin {c['spin']:.2f} deg/s axis {c['axis']}, "
              f"outcome {r['outcome']}, abort {r.get('abort_reason')}@{r.get('abort_phase')}, push "
              f"{1e3 * r['push_dv']:.2f} mm/s, phase_end {r.get('phase_end')}, flyin_s {r.get('flyin_s')}, "
              f"cp min {sim.cp_min:.3f}, allow max {sim.allow_max:.3f}")
        print(f"path legs {[round(x) for x in fp.T]} s, pts {[np.round(p, 2).tolist() for p in fp.pts]}")
        print(f"pre {sim.pre_out}")
        R = np.array(sim.rows, float)
        if not len(R):
            continue
        i = int(np.nanargmin(R[:, 5]))
        print(f"min cp {R[i, 5]:.3f} at t {R[i, 0]:.0f} (path alone there {R[i, 7]:.3f}, allowance {R[i, 6]:.3f}); "
              f"min cp_path {np.nanmin(R[:, 7]):.3f}; max ep {R[:, 8].max():.0f} mm; max fv {R[:, 11].max():.2f} mm/s")
        print(hdr)
        step = max(len(R) // 40, 1)
        for j in sorted(set(list(range(0, len(R), step)) + list(range(max(i - 4, 0), min(i + 3, len(R)))) + [len(R) - 1])):
            x = R[j]
            print(f"{x[0]:6.0f} {x[1]:6.1f} {x[2]:4.2f} {x[3]:6.2f} {x[4]:6.3f} {x[5]:6.3f} {x[6]:6.3f} {x[7]:7.3f} "
                  f"{x[8]:6.0f} {x[9]:5.1f} {x[10]:6.0f} {x[11]:5.2f} {x[12]:6.4f}")


if __name__ == "__main__":
    main()
