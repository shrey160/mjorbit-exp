"""P5.2 step 1: why does the predicted-clearance monitor (clear_pred) abort fly-ins?

One seed at a time, bus on the relative-motion filter (as filt_eval.py --loop closed) or, with
--truth, on the true client state. Through the fly-in it logs, every 0.5 s:
  clr      true clearance now (servicer radius and margin taken off)
  cp       the monitor's predicted free-drift clearance (true state), tau = time ahead of its minimum
  cp_ref   the same prediction for a bus exactly on its reference (true client state): the margin
           the planned path has by itself
  cp_v     prediction from the true position but the reference velocity (isolates velocity error)
  ep, ev   bus position / velocity error against the true-state reference
  fp, fv   filter position / velocity error

python dbg_clear.py 10 0.5 5000023 [--truth]
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parent))
from filt_eval import FiltSim  # noqa: E402
from evaluate_e3 import case_draws  # noqa: E402
from grasp_sim import GraspCfg  # noqa: E402
from safety import box_clearance  # noqa: E402


def coast(sim, r0, v, Rc, w, horizon=30.0):
    best, tb = np.inf, 0.0
    for tau in np.arange(0.0, horizon + 1e-9, 1.0):
        Rt = Rotation.from_rotvec(w * tau).as_matrix()
        c = box_clearance((Rt @ Rc).T @ (r0 + v * tau), sim.boxes)
        if c < best:
            best, tb = c, tau
    return best - sim.r_serv - sim.cfg.clear_margin, tb


class Trace(FiltSim):
    rows: list = []

    def _hook(self, loc):
        super()._hook(loc)
        t, ph = loc["t"], loc["phase"]
        if ph != "flyin" or self._flyin_t is None or int(round(t * 100)) % 50:
            return
        kd, d = self.kd, self.data
        pc, Rc, vc, wc = self._truth()
        ps, vs = kd.xpos[self.bid_chaser].copy(), d.qvel[0:3].copy()
        p_ref, v_ref, _, _, _ = self.flyin_path.reference(self._flyin_t, pc, vc, Rc, wc, self.R_rel0, k=self._flyin_k)
        cp, tau = coast(self, ps - pc, vs - vc, Rc, wc)
        cpr, _ = coast(self, p_ref - pc, v_ref - vc, Rc, wc)
        cpv, _ = coast(self, ps - pc, v_ref - vc, Rc, wc)
        fe = self.filt_est
        fp = fv = np.nan
        if fe is not None and fe.out is not None:
            fp = np.linalg.norm(fe.out[0] - pc)
            mv = kd.subtree_linvel[self.bid_chaser]
            fv = np.linalg.norm((fe.out[2] - mv) - (vc - mv))
        leg = int(np.searchsorted(np.cumsum(self.flyin_path.T), self._flyin_t, side="right")) + 1
        self.rows.append((t, self._flyin_t, leg, self._flyin_k, np.linalg.norm(ps - pc), self.clearance_now(), cp, tau,
                          cpr, cpv, 1e3 * np.linalg.norm(ps - p_ref), 1e3 * np.linalg.norm(vs - v_ref), 1e3 * fp,
                          1e3 * fv, self._pre > 0))


def main():
    a = [x for x in sys.argv[1:] if not x.startswith("--")]
    truth = "--truth" in sys.argv
    mp, ma, seeds = float(a[0]), float(a[1]), [int(s) for s in a[2:]]
    pre = 0.0 if truth else 30.0
    sim = Trace(GraspCfg(scene="p3e_espa_arm2_scene.xml", flyin=True, hold_s=5.0 + (pre + 20.0 if not truth else 0.0),
                         mon=True, mon_act=True))
    sim.closed, sim.preroll = not truth, 30.0
    hdr = (f"{'t':>6s} {'path':>6s} leg {'k':>4s} {'range':>6s} {'clr':>6s} {'cp':>6s} {'tau':>4s} {'cp_ref':>6s} "
           f"{'cp_v':>6s} {'ep mm':>6s} {'ev':>5s} {'fp mm':>6s} {'fv':>5s}")
    for seed in seeds:
        c = case_draws(seed, 0.0)
        sim.meas, sim.noise_seed, sim.rows = (mp * 1e-3, np.radians(ma)), seed, []
        Trace.rows = sim.rows
        r = sim.run(np.random.default_rng(seed), spin_deg_s=c["spin"], spin_axis=c["axis"])
        R = np.array(sim.rows, float)
        print(f"\n=== seed {seed} {'truth' if truth else f'filter {mp} mm {ma} deg'}: spin {c['spin']:.2f} deg/s axis "
              f"{c['axis']}, outcome {r['outcome']}, abort {r.get('abort_reason')}@{r.get('abort_phase')}, "
              f"t_detect {r.get('t_detect')}, path legs {[round(x) for x in sim.flyin_path.T]} s, "
              f"pts {[np.round(p, 2).tolist() for p in sim.flyin_path.pts]}, pre {sim.pre_out}")
        R = R[R[:, 14] == 0]
        i_min = int(np.argmin(R[:, 6]))
        i_ref = int(np.argmin(R[:, 8]))
        print(f"min cp {R[i_min, 6]:.3f} at t {R[i_min, 0]:.1f}; min cp_ref {R[i_ref, 8]:.3f} at t {R[i_ref, 0]:.1f}; "
              f"min true clr {R[:, 5].min():.3f}; cp - cp_ref: min {np.min(R[:, 6] - R[:, 8]):.3f}; "
              f"max ep {R[:, 10].max():.0f} mm, max ev {R[:, 11].max():.1f} mm/s")
        print(hdr)
        show = sorted(set(list(range(0, len(R), 20)) + list(range(max(len(R) - 12, 0), len(R))) +
                          list(range(max(i_ref - 3, 0), min(i_ref + 4, len(R))))))
        for i in show:
            x = R[i]
            print(f"{x[0]:6.1f} {x[1]:6.1f} {int(x[2]):3d} {x[3]:4.2f} {x[4]:6.2f} {x[5]:6.3f} {x[6]:6.3f} {x[7]:4.0f} "
                  f"{x[8]:6.3f} {x[9]:6.3f} {x[10]:6.0f} {x[11]:5.1f} {x[12]:6.0f} {x[13]:5.2f}")


if __name__ == "__main__":
    main()
