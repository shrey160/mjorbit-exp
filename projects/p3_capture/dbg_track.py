"""Replay E3 missions through the fly-in, logging feed-forward thrust demand, saturation and
tracking error per second (diagnosis of the 'track' false alarms at top spin)."""
import json, sys
from multiprocessing import Pool
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
import evaluate_e3 as E

D = Path(__file__).parent

def run(c):
    import grasp_sim
    log = []
    orig = grasp_sim.GraspSim.bus_ctrl
    def bc(self):
        F, T = orig(self)
        fp = getattr(self, "flyin_path", None)
        if fp is not None and self._flyin_t is not None:
            t = self._flyin_t
            if abs(t - round(t)) < 0.005:
                kd = self.kd
                p_ref = self._bus_ref[0]
                ep = float(np.linalg.norm(kd.xpos[self.bid_chaser] - p_ref))
                Rc = kd.xmat[self.bid_target].reshape(3, 3); pc = kd.xpos[self.bid_target]
                w_c = Rc @ self.data.qvel[17:20]
                _, _, a_ff, _, _ = fp.reference(t, pc, self.data.qvel[14:17], Rc, w_c, self.R_rel0)
                leg = int(np.searchsorted(np.cumsum(fp.T), t))
                Rs = kd.xmat[self.bid_chaser].reshape(3, 3)
                from scipy.spatial.transform import Rotation
                ea = float(np.degrees(np.linalg.norm(Rotation.from_matrix(self._bus_ref[2] @ Rs.T).as_rotvec())))
                wn = float(np.degrees(np.linalg.norm(w_c)))
                log.append((round(t), leg, round(float(np.max(np.abs(self.m_serv * a_ff))), 3),
                            round(float(np.max(np.abs(self._F_raw))), 3), round(ep, 3), round(ea, 2), round(wn, 3)))
        return F, T
    grasp_sim.GraspSim.bus_ctrl = bc
    r = E._mission(c)
    worst = log[-40::4]
    ffmax = max(log, key=lambda x: x[2]) if log else None
    return dict(seed=c["seed"], spin=round(c["spin"], 3), axis=c["axis"], outcome=E.classify(c, r),
                reason=r.get("g_abort_reason"), m=None, legs=None, ff_peak=ffmax, worst_ep=worst,
                n_sat=sum(x[3] > 0.75 for x in log), n=len(log))

if __name__ == "__main__":
    seeds = [int(s) for s in sys.argv[1:]]
    cases = [c for c in json.loads((D / "e3_handoff_validation.json").read_text())["cases"] if c["seed"] in seeds]
    with Pool(len(cases)) as p:
        for o in p.imap_unordered(run, cases):
            print(json.dumps(o), flush=True)
