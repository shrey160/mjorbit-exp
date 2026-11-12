"""P4.3 diagnosis: how much of the estimator's error is the per-episode camera/lidar mount
randomisation (+-1 cm, +-1 deg), which no image can reveal?

From the privileged labels: bar pose in the bus frame (priv 0:9) and in the bus camera (21:30)
and lidar (30:39) frames give each sensor's actual pose on the bus, per episode (fixed to the bus
body). Nominal mount = the median over episodes. Floor = error of a PERFECT in-sensor estimate
composed with the nominal mount, i.e. what an estimator that cannot see the mount error gets.
python mount_floor.py <pack_dir>"""
import sys
from pathlib import Path

import numpy as np

PH = ("settle", "reach", "approach", "close", "hold")


def R6(r):
    a = r[:, 0:3] / np.linalg.norm(r[:, 0:3], axis=1, keepdims=True)
    b = r[:, 3:6] - (r[:, 3:6] * a).sum(1, keepdims=True) * a
    b /= np.linalg.norm(b, axis=1, keepdims=True)
    return np.stack([a, b, np.cross(a, b)], -1)


def ang(R):
    return np.degrees(np.arccos(np.clip((np.trace(R, axis1=-2, axis2=-1) - 1) / 2, -1, 1)))


d = Path(sys.argv[1])
pv = np.load(d / "priv.npy")
ep, ph = np.load(d / "ep.npy"), np.load(d / "phase.npy")
pbb, Rbb = pv[:, 0:3].astype(np.float64), R6(pv[:, 3:9].astype(np.float64))
neps = ep.max() + 1
for name, s in (("bus camera", 21), ("lidar", 30)):
    pcb, Rcb = pv[:, s:s + 3].astype(np.float64), R6(pv[:, s + 3:s + 9].astype(np.float64))
    Rbc = Rbb @ np.transpose(Rcb, (0, 2, 1))  # sensor orientation in the bus frame
    pbc = pbb - np.einsum("nij,nj->ni", Rbc, pcb)  # sensor position in the bus frame
    # per-episode mount (first settle frame), nominal = median over episodes
    first = np.array([np.flatnonzero(ep == k)[0] for k in range(neps)])
    pm, Rm = pbc[first], Rbc[first]
    p_nom = np.median(pm, 0)
    U, _, Vt = np.linalg.svd(np.median(Rm, 0))
    R_nom = U @ Vt
    dpos = np.linalg.norm(pm - p_nom, axis=1) * 1000
    drot = ang(np.transpose(Rm, (0, 2, 1)) @ R_nom)
    # check the mount is fixed within an episode
    drift = np.linalg.norm(pbc - pm[ep], axis=1) * 1000
    print(f"{name}: mount offset per episode |dp| median {np.median(dpos):.1f} mm (max {dpos.max():.1f}), "
          f"rotation median {np.median(drot):.2f} deg (max {drot.max():.2f}); within-episode drift max {drift.max():.2f} mm")
    # floor: perfect in-sensor estimate, nominal mount
    p_hat = p_nom + np.einsum("ij,nj->ni", R_nom, pcb)
    e = np.linalg.norm(p_hat - pbb, axis=1) * 1000
    out = [f"all {np.median(e):.1f}/{np.percentile(e, 95):.1f}"]
    for k, n in enumerate(PH):
        m = ph == k
        out.append(f"{n} {np.median(e[m]):.1f}/{np.percentile(e[m], 95):.1f}")
    print(f"  floor with a perfect {name} estimate and the nominal mount (median/p95 mm): " + "; ".join(out))
    va = ep >= 450
    print(f"  validation episodes only: median {np.median(e[va]):.1f} mm; approach {np.median(e[va & (ph == 2)]):.1f} mm")
