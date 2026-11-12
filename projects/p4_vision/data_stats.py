"""P4.3 groundwork: how much does the bar pose vary in the bus frame across demos_v1, per
phase, and within an episode? Sets the "predict the mean" floor a pose estimator must beat.
python data_stats.py <demo_dir> [n_episodes]"""
import sys
from pathlib import Path

import numpy as np

PH = ("settle", "reach", "approach", "close", "hold")
d = Path(sys.argv[1])
files = sorted(d.glob("ep_*.npz"))[: int(sys.argv[2]) if len(sys.argv) > 2 else None]
P, R, PHS, EP = [], [], [], []
spin = []
for k, f in enumerate(files):
    z = np.load(f)
    pr = z["priv"]
    P.append(pr[:, 0:3])
    R.append(pr[:, 3:9])
    PHS.append(z["phase"])
    EP.append(np.full(len(pr), k))
    spin.append(np.linalg.norm(pr[:, 9:12], axis=1).mean())
P, R, PHS, EP = map(np.concatenate, (P, R, PHS, EP))
print(f"{len(files)} episodes, {len(P)} frames; client rate median {np.degrees(np.median(spin)):.2f} deg/s")
mu = P.mean(0)
print("bar position in bus frame: mean", np.round(mu, 3), "std", np.round(P.std(0) * 1000, 1), "mm")


def ang(r6, ref):
    a = r6[:, 0:3] / np.linalg.norm(r6[:, 0:3], axis=1, keepdims=True)
    b = r6[:, 3:6] - (r6[:, 3:6] * a).sum(1, keepdims=True) * a
    b /= np.linalg.norm(b, axis=1, keepdims=True)
    Rm = np.stack([a, b, np.cross(a, b)], -1)
    rel = np.einsum("nji,jk->nik", Rm, ref)
    return np.degrees(np.arccos(np.clip((np.trace(rel, axis1=1, axis2=2) - 1) / 2, -1, 1)))


r0 = R.mean(0)
a = r0[:3] / np.linalg.norm(r0[:3])
b = r0[3:] - (r0[3:] @ a) * a
b /= np.linalg.norm(b)
Rref = np.stack([a, b, np.cross(a, b)], -1)
A = ang(R, Rref)
print("mean-pose predictor error (all frames): position median %.1f mm, p95 %.1f mm; rotation median %.2f deg, p95 %.2f deg"
      % (np.median(np.linalg.norm(P - mu, axis=1)) * 1000, np.percentile(np.linalg.norm(P - mu, axis=1), 95) * 1000,
         np.median(A), np.percentile(A, 95)))
for p in range(5):
    m = PHS == p
    e = np.linalg.norm(P[m] - mu, axis=1) * 1000
    print(f"  {PH[p]:8s} frames {m.sum():6d}: |bar - mean| median {np.median(e):.1f} mm, p95 {np.percentile(e, 95):.1f} mm; "
          f"rot median {np.median(A[m]):.2f} deg")
# within-episode motion (how much the bar moves in the bus frame during one episode)
w = [np.linalg.norm(P[EP == k] - P[EP == k][0], axis=1).max() * 1000 for k in range(len(files))]
print("within-episode bar motion in bus frame: median %.1f mm, max %.1f mm" % (np.median(w), np.max(w)))
# bar position at the end of settle (where the classical grasp sets its IK goal)
s_end = [P[(EP == k) & (PHS == 0)][-1] for k in range(len(files))]
print("bar at end of settle, std across episodes:", np.round(np.std(s_end, 0) * 1000, 1), "mm")
