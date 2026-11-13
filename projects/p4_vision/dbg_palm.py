"""P4.8 diagnosis: one closed-loop episode with PalmPoseEst; at each palm update prints the true
bar position in the grip frame, the palm network's output and the v3 estimate in the same frame.
bash run_p4.sh dbg_palm.py <seed> [mount_mm=10] [norot=1]"""
import sys
from pathlib import Path

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "p3_capture"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from cams import CamRig  # noqa: E402
from gen_demos import SCENE  # noqa: E402
from grasp_sim import GraspCfg, GraspSim  # noqa: E402
from pose_est import NetPoseEst, PalmPoseEst  # noqa: E402
from pose_net import rot6_to_R  # noqa: E402

seed = int(sys.argv[1])
mm = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
norot = (sys.argv[3] if len(sys.argv) > 3 else "1") == "1"
D = "/home/shrey31/p4_data"
sim = GraspSim(GraspCfg(scene=SCENE, hold_s=5.0, mon=True, mon_act=True))
rig = CamRig(sim.km, mount_m=mm / 1000, mount_deg=mm / 10)
rows = []


class Dbg(PalmPoseEst):
    def update(self, sim, t, phase):
        self._t = t
        super().update(sim, t, phase)

    @torch.no_grad()
    def _infer(self, sim):
        out = super()._infer(sim)
        if self._phase == "approach":
            kd = sim.kd
            Rs, ps = kd.xmat[sim.bid_chaser].reshape(3, 3), kd.xpos[sim.bid_chaser]
            pg, Rg = sim.site_pose(sim.sid_grip)
            pb, Rb = sim.site_pose(sim.sid_bar)
            true = Rg.T @ (pb - pg)
            p_v3, _ = NetPoseEst._infer(self, sim)
            v3 = Rg.T @ (ps + Rs @ p_v3 - pg)
            img = torch.from_numpy(np.ascontiguousarray(self.rig.render_palm(kd)))[None].permute(0, 3, 1, 2)
            p, r6 = self.palm(img)
            Rt = Rg.T @ Rb
            Rp = rot6_to_R(r6[0].numpy().astype(np.float64))
            ang = np.degrees(np.arccos(np.clip((np.trace(Rp.T @ Rt) - 1) / 2, -1, 1)))
            angI = np.degrees(np.arccos(np.clip((np.trace(Rt) - 1) / 2, -1, 1)))
            rows.append((self._t, true * 1000, p[0].numpy() * 1000, v3 * 1000, ang, angI, float(img.float().mean())))
        return out


est = Dbg(f"{D}/pose_v3/best.pt", f"{D}/palm_net_v1/best.pt", rig, use_rot=not norot)
draw = np.random.default_rng(seed + 12345)
spin, axis = float(draw.uniform(0.0, 2.0)), int(draw.integers(3))
rig.randomize(draw)
est.reset()
sim.pose_est = est
r = sim.run(np.random.default_rng(seed), spin_deg_s=spin, spin_axis=axis)
print(f"seed {seed} spin {spin:.2f} axis {axis}: {r['outcome']} abort {r['abort_reason']} push {r['push_dv'] * 1000:.2f} mm/s")
print("   t     true x/y/z mm        palm x/y/z mm        v3 x/y/z mm       palm rot err, true rel rot deg, img mean")
for k, (t, a, b, c, ang, angI, im) in enumerate(rows):
    if k % 5 == 0 or k > len(rows) - 6:
        print(f" {t:6.1f}  {a[0]:6.1f} {a[1]:6.1f} {a[2]:6.1f}   {b[0]:6.1f} {b[1]:6.1f} {b[2]:6.1f}   {c[0]:6.1f} {c[1]:6.1f} {c[2]:6.1f}   {ang:5.1f} {angI:5.1f}  {im:5.1f}")
