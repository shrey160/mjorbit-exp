"""P4.3: the trained bar-pose network as GraspSim.pose_est (classical grasp on an estimated
pose). At 10 Hz while settling, reaching and approaching it renders the cameras and lidar,
estimates the bar pose in the bus frame and smooths it (exponential, alpha per update). At
the close command the estimate is latched: the jaws then cover the bar, and the bus holds the
relative pose, so the latched bus-frame estimate stays valid.

World pose = the servicer's own pose (known to it) composed with the bus-frame estimate. The
client frame follows from the bar pose through the client's known geometry. Velocity
feed-forward uses the bus's own motion (the bus station-keeps on the client, so a point fixed
in the client moves with the bus to first order), so no noisy differentiation is needed.
Each update also logs the true bar pose, for the error report."""
from __future__ import annotations

import numpy as np
import torch

from pose_net import PoseNet, rot6_to_R

ARM_PH = ("settle", "reach", "approach")


class NetPoseEst:
    def __init__(self, ckpt, rig, every=10, alpha=0.5, device="cpu"):
        self.net = PoseNet()
        self.net.load_state_dict(torch.load(ckpt, map_location=device))
        self.net.eval().to(device)
        self.rig, self.every, self.alpha, self.dev = rig, every, alpha, device
        self.reset()

    def reset(self):
        self.p_b = self.R_b = None  # smoothed bar pose in the bus frame
        self.k = 0
        self.latched = False
        self.log = []  # (t, phase, pos err m, rot err deg) per update
        self.geo = None

    def _geometry(self, sim):
        """Bar site relative to the client body: fixed by the client's (known) design."""
        kd = sim.kd
        Rc, pc = kd.xmat[sim.bid_target].reshape(3, 3), kd.xpos[sim.bid_target]
        pb, Rb = sim.site_pose(sim.sid_bar)
        self.geo = (Rc.T @ (pb - pc), Rc.T @ Rb)

    @torch.no_grad()
    def _infer(self, sim):
        kd = sim.kd
        im = self.rig.render(kd)
        li = self.rig.lidar(kd)
        t = lambda a: torch.from_numpy(np.ascontiguousarray(a))[None].to(self.dev)  # noqa: E731
        q = torch.tensor(sim.data.qpos[sim_arm_q(sim)], dtype=torch.float32)[None].to(self.dev)
        p, r6 = self.net(t(im["wrist_cam"]).permute(0, 3, 1, 2), t(im["bus_cam"]).permute(0, 3, 1, 2),
                         t(li.astype(np.int32)), q)
        return p[0].cpu().numpy().astype(np.float64), r6[0].cpu().numpy().astype(np.float64)

    def update(self, sim, t, phase):
        if self.geo is None:
            self._geometry(sim)
        if self.latched or phase not in ARM_PH:
            if phase in ("close", "hold", "abort") and self.p_b is not None:
                self.latched = True
            return
        self.k += 1
        if self.p_b is not None and (self.k - 1) % self.every:
            return
        p, r6 = self._infer(sim)
        if self.p_b is None:
            self.p_b, self.r6 = p, r6
        else:
            a = self.alpha
            self.p_b = a * p + (1 - a) * self.p_b
            self.r6 = a * r6 + (1 - a) * self.r6
        self.R_b = rot6_to_R(self.r6)
        # error log against the truth (bus frame)
        kd = sim.kd
        Rs, ps = kd.xmat[sim.bid_chaser].reshape(3, 3), kd.xpos[sim.bid_chaser]
        pb, Rb = sim.site_pose(sim.sid_bar)
        e_p = float(np.linalg.norm(self.p_b - Rs.T @ (pb - ps)))
        Rt = Rs.T @ Rb
        e_r = float(np.degrees(np.arccos(np.clip((np.trace(self.R_b.T @ Rt) - 1) / 2, -1, 1))))
        self.log.append((t, phase, e_p, e_r))

    def bar(self, sim):
        """World bar position and client rotation (what grasp_target needs)."""
        kd = sim.kd
        Rs, ps = kd.xmat[sim.bid_chaser].reshape(3, 3), kd.xpos[sim.bid_chaser]
        p_bar = ps + Rs @ self.p_b
        Rc = Rs @ self.R_b @ self.geo[1].T
        return p_bar, Rc

    def client(self, sim):
        kd = sim.kd
        p_bar, Rc = self.bar(sim)
        p_c = p_bar - Rc @ self.geo[0]
        ps = kd.xpos[sim.bid_chaser]
        Rs = kd.xmat[sim.bid_chaser].reshape(3, 3)
        ws = Rs @ sim.data.qvel[3:6]
        vc = sim.data.qvel[0:3] + np.cross(ws, p_c - ps)
        return p_c, Rc, vc, ws


class PalmPoseEst(NetPoseEst):
    """P4.8: the P4.3 estimate while settling and reaching; in approach the bar pose comes from
    the palm camera instead: the arm's own grip-point pose (forward kinematics, taken as
    calibrated) composed with the bar's pose in the grip-point frame from palm_net.PalmNet.
    That image shows the pads and the bar together, so camera-mount errors drop out. Same
    smoothing, latch and interface as NetPoseEst. A palm output outside the trained range
    (more than max_x ahead or max_lat to the side) is ignored for that update."""

    def __init__(self, ckpt, palm_ckpt, rig, every=10, alpha=0.5, device="cpu", max_x=0.20, max_lat=0.06, agree=0.04,
                 use_rot=True):
        super().__init__(ckpt, rig, every=every, alpha=alpha, device=device)
        from palm_net import PalmNet
        self.palm = PalmNet()
        self.palm.load_state_dict(torch.load(palm_ckpt, map_location=device))
        self.palm.eval().to(device)
        self.max_x, self.max_lat, self.agree, self.use_rot = max_x, max_lat, agree, use_rot
        self._phase = None

    def reset(self):
        super().reset()
        self.n_palm = self.n_rej = 0

    def update(self, sim, t, phase):
        self._phase = phase
        super().update(sim, t, phase)

    @torch.no_grad()
    def _infer(self, sim):
        p_v3, r6_v3 = super()._infer(sim)
        if self._phase != "approach":
            return p_v3, r6_v3
        img = torch.from_numpy(np.ascontiguousarray(self.rig.render_palm(sim.kd)))[None].permute(0, 3, 1, 2).to(self.dev)
        p, r6 = self.palm(img)
        p, r6 = p[0].cpu().numpy().astype(np.float64), r6[0].cpu().numpy().astype(np.float64)
        kd = sim.kd
        Rs, ps = kd.xmat[sim.bid_chaser].reshape(3, 3), kd.xpos[sim.bid_chaser]
        pg, Rg = sim.site_pose(sim.sid_grip)
        # Coarse-to-fine gate (13 Nov, dbg_palm.py seed 6000013): with the bar out of its view or
        # the image dark, the palm network still answers "centred, 5-8 cm ahead", and the approach
        # then runs away from the bar. The P4.3 estimate is coarse but bounded (30 mm at a 10 mm
        # mount error), so the palm value is used only where P4.3 puts the bar inside the palm
        # camera's trained view and the two agree within `agree` on depth and across the jaws.
        v3g = Rg.T @ (ps + Rs @ p_v3 - pg)
        ok = (0.0 < v3g[0] < self.max_x and abs(v3g[2]) < self.max_lat
              and abs(p[0] - v3g[0]) < self.agree and abs(p[2] - v3g[2]) < self.agree)
        if not ok:
            self.n_rej += 1
            return p_v3, r6_v3
        self.n_palm += 1
        # along the bar (grip-frame y) a close-up of a uniform bar carries no information
        # (validation: 11 mm median error there, 2 mm across it): keep the P4.3 value on that axis
        p[1] = v3g[1]
        p_b = Rs.T @ (pg + Rg @ p - ps)
        if not self.use_rot:
            return p_b, r6_v3
        R_b = Rs.T @ Rg @ rot6_to_R(r6)
        return p_b, np.concatenate([R_b[:, 0], R_b[:, 1]])


def sim_arm_q(sim):
    from grasp_sim import ARM_Q
    return ARM_Q
