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


def sim_arm_q(sim):
    from grasp_sim import ARM_Q
    return ARM_Q
