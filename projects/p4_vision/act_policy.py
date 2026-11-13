"""P4.4e (SPEC_4 4.4): a LeRobot ACT checkpoint as a GraspSim.policy.

The demos (gen_demos.py, to_lerobot.py) are 10 Hz frames: wrist, bus (192 -> 128 area resize)
and lidar (to_lerobot.pack_lidar) images, the 29-D state, and the expert grip target in the bus
frame (3 + rot6) with a close flag. GraspSim calls the policy every 10 ms control step in reach
and approach, so:

  - every 0.1 s (on the demos' 10 Hz grid, t = 0, 0.1, ...) it renders the cameras, builds the
    state like gen_demos.Recorder and predicts a chunk of H actions (frames f .. f+H-1);
  - temporal ensembling (ACT, Zhao et al. 2023): the action for frame f is the weighted mean of
    every stored chunk's prediction for f, weights exp(-m * i), i = 0 for the oldest chunk;
  - between frames the bus-frame target moves linearly (rotation: slerp) from the last commanded
    target to the ensembled target for frame f+1, so the commanded target is continuous (GraspSim
    differentiates it for the impedance reference velocity);
  - the bus-frame target is mapped to the world at the current bus pose each control step;
  - close: the ensembled close flag for frame f > 0.5 during approach (GraspSim acts on it
    one step later and then runs its own close and hold).

ens="none" executes only the newest chunk (no averaging); with replan=k it predicts every k frames
(chunk execution, ACT n_action_steps = k), otherwise every frame (receding horizon).
Runs in the LeRobot env, which also imports mjorbit from simulator/src (both Python 3.12,
mujoco 3.7.0): see eval_act.py.
"""
from __future__ import annotations

import cv2
import numpy as np
import torch
from scipy.spatial.transform import Rotation, Slerp

from gen_demos import ARM_Q, GRIP_Q, rot6
from to_lerobot import TASK, pack_lidar

FPS_DT = 0.1


def rot6_to_R(r6):
    a, b = r6[0:3], r6[3:6]
    x = a / np.linalg.norm(a)
    b = b - (x @ b) * x
    y = b / np.linalg.norm(b)
    return np.stack([x, y, np.cross(x, y)], -1)


class ACTGraspPolicy:
    def __init__(self, ckpt, rig, device="cpu", ens="temporal", m=0.01, close_thr=0.5, replan=1, shadow=False, expert_reach=False):
        from lerobot.policies.act.modeling_act import ACTPolicy
        from lerobot.policies.factory import make_pre_post_processors
        self.rig, self.device, self.ens, self.m, self.close_thr = rig, device, ens, m, close_thr
        self.expert_reach = expert_reach
        self.shadow = shadow  # diagnostic: execute the expert, only log the ACT prediction (live open-loop error)
        self.replan = replan  # frames between predictions (1 = every 0.1 s; ens "none" + 10 = execute 10 of each chunk)
        self.net = ACTPolicy.from_pretrained(ckpt).to(device).eval()
        self.pre, self.post = make_pre_post_processors(
            self.net.config, pretrained_path=ckpt, preprocessor_overrides={"device_processor": {"device": device}})
        self.H = self.net.config.chunk_size
        self.reset()

    def reset(self, rng=None):
        self.chunks = {}  # frame index of the chunk's first action -> (H, 10) bus-frame actions
        self.frame = None  # current 10 Hz frame index
        self.cmd = None  # last commanded bus-frame target (p, R)
        self.seg = None  # (p0, R0, p1, R1, t0) segment of the current frame
        self.close = False
        self.last = None
        self.log = []  # (t, phase, |target - expert| m, deg, close flag, expert close test)
        self.n_pred = 0

    # ---------------------------------------------------------------- observation
    def _obs(self, sim):
        kd = sim.kd
        Rs, ps = kd.xmat[sim.bid_chaser].reshape(3, 3), kd.xpos[sim.bid_chaser]
        ws = Rs @ kd.qvel[3:6]
        vs = kd.qvel[0:3]
        pg, Rg = sim.site_pose(sim.sid_grip)
        vg, wg = sim.site_vel(sim.sid_grip)
        v_rel = Rs.T @ (vg - vs - np.cross(ws, pg - ps))
        w_rel = Rs.T @ (wg - ws)
        state = np.concatenate([kd.qpos[ARM_Q], kd.qvel[6:12], kd.qpos[GRIP_Q],
                                Rs.T @ (pg - ps), rot6(Rs.T @ Rg), v_rel, w_rel]).astype(np.float32)
        im = self.rig.render(kd)
        imgs = {"wrist": im["wrist_cam"],
                "bus": cv2.resize(im["bus_cam"], (128, 128), interpolation=cv2.INTER_AREA),
                "lidar": pack_lidar(self.rig.lidar(kd))}
        b = {f"observation.images.{k}": torch.from_numpy(np.ascontiguousarray(v)).permute(2, 0, 1)
             .float().div(255.0)[None] for k, v in imgs.items()}
        b["observation.state"] = torch.from_numpy(state)[None]
        b["task"] = [TASK]
        return b

    def _predict(self, sim, f):
        with torch.no_grad():
            chunk = self.post(self.net.predict_action_chunk(self.pre(self._obs(sim))))
        self.chunks[f] = chunk[0].float().cpu().numpy()
        self.n_pred += 1
        for k in [k for k in self.chunks if k + self.H <= f]:
            del self.chunks[k]

    def _action(self, f):
        """Ensembled 10-D action for frame f (None if no chunk covers it)."""
        if self.ens == "none":
            k = max(self.chunks)
            return self.chunks[k][min(f - k, self.H - 1)]
        ks = sorted(k for k in self.chunks if k <= f < k + self.H)
        if not ks:
            return None
        w = np.exp(-self.m * np.arange(len(ks)))
        A = np.stack([self.chunks[k][f - k] for k in ks])
        a = (w[:, None] * A).sum(0) / w.sum()
        # rotation: average the rot6 rows then re-orthonormalise (rot6_to_R): fine for the
        # sub-degree spread between chunks
        return a

    # ---------------------------------------------------------------- policy
    def __call__(self, sim, t, phase, t_phase):
        kd = sim.kd
        Rs, ps = kd.xmat[sim.bid_chaser].reshape(3, 3), kd.xpos[sim.bid_chaser]
        if self.expert_reach and phase == "reach":
            # hybrid diagnostic: the classical reach (true pose, as the oracle); ACT from approach on.
            # The commanded target is kept so ACT's first segment starts where the expert left off.
            pe, Re = sim.expert_suggest(t, phase, t_phase)
            self.cmd = (Rs.T @ (pe - ps), Rs.T @ Re)
            self.last = (pe, Re)
            return pe, Re, False
        f = int(np.floor(t / FPS_DT + 1e-6))
        if f != self.frame:
            self.frame = f
            if not self.chunks or f - max(self.chunks) >= self.replan:
                self._predict(sim, f)
            a0, a1 = self._action(f), self._action(f + 1)
            if self.cmd is None:
                self.cmd = (a0[:3].copy(), rot6_to_R(a0[3:9]))
            self.close = bool(a0[9] > self.close_thr)
            self.seg = (self.cmd[0].copy(), self.cmd[1].copy(), a1[:3].copy(), rot6_to_R(a1[3:9]), f * FPS_DT)
            # log ACT's own target for this frame (the label is the expert target at this time)
            pe, Re = sim.expert_suggest(t, phase, t_phase)
            from policies import close_test
            pa, Ra = ps + Rs @ a0[:3], Rs @ rot6_to_R(a0[3:9])
            ang = float(np.degrees(np.linalg.norm(Rotation.from_matrix(Ra @ Re.T).as_rotvec())))
            pg, _ = sim.site_pose(sim.sid_grip)
            self.log.append((t, phase, float(np.linalg.norm(pa - pe)), ang, self.close, bool(close_test(sim)),
                             float(np.linalg.norm(pa - pg)), float(np.linalg.norm(pe - pg))))
        p0, R0, p1, R1, t0 = self.seg
        s = float(np.clip((t - t0) / FPS_DT + 0.1, 0.0, 1.0))  # 0.1: reach the frame target at its last step
        p = (1 - s) * p0 + s * p1
        R = Slerp([0.0, 1.0], Rotation.from_matrix(np.stack([R0, R1])))([s]).as_matrix()[0]
        self.cmd = (p, R)
        p_w, R_w = ps + Rs @ p, Rs @ R
        self.last = (p_w, R_w)
        if self.shadow:
            from policies import close_test
            self.cmd = (Rs.T @ (pe_ := sim.expert_suggest(t, phase, t_phase))[0] - Rs.T @ ps, Rs.T @ pe_[1])
            return pe_[0], pe_[1], phase == "approach" and close_test(sim)
        return p_w, R_w, phase == "approach" and self.close
