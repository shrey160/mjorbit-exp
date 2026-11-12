"""Project 2B step B4b: residual policy on top of CW epoch guidance (SPEC_2 sections 14, 16).

Each 20 s epoch the env runs the 9-state bias filter (1 Hz navigation, nav_filter) and
computes the classical command

    dv_CW = v_req(filter p, v, b; t_go = t_arrive - t) - v_filter   (deadband 2 mm/s)

The policy adds a bounded correction in LVLH:

    dv = dv_CW + res_scale * a,   a in [-1, 1]^3,  res_scale = 0.02 m/s

then the sum is rotated to world, clipped to dv_max per world axis, and flown exactly as
in ManeuverEnv (finite burn, coast). With a = 0 this is "CW epoch nav9 db2", the B3
benchmark, so training starts from the benchmark's behaviour.

Observation (25), all from the filter (never the true state):
  (p - hold)/50, v/0.2, b/2e-5, clip((p - hold)/0.5), sqrt(diag P) for p, v, b
  (scaled by 0.05 m, 1 mm/s, 2e-5), time remaining, previous residual action,
  dv_CW / 0.02.
Reward, success and failure: ManeuverEnv's, on the true state.
"""

from __future__ import annotations

import dataclasses

import numpy as np
import torch
from tensordict import TensorDict

from approach_env import HOLD
from maneuver_env import ManeuverEnv, ManeuverEnvCfg
from guidance import glideslope_dv
from nav_filter import BiasKalman, BiasKalmanTorch, v_required_bias


@dataclasses.dataclass
class ResidualEnvCfg(ManeuverEnvCfg):
    res_scale: float = 0.02  # m/s per LVLH axis at |a| = 1
    t_arrive: float = 480.0  # CW arrival time, s
    cw_deadband: float = 0.002  # m/s
    use_bias: bool = True  # bias-compensated CW
    torch_filter: bool = True  # run the filter in torch on cfg.device (same maths)
    base: str = "cw"  # "cw" (B4b) or "gs": glideslope hybrid (guidance.py, SPEC_2 19)
    gs_k: float = 0.03
    gs_v_max: float = 0.1
    gs_v_min: float = 0.003
    gs_r_switch: float = 3.0
    gs_t_arrive: float = 440.0


class ResidualEnv(ManeuverEnv):
    def __init__(self, cfg: ResidualEnvCfg) -> None:
        self.kf = None
        self._fresh = np.zeros(cfg.num_envs, bool)
        super().__init__(cfg)
        if cfg.torch_filter:
            self.kf = BiasKalmanTorch(cfg.num_envs, self.mean_motion, cfg.nav_pos_sigma,
                                      cfg.nav_vel_sigma, device=cfg.device)
        else:
            self.kf = BiasKalman(cfg.num_envs, self.mean_motion, cfg.nav_pos_sigma,
                                 cfg.nav_vel_sigma)
        self.nav_log = []
        self._fresh[:] = True
        self._cw_dv = np.zeros((cfg.num_envs, 3))
        self.last_total_dv = np.zeros((cfg.num_envs, 3))
        self._obs = self._compute_obs()

    def reset_worlds(self, idx, p0=None, v0=None) -> None:
        super().reset_worlds(idx, p0, v0)
        self._fresh[idx] = True

    # ------------------------------------------------------------ navigation + CW
    def _nav_update(self) -> None:
        cfg = self.cfg
        if self.nav_log:
            if isinstance(self.kf, BiasKalmanTorch):
                self.kf.run_log(cfg.physics_dt, np.stack([e[0] for e in self.nav_log]),
                                np.stack([e[1] for e in self.nav_log]))
            else:
                for u, z, _ in self.nav_log:
                    self.kf.predict(cfg.physics_dt, u=u)
                    self.kf.update(z)
            self.nav_log.clear()
        if self._fresh.any():
            idx = np.flatnonzero(self._fresh)
            z = np.concatenate([self.meas_p, self.meas_v], axis=1)
            self.kf.reset(idx, z)
            self._fresh[idx] = False

    def _cw_command(self) -> np.ndarray:
        cfg = self.cfg
        x = self.kf.x
        p, v, b = x[:, :3], x[:, 3:6], x[:, 6:9]
        t = self.episode_length_buf.cpu().numpy() * cfg.control_dt
        if cfg.base == "gs":
            return glideslope_dv(self.mean_motion, p, v, b if cfg.use_bias else None,
                                 dt=cfg.control_dt, k=cfg.gs_k, v_max=cfg.gs_v_max,
                                 v_min=cfg.gs_v_min, deadband=cfg.cw_deadband,
                                 r_switch=cfg.gs_r_switch, t_arrive=cfg.gs_t_arrive, t=t)
        t_go = np.maximum(cfg.t_arrive - t, cfg.control_dt)
        dv = v_required_bias(self.mean_motion, p, t_go, b if cfg.use_bias else None) - v
        return np.where(np.abs(dv).max(axis=1, keepdims=True) > cfg.cw_deadband, dv, 0.0)

    def _compute_obs(self) -> TensorDict:
        if self.kf is None:  # during base-class construction
            return super()._compute_obs()
        cfg = self.cfg
        self._measure()
        self._nav_update()
        self._cw_dv = self._cw_command()
        x, P = self.kf.x, self.kf.P
        e = x[:, :3] - HOLD
        sd = np.sqrt(np.maximum(np.diagonal(P, axis1=1, axis2=2), 0.0))
        obs = np.concatenate([
            e / cfg.pos_scale, x[:, 3:6] / cfg.vel_scale, x[:, 6:9] / 2e-5,
            np.clip(e / cfg.fine_scale, -1.0, 1.0),
            sd[:, 0:3] / 0.05, sd[:, 3:6] / 1e-3, sd[:, 6:9] / 2e-5,
            self.time_remaining()[:, None], self._last_actions,
            np.clip(self._cw_dv / cfg.res_scale, -5.0, 5.0)], axis=1)
        obs = np.nan_to_num(obs, nan=0.0, posinf=0.0, neginf=0.0)
        t = torch.as_tensor(obs, dtype=torch.float32, device=self.device)
        return TensorDict({"policy": t}, batch_size=[self.num_envs])

    # ------------------------------------------------------------------ step
    def step(self, actions: torch.Tensor):
        cfg = self.cfg
        acts = actions.detach().to("cpu", torch.float64).clamp(-1.0, 1.0).numpy()
        dv_lvlh = self._cw_dv + cfg.res_scale * acts
        C, _, _, _ = self.lvlh_state()
        dv_w = np.clip(np.einsum("nij,nj->ni", C, dv_lvlh), -cfg.dv_max, cfg.dv_max)
        self.last_total_dv = dv_lvlh
        return self.step_world_dv(dv_w, acts)
