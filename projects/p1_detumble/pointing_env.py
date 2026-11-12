"""Project 1B: point to a requested attitude and hold it (mjorbit_warp, rsl-rl >= 5 VecEnv).

Same spacecraft, actuator and stepping as Project 1A (DetumbleEnv); only the task changes.

  observation (9) = [e (3), body w / 0.3 (3), previous action (3)]
      e = vector part of the body-frame error quaternion q_e = conj(q_cur) * q_target,
          sign-fixed so its scalar part is >= 0. |e| = sin(angle/2); e points along the
          body-frame axis to rotate about.
  action (3)      = normalized body torques, tau = tau_max * clip(a, -1, 1), held 0.1 s
  episode         = 40 s (400 control steps)
  success         = angle error < 5 deg and |w| < 0.01 rad/s for the final 5 s
  reset           = random attitude, small random spin, target = current attitude rotated
                    by a random axis and an angle in [err_min, err_max]
"""

from __future__ import annotations

import dataclasses

import numpy as np
import torch
from tensordict import TensorDict

from detumble_env import DetumbleEnv, DetumbleEnvCfg


# --------------------------------------------------------------- quaternions (w, x, y, z)
def qmul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a[..., 0], a[..., 1], a[..., 2], a[..., 3]
    bw, bx, by, bz = b[..., 0], b[..., 1], b[..., 2], b[..., 3]
    return np.stack([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ], axis=-1)


def qconj(q: np.ndarray) -> np.ndarray:
    return q * np.array([1.0, -1.0, -1.0, -1.0])


def qexp(axis: np.ndarray, angle: np.ndarray) -> np.ndarray:
    h = 0.5 * angle[..., None]
    return np.concatenate([np.cos(h), np.sin(h) * axis], axis=-1)


def error_quat(q_cur: np.ndarray, q_tgt: np.ndarray) -> np.ndarray:
    """Body-frame rotation from current to target, sign-fixed to scalar part >= 0."""
    qe = qmul(qconj(q_cur), q_tgt)
    qe = qe / np.linalg.norm(qe, axis=-1, keepdims=True)
    return np.where(qe[..., :1] < 0.0, -qe, qe)


def error_angle(qe: np.ndarray) -> np.ndarray:
    return 2.0 * np.arctan2(np.linalg.norm(qe[..., 1:], axis=-1), np.abs(qe[..., 0]))


@dataclasses.dataclass
class PointingEnvCfg(DetumbleEnvCfg):
    episode_length_s: float = 40.0
    # small residual spin at reset (each axis uniform in +-spin_max)
    spin_mode: str = "box"
    spin_min: float = 0.0
    spin_max: float = 0.02
    # target = current attitude rotated by err in [err_min, err_max] (rad) about a random axis
    err_min: float = 0.0
    err_max: float = np.pi / 6
    success_angle_deg: float = 5.0

    # Reward version. "v1" = original (SPEC_1B section 2); "v2" = reshaped (section 9);
    # "v3" = v1 + fine precision bonus (section 15).
    reward_version: str = "v1"
    # Observation version. "v1" = 9 values; "v2" appends a zoomed copy of the attitude
    # error, clip(e / fine_obs_scale, -1, 1), so the network has high gain near the
    # target (SPEC_1B section 11). Baseline indices 0:6 are unchanged.
    obs_version: str = "v1"
    fine_obs_scale: float = 0.02  # sin(angle/2) ~ 0.02 -> ~2.3 deg saturation

    # v1: attitude error dominates, rate keeps it from overshooting.
    w_att: float = 1.0
    w_rate: float = 0.2
    w_effort: float = 0.02
    w_smooth: float = 0.02
    w_point_bonus: float = 0.5  # angle < 5 deg
    w_rest_bonus: float = 0.5  # angle < 5 deg and |w| < 0.01

    # v2 extras: graded precision bonuses instead of flat thresholds, and a rate
    # penalty only near the target (so fast slews are not penalized).
    v2_coarse_deg: float = 5.0  # width of the coarse precision bonus
    v2_fine_deg: float = 0.5  # width of the fine precision bonus
    w_coarse: float = 0.5
    w_fine: float = 1.0
    v2_rate_gate_deg: float = 15.0  # rate penalty applies within ~this angle
    w_rate_near: float = 0.5
    w_still: float = 0.5  # fine-pointed and still: exp(-(w/0.005)^2)

    # v3 = v1 + fine precision bonus w_fine_v3 * exp(-(deg / v2_fine_deg)^2)
    w_fine_v3: float = 0.5


class PointingEnv(DetumbleEnv):
    """Slew to a requested attitude and hold it with ideal body torques."""

    def __init__(self, cfg: PointingEnvCfg) -> None:
        self._q_tgt = np.tile([1.0, 0.0, 0.0, 0.0], (cfg.num_envs, 1))
        self._init_angle = np.zeros(cfg.num_envs)
        self._pending_targets: np.ndarray | None = None
        super().__init__(cfg)

    # ---------------------------------------------------------------- state access
    def attitudes(self) -> np.ndarray:
        q = np.asarray(self.data.qpos[:, 3:7], dtype=float)
        return q / np.linalg.norm(q, axis=1, keepdims=True)

    def pointing_error(self) -> tuple[np.ndarray, np.ndarray]:
        """(sign-fixed error quaternion (n,4), angle error in rad (n,))."""
        qe = error_quat(self.attitudes(), self._q_tgt)
        return qe, error_angle(qe)

    # ---------------------------------------------------------------- hooks
    def _on_reset(self, idx: np.ndarray, quat: np.ndarray) -> None:
        cfg = self.cfg
        n = idx.size
        if self._pending_targets is not None:
            self._q_tgt[idx] = self._pending_targets
            self._pending_targets = None
        else:
            axis = self._rng.normal(size=(n, 3))
            axis /= np.linalg.norm(axis, axis=1, keepdims=True)
            ang = self._rng.uniform(cfg.err_min, cfg.err_max, n)
            self._q_tgt[idx] = qmul(np.asarray(quat, dtype=float), qexp(axis, ang))
        self._init_angle[idx] = error_angle(error_quat(np.asarray(quat, dtype=float),
                                                       self._q_tgt[idx]))

    def _task_terms(self, acts: np.ndarray, w: np.ndarray, speed: np.ndarray):
        cfg = self.cfg
        _, ang = self.pointing_error()
        pointed = ang < np.radians(cfg.success_angle_deg)
        ok = pointed & (speed < cfg.success_rate)
        common = (
            -cfg.w_att * ang / np.pi
            - cfg.w_effort * np.sum(acts**2, axis=1)
            - cfg.w_smooth * np.sum((acts - self._last_actions) ** 2, axis=1)
        )
        if cfg.reward_version in ("v1", "v3"):
            rew = (common
                   - cfg.w_rate * speed / cfg.w_scale
                   + cfg.w_point_bonus * pointed
                   + cfg.w_rest_bonus * ok)
            if cfg.reward_version == "v3":  # v1 + fine precision bonus only (SPEC_1B 15)
                rew = rew + cfg.w_fine_v3 * np.exp(-(np.degrees(ang) / cfg.v2_fine_deg) ** 2)
        elif cfg.reward_version == "v2":
            deg = np.degrees(ang)
            fine = np.exp(-(deg / cfg.v2_fine_deg) ** 2)
            near = np.exp(-(deg / cfg.v2_rate_gate_deg) ** 2)
            rew = (common
                   - cfg.w_rate_near * near * speed / cfg.w_scale
                   + cfg.w_coarse * np.exp(-(deg / cfg.v2_coarse_deg) ** 2)
                   + cfg.w_fine * fine
                   + cfg.w_still * fine * np.exp(-(speed / 0.005) ** 2))
        else:
            raise ValueError(f"unknown reward_version {cfg.reward_version!r}")
        log = {"/metrics/angle_err_deg_mean": float(np.degrees(np.nanmean(ang))),
               "/metrics/pointed_frac": float(pointed.mean())}
        return ok, rew, log

    def _compute_obs(self) -> TensorDict:
        qe, _ = self.pointing_error()
        parts = [qe[:, 1:], self.body_rates() / self.cfg.w_scale, self._last_actions]
        if self.cfg.obs_version == "v2":
            parts.append(np.clip(qe[:, 1:] / self.cfg.fine_obs_scale, -1.0, 1.0))
        obs = np.concatenate(parts, axis=1)
        obs = np.nan_to_num(obs, nan=0.0, posinf=0.0, neginf=0.0)
        t = torch.as_tensor(obs, dtype=torch.float32, device=self.device)
        return TensorDict({"policy": t}, batch_size=[self.num_envs])

    # ---------------------------------------------------------------- evaluation
    def set_pointing_conditions(self, w0: np.ndarray, quat: np.ndarray,
                                q_tgt: np.ndarray) -> TensorDict:
        """Reset every world to the given spin, attitude and target attitude."""
        self._pending_targets = np.asarray(q_tgt, dtype=float)
        return self.set_initial_conditions(w0, quat)
