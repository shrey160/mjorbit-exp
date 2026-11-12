"""Project 1A: batched detumble environment on mjorbit_warp (rsl-rl >= 5 VecEnv).

Implements the contract in SPEC.md:
  observation (6) = [body angular velocity / 0.3, previous action]
  action (3)      = normalized body torques, tau = 0.01 * clip(a, -1, 1) N*m, held 0.1 s
  episode         = 30 s (300 control steps); time-out is a truncation
  terminate early = |w| > 1 rad/s or non-finite state (failure)
  success         = |w| < 0.01 rad/s for the final 5 s

Import ``mjorbit`` before torch so the native bindings get the env's libstdc++.
"""

from __future__ import annotations

import dataclasses
from pathlib import Path

import mjorbit  # noqa: F401

# isort: split

import numpy as np
import torch
from rsl_rl.env import VecEnv
from tensordict import TensorDict

from mjorbit.constants import GM_EARTH, R_EARTH
from mjorbit_warp import MjoModel, OrbitInit, mjo_forward, mjo_pull, mjo_step, mjo_upload

XML_PATH = Path(__file__).with_name("p1_spacecraft.xml")


@dataclasses.dataclass
class DetumbleEnvCfg:
    num_envs: int = 1024
    episode_length_s: float = 30.0
    control_dt: float = 0.1
    physics_dt: float = 0.01
    altitude_km: float = 400.0
    tau_max: float = 0.01  # N*m per axis

    # Initial spin. "one_axis": one random body axis, |w| in [spin_min, spin_max].
    # "box": each axis uniform in [-spin_max, spin_max].
    spin_mode: str = "one_axis"
    spin_min: float = 0.05
    spin_max: float = 0.1

    w_scale: float = 0.3  # observation scaling of angular velocity (rad/s)
    success_rate: float = 0.01  # rad/s
    success_hold_s: float = 5.0
    term_speed: float = 1.0  # rad/s, early termination (failure)

    # Reward weights (SPEC section 7).
    w_rate: float = 1.0
    w_effort: float = 0.02
    w_smooth: float = 0.02
    w_rest_bonus: float = 0.5
    fail_penalty: float = 10.0

    auto_reset: bool = True
    device: str = "cuda"
    seed: int = 0


def random_quats(rng: np.random.Generator, n: int) -> np.ndarray:
    q = rng.normal(size=(n, 4))
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    return q


def sample_spins(rng: np.random.Generator, n: int, mode: str, lo: float, hi: float) -> np.ndarray:
    if mode == "one_axis":
        w = np.zeros((n, 3))
        axis = rng.integers(0, 3, n)
        mag = rng.uniform(lo, hi, n) * rng.choice([-1.0, 1.0], n)
        w[np.arange(n), axis] = mag
        return w
    if mode == "box":
        return rng.uniform(-hi, hi, (n, 3))
    raise ValueError(f"unknown spin_mode {mode!r}")


class DetumbleEnv(VecEnv):
    """Bring a randomly spinning rigid spacecraft to rest with ideal body torques."""

    def __init__(self, cfg: DetumbleEnvCfg) -> None:
        if cfg.num_envs < 2:
            raise ValueError("num_envs must be >= 2 (nworld=1 uses unbatched buffers)")
        dec = cfg.control_dt / cfg.physics_dt
        if abs(dec - round(dec)) > 1e-9:
            raise ValueError("control_dt must be an integer multiple of physics_dt")

        self.cfg = cfg
        self.device = cfg.device
        self.num_envs = cfg.num_envs
        self.num_actions = 3
        self.max_episode_length = int(round(cfg.episode_length_s / cfg.control_dt))
        self.episode_length_buf = torch.zeros(cfg.num_envs, dtype=torch.long, device=cfg.device)
        self._decimation = int(round(dec))
        self._hold_steps = int(round(cfg.success_hold_s / cfg.control_dt))
        self._rng = np.random.default_rng(cfg.seed)

        self.model = MjoModel.from_xml_path(str(XML_PATH), mj_timestep=cfg.physics_dt)
        self.data = self.model.make_data(orbit=self._sample_orbits(cfg.num_envs),
                                         nworld=cfg.num_envs)
        m = self.model.mj_model
        self.inertia = np.asarray(m.body_inertia[m.body("sc").id], dtype=float).copy()
        self._qpos0 = m.qpos0.copy()

        n = cfg.num_envs
        self._last_actions = np.zeros((n, 3))
        self._below_count = np.zeros(n, dtype=np.int64)  # consecutive steps below threshold
        self._ep_effort = np.zeros(n)
        self._success_ema = 0.0
        self._n_done = 0

        self.reset_worlds(np.arange(n))
        self._obs = self._compute_obs()

    # ------------------------------------------------------------------ VecEnv
    def get_observations(self) -> TensorDict:
        return self._obs

    def step(self, actions: torch.Tensor) -> tuple[TensorDict, torch.Tensor, torch.Tensor, dict]:
        cfg = self.cfg
        acts = actions.detach().to("cpu", torch.float64).clamp(-1.0, 1.0).numpy()
        self.data.ctrl[:, 0:3] = cfg.tau_max * acts
        mjo_upload(self.model, self.data, fields=("ctrl",))
        for _ in range(self._decimation):
            mjo_step(self.model, self.data)
        mjo_pull(self.model, self.data, fields=("qpos", "qvel"))
        self.episode_length_buf += 1

        w = self.body_rates()
        speed = np.linalg.norm(w, axis=1)
        finite = np.isfinite(self.data.qpos).all(axis=1) & np.isfinite(self.data.qvel).all(axis=1)
        failed = (~finite) | (speed > cfg.term_speed)

        rest, rew, task_log = self._task_terms(acts, w, speed)
        self._below_count = np.where(rest, self._below_count + 1, 0)
        self._ep_effort += np.abs(acts).sum(axis=1) * cfg.tau_max * cfg.control_dt
        rew = np.where(failed, -cfg.fail_penalty, rew)
        rew = np.nan_to_num(rew, nan=-cfg.fail_penalty)

        time_out = (self.episode_length_buf >= self.max_episode_length).cpu().numpy() & ~failed
        dones = time_out | failed
        self._last_actions = acts

        # Episode-level success statistics (EMA over finished episodes).
        if dones.any():
            succ = time_out & (self._below_count >= self._hold_steps)
            batch = float(succ[dones].mean())
            k = int(dones.sum())
            a = min(1.0, k / 512.0)
            self._success_ema = batch if self._n_done == 0 else (1 - a) * self._success_ema + a * batch
            self._n_done += k

        log = {
            "/metrics/speed_mean": float(np.nanmean(speed)),
            "/metrics/at_rest_frac": float(rest.mean()),
            "/metrics/torque_sat_frac": float((np.abs(acts) > 0.999).any(axis=1).mean()),
            "/metrics/success_ema": self._success_ema,
            "/metrics/fail_frac": float(failed.mean()),
            **task_log,
        }

        done_idx = np.flatnonzero(dones)
        if done_idx.size and cfg.auto_reset:
            self.reset_worlds(done_idx)

        self._obs = self._compute_obs()
        extras = {
            "time_outs": torch.as_tensor(time_out, dtype=torch.bool, device=self.device),
            "log": log,
        }
        return (
            self._obs,
            torch.as_tensor(rew, dtype=torch.float32, device=self.device),
            torch.as_tensor(dones, dtype=torch.bool, device=self.device),
            extras,
        )

    # ------------------------------------------------------------- task hooks
    def _task_terms(self, acts: np.ndarray, w: np.ndarray, speed: np.ndarray):
        """Per-step (success-condition-now, reward, extra log). Subclasses override."""
        cfg = self.cfg
        rest = speed < cfg.success_rate
        rew = (
            -cfg.w_rate * speed / cfg.w_scale
            - cfg.w_effort * np.sum(acts**2, axis=1)
            - cfg.w_smooth * np.sum((acts - self._last_actions) ** 2, axis=1)
            + cfg.w_rest_bonus * rest
        )
        return rest, rew, {}

    def _on_reset(self, idx: np.ndarray, quat: np.ndarray) -> None:
        """Called after worlds `idx` get new attitudes `quat`. Subclasses override."""

    # ------------------------------------------------------------- helpers
    def body_rates(self) -> np.ndarray:
        """Body-frame angular velocity (rad/s), host copy (call after a pull)."""
        return np.asarray(self.data.qvel[:, 3:6], dtype=float)

    def _compute_obs(self) -> TensorDict:
        obs = np.concatenate([self.body_rates() / self.cfg.w_scale, self._last_actions], axis=1)
        obs = np.nan_to_num(obs, nan=0.0, posinf=0.0, neginf=0.0)
        t = torch.as_tensor(obs, dtype=torch.float32, device=self.device)
        return TensorDict({"policy": t}, batch_size=[self.num_envs])

    def _sample_orbits(self, n: int) -> list[OrbitInit]:
        radius = R_EARTH + self.cfg.altitude_km
        speed = float(np.sqrt(GM_EARTH / radius))
        phi = self._rng.uniform(0.0, 2.0 * np.pi, n)
        return [
            OrbitInit(R_eci=radius * np.array([np.cos(p), 0.0, np.sin(p)]),
                      V_eci=speed * np.array([-np.sin(p), 0.0, np.cos(p)]))
            for p in phi
        ]

    def reset_worlds(self, idx: np.ndarray, w0: np.ndarray | None = None,
                     quat: np.ndarray | None = None) -> None:
        """Reset worlds `idx`. Random spin/attitude unless `w0` (n,3) / `quat` (n,4) given."""
        cfg = self.cfg
        n = idx.size
        if w0 is None:
            w0 = sample_spins(self._rng, n, cfg.spin_mode, cfg.spin_min, cfg.spin_max)
        if quat is None:
            quat = random_quats(self._rng, n)
        qpos = np.tile(self._qpos0, (n, 1))
        qpos[:, 0:3] = 0.0
        qpos[:, 3:7] = quat
        qvel = np.zeros((n, 6))
        qvel[:, 3:6] = w0
        self.data.qpos[idx] = qpos
        self.data.qvel[idx] = qvel
        self.data.ctrl[idx] = 0.0
        self._last_actions[idx] = 0.0
        self._below_count[idx] = 0
        self._ep_effort[idx] = 0.0
        self.episode_length_buf[torch.as_tensor(idx, device=self.device)] = 0
        self._on_reset(idx, quat)
        mjo_upload(self.model, self.data, fields=("qpos", "qvel", "ctrl"))
        mjo_forward(self.model, self.data)
        mjo_pull(self.model, self.data, fields=("qpos", "qvel"))

    def set_initial_conditions(self, w0: np.ndarray, quat: np.ndarray) -> TensorDict:
        """Evaluation helper: reset every world to the given spins/attitudes."""
        self.reset_worlds(np.arange(self.num_envs), w0=w0, quat=quat)
        self._obs = self._compute_obs()
        return self._obs
