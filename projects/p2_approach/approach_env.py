"""Project 2A: batched approach-and-hold environment on mjorbit_warp (rsl-rl >= 5 VecEnv).

Implements SPEC_2:
  observation (13) = [(p - hold)/50, v_lvlh/0.2, time remaining/600, previous action,
                      clip((p - hold)/0.5, -1, 1)]          all in LVLH
  action (3)       = LVLH force command a in [-1, 1]^3 x 0.05 N, rotated to world and
                     clipped per world axis (the thrusters are world-fixed)
  episode          = 600 s at 1 Hz (physics dt 1 s); keep-out entry (< 2 m) ends it (failure)
  success          = |p - hold| < 0.25 m and |v| < 0.01 m/s for the final 60 s

Time remaining is in the observation, so the episode end is part of the state: it is a
true termination, not a truncation, and `time_outs` is never set (no bootstrapping).

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

XML_PATH = Path(__file__).with_name("p2_scene.xml")
HOLD = np.array([0.0, -3.0, 0.0])


@dataclasses.dataclass
class ApproachEnvCfg:
    num_envs: int = 4096
    episode_length_s: float = 600.0
    control_dt: float = 1.0
    physics_dt: float = 1.0
    altitude_km: float = 400.0
    mass: float = 10.0
    f_max: float = 0.05  # N per world axis
    # Policy-action deadzone: |a| < deadzone gives zero thrust, the rest rescales to full
    # range. Makes "coast" a wide basin, so exploration noise near zero burns no fuel
    # (SPEC_2 section 8: without it the fuel cost collapsed exploration).
    deadzone: float = 0.0

    # initial state: distance in [r_min, r_max] within a cone around -V-bar
    r_min: float = 20.0
    r_max: float = 50.0
    cone_deg: float = 60.0
    v0_max: float = 0.02

    keep_out: float = 2.0
    pos_tol: float = 0.25
    vel_tol: float = 0.01
    hold_s: float = 60.0

    pos_scale: float = 50.0
    vel_scale: float = 0.2
    fine_scale: float = 0.5

    # reward v1 (SPEC_2 section 6, finalized in section 8)
    gamma_shaping: float = 0.998  # must equal the PPO gamma for policy invariance
    # potential-based shaping (policy-invariant in the discounted return):
    #   phi = -w_shape |e| + g (w_fine_pot + w_still_pot exp(-(|v|/vel_tol)^2)),
    #   g = exp(-(|e|/pos_tol)^2)
    # v1 used w_shape 0.1 and no fine terms, with w_fuel 0.2: the policy collapsed to
    # "never thrust" (SPEC_2 section 8).
    w_shape: float = 1.0  # per m
    w_fine_pot: float = 5.0
    w_still_pot: float = 5.0
    w_fuel: float = 0.02  # per step, x |F|_1 / f_max (raise in later stages)
    w_smooth: float = 0.01
    w_near: float = 0.5  # exp(-(|e|/pos_tol)^2), only in the final hold window
    w_hold: float = 1.0  # success condition, only in the final hold window
    w_success: float = 20.0  # terminal, held the whole final window
    fail_penalty: float = 10.0

    auto_reset: bool = True
    device: str = "cuda"
    seed: int = 0

    # Uncertainty (SPEC_2 section 13). All zero = the ideal setting. Sampled per episode,
    # unknown to every controller; success and reward use the true state.
    gain_err: float = 0.0  # per-axis thrust gain uniform in 1 +- gain_err (capped at f_max)
    misalign_deg: float = 0.0  # thrust misalignment: random axis, angle uniform up to this
    dist_acc: float = 0.0  # constant LVLH disturbance accel, per axis uniform +-, m/s^2
    nav_pos_sigma: float = 0.0  # m, white noise on the measured LVLH position
    nav_vel_sigma: float = 0.0  # m/s, white noise on the measured LVLH velocity


# Uncertainty presets (SPEC_2 section 13)
UNCERTAINTY = {
    "none": {},
    # thrust gain +-10 %/axis, misalignment up to 3 deg, constant LVLH disturbance up to
    # 2e-5 m/s^2/axis (~10x differential drag here), nav noise 5 cm / 1 mm/s (1 sigma)
    "u1": dict(gain_err=0.10, misalign_deg=3.0, dist_acc=2e-5,
               nav_pos_sigma=0.05, nav_vel_sigma=0.001),
    # diagnostics (SPEC_2 19): u1 split into its navigation and dynamics parts
    "u1nav": dict(nav_pos_sigma=0.05, nav_vel_sigma=0.001),
    "u1dyn": dict(gain_err=0.10, misalign_deg=3.0, dist_acc=2e-5),
}


# ------------------------------------------------------------------- LVLH helpers
def lvlh_frames(R_km: np.ndarray, V_kms: np.ndarray):
    """Per-world LVLH basis C (n,3,3, columns = LVLH axes in world) and omega (n,3) rad/s."""
    h = np.cross(R_km, V_kms)
    x = R_km / np.linalg.norm(R_km, axis=1, keepdims=True)
    z = h / np.linalg.norm(h, axis=1, keepdims=True)
    y = np.cross(z, x)
    C = np.stack([x, y, z], axis=2)
    omega = h / np.sum(R_km**2, axis=1, keepdims=True)
    return C, omega


def world_to_lvlh(C, omega, r_w, v_w):
    p = np.einsum("nji,nj->ni", C, r_w)
    v = np.einsum("nji,nj->ni", C, v_w - np.cross(omega, r_w))
    return p, v


def lvlh_to_world(C, omega, p, v):
    r_w = np.einsum("nij,nj->ni", C, p)
    v_w = np.einsum("nij,nj->ni", C, v) + np.cross(omega, r_w)
    return r_w, v_w


def sample_starts(rng, n, r_min, r_max, cone_deg, v0_max):
    r = rng.uniform(r_min, r_max, n)
    cos_t = rng.uniform(np.cos(np.radians(cone_deg)), 1.0, n)
    sin_t = np.sqrt(1 - cos_t**2)
    phi = rng.uniform(0, 2 * np.pi, n)
    p = np.stack([r * sin_t * np.cos(phi), -r * cos_t, r * sin_t * np.sin(phi)], axis=1)
    v = rng.uniform(-v0_max, v0_max, (n, 3))
    return p, v


class ApproachEnv(VecEnv):
    """Fly a thruster-only chaser to a V-bar hold point behind a co-orbiting target."""

    xml_path = XML_PATH

    # Geometry hooks (Project 3 overrides them for a box-shaped client, P3 SPEC).
    def goal_lvlh(self) -> np.ndarray:
        """Goal position per world (n,3), LVLH."""
        return np.broadcast_to(HOLD, (self.num_envs, 3))

    def goal_vel_lvlh(self) -> np.ndarray:
        """Goal velocity per world (n,3), LVLH (success uses the speed relative to it)."""
        return np.zeros((self.num_envs, 3))

    def clearance(self, p: np.ndarray) -> np.ndarray:
        """Signed distance outside the keep-out volume (m); < 0 is a collision."""
        return np.linalg.norm(p, axis=1) - self.cfg.keep_out

    def proximity(self, p: np.ndarray) -> np.ndarray:
        """Quantity whose episode minimum is reported as min_dist."""
        return np.linalg.norm(p, axis=1)

    def __init__(self, cfg: ApproachEnvCfg) -> None:
        if cfg.num_envs < 2:
            raise ValueError("num_envs must be >= 2")
        self.cfg = cfg
        self.device = cfg.device
        self.num_envs = n = cfg.num_envs
        self.num_actions = 3
        self.max_episode_length = int(round(cfg.episode_length_s / cfg.control_dt))
        self.episode_length_buf = torch.zeros(n, dtype=torch.long, device=cfg.device)
        self._decimation = int(round(cfg.control_dt / cfg.physics_dt))
        self._hold_steps = int(round(cfg.hold_s / cfg.control_dt))
        self._rng = np.random.default_rng(cfg.seed)
        self.mean_motion = float(np.sqrt(GM_EARTH / (R_EARTH + cfg.altitude_km) ** 3))

        self.model = MjoModel.from_xml_path(str(self.xml_path), mj_timestep=cfg.physics_dt)
        self.data = self.model.make_data(orbit=self._sample_orbits(n), nworld=n)
        self._qpos0 = self.model.mj_model.qpos0.copy()

        self._last_actions = np.zeros((n, 3))
        self._ok_count = np.zeros(n, dtype=np.int64)
        self.ep_dv = np.zeros(n)  # m/s, per-axis 1-norm
        self.ep_min_dist = np.full(n, np.inf)
        self.ep_close_max = np.zeros(n)
        self._success_ema = self._collide_ema = 0.0
        self._dv_ema = np.nan
        self._n_done = 0
        self._pending: tuple[np.ndarray, np.ndarray] | None = None
        self._phi = np.zeros(n)
        self._gain = np.ones((n, 3))
        self._mis = np.tile(np.eye(3), (n, 1, 1))
        self._adist = np.zeros((n, 3))
        self._unc_rng = np.random.default_rng(cfg.seed + 7919)
        self._nav_rng = np.random.default_rng(cfg.seed + 104729)
        self.meas_p = np.zeros((n, 3))
        self.meas_v = np.zeros((n, 3))

        mjo_pull(self.model, self.data, fields=("qpos", "qvel", "orbit"))
        self.reset_worlds(np.arange(n))
        self._obs = self._compute_obs()

    # ------------------------------------------------------------------ state
    def lvlh_state(self):
        """(C, omega, p_lvlh, v_lvlh) for the chaser in every world (after a pull)."""
        C, om = lvlh_frames(np.asarray(self.data.orbit.R_eci, float),
                            np.asarray(self.data.orbit.V_eci, float))
        r_w = np.asarray(self.data.qpos[:, 0:3], float)
        v_w = np.asarray(self.data.qvel[:, 0:3], float)
        p, v = world_to_lvlh(C, om, r_w, v_w)
        return C, om, p, v

    def measured_state(self):
        """(C, omega, p, v) as the controllers see it: navigation noise on p and v."""
        C, om, _, _ = self.lvlh_state()
        return C, om, self.meas_p, self.meas_v

    def _measure(self) -> None:
        cfg = self.cfg
        _, _, p, v = self.lvlh_state()
        n = self.num_envs
        self.meas_p = p + (self._nav_rng.normal(0.0, cfg.nav_pos_sigma, (n, 3))
                           if cfg.nav_pos_sigma > 0 else 0.0)
        self.meas_v = v + (self._nav_rng.normal(0.0, cfg.nav_vel_sigma, (n, 3))
                           if cfg.nav_vel_sigma > 0 else 0.0)

    def actual_force(self, F_cmd: np.ndarray, C: np.ndarray) -> np.ndarray:
        """Force the thrusters really deliver (gain error, misalignment, cap) plus the
        unmodelled disturbance, all in world axes."""
        cfg = self.cfg
        F = np.einsum("nij,nj->ni", self._mis, self._gain * F_cmd)
        F = np.clip(F, -cfg.f_max, cfg.f_max)
        return F + cfg.mass * np.einsum("nij,nj->ni", C, self._adist)

    def _sample_uncertainty(self, idx: np.ndarray) -> None:
        cfg = self.cfg
        k = idx.size
        rng = self._unc_rng
        self._gain[idx] = rng.uniform(1 - cfg.gain_err, 1 + cfg.gain_err, (k, 3))
        ax = rng.normal(size=(k, 3))
        ax /= np.linalg.norm(ax, axis=1, keepdims=True)
        ang = np.radians(rng.uniform(0.0, cfg.misalign_deg, k)) if cfg.misalign_deg > 0             else np.zeros(k)
        K = np.zeros((k, 3, 3))
        K[:, 0, 1], K[:, 0, 2], K[:, 1, 2] = -ax[:, 2], ax[:, 1], -ax[:, 0]
        K[:, 1, 0], K[:, 2, 0], K[:, 2, 1] = ax[:, 2], -ax[:, 1], ax[:, 0]
        s, c = np.sin(ang)[:, None, None], np.cos(ang)[:, None, None]
        self._mis[idx] = np.eye(3) + s * K + (1 - c) * (K @ K)
        self._adist[idx] = rng.uniform(-cfg.dist_acc, cfg.dist_acc, (k, 3))

    def time_remaining(self) -> np.ndarray:
        return 1.0 - self.episode_length_buf.cpu().numpy() / self.max_episode_length

    # ------------------------------------------------------------------ VecEnv
    def get_observations(self) -> TensorDict:
        return self._obs

    def step(self, actions: torch.Tensor):
        acts = actions.detach().to("cpu", torch.float64).clamp(-1.0, 1.0).numpy()
        dz = self.cfg.deadzone
        cmd = np.sign(acts) * np.maximum(np.abs(acts) - dz, 0.0) / (1.0 - dz) if dz > 0 else acts
        C, _, _, _ = self.lvlh_state()
        F = np.einsum("nij,nj->ni", C, cmd * self.cfg.f_max)
        return self._advance(F, acts)

    def step_world_force(self, F_world: np.ndarray):
        """Baseline/evaluation path: command world-frame forces directly (N)."""
        C, _, _, _ = self.lvlh_state()
        acts = np.einsum("nji,nj->ni", C, F_world) / self.cfg.f_max
        return self._advance(F_world, acts)

    def _advance(self, F_world: np.ndarray, acts: np.ndarray):
        cfg = self.cfg
        F = np.clip(F_world, -cfg.f_max, cfg.f_max)
        C0, _, _, _ = self.lvlh_state()
        self.data.ctrl[:, 0:3] = self.actual_force(F, C0)
        mjo_upload(self.model, self.data, fields=("ctrl",))
        for _ in range(self._decimation):
            mjo_step(self.model, self.data)
        mjo_pull(self.model, self.data, fields=("qpos", "qvel", "orbit"))
        self.episode_length_buf += 1

        _, _, p, v = self.lvlh_state()
        e = np.linalg.norm(p - HOLD, axis=1)
        speed = np.linalg.norm(v, axis=1)
        r = np.linalg.norm(p, axis=1)
        finite = np.isfinite(p).all(axis=1) & np.isfinite(v).all(axis=1)
        collided = finite & (r < cfg.keep_out)
        failed = collided | ~finite
        ok = (e < cfg.pos_tol) & (speed < cfg.vel_tol)
        self._ok_count = np.where(ok, self._ok_count + 1, 0)

        steps = self.episode_length_buf.cpu().numpy()
        in_window = steps > self.max_episode_length - self._hold_steps
        time_up = (steps >= self.max_episode_length) & ~failed
        success = time_up & (self._ok_count >= self._hold_steps)

        fuel = np.abs(F).sum(axis=1) / cfg.f_max
        self.ep_dv += np.abs(F).sum(axis=1) / cfg.mass * cfg.control_dt
        self.ep_min_dist = np.minimum(self.ep_min_dist, np.where(finite, r, 0.0))
        close = np.where(r < 10.0, -np.sum(p * v, axis=1) / np.maximum(r, 1e-9), 0.0)
        self.ep_close_max = np.maximum(self.ep_close_max, close)

        # potential-based shaping: gamma * phi(s') - phi(s); phi(terminal) = 0
        phi_prev = self._phi
        phi = np.where(failed | time_up, 0.0, self._potential(e, speed))
        rew = (cfg.gamma_shaping * phi - phi_prev
               - cfg.w_fuel * fuel
               - cfg.w_smooth * np.sum((acts - self._last_actions) ** 2, axis=1)
               + in_window * (cfg.w_near * np.exp(-(e / cfg.pos_tol) ** 2) + cfg.w_hold * ok)
               + cfg.w_success * success)
        rew = np.where(failed, -cfg.fail_penalty, rew)
        rew = np.nan_to_num(rew, nan=-cfg.fail_penalty)
        self._last_actions = acts
        self._phi = np.nan_to_num(phi)

        dones = time_up | failed
        self.last_done_info = None
        if dones.any():
            idx = np.flatnonzero(dones)
            self.last_done_info = dict(idx=idx, success=success[idx], collided=collided[idx],
                                       dv=self.ep_dv[idx].copy(),
                                       min_dist=self.ep_min_dist[idx].copy(),
                                       close_max=self.ep_close_max[idx].copy(),
                                       final_err=e[idx].copy(), final_speed=speed[idx].copy())
            k = idx.size
            a = min(1.0, k / 1024.0)
            bs, bc = float(success[idx].mean()), float(collided[idx].mean())
            sd = success[idx]
            if self._n_done == 0:
                self._success_ema, self._collide_ema = bs, bc
            else:
                self._success_ema = (1 - a) * self._success_ema + a * bs
                self._collide_ema = (1 - a) * self._collide_ema + a * bc
            if sd.any():
                bd = float(self.ep_dv[idx][sd].mean())
                self._dv_ema = bd if np.isnan(self._dv_ema) else 0.9 * self._dv_ema + 0.1 * bd
            self._n_done += k

        log = {
            "/metrics/success_ema": self._success_ema,
            "/metrics/collide_ema": self._collide_ema,
            "/metrics/dv_success_ema": 0.0 if np.isnan(self._dv_ema) else self._dv_ema,
            "/metrics/pos_err_mean": float(np.nanmean(e)),
            "/metrics/ok_frac": float(ok.mean()),
            "/metrics/thrust_sat_frac": float((np.abs(F) >= cfg.f_max * 0.999).any(axis=1).mean()),
        }
        done_idx = np.flatnonzero(dones)
        if done_idx.size and cfg.auto_reset:
            self.reset_worlds(done_idx)

        self._obs = self._compute_obs()
        extras = {"time_outs": torch.zeros(self.num_envs, dtype=torch.bool, device=self.device),
                  "log": log}
        return (self._obs,
                torch.as_tensor(rew, dtype=torch.float32, device=self.device),
                torch.as_tensor(dones, dtype=torch.bool, device=self.device),
                extras)

    # ------------------------------------------------------------- helpers
    def _potential(self, e: np.ndarray, speed: np.ndarray) -> np.ndarray:
        cfg = self.cfg
        g = np.exp(-(e / cfg.pos_tol) ** 2)
        return -cfg.w_shape * e + g * (cfg.w_fine_pot
                                       + cfg.w_still_pot * np.exp(-(speed / cfg.vel_tol) ** 2))

    def _refresh_phi(self, idx: np.ndarray) -> None:
        _, _, p, v = self.lvlh_state()
        e = np.linalg.norm(p[idx] - self.goal_lvlh()[idx], axis=1)
        self._phi[idx] = self._potential(
            e, np.linalg.norm(v[idx] - self.goal_vel_lvlh()[idx], axis=1))

    def _compute_obs(self) -> TensorDict:
        cfg = self.cfg
        self._measure()
        p, v = self.meas_p, self.meas_v
        e = p - HOLD
        obs = np.concatenate([e / cfg.pos_scale, v / cfg.vel_scale,
                              self.time_remaining()[:, None], self._last_actions,
                              np.clip(e / cfg.fine_scale, -1.0, 1.0)], axis=1)
        obs = np.nan_to_num(obs, nan=0.0, posinf=0.0, neginf=0.0)
        t = torch.as_tensor(obs, dtype=torch.float32, device=self.device)
        return TensorDict({"policy": t}, batch_size=[self.num_envs])

    def _sample_orbits(self, n: int) -> list[OrbitInit]:
        radius = R_EARTH + self.cfg.altitude_km
        speed = float(np.sqrt(GM_EARTH / radius))
        phi = self._rng.uniform(0.0, 2.0 * np.pi, n)
        return [OrbitInit(R_eci=radius * np.array([np.cos(p), 0.0, np.sin(p)]),
                          V_eci=speed * np.array([-np.sin(p), 0.0, np.cos(p)]))
                for p in phi]

    def reset_worlds(self, idx: np.ndarray, p0: np.ndarray | None = None,
                     v0: np.ndarray | None = None) -> None:
        """Reset worlds `idx` to LVLH start states (random unless p0/v0 given)."""
        cfg = self.cfg
        n = idx.size
        if p0 is None:
            p0, v0 = sample_starts(self._rng, n, cfg.r_min, cfg.r_max, cfg.cone_deg, cfg.v0_max)
        C, om = lvlh_frames(np.asarray(self.data.orbit.R_eci, float)[idx],
                            np.asarray(self.data.orbit.V_eci, float)[idx])
        r_w, v_w = lvlh_to_world(C, om, p0, v0)
        qpos = np.tile(self._qpos0, (n, 1))
        qpos[:, 0:3] = r_w
        qpos[:, 3:7] = [1.0, 0.0, 0.0, 0.0]
        qpos[:, 7:10] = 0.0
        qpos[:, 10:14] = [1.0, 0.0, 0.0, 0.0]
        qvel = np.zeros((n, 12))
        qvel[:, 0:3] = v_w
        self.data.qpos[idx] = qpos
        self.data.qvel[idx] = qvel
        self.data.ctrl[idx] = 0.0
        self._sample_uncertainty(idx)
        self._last_actions[idx] = 0.0
        self._ok_count[idx] = 0
        self.ep_dv[idx] = 0.0
        self.ep_min_dist[idx] = np.inf
        self.ep_close_max[idx] = 0.0
        self.episode_length_buf[torch.as_tensor(idx, device=self.device)] = 0
        mjo_upload(self.model, self.data, fields=("qpos", "qvel", "ctrl"))
        mjo_forward(self.model, self.data)
        mjo_pull(self.model, self.data, fields=("qpos", "qvel", "orbit"))
        self._refresh_phi(idx)

    def set_initial_conditions(self, p0: np.ndarray, v0: np.ndarray,
                               unc_seed: int | None = None) -> TensorDict:
        """Reset every world. unc_seed makes the per-world uncertainty draws and the
        navigation-noise sequence reproducible (matched across controllers)."""
        if unc_seed is not None:
            self._unc_rng = np.random.default_rng(unc_seed)
            self._nav_rng = np.random.default_rng(unc_seed + 1)
        self.reset_worlds(np.arange(self.num_envs), p0=p0, v0=v0)
        self._obs = self._compute_obs()
        return self._obs


# ------------------------------------------------------------------- baseline
def baseline_force(env: ApproachEnv, Kp=0.05, Kd=0.5, beta=0.8, v_max=None,
                   plan_s=None, d0=None) -> np.ndarray:
    """Vectorized SPEC_2 section 4 controller -> world force (N). For the planned
    variant pass plan_s and d0 (initial distance to hold, per world)."""
    cfg, n_ = env.cfg, env.mean_motion
    C, _, p, v = env.measured_state()
    a_min = cfg.f_max / cfg.mass - 3 * n_**2 * 50.0
    e = HOLD - p
    d = np.linalg.norm(e, axis=1)
    vmax = d0 / plan_s if plan_s is not None else np.full(len(d), v_max if v_max else 0.2)
    spd = np.minimum(np.minimum(Kp * d, np.sqrt(2 * beta * a_min * d)), vmax)
    v_ref = e / np.maximum(d, 1e-9)[:, None] * spd[:, None]
    cw = np.stack([3 * n_**2 * p[:, 0] + 2 * n_ * v[:, 1], -2 * n_ * v[:, 0],
                   -n_**2 * p[:, 2]], axis=1)
    u = Kd * (v_ref - v) - cw
    b = getattr(env, "est_bias", None)  # bias feed-forward when a bias filter is used
    if b is not None:
        u = u - b
    return np.clip(cfg.mass * np.einsum("nij,nj->ni", C, u), -cfg.f_max, cfg.f_max)
