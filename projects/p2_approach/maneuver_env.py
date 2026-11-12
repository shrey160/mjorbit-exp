"""Project 2A, maneuver-epoch version (SPEC_2 section 11).

The policy decides once per maneuver epoch (default 20 s, so 30 decisions per 600 s
episode) instead of every second:

  action (3)  = Δv request in LVLH, a in [-1, 1]^3 x dv_max (0.1 m/s). It is rotated to
                world and clipped per world axis, then flown as a finite burn at full
                thrust (5 mm/s^2 per axis) from the start of the epoch; the rest of the
                epoch is a coast. At 0.1 m/s an axis burns for the whole 20 s epoch.
  observation = same 13 values as the 1 Hz env (previous action = previous Δv request).
  reward      = potential shaping (gamma per decision) - w_fuel |Δv|_1 / dv_max
                + hold-window terms summed over the epoch's seconds + terminal success
                bonus; keep-out entry at any second is a failure.

Physics, start states, contract and metrics are those of ApproachEnv (1 s physics step).
"""

from __future__ import annotations

import dataclasses

import numpy as np
import torch

from approach_env import HOLD, ApproachEnv, ApproachEnvCfg
from mjorbit_warp import mjo_pull, mjo_step, mjo_upload


@dataclasses.dataclass
class ManeuverEnvCfg(ApproachEnvCfg):
    num_envs: int = 2048
    control_dt: float = 20.0  # maneuver epoch, s
    dv_max: float = 0.1  # m/s per LVLH axis per decision
    gamma_shaping: float = 0.99  # per decision; must equal the PPO gamma
    w_fuel: float = 2.0  # x |Δv|_1 / dv_max per decision
    w_near: float = 0.5  # per second in the final 60 s
    w_hold: float = 1.0  # per second in the final 60 s


class ManeuverEnv(ApproachEnv):
    def __init__(self, cfg: ManeuverEnvCfg) -> None:
        self.trace: list | None = None  # set to [] to record per-second (p, v, F_lvlh)
        # set to [] to log a 1 Hz navigation measurement each second (SPEC_2 15):
        # entries (commanded LVLH accel, z = [p, v] + noise, true [p, v, b])
        self.nav_log: list | None = None
        self.shield = None  # shield.Shield: filters every world Δv request (SPEC_2 17)
        super().__init__(cfg)
        self._nav1_rng = np.random.default_rng(cfg.seed + 15485863)
        n = cfg.num_envs
        self._ok_sec = np.zeros(n, dtype=np.int64)  # consecutive ok seconds
        self._last_bad_sec = np.zeros(n, dtype=np.int64)
        self._total_s = int(round(cfg.episode_length_s / cfg.physics_dt))

    def reset_worlds(self, idx, p0=None, v0=None) -> None:
        super().reset_worlds(idx, p0, v0)
        if hasattr(self, "_ok_sec"):
            self._ok_sec[idx] = 0
            self._last_bad_sec[idx] = 0

    def set_initial_conditions(self, p0, v0, unc_seed=None):
        if unc_seed is not None:
            self._nav1_rng = np.random.default_rng(unc_seed + 2)
        return super().set_initial_conditions(p0, v0, unc_seed=unc_seed)

    def step(self, actions: torch.Tensor):
        acts = actions.detach().to("cpu", torch.float64).clamp(-1.0, 1.0).numpy()
        C, _, _, _ = self.lvlh_state()
        dv_w = np.clip(np.einsum("nij,nj->ni", C, acts * self.cfg.dv_max),
                       -self.cfg.dv_max, self.cfg.dv_max)
        return self.step_world_dv(dv_w, acts)

    def step_world_dv(self, dv_w: np.ndarray, acts: np.ndarray | None = None):
        """Fly world-frame Δv requests (m/s) as full-thrust burns, then coast."""
        cfg = self.cfg
        n = self.num_envs
        if self.shield is not None:
            dv_w = self.shield(np.asarray(dv_w, float))
        if acts is None:
            C, _, _, _ = self.lvlh_state()
            acts = np.einsum("nji,nj->ni", C, dv_w) / cfg.dv_max
        remaining = np.asarray(dv_w, float).copy()
        a_step = cfg.f_max / cfg.mass * cfg.physics_dt
        failed = np.zeros(n, bool)
        collided = np.zeros(n, bool)
        win_rew = np.zeros(n)
        dv_used = np.zeros(n)
        base_s = self.episode_length_buf.cpu().numpy() * self._decimation
        for k in range(self._decimation):
            step_dv = np.clip(remaining, -a_step, a_step)
            remaining -= step_dv
            F = np.where(failed[:, None], 0.0, step_dv * cfg.mass / cfg.physics_dt)
            Cb, _, _, _ = self.lvlh_state()
            self.data.ctrl[:, 0:3] = self.actual_force(F, Cb)
            mjo_upload(self.model, self.data, fields=("ctrl",))
            mjo_step(self.model, self.data)
            mjo_pull(self.model, self.data, fields=("qpos", "qvel", "orbit"))
            dv_used += np.abs(F).sum(axis=1) / cfg.mass * cfg.physics_dt
            Ck, _, p, v = self.lvlh_state()
            if self.trace is not None:
                self.trace.append((p.copy(), v.copy(), np.einsum("nji,nj->ni", Ck, F)))
            if self.nav_log is not None:
                z = np.concatenate([
                    p + self._nav1_rng.normal(0.0, max(cfg.nav_pos_sigma, 0.0), (n, 3)),
                    v + self._nav1_rng.normal(0.0, max(cfg.nav_vel_sigma, 0.0), (n, 3))], 1)
                u = np.einsum("nji,nj->ni", Cb, F) / cfg.mass
                self.nav_log.append((u, z, np.concatenate([p, v, self._adist], 1)))
            e = np.linalg.norm(p - self.goal_lvlh(), axis=1)
            speed = np.linalg.norm(v - self.goal_vel_lvlh(), axis=1)
            r = np.linalg.norm(p, axis=1)
            finite = np.isfinite(p).all(axis=1) & np.isfinite(v).all(axis=1)
            hit = finite & (self.clearance(p) < 0.0) & ~failed
            collided |= hit
            failed |= hit | ~finite
            ok = (e < cfg.pos_tol) & (speed < cfg.vel_tol) & ~failed
            sec = base_s + k + 1
            self._ok_sec = np.where(ok, self._ok_sec + 1, 0)
            self._last_bad_sec = np.where(ok, self._last_bad_sec, sec)
            in_window = sec > self._total_s - int(cfg.hold_s / cfg.physics_dt)
            win_rew += in_window * (cfg.w_near * np.exp(-(e / cfg.pos_tol) ** 2) + cfg.w_hold * ok)
            self.ep_min_dist = np.minimum(self.ep_min_dist,
                                          np.where(finite, self.proximity(p), 0.0))
            close = np.where(r < 10.0, -np.sum(p * v, axis=1) / np.maximum(r, 1e-9), 0.0)
            self.ep_close_max = np.maximum(self.ep_close_max, np.where(failed, 0.0, close))
        self.episode_length_buf += 1
        self.ep_dv += dv_used

        steps = self.episode_length_buf.cpu().numpy()
        time_up = (steps >= self.max_episode_length) & ~failed
        hold_s = int(cfg.hold_s / cfg.physics_dt)
        success = time_up & (self._ok_sec >= hold_s)

        phi = np.where(failed | time_up, 0.0, self._potential(e, speed))
        rew = (cfg.gamma_shaping * phi - self._phi
               - cfg.w_fuel * dv_used / cfg.dv_max
               - cfg.w_smooth * np.sum((acts - self._last_actions) ** 2, axis=1)
               + win_rew + cfg.w_success * success)
        rew = np.where(failed, -cfg.fail_penalty, rew)
        rew = np.nan_to_num(rew, nan=-cfg.fail_penalty)
        self._last_actions = acts
        self._phi = np.nan_to_num(phi)

        dones = time_up | failed
        self.last_done_info = None
        if dones.any():
            idx = np.flatnonzero(dones)
            settle = np.where(success, (self._last_bad_sec + 1) * cfg.physics_dt, np.nan).astype(float)
            self.last_done_info = dict(idx=idx, success=success[idx], collided=collided[idx],
                                       dv=self.ep_dv[idx].copy(),
                                       min_dist=self.ep_min_dist[idx].copy(),
                                       close_max=self.ep_close_max[idx].copy(),
                                       final_err=e[idx].copy(), final_speed=speed[idx].copy(),
                                       settle=settle[idx])
            k = idx.size
            a = min(1.0, k / 1024.0)
            bs, bc = float(success[idx].mean()), float(collided[idx].mean())
            if self._n_done == 0:
                self._success_ema, self._collide_ema = bs, bc
            else:
                self._success_ema = (1 - a) * self._success_ema + a * bs
                self._collide_ema = (1 - a) * self._collide_ema + a * bc
            sd = success[idx]
            if sd.any():
                bd = float(self.ep_dv[idx][sd].mean())
                self._dv_ema = bd if np.isnan(self._dv_ema) else 0.9 * self._dv_ema + 0.1 * bd
            self._n_done += k

        log = {
            "/metrics/success_ema": self._success_ema,
            "/metrics/collide_ema": self._collide_ema,
            "/metrics/dv_success_ema": 0.0 if np.isnan(self._dv_ema) else self._dv_ema,
            "/metrics/pos_err_mean": float(np.nanmean(e)),
            "/metrics/ok_frac": float((self._ok_sec > 0).mean()),
            "/metrics/thrust_sat_frac": float((np.abs(acts) > 0.999).any(axis=1).mean()),
        }
        done_idx = np.flatnonzero(dones)
        if done_idx.size and cfg.auto_reset:
            self.reset_worlds(done_idx)
        self._obs = self._compute_obs()
        extras = {"time_outs": torch.zeros(n, dtype=torch.bool, device=self.device), "log": log}
        return (self._obs,
                torch.as_tensor(rew, dtype=torch.float32, device=self.device),
                torch.as_tensor(dones, dtype=torch.bool, device=self.device),
                extras)
