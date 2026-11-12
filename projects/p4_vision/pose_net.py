"""P4.3 bar-pose estimator: wrist + bus camera images, bus flash lidar and the arm joint
angles -> bar position (3) and rotation (6, first two matrix columns) in the bus frame.

One small CNN per sensor. Feature maps are flattened, not pooled, so the network keeps where
in the image the bar is. CoordConv channels give each pixel its position. The joint angles
tell the network where the wrist camera is."""
from __future__ import annotations

import numpy as np
import torch
from torch import nn


def _block(cin, cout, k=3, s=2):
    return nn.Sequential(nn.Conv2d(cin, cout, k, s, k // 2, bias=False), nn.GroupNorm(8, cout), nn.SiLU())


class Enc(nn.Module):
    """(B, C, S, S) -> (B, d): four stride-2 blocks down to S/16 (128 -> 8, 192 -> 12)."""

    def __init__(self, cin, size, d=256, w=(32, 64, 96, 128)):
        super().__init__()
        assert size % 16 == 0, size
        chans = [cin + 2, *w]
        layers = [_block(chans[0], chans[1], 5, 2)] + [_block(chans[i], chans[i + 1]) for i in range(1, len(w))]
        self.conv = nn.Sequential(*layers, nn.Conv2d(chans[-1], 32, 1))
        g = torch.linspace(-1, 1, size)
        self.register_buffer("xy", torch.stack(torch.meshgrid(g, g, indexing="xy"))[None], persistent=False)
        self.fc = nn.Sequential(nn.Flatten(), nn.Linear(32 * (size // 16) ** 2, d), nn.SiLU())

    def forward(self, x):
        x = torch.cat([x, self.xy.expand(x.shape[0], -1, -1, -1)], 1)
        return self.fc(self.conv(x))


class PoseNet(nn.Module):
    def __init__(self, wrist=128, bus=192, lid=128, d=256):
        super().__init__()
        self.wrist, self.bus = Enc(3, wrist, d), Enc(3, bus, d)
        self.lidar = Enc(2, lid, d)
        self.q = nn.Sequential(nn.Linear(12, 64), nn.SiLU())
        self.head = nn.Sequential(nn.Linear(3 * d + 64, 512), nn.SiLU(), nn.Linear(512, 256), nn.SiLU(), nn.Linear(256, 9))
        # target normalisation (set from the training set, saved with the weights)
        self.register_buffer("p_mu", torch.zeros(3))
        self.register_buffer("p_sd", torch.ones(3))

    def forward(self, wrist, bus, lidar, q):
        """wrist, bus: (B, 3, S, S) uint8 or float in [0, 255]; lidar: (B, S, S) mm; q: (B, 6) rad.
        Returns position (B, 3) in m and rot6 (B, 6) in the bus frame."""
        f = lambda im: im.float() / 127.5 - 1.0  # noqa: E731
        r = lidar.float() / 1000.0
        valid = (r > 0).float()
        L = torch.stack([valid * (r - 1.25) / 0.5, valid], 1)
        h = torch.cat([self.wrist(f(wrist)), self.bus(f(bus)), self.lidar(L),
                       self.q(torch.cat([torch.sin(q), torch.cos(q)], 1))], 1)
        o = self.head(h)
        return self.p_mu + self.p_sd * o[:, :3], o[:, 3:]


def rot6_to_R(r6):
    """(..., 6) first two columns -> (..., 3, 3) rotation (Gram-Schmidt). numpy or torch."""
    if isinstance(r6, torch.Tensor):
        a = torch.nn.functional.normalize(r6[..., 0:3], dim=-1)
        b = r6[..., 3:6] - (r6[..., 3:6] * a).sum(-1, keepdim=True) * a
        b = torch.nn.functional.normalize(b, dim=-1)
        return torch.stack([a, b, torch.cross(a, b, dim=-1)], -1)
    a = r6[..., 0:3] / np.linalg.norm(r6[..., 0:3], axis=-1, keepdims=True)
    b = r6[..., 3:6] - (r6[..., 3:6] * a).sum(-1, keepdims=True) * a
    b = b / np.linalg.norm(b, axis=-1, keepdims=True)
    return np.stack([a, b, np.cross(a, b)], -1)


def rot_err_deg(R1, R2):
    """Angle between rotations, torch (..., 3, 3)."""
    tr = (R1.transpose(-1, -2) @ R2).diagonal(dim1=-2, dim2=-1).sum(-1)
    return torch.rad2deg(torch.arccos(torch.clamp((tr - 1) / 2, -1.0, 1.0)))
