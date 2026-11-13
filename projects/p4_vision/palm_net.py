"""P4.8 palm-camera offset network: one palm_cam image -> the bar's position (3) [m] and
rotation (6, first two matrix columns) in the grip-point frame. The image shows both pads and
the bar between them, so the answer does not depend on any camera's mount calibration.
Same encoder as the P4.3 estimator (pose_net.Enc: flattened feature map, CoordConv)."""
from __future__ import annotations

import torch
from torch import nn

from pose_net import Enc


class PalmNet(nn.Module):
    def __init__(self, size=128, d=256):
        super().__init__()
        self.enc = Enc(3, size, d)
        self.head = nn.Sequential(nn.Linear(d, 256), nn.SiLU(), nn.Linear(256, 9))
        self.register_buffer("p_mu", torch.zeros(3))  # label normalisation, saved with the weights
        self.register_buffer("p_sd", torch.ones(3))

    def forward(self, img):
        """img: (B, 3, S, S) uint8 or float in [0, 255]. Returns position (B, 3) m, rot6 (B, 6)."""
        o = self.head(self.enc(img.float() / 127.5 - 1.0))
        return self.p_mu + self.p_sd * o[:, :3], o[:, 3:]
