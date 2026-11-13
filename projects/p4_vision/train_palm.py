"""P4.8: train the palm-camera offset network (palm_net.PalmNet) on gen_palm.py data.
CPU only (laptop thermal limit: no sustained GPU training), --threads 8. The last 15% of the
episodes (by seed) are validation. Approach frames only by default: the correction is used
before the jaws close (in close and hold the jaws hide the bar). Photometric augmentation only
(no shifts or flips: where the bar sits in the image is the signal). Saves best.pt (lowest
validation median position error) and last.pt; resumable from last.pt.

  bash run_p4.sh train_palm.py --data /home/shrey31/p4_data/palm_v1 --out /home/shrey31/p4_data/palm_net_v1
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parent))
from palm_net import PalmNet  # noqa: E402
from pose_net import rot6_to_R, rot_err_deg  # noqa: E402


def load(files, phases):
    P, Y = [], []
    for f in files:
        z = np.load(f)
        m = np.isin(z["phase"], phases)
        P.append(z["palm"][m])
        Y.append(z["rel"][m])
    return np.concatenate(P), np.concatenate(Y).astype(np.float32)


@torch.no_grad()
def evaluate(net, P, Y, bs=256):
    net.eval()
    ep, er = [], []
    for i in range(0, len(P), bs):
        x = torch.from_numpy(P[i:i + bs]).permute(0, 3, 1, 2)
        y = torch.from_numpy(Y[i:i + bs])
        p, r6 = net(x)
        ep.append((p - y[:, :3]).numpy())
        er.append(rot_err_deg(rot6_to_R(r6), rot6_to_R(y[:, 3:])).numpy())
    ep, er = np.concatenate(ep) * 1000, np.concatenate(er)
    n = np.linalg.norm(ep, axis=1)
    near = Y[:, 0] < 0.06  # the last 6 cm, where centring matters
    return dict(pos_mm_med=float(np.median(n)), pos_mm_p95=float(np.percentile(n, 95)),
                axis_mm_med=[float(v) for v in np.median(np.abs(ep), 0)],
                near_pos_mm_med=float(np.median(n[near])) if near.any() else float("nan"),
                near_z_mm_p95=float(np.percentile(np.abs(ep[near, 2]), 95)) if near.any() else float("nan"),
                rot_deg_med=float(np.median(er)), n=int(len(n)), n_near=int(near.sum()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--epochs", type=int, default=20)
    ap.add_argument("--bs", type=int, default=64)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--threads", type=int, default=8)
    ap.add_argument("--phases", default="2", help="phase indices used (2 approach, 3 close, 4 hold)")
    a = ap.parse_args()
    torch.set_num_threads(a.threads)
    torch.manual_seed(0)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    files = sorted(Path(a.data).glob("ep_*.npz"))
    files = [f for f in files if not f.name.endswith(".tmp.npz")]
    n_val = max(1, round(0.15 * len(files)))
    phases = [int(p) for p in a.phases.split(",")]
    Ptr, Ytr = load(files[:-n_val], phases)
    Pva, Yva = load(files[-n_val:], phases)
    print(f"{len(files) - n_val} train episodes, {len(Ptr)} frames; {n_val} val episodes, {len(Pva)} frames; "
          f"label x {Ytr[:, 0].min() * 1000:.0f}..{Ytr[:, 0].max() * 1000:.0f} mm, |y| <= {np.abs(Ytr[:, 1]).max() * 1000:.0f}, "
          f"|z| <= {np.abs(Ytr[:, 2]).max() * 1000:.0f}", flush=True)
    net = PalmNet()
    net.p_mu.copy_(torch.from_numpy(Ytr[:, :3].mean(0)))
    net.p_sd.copy_(torch.from_numpy(Ytr[:, :3].std(0)))
    opt = torch.optim.AdamW(net.parameters(), lr=a.lr, weight_decay=1e-4)
    steps = a.epochs * (len(Ptr) // a.bs)
    sched = torch.optim.lr_scheduler.OneCycleLR(opt, max_lr=a.lr, total_steps=steps, pct_start=0.1)
    ep0, best, hist = 0, np.inf, []
    last = out / "last.pt"
    if last.exists():
        s = torch.load(last, map_location="cpu")
        net.load_state_dict(s["net"]); opt.load_state_dict(s["opt"]); sched.load_state_dict(s["sched"])
        ep0, best, hist = s["epoch"], s["best"], s["hist"]
        print(f"resumed after epoch {ep0}", flush=True)
    g = torch.Generator().manual_seed(1)
    Xtr, Ytr_t = torch.from_numpy(Ptr), torch.from_numpy(Ytr)
    for ep in range(ep0, a.epochs):
        net.train()
        t0, tot, nb = time.time(), 0.0, 0
        perm = torch.randperm(len(Xtr), generator=g)
        for i in range(0, len(perm) - a.bs + 1, a.bs):
            idx = perm[i:i + a.bs]
            x = Xtr[idx].permute(0, 3, 1, 2).float()
            # exposure, contrast and noise (the rig already varies them per episode; this adds per-frame variety)
            gain = torch.empty(len(idx), 1, 1, 1).uniform_(0.7, 1.3, generator=g)
            off = torch.empty(len(idx), 1, 1, 1).uniform_(-15, 15, generator=g)
            x = (x * gain + off + torch.randn(x.shape, generator=g) * 3.0).clamp(0, 255)
            y = Ytr_t[idx]
            p, r6 = net(x)
            loss = torch.nn.functional.smooth_l1_loss((p - net.p_mu) / net.p_sd, (y[:, :3] - net.p_mu) / net.p_sd, beta=0.1) \
                + torch.nn.functional.mse_loss(r6, y[:, 3:])
            opt.zero_grad(set_to_none=True)
            loss.backward()
            opt.step()
            sched.step()
            tot, nb = tot + float(loss), nb + 1
        v = evaluate(net, Pva, Yva)
        hist.append(dict(epoch=ep + 1, loss=tot / nb, **v))
        tag = ""
        if v["pos_mm_med"] < best:
            best, tag = v["pos_mm_med"], " *"
            torch.save(net.state_dict(), out / "best.pt")
        torch.save(dict(net=net.state_dict(), opt=opt.state_dict(), sched=sched.state_dict(), epoch=ep + 1, best=best, hist=hist), last)
        (out / "hist.json").write_text(json.dumps(hist, indent=1))
        print(f"epoch {ep + 1:2d} loss {tot / nb:.4f} val pos {v['pos_mm_med']:.2f} / {v['pos_mm_p95']:.2f} mm (x {v['axis_mm_med'][0]:.2f} "
              f"y {v['axis_mm_med'][1]:.2f} z {v['axis_mm_med'][2]:.2f}) near {v['near_pos_mm_med']:.2f} mm, z p95 {v['near_z_mm_p95']:.2f}; "
              f"rot {v['rot_deg_med']:.2f} deg; {(time.time() - t0) / 60:.1f} min{tag}", flush=True)


if __name__ == "__main__":
    main()
