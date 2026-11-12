"""P4.3: train the bar-pose estimator (pose_net.PoseNet) on a packed demo set.
python train_pose.py --pack /home/.../demos_v1_pack --out /home/.../pose_v1 [--epochs 20]
Episodes < --n-train train; the rest select the checkpoint (validation, by episode).
Reports the bar position / rotation error per phase on the validation episodes."""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np
import torch
from torch.utils.data import DataLoader, Dataset, Sampler

from pose_net import PoseNet, rot6_to_R, rot_err_deg

PH = ("settle", "reach", "approach", "close", "hold")


class Batches(Sampler):
    def __init__(self, idx, bs, shuffle, seed=0):
        self.idx, self.bs, self.shuffle = idx, bs, shuffle
        self.rng = np.random.default_rng(seed)

    def __iter__(self):
        idx = self.rng.permutation(self.idx) if self.shuffle else self.idx
        for i in range(0, len(idx), self.bs):
            yield np.sort(idx[i:i + self.bs])

    def __len__(self):
        return (len(self.idx) + self.bs - 1) // self.bs


class Pack(Dataset):
    """Returns a whole batch per call (sorted memmap reads)."""

    def __init__(self, d):
        self.d = Path(d)
        self.mm = None

    def _open(self):
        self.mm = {k: np.load(self.d / f"{k}.npy", mmap_mode="r")
                   for k in ("wrist", "bus", "lidar", "state", "priv", "phase")}

    def __getitem__(self, b):
        if self.mm is None:
            self._open()
        m = self.mm
        img = lambda k: torch.from_numpy(np.ascontiguousarray(m[k][b])).permute(0, 3, 1, 2)  # noqa: E731
        return dict(wrist=img("wrist"), bus=img("bus"),
                    lidar=torch.from_numpy(m["lidar"][b].astype(np.int32)),
                    q=torch.from_numpy(m["state"][b][:, 0:6].copy()),
                    p=torch.from_numpy(m["priv"][b][:, 0:3].copy()), r6=torch.from_numpy(m["priv"][b][:, 3:9].copy()),
                    phase=torch.from_numpy(m["phase"][b].astype(np.int64)))


def augment(x, dev):
    """Training-only photometric augmentation on the GPU (v2, 12 Nov). No geometric shifts or
    crops: where the bar sits in the image IS the pose signal, so moving the image would
    corrupt the label. Per sample and camera: brightness and contrast 0.8-1.2, colour gain
    0.9-1.1 per channel, pixel noise 0-4 DN. Lidar: extra dropout 0-5%, range noise 0-5 mm."""
    y = dict(x)
    for k in ("wrist", "bus"):
        im = x[k].to(dev, non_blocking=True).float()
        B = im.shape[0]
        u = lambda lo, hi, *s: torch.empty(B, *s, device=dev).uniform_(lo, hi)  # noqa: E731
        mean = im.mean((1, 2, 3), keepdim=True)
        im = (im - mean) * u(0.8, 1.2, 1, 1, 1) + mean * u(0.8, 1.2, 1, 1, 1)
        im = im * u(0.9, 1.1, 3, 1, 1) + torch.randn_like(im) * u(0.0, 4.0, 1, 1, 1)
        y[k] = im.clamp(0, 255)
    L = x["lidar"].to(dev, non_blocking=True).float()
    B = L.shape[0]
    noise = torch.randn_like(L) * torch.empty(B, 1, 1, device=dev).uniform_(0, 5)
    drop = torch.rand_like(L) < torch.empty(B, 1, 1, device=dev).uniform_(0, 0.05)
    y["lidar"] = torch.where((L > 0) & ~drop, L + noise, torch.zeros_like(L))
    return y


def loss_fn(net, x, dev):
    p, r6 = net(x["wrist"].to(dev, non_blocking=True), x["bus"].to(dev, non_blocking=True),
                x["lidar"].to(dev, non_blocking=True), x["q"].to(dev))
    pt, rt = x["p"].to(dev), x["r6"].to(dev)
    lp = torch.nn.functional.smooth_l1_loss((p - pt) / net.p_sd, torch.zeros_like(p), beta=0.05)
    lr = ((r6 - rt) ** 2).sum(1).mean()
    return lp + 10.0 * lr, p, r6


@torch.no_grad()
def evaluate(net, dl, dev):
    net.eval()
    E, A, P = [], [], []
    for x in dl:
        _, p, r6 = loss_fn(net, x, dev)
        E.append((p - x["p"].to(dev)).norm(dim=1).cpu())
        A.append(rot_err_deg(rot6_to_R(r6), rot6_to_R(x["r6"].to(dev))).cpu())
        P.append(x["phase"])
    net.train()
    E, A, P = (torch.cat(v).numpy() for v in (E, A, P))
    rep = {"all": dict(pos_mm_med=float(np.median(E) * 1000), pos_mm_p95=float(np.percentile(E, 95) * 1000),
                       rot_deg_med=float(np.median(A)), rot_deg_p95=float(np.percentile(A, 95)))}
    for k, n in enumerate(PH):
        m = P == k
        if m.any():
            rep[n] = dict(pos_mm_med=float(np.median(E[m]) * 1000), pos_mm_p95=float(np.percentile(E[m], 95) * 1000),
                          rot_deg_med=float(np.median(A[m])), rot_deg_p95=float(np.percentile(A[m], 95)))
    return rep


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pack", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--epochs", type=int, default=20)
    ap.add_argument("--bs", type=int, default=128)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--n-train", type=int, default=450)
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--max-steps", type=int, default=0, help="stop after this many steps (smoke test)")
    ap.add_argument("--augment", type=int, default=1, help="photometric augmentation (1) or none (0)")
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    dev = "cuda"
    torch.backends.cudnn.benchmark = True
    # bf16 needs Ampere or newer (RTX 4060, L4, A100); a T4 (Colab free tier) uses fp16 + loss scaling
    amp = torch.bfloat16 if torch.cuda.get_device_capability()[0] >= 8 else torch.float16
    ep = np.load(Path(a.pack) / "ep.npy")
    priv = np.load(Path(a.pack) / "priv.npy", mmap_mode="r")
    tr, va = np.flatnonzero(ep < a.n_train), np.flatnonzero(ep >= a.n_train)
    ds = Pack(a.pack)
    mk = lambda idx, sh: DataLoader(ds, sampler=Batches(idx, a.bs, sh), batch_size=None, num_workers=a.workers,  # noqa: E731
                                    pin_memory=True, persistent_workers=True, prefetch_factor=4)
    dl_tr, dl_va = mk(tr, True), mk(va[::2], False)
    net = PoseNet().to(dev)
    P = np.asarray(priv[tr][:, 0:3])
    net.p_mu[:] = torch.tensor(P.mean(0), dtype=torch.float32)
    net.p_sd[:] = torch.tensor(P.std(0), dtype=torch.float32)
    opt = torch.optim.AdamW(net.parameters(), lr=a.lr, weight_decay=1e-4)
    steps = a.epochs * len(dl_tr) if not a.max_steps else a.max_steps
    sched = torch.optim.lr_scheduler.OneCycleLR(opt, a.lr, total_steps=steps, pct_start=0.05)
    scaler = torch.amp.GradScaler()
    print(f"GPU {torch.cuda.get_device_name(0)}, autocast {amp}, augment {a.augment}", flush=True)
    print(f"train frames {len(tr)} (episodes < {a.n_train}), val frames {len(va[::2])}; "
          f"params {sum(p.numel() for p in net.parameters()) / 1e6:.2f}M; steps {steps}", flush=True)
    best, step, t0, log, e0 = np.inf, 0, time.time(), [], 0
    if (out / "state.pt").exists():  # resume after an interruption (power loss, 12 Nov)
        S = torch.load(out / "state.pt", map_location=dev, weights_only=False)
        net.load_state_dict(S["net"])
        opt.load_state_dict(S["opt"])
        sched.load_state_dict(S["sched"])
        scaler.load_state_dict(S["scaler"])
        best, step, log, e0 = S["best"], S["step"], S["log"], S["epoch"]
        print(f"resumed after epoch {e0}, step {step}", flush=True)
    for e in range(e0, a.epochs):
        dl_tr.sampler.rng = np.random.default_rng(e)  # same batches whether resumed or not
        run = 0.0
        for i, x in enumerate(dl_tr):
            if a.augment:
                x = augment(x, dev)
            with torch.autocast("cuda", amp):
                loss, _, _ = loss_fn(net, x, dev)
            opt.zero_grad(set_to_none=True)
            scaler.scale(loss).backward()
            scaler.step(opt)
            scaler.update()
            sched.step()
            run += loss.item()
            step += 1
            if step % 200 == 0:
                print(f"  step {step}/{steps} loss {run / (i + 1):.4f} {(time.time() - t0) / 60:.1f} min", flush=True)
            if step >= steps:
                break
        rep = evaluate(net, dl_va, dev)
        rep.update(epoch=e + 1, train_loss=run / max(i + 1, 1), min=round((time.time() - t0) / 60, 1))
        log.append(rep)
        score = rep["approach"]["pos_mm_med"] + rep["all"]["pos_mm_med"] if "approach" in rep else rep["all"]["pos_mm_med"]
        tag = ""
        if score < best:
            best = score
            torch.save(net.state_dict(), out / "best.pt")
            tag = " *"
        print(f"epoch {e + 1}: " + "; ".join(f"{k} {v['pos_mm_med']:.1f}/{v['pos_mm_p95']:.1f} mm "
                                             f"{v['rot_deg_med']:.2f}/{v['rot_deg_p95']:.2f} deg"
                                             for k, v in rep.items() if isinstance(v, dict)) + tag, flush=True)
        (out / "log.json").write_text(json.dumps(log, indent=1))
        torch.save(dict(net=net.state_dict(), opt=opt.state_dict(), sched=sched.state_dict(),
                        scaler=scaler.state_dict(), best=best, step=step, log=log, epoch=e + 1),
                   out / "state.tmp.pt")
        (out / "state.tmp.pt").replace(out / "state.pt")
        if step >= steps:
            break
    torch.save(net.state_dict(), out / "last.pt")


if __name__ == "__main__":
    main()
