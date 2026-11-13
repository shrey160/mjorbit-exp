"""P4.4d checkpoint selection (SPEC_4 4.4): offline action error of a LeRobot ACT checkpoint on
the validation episodes of a converted dataset (to_lerobot.py sidecar index, split "val").
For every k-th frame: predict the chunk, compare its first action (and the whole chunk) with
the recorded expert labels. Reports the grip-target position error [mm] and rotation error
[deg] per phase (phase from the source npz), and close-flag accuracy. Runs in the LeRobot env.

python val_act.py <pretrained_model dir> <dataset root> [--every 5] [--device cuda] [--out f.json]
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import torch

PH = ("settle", "reach", "approach", "close", "hold")


def rot6_to_R(r6):
    a, b = r6[..., 0:3], r6[..., 3:6]
    x = a / np.linalg.norm(a, axis=-1, keepdims=True)
    b = b - (x * b).sum(-1, keepdims=True) * x
    y = b / np.linalg.norm(b, axis=-1, keepdims=True)
    return np.stack([x, y, np.cross(x, y)], -1)


def rot_err_deg(Ra, Rb):
    c = (np.einsum("...ij,...ij->...", Ra, Rb) - 1.0) / 2.0
    return np.degrees(np.arccos(np.clip(c, -1.0, 1.0)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ckpt")
    ap.add_argument("root")
    ap.add_argument("--every", type=int, default=5)
    ap.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    ap.add_argument("--bs", type=int, default=64)
    ap.add_argument("--max-episodes", type=int, default=0)
    ap.add_argument("--out", default="")
    a = ap.parse_args()
    from lerobot.datasets import LeRobotDataset
    from lerobot.policies.act.modeling_act import ACTPolicy
    from lerobot.policies.factory import make_pre_post_processors

    root = Path(a.root)
    index = [e for e in json.loads((root.parent / f"{root.name}_episodes.json").read_text()) if e["split"] == "val"]
    if a.max_episodes:
        index = index[:a.max_episodes]
    ds = LeRobotDataset("local/mjorbit_demos_v4", root=root, episodes=[e["episode"] for e in index])
    policy = ACTPolicy.from_pretrained(a.ckpt).to(a.device).eval()
    pre, post = make_pre_post_processors(policy.config, pretrained_path=a.ckpt,
                                         preprocessor_overrides={"device_processor": {"device": a.device}})
    H = policy.config.chunk_size
    keys = [k for k in ds.features if k.startswith("observation.")]
    # positions of each episode inside this (filtered) dataset
    epi = np.asarray(ds.hf_dataset["episode_index"])
    E, A, C, P, Ech = [], [], [], [], []
    for e in index:
        rows = np.flatnonzero(epi == e["episode"])
        # labels from the dataset itself; phase from the index (add_phase_index.py), so this
        # also runs on Colab without the source npz files
        act = np.stack([np.asarray(x) for x in ds.hf_dataset.select(rows.tolist())["action"]]).astype(np.float32)
        phase = np.asarray(e["phase"], np.int8)
        assert len(phase) == len(rows), (e["episode"], len(phase), len(rows))
        sel = list(range(0, len(rows), a.every))
        for s in range(0, len(sel), a.bs):
            fr = sel[s:s + a.bs]
            items = [ds[int(rows[i])] for i in fr]
            batch = {k: torch.stack([it[k] for it in items]) for k in keys}
            batch["task"] = [it["task"] for it in items]
            with torch.no_grad():
                obs = pre(batch)
                chunk = policy.predict_action_chunk(obs)  # (B, H, 10), normalised
                chunk = post(chunk).cpu().numpy()
            for j, i in enumerate(fr):
                lab = act[i:i + H]
                pred = chunk[j, :len(lab)]
                E.append(np.linalg.norm(pred[0, :3] - lab[0, :3]) * 1000)
                A.append(rot_err_deg(rot6_to_R(pred[0, 3:9]), rot6_to_R(lab[0, 3:9])))
                C.append((pred[0, 9] > 0.5) == (lab[0, 9] > 0.5))
                Ech.append(np.linalg.norm(pred[:, :3] - lab[:, :3], axis=1).mean() * 1000)
                P.append(phase[i])
    E, A, C, P, Ech = map(np.asarray, (E, A, C, P, Ech))
    rep = {"all": dict(n=int(len(E)), pos_mm_med=float(np.median(E)), pos_mm_p95=float(np.percentile(E, 95)),
                       rot_deg_med=float(np.median(A)), close_acc=float(C.mean()), chunk_pos_mm_mean=float(Ech.mean()))}
    for k, n in enumerate(PH):
        m = P == k
        if m.any():
            rep[n] = dict(n=int(m.sum()), pos_mm_med=float(np.median(E[m])), pos_mm_p95=float(np.percentile(E[m], 95)),
                          rot_deg_med=float(np.median(A[m])), close_acc=float(C[m].mean()),
                          chunk_pos_mm_mean=float(Ech[m].mean()))
    rep["ckpt"] = a.ckpt
    for k, v in rep.items():
        if isinstance(v, dict):
            print(f"{k:>9}: n {v['n']:5d}  pos {v['pos_mm_med']:6.1f} / {v['pos_mm_p95']:6.1f} mm  rot {v['rot_deg_med']:.2f} deg"
                  f"  close acc {v['close_acc']:.3f}  chunk pos mean {v['chunk_pos_mm_mean']:6.1f} mm")
    if a.out:
        Path(a.out).write_text(json.dumps(rep, indent=1))


if __name__ == "__main__":
    main()
