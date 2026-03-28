#!/usr/bin/env python3
"""
Packed-dot pictogram for bad-leader success counts.

One dot = one successful trial reaching >=80% coverage.
No text labels inside the figure, per user request.
"""

import argparse
import glob
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


DEFAULT_RUN_DIR = Path("experiments/run_20260228_014625_multiroom_100trials_all")
SUCCESS_THRESHOLD = 80.0
CLR_REIP = "#71c7f2"
CLR_RAFT = "#ef6a6a"


plt.rcParams.update({
    "figure.dpi": 300,
    "savefig.dpi": 600,
    "axes.linewidth": 0.0,
})


def resolve_results_file(path: Path) -> Path:
    if path.is_file():
        return path
    matches = sorted(glob.glob(str(path / "results_final_*.json")))
    if not matches:
        raise FileNotFoundError(f"No results_final_*.json found in {path}")
    return Path(matches[-1])


def load_results(path: Path):
    with path.open() as f:
        return json.load(f)


def success_count(results, controller: str) -> tuple[int, int]:
    subset = [
        r for r in results
        if r.get("controller") == controller
        and (r.get("fault_type") or "none") == "bad_leader"
    ]
    successes = sum(1 for r in subset if (r.get("final_coverage") or 0) >= SUCCESS_THRESHOLD)
    return successes, len(subset)


def hex_pack(count: int, spacing: float = 1.0):
    pts = []
    row = 0
    while len(pts) < count:
        n_cols = 7 + (row % 2)
        x_offset = 0.5 * spacing if row % 2 else 0.0
        for col in range(n_cols):
            if len(pts) >= count:
                break
            x = col * spacing + x_offset
            y = row * spacing * 0.86
            pts.append((x, y))
        row += 1
    pts = np.array(pts, dtype=float)
    pts[:, 0] -= np.mean(pts[:, 0])
    pts[:, 1] -= np.mean(pts[:, 1])
    return pts


def draw_blob(ax, count: int, color: str):
    pts = hex_pack(count, spacing=1.0)
    ax.scatter(pts[:, 0], pts[:, 1], s=28, color=color, edgecolors="white", linewidths=0.3)
    ax.text(0, 0, str(count), ha="center", va="center",
            fontsize=18, fontweight="bold", color="#111111")
    pad = 1.1
    ax.set_xlim(np.min(pts[:, 0]) - pad, np.max(pts[:, 0]) + pad)
    ax.set_ylim(np.min(pts[:, 1]) - pad, np.max(pts[:, 1]) + pad)
    ax.set_aspect("equal")
    ax.axis("off")


def main():
    parser = argparse.ArgumentParser(description="Generate packed-dot pictogram for >=80% success counts.")
    parser.add_argument("run_dir", nargs="?", default=str(DEFAULT_RUN_DIR),
                        help="Experiment run directory or explicit results_final JSON path")
    parser.add_argument("--output-dir", default="paper_docs/Ryker_Kollmyer___UPDATED_PAPER",
                        help="Directory to save the figure")
    args = parser.parse_args()

    run_path = Path(args.run_dir)
    results_file = resolve_results_file(run_path)
    output_dir = Path(args.output_dir)

    results = load_results(results_file)
    reip_count, reip_total = success_count(results, "reip")
    raft_count, raft_total = success_count(results, "raft")
    if reip_total == 0 or raft_total == 0:
        raise RuntimeError("Need bad_leader trials for both REIP and Raft.")

    fig, axes = plt.subplots(1, 2, figsize=(3.1, 1.6))
    draw_blob(axes[0], reip_count, CLR_REIP)
    draw_blob(axes[1], raft_count, CLR_RAFT)
    fig.tight_layout(pad=0.2, w_pad=0.7)

    output_dir.mkdir(parents=True, exist_ok=True)
    for ext in ("png", "pdf"):
        out = output_dir / f"fault_plum_pudding.{ext}"
        fig.savefig(out, facecolor="white", bbox_inches="tight", pad_inches=0.02)
        print(f"Saved: {out}")
    plt.close(fig)


if __name__ == "__main__":
    main()
