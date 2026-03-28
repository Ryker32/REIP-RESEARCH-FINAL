#!/usr/bin/env python3
"""
SHAP-style mirrored beeswarm for trial coverage distributions.

Each panel uses discrete coverage bins as rows:
  - row = coverage bin
  - dot = one trial
  - left side = REIP
  - right side = Raft
  - horizontal spread = density within that coverage bin
"""

import json
import os
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


DATA_DIR = Path("experiments/run_20260228_014625_multiroom_100trials_all/logs")
OUTPUT_DIR = Path("paper_docs/Ryker_Kollmyer___UPDATED_PAPER")
TIMESTAMPS = [30, 40, 45, 50]

CLR_REIP = "#1f4fff"
CLR_RAFT = "#de6464"
CLR_GUIDE = "#7c8188"
CLR_GRID = "#e3e6eb"

BIN_SIZE = 10.0
ROW_CENTERS = np.arange(BIN_SIZE / 2, 100, BIN_SIZE)  # 5, 15, ..., 95


plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["Times New Roman", "Times", "DejaVu Serif"],
    "font.size": 10,
    "axes.labelsize": 11,
    "axes.titlesize": 11,
    "xtick.labelsize": 9,
    "ytick.labelsize": 9,
    "axes.linewidth": 0.7,
    "figure.dpi": 300,
    "savefig.dpi": 600,
})


def load_timelines(controller: str, fault_tag: str = "BadLeader"):
    timelines = []
    prefix = f"multiroom_{controller}_{fault_tag}_"
    for exp_dir in sorted(os.listdir(DATA_DIR)):
        if not exp_dir.startswith(prefix):
            continue
        tl_file = DATA_DIR / exp_dir / "coverage_timeline.json"
        if not tl_file.exists():
            continue
        with tl_file.open() as f:
            data = json.load(f)
        tl = data.get("timeline", [])
        if tl:
            timelines.append(tl)
    return timelines


def coverage_at_time(timeline, target_t: float):
    idx = 0
    while idx < len(timeline) - 1 and timeline[idx + 1][0] <= target_t:
        idx += 1
    return timeline[idx][1] if timeline else 0.0


def bin_center(value: float):
    clipped = min(max(value, 0.0), 99.999)
    bin_idx = int(clipped // BIN_SIZE)
    return ROW_CENTERS[bin_idx]


def shap_like_offsets(n: int, step: float = 0.0064):
    """Pack dots outward from the center like a mirrored beeswarm."""
    if n <= 0:
        return np.array([])
    positions = [0.0]
    k = 1
    while len(positions) < n:
        positions.append(k * step)
        if len(positions) < n:
            positions.append((k + 0.35) * step)
        k += 1
    return np.array(positions[:n])


def kde_density(values, y_grid, bandwidth: float = 4.5):
    """Simple Gaussian KDE without scipy."""
    vals = np.asarray(values, dtype=float)[:, None]
    grid = y_grid[None, :]
    z = (grid - vals) / bandwidth
    dens = np.exp(-0.5 * z * z).sum(axis=0)
    if dens.max() > 0:
        dens = dens / dens.max()
    return dens


def make_swarm(values, side: str, seed: int):
    values = np.asarray(values, dtype=float)
    y = np.array([bin_center(v) for v in values], dtype=float)
    x = np.zeros(len(values), dtype=float)
    y_out = y.copy()
    rng = np.random.default_rng(seed)

    # preserve a stable order within each row so the outward bloom looks smooth
    row_to_indices = {}
    for i, yc in enumerate(y):
        row_to_indices.setdefault(yc, []).append(i)

    for yc, idxs in row_to_indices.items():
        idxs_sorted = sorted(idxs, key=lambda i: values[i])
        offsets = shap_like_offsets(len(idxs_sorted))
        if side == "left":
            offsets = -offsets
        for idx, off in zip(idxs_sorted, offsets):
            x[idx] = off
            y_out[idx] = y[idx] + rng.uniform(-BIN_SIZE * 0.22, BIN_SIZE * 0.22)

    return x, y_out


def draw_panel(ax, reip_vals, raft_vals, timestamp):
    reip_x, reip_y = make_swarm(reip_vals, "left", 100 + timestamp)
    raft_x, raft_y = make_swarm(raft_vals, "right", 200 + timestamp)

    for y in range(0, 101, 10):
        ax.axhline(y, color=CLR_GRID, linewidth=0.45, alpha=0.28, zorder=0)

    ax.axvline(0, color=CLR_GUIDE, linewidth=0.95, alpha=0.9, zorder=1)

    ax.scatter(
        reip_x, reip_y, s=9, marker="o", linewidths=0,
        color=CLR_REIP, alpha=0.92, zorder=3
    )
    ax.scatter(
        raft_x, raft_y, s=9, marker="o", linewidths=0,
        color=CLR_RAFT, alpha=0.92, zorder=3
    )

    ax.text(-0.063, 103.5, "REIP", color=CLR_REIP, fontsize=9.5, fontweight="bold",
            ha="center", va="bottom")
    ax.text(0.063, 103.5, "Raft", color=CLR_RAFT, fontsize=9.5, fontweight="bold",
            ha="center", va="bottom")

    ax.set_title(f"{timestamp}s", fontweight="bold", pad=10)
    ax.set_xlim(-0.14, 0.14)
    ax.set_ylim(0, 106)
    ax.set_xticks([])
    ax.set_yticks(np.arange(0, 101, 10))
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.spines["bottom"].set_visible(False)
    ax.spines["left"].set_linewidth(0.8)


def main():
    reip_tls = load_timelines("reip")
    raft_tls = load_timelines("raft")

    fig, axes = plt.subplots(1, len(TIMESTAMPS), figsize=(9.4, 3.2), sharey=True)

    for ax, t in zip(axes, TIMESTAMPS):
        reip_vals = [coverage_at_time(tl, t) for tl in reip_tls]
        raft_vals = [coverage_at_time(tl, t) for tl in raft_tls]
        draw_panel(ax, reip_vals, raft_vals, t)

    axes[0].set_ylabel("Coverage (%)", fontweight="bold")
    fig.suptitle("Coverage Distribution by Trial Under Bad-Leader Faults",
                 fontsize=13, fontweight="bold", y=1.02)
    fig.tight_layout(rect=[0, 0.01, 1, 0.95], w_pad=0.75)

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    for ext in ("png", "pdf"):
        out = OUTPUT_DIR / f"fault_shap_style_swarm.{ext}"
        fig.savefig(out, facecolor="white", bbox_inches="tight")
        print(f"Saved: {out}")
    plt.close(fig)


if __name__ == "__main__":
    main()
