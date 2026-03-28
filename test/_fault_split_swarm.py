#!/usr/bin/env python3
"""
Poster-quality SHAP-style mirrored split beeswarm coverage figure.

Each panel shows per-trial coverage at a fixed timestamp under bad-leader faults:
  left  = REIP
  right = Raft
  one dot = one trial
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
CLR_GRID = "#d8dce2"
CLR_GUIDE = "#6b7078"


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


def beeswarm_offsets(values, side: str, row_step: float = 0.022, y_bin: float = 2.2):
    """
    SHAP-like density packing:
    points sharing nearby y-values stack outward from the centerline.
    """
    values = np.asarray(values, dtype=float)
    order = np.argsort(values)
    x = np.zeros(len(values), dtype=float)
    bins = {}
    for idx in order:
        key = int(round(values[idx] / y_bin))
        bins.setdefault(key, []).append(idx)

    for key, idxs in bins.items():
        positions = [0.0]
        k = 1
        while len(positions) < len(idxs):
            positions.append(k * row_step)
            if len(positions) < len(idxs):
                positions.append((k + 0.35) * row_step)
            k += 1
        positions = np.array(positions[:len(idxs)])
        if side == "left":
            positions = -positions
        idxs_sorted = sorted(idxs, key=lambda i: values[i])
        for i, pos in zip(idxs_sorted, positions):
            x[i] = pos
    return x


def draw_panel(ax, reip_vals, raft_vals, t_label):
    reip_x = beeswarm_offsets(reip_vals, "left")
    raft_x = beeswarm_offsets(raft_vals, "right")

    # Very light horizontal guides
    for y in range(0, 101, 10):
        ax.axhline(y, color=CLR_GRID, linewidth=0.45, alpha=0.22, zorder=0)

    # Center divider
    ax.axvline(0, color=CLR_GUIDE, linewidth=0.95, alpha=0.9, zorder=1)

    # Trial dots: small, dense, SHAP-like
    ax.scatter(reip_x, reip_vals, s=10, color=CLR_REIP, alpha=0.92,
               edgecolors="none", zorder=3)
    ax.scatter(raft_x, raft_vals, s=10, color=CLR_RAFT, alpha=0.92,
               edgecolors="none", zorder=3)

    # Header labels inside panel
    ax.text(-0.105, 103.5, "REIP", color=CLR_REIP, fontsize=9.5, fontweight="bold",
            ha="center", va="bottom")
    ax.text(0.105, 103.5, "Raft", color=CLR_RAFT, fontsize=9.5, fontweight="bold",
            ha="center", va="bottom")

    ax.set_title(f"Coverage at {t_label}s", fontweight="bold", pad=10)
    ax.set_xlim(-0.17, 0.17)
    ax.set_ylim(0, 106)
    ax.set_xticks([])
    ax.set_yticks([0, 20, 40, 60, 80, 100])
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.spines["bottom"].set_visible(False)
    ax.spines["left"].set_linewidth(0.8)


def main():
    reip_tls = load_timelines("reip")
    raft_tls = load_timelines("raft")

    fig, axes = plt.subplots(1, len(TIMESTAMPS), figsize=(9.3, 3.4), sharey=True)

    for ax, t in zip(axes, TIMESTAMPS):
        reip_vals = [coverage_at_time(tl, t) for tl in reip_tls]
        raft_vals = [coverage_at_time(tl, t) for tl in raft_tls]
        draw_panel(ax, reip_vals, raft_vals, t)

    axes[0].set_ylabel("Coverage (%)", fontweight="bold")
    fig.suptitle("Coverage Distributions Across 100 Trials Under Bad-Leader Faults",
                 fontsize=13, fontweight="bold", y=1.02)
    fig.tight_layout(rect=[0, 0.02, 1, 0.97], w_pad=0.9)

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    for ext in ("png", "pdf"):
        out = OUTPUT_DIR / f"fault_split_swarm.{ext}"
        fig.savefig(out, facecolor="white", bbox_inches="tight")
        print(f"Saved: {out}")
    plt.close(fig)


if __name__ == "__main__":
    main()
