#!/usr/bin/env python3
"""
Timestamp KDE from the same source run used by the main resilience figures.

Uses the original bad-leader trial set and shows mirrored 1D KDEs of coverage
at key timestamps for REIP vs Raft.
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
CLR_GRID = "#e3e6eb"
CLR_GUIDE = "#7c8188"


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


def kde_1d(values, y_grid, bandwidth: float = 6.0):
    vals = np.asarray(values, dtype=float)[:, None]
    grid = y_grid[None, :]
    z = (grid - vals) / bandwidth
    dens = np.exp(-0.5 * z * z).sum(axis=0)
    if dens.max() > 0:
        dens = dens / dens.max()
    return dens


def draw_panel(ax, reip_vals, raft_vals, timestamp):
    y_grid = np.linspace(0, 100, 500)
    reip_d = kde_1d(reip_vals, y_grid)
    raft_d = kde_1d(raft_vals, y_grid)
    max_width = 0.16

    for y in range(0, 101, 10):
        ax.axhline(y, color=CLR_GRID, linewidth=0.45, alpha=0.28, zorder=0)

    ax.fill_betweenx(y_grid, -reip_d * max_width, 0,
                     color=CLR_REIP, alpha=0.18, linewidth=0, zorder=1)
    ax.fill_betweenx(y_grid, 0, raft_d * max_width,
                     color=CLR_RAFT, alpha=0.18, linewidth=0, zorder=1)
    ax.plot(-reip_d * max_width, y_grid, color=CLR_REIP, alpha=0.85, linewidth=1.0, zorder=2)
    ax.plot(raft_d * max_width, y_grid, color=CLR_RAFT, alpha=0.85, linewidth=1.0, zorder=2)

    ax.axvline(0, color=CLR_GUIDE, linewidth=0.95, alpha=0.9, zorder=1)

    # light interior dots to show the exact trials behind the density
    rng = np.random.default_rng(1000 + timestamp)
    reip_x = -np.interp(reip_vals, y_grid, reip_d) * max_width * rng.uniform(0.08, 0.95, size=len(reip_vals))
    raft_x = np.interp(raft_vals, y_grid, raft_d) * max_width * rng.uniform(0.08, 0.95, size=len(raft_vals))
    reip_y = np.asarray(reip_vals) + rng.normal(0, 0.45, size=len(reip_vals))
    raft_y = np.asarray(raft_vals) + rng.normal(0, 0.45, size=len(raft_vals))
    ax.scatter(reip_x, reip_y, s=5, color=CLR_REIP, alpha=0.40, edgecolors="none", zorder=3)
    ax.scatter(raft_x, raft_y, s=5, color=CLR_RAFT, alpha=0.40, edgecolors="none", zorder=3)

    reip_med = np.median(reip_vals)
    raft_med = np.median(raft_vals)
    ax.plot([-0.025, -0.002], [reip_med, reip_med], color=CLR_REIP, linewidth=2.2, zorder=4)
    ax.plot([0.002, 0.025], [raft_med, raft_med], color=CLR_RAFT, linewidth=2.2, zorder=4)

    ax.text(-0.085, 103.5, "REIP", color=CLR_REIP, fontsize=9.5, fontweight="bold",
            ha="center", va="bottom")
    ax.text(0.085, 103.5, "Raft", color=CLR_RAFT, fontsize=9.5, fontweight="bold",
            ha="center", va="bottom")

    ax.set_title(f"{timestamp}s", fontweight="bold", pad=10)
    ax.set_xlim(-0.19, 0.19)
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

    fig, axes = plt.subplots(1, len(TIMESTAMPS), figsize=(9.0, 3.2), sharey=True)

    for ax, t in zip(axes, TIMESTAMPS):
        reip_vals = [coverage_at_time(tl, t) for tl in reip_tls]
        raft_vals = [coverage_at_time(tl, t) for tl in raft_tls]
        draw_panel(ax, reip_vals, raft_vals, t)

    axes[0].set_ylabel("Coverage (%)", fontweight="bold")
    fig.suptitle("Coverage KDE by Trial Under Bad-Leader Faults",
                 fontsize=13, fontweight="bold", y=1.02)
    fig.tight_layout(rect=[0, 0.01, 1, 0.95], w_pad=0.75)

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    for ext in ("png", "pdf"):
        out = OUTPUT_DIR / f"fault_timestamp_kde.{ext}"
        fig.savefig(out, facecolor="white", bbox_inches="tight")
        print(f"Saved: {out}")
    plt.close(fig)


if __name__ == "__main__":
    main()
