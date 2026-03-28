#!/usr/bin/env python3
"""
2D KDE contour figure from the original bad-leader dataset used by
`resilience_contrast.png`.

Each trial contributes one point:
  x = coverage at 50s
  y = final coverage
"""

import json
import os
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


DATA_DIR = Path("experiments/run_20260228_014625_multiroom_100trials_all/logs")
RESULTS_FILE = Path("experiments/run_20260228_014625_multiroom_100trials_all/results_final_20260228_035139.json")
OUTPUT_DIR = Path("paper_docs/Ryker_Kollmyer___UPDATED_PAPER")
SNAPSHOT_T = 50.0

CLR_REIP = "#1f4fff"
CLR_RAFT = "#de6464"


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
        timeline = data.get("timeline", [])
        if timeline:
            timelines.append((data.get("trial"), timeline))
    return timelines


def load_final_coverage(controller: str):
    with RESULTS_FILE.open() as f:
        results = json.load(f)
    return {
        r.get("trial"): float(r.get("final_coverage") or 0.0)
        for r in results
        if r.get("controller") == controller and (r.get("fault_type") or "none") == "bad_leader"
    }


def coverage_at_time(timeline, target_t: float):
    idx = 0
    while idx < len(timeline) - 1 and timeline[idx + 1][0] <= target_t:
        idx += 1
    return timeline[idx][1] if timeline else 0.0


def paired_metric_points(controller: str):
    finals = load_final_coverage(controller)
    xs, ys = [], []
    for trial, tl in load_timelines(controller):
        if trial not in finals:
            continue
        xs.append(coverage_at_time(tl, SNAPSHOT_T))
        ys.append(finals[trial])
    return np.asarray(xs, dtype=float), np.asarray(ys, dtype=float)


def kde2d(points_x, points_y, x_grid, y_grid, bw_x=5.5, bw_y=5.5):
    px = np.asarray(points_x, dtype=float)[:, None, None]
    py = np.asarray(points_y, dtype=float)[:, None, None]
    X = x_grid[None, :, :]
    Y = y_grid[None, :, :]
    zx = (X - px) / bw_x
    zy = (Y - py) / bw_y
    dens = np.exp(-0.5 * (zx * zx + zy * zy)).sum(axis=0)
    if np.max(dens) > 0:
        dens = dens / np.max(dens)
    return dens


def rgba_steps(color):
    return [
        matplotlib.colors.to_rgba(color, 0.10),
        matplotlib.colors.to_rgba(color, 0.18),
        matplotlib.colors.to_rgba(color, 0.28),
        matplotlib.colors.to_rgba(color, 0.40),
        matplotlib.colors.to_rgba(color, 0.56),
        matplotlib.colors.to_rgba(color, 0.76),
    ]


def draw_panel(ax, x_vals, y_vals, color, title):
    x_mesh = np.linspace(0, 100, 240)
    y_mesh = np.linspace(0, 100, 240)
    X, Y = np.meshgrid(x_mesh, y_mesh)
    dens = kde2d(x_vals, y_vals, X, Y)
    levels = [0.08, 0.18, 0.32, 0.50, 0.72, 0.90]
    ax.contourf(
        X, Y, dens, levels=levels + [1.01],
        colors=rgba_steps(color), zorder=1
    )
    ax.contour(
        X, Y, dens, levels=levels,
        colors=[matplotlib.colors.to_rgba(color, 0.85)],
        linewidths=0.7, zorder=2
    )
    ax.scatter(x_vals, y_vals, s=10, color=color, alpha=0.18, edgecolors="none", zorder=3)

    mean_x = float(np.mean(x_vals))
    mean_y = float(np.mean(y_vals))
    ax.axvline(mean_x, color=color, linestyle="--", linewidth=1.0, alpha=0.8, zorder=4)
    ax.axhline(mean_y, color=color, linestyle="--", linewidth=1.0, alpha=0.8, zorder=4)
    ax.text(0.98, 0.95, f"mean: ({mean_x:.1f}, {mean_y:.1f})",
            transform=ax.transAxes, ha="right", va="top",
            fontsize=8.2, color="#333333", fontweight="bold")

    ax.set_title(title, color=color, fontweight="bold", pad=8)
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 100)
    ax.grid(True, alpha=0.15, linewidth=0.35)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.set_xlabel(f"Coverage at {int(SNAPSHOT_T)}s (%)")


def main():
    reip_x, reip_y = paired_metric_points("reip")
    raft_x, raft_y = paired_metric_points("raft")

    fig, axes = plt.subplots(1, 2, figsize=(7.2, 3.25), sharex=True, sharey=True)
    draw_panel(axes[0], reip_x, reip_y, CLR_REIP, "REIP")
    draw_panel(axes[1], raft_x, raft_y, CLR_RAFT, "Raft")
    axes[0].set_ylabel("Final Coverage (%)")

    fig.suptitle("2D KDE of Mid-Mission vs Final Coverage Under Bad-Leader Faults",
                 fontsize=13, fontweight="bold", y=1.03)
    fig.tight_layout(rect=[0, 0, 1, 0.95], w_pad=0.7)

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    for ext in ("png", "pdf"):
        out = OUTPUT_DIR / f"fault_2d_kde.{ext}"
        fig.savefig(out, facecolor="white", bbox_inches="tight")
        print(f"Saved: {out}")
    plt.close(fig)


if __name__ == "__main__":
    main()
