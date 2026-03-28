#!/usr/bin/env python3
"""
Poster-quality split raincloud plot for bad-leader trial coverage.

Three panels:
  - 40s
  - 45s
  - 50s

Left half-cloud = REIP
Right half-cloud = Raft
"""

import argparse
import json
import os
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


DEFAULT_RUN_DIR = Path("experiments/run_20260228_014625_multiroom_100trials_all")
TIMESTAMPS = [40, 45, 50]

CLR_REIP = "#1f4fff"
CLR_RAFT = "#de6464"
CLR_GRID = "#d8dde5"
CLR_GUIDE = "#757b84"


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


def load_timelines(logs_dir: Path, controller: str, condition_tag: str = "BadLeader"):
    timelines = []
    prefix = f"multiroom_{controller}_{condition_tag}_"
    for exp_dir in sorted(os.listdir(logs_dir)):
        if not exp_dir.startswith(prefix):
            continue
        tl_file = logs_dir / exp_dir / "coverage_timeline.json"
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


def kde_density(values, y_grid, bandwidth: float = 4.6):
    vals = np.asarray(values, dtype=float)[:, None]
    grid = y_grid[None, :]
    z = (grid - vals) / bandwidth
    dens = np.exp(-0.5 * z * z).sum(axis=0)
    if dens.max() > 0:
        dens = dens / dens.max()
    return dens


def local_widths(values, y_grid, density, max_width):
    """Map each value to the local half-violin width at that y."""
    widths = np.interp(values, y_grid, density) * max_width
    return np.maximum(widths, 0.003)


def dot_positions(values, side, y_grid, density, max_width, seed):
    rng = np.random.default_rng(seed)
    values = np.asarray(values, dtype=float)
    widths = local_widths(values, y_grid, density, max_width)

    # sample inside the local density shape; concentrate points near the center
    mag = widths * (rng.beta(2.0, 5.0, size=len(values)))
    x = -mag if side == "left" else mag
    y = values + rng.normal(0, 0.28, size=len(values))
    return x, y


def median_iqr(vals):
    vals = np.asarray(vals, dtype=float)
    return np.median(vals), np.percentile(vals, 25), np.percentile(vals, 75)


def draw_panel(ax, reip_vals, raft_vals, timestamp, seed_offset=0):
    y_grid = np.linspace(0, 100, 500)
    max_width = 0.19
    reip_d = kde_density(reip_vals, y_grid)
    raft_d = kde_density(raft_vals, y_grid)

    # soft grid
    for y in range(0, 101, 10):
        ax.axhline(y, color=CLR_GRID, linewidth=0.45, alpha=0.26, zorder=0)

    # split half-violins
    ax.fill_betweenx(y_grid, -reip_d * max_width, 0,
                     color=CLR_REIP, alpha=0.18, linewidth=0, zorder=1)
    ax.fill_betweenx(y_grid, 0, raft_d * max_width,
                     color=CLR_RAFT, alpha=0.18, linewidth=0, zorder=1)

    ax.axvline(0, color=CLR_GUIDE, linewidth=0.95, alpha=0.9, zorder=2)

    rx, ry = dot_positions(reip_vals, "left", y_grid, reip_d, max_width, 100 + seed_offset)
    bx, by = dot_positions(raft_vals, "right", y_grid, raft_d, max_width, 200 + seed_offset)
    ax.scatter(rx, ry, s=12, color=CLR_REIP, alpha=0.92, edgecolors="none", zorder=3)
    ax.scatter(bx, by, s=12, color=CLR_RAFT, alpha=0.92, edgecolors="none", zorder=3)

    # subtle median markers
    for side, vals, color in [("left", reip_vals, CLR_REIP), ("right", raft_vals, CLR_RAFT)]:
        med, q1, q3 = median_iqr(vals)
        x0 = -0.11 if side == "left" else 0.11
        ax.plot([x0, x0], [q1, q3], color=color, linewidth=2.1, zorder=4, solid_capstyle="round")
        ax.plot([x0 - 0.018, x0 + 0.018], [med, med], color="black", linewidth=1.5, zorder=5)

    ax.text(-0.11, 103.2, "REIP", color=CLR_REIP, fontsize=9.5, fontweight="bold",
            ha="center", va="bottom")
    ax.text(0.11, 103.2, "Raft", color=CLR_RAFT, fontsize=9.5, fontweight="bold",
            ha="center", va="bottom")

    ax.set_title(f"{timestamp}s", fontweight="bold", pad=8)
    ax.set_xlim(-0.22, 0.22)
    ax.set_ylim(0, 106)
    ax.set_xticks([])
    ax.set_yticks([0, 20, 40, 60, 80, 100])
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.spines["bottom"].set_visible(False)
    ax.spines["left"].set_linewidth(0.8)


def main():
    parser = argparse.ArgumentParser(description="Generate the split raincloud figure from a run directory.")
    parser.add_argument("run_dir", nargs="?", default=str(DEFAULT_RUN_DIR),
                        help="Experiment run directory containing logs/")
    parser.add_argument("--output-dir", default=None,
                        help="Directory to save figures (default: <run_dir>/curated_figures)")
    parser.add_argument("--condition-tag", default="BadLeader",
                        help="Experiment condition tag prefix inside logs/ (default: BadLeader)")
    args = parser.parse_args()

    run_dir = Path(args.run_dir)
    logs_dir = run_dir / "logs"
    output_dir = Path(args.output_dir) if args.output_dir else run_dir / "curated_figures"

    reip_tls = load_timelines(logs_dir, "reip", args.condition_tag)
    raft_tls = load_timelines(logs_dir, "raft", args.condition_tag)
    if not reip_tls or not raft_tls:
        raise RuntimeError(
            f"Need matching {args.condition_tag} logs for both REIP and Raft in {logs_dir}"
        )

    fig, axes = plt.subplots(1, len(TIMESTAMPS), figsize=(7.4, 3.45), sharey=True)

    for i, (ax, t) in enumerate(zip(axes, TIMESTAMPS)):
        reip_vals = [coverage_at_time(tl, t) for tl in reip_tls]
        raft_vals = [coverage_at_time(tl, t) for tl in raft_tls]
        draw_panel(ax, reip_vals, raft_vals, t, seed_offset=i)

    axes[0].set_ylabel("Coverage (%)", fontweight="bold")
    fig.suptitle("Coverage Distribution Across Trials Under Bad-Leader Faults",
                 fontsize=13, fontweight="bold", y=1.02)
    fig.tight_layout(rect=[0, 0.01, 1, 0.96], w_pad=0.9)

    output_dir.mkdir(parents=True, exist_ok=True)
    for ext in ("png", "pdf"):
        out = output_dir / f"fault_raincloud.{ext}"
        fig.savefig(out, facecolor="white", bbox_inches="tight")
        print(f"Saved: {out}")
    plt.close(fig)


if __name__ == "__main__":
    main()
