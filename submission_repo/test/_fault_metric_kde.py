#!/usr/bin/env python3
"""
Metric KDEs from the original bad-leader dataset used by resilience_contrast.

This is a true metric-density figure, not a beeswarm hybrid:
  - x axis = metric value
  - y axis = density
  - REIP vs Raft shown as overlapping KDE curves
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
PANELS = [
    ("Coverage at 40s", 40.0, "cov40"),
    ("Coverage at 50s", 50.0, "cov50"),
    ("Final Coverage", None, "final"),
]

CLR_REIP = "#1f4fff"
CLR_RAFT = "#de6464"
CLR_GRID = "#e3e6eb"


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


def kde_1d(values, x_grid, bandwidth=4.8):
    vals = np.asarray(values, dtype=float)[:, None]
    grid = x_grid[None, :]
    z = (grid - vals) / bandwidth
    dens = np.exp(-0.5 * z * z).sum(axis=0) / (len(vals) * bandwidth * np.sqrt(2.0 * np.pi))
    return dens


def load_final_coverage(controller: str):
    with RESULTS_FILE.open() as f:
        results = json.load(f)
    return [
        float(r.get("final_coverage") or 0.0)
        for r in results
        if r.get("controller") == controller and (r.get("fault_type") or "none") == "bad_leader"
    ]


def panel_values(controller: str, t_or_none):
    if t_or_none is None:
        return load_final_coverage(controller)
    tls = load_timelines(controller)
    return [coverage_at_time(tl, t_or_none) for tl in tls]


def draw_panel(ax, title, t_or_none, y_max):
    reip = panel_values("reip", t_or_none)
    raft = panel_values("raft", t_or_none)
    x_grid = np.linspace(0, 100, 500)
    reip_d = kde_1d(reip, x_grid)
    raft_d = kde_1d(raft, x_grid)

    # Highlight the "good outcome" region directly.
    hi_mask = x_grid >= 80.0
    ax.axvspan(80, 100, color="#f3f4f7", alpha=0.9, zorder=0)
    ax.text(0.98, 0.06, ">=80% region", transform=ax.transAxes,
            ha="right", va="bottom", fontsize=8.0, color="#666666")

    ax.fill_between(x_grid, 0, reip_d, color=CLR_REIP, alpha=0.20, linewidth=0, zorder=1)
    ax.fill_between(x_grid, 0, raft_d, color=CLR_RAFT, alpha=0.20, linewidth=0, zorder=1)
    ax.plot(x_grid, reip_d, color=CLR_REIP, linewidth=2.0, label="REIP", zorder=3)
    ax.plot(x_grid, raft_d, color=CLR_RAFT, linewidth=2.0, label="Raft", zorder=3)
    ax.fill_between(x_grid[hi_mask], 0, reip_d[hi_mask], color=CLR_REIP, alpha=0.36, linewidth=0, zorder=2)
    ax.fill_between(x_grid[hi_mask], 0, raft_d[hi_mask], color=CLR_RAFT, alpha=0.36, linewidth=0, zorder=2)

    reip_mean = float(np.mean(reip))
    raft_mean = float(np.mean(raft))
    ax.axvline(reip_mean, color=CLR_REIP, linewidth=1.2, linestyle="--", alpha=0.9, zorder=2)
    ax.axvline(raft_mean, color=CLR_RAFT, linewidth=1.2, linestyle="--", alpha=0.9, zorder=2)
    delta = reip_mean - raft_mean
    ax.text(0.98, 0.95, f"Delta mean: {delta:+.1f} pp", transform=ax.transAxes,
            ha="right", va="top", fontsize=8.5, fontweight="bold", color="#333333")
    reip_hi = 100.0 * np.mean(np.asarray(reip) >= 80.0)
    raft_hi = 100.0 * np.mean(np.asarray(raft) >= 80.0)
    ax.text(0.98, 0.84, f">=80%: {reip_hi:.0f}% vs {raft_hi:.0f}%", transform=ax.transAxes,
            ha="right", va="top", fontsize=8.3, fontweight="bold", color="#333333")

    ax.set_title(title, fontweight="bold", pad=8)
    ax.set_xlim(0, 100)
    ax.set_ylim(0, y_max)
    ax.set_xlabel("Coverage (%)")
    ax.grid(True, axis="x", alpha=0.20, linewidth=0.4)
    ax.set_yticks([])
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.spines["left"].set_visible(False)


def main():
    panel_data = []
    global_ymax = 0.0
    for _, t_or_none, _ in PANELS:
        reip = panel_values("reip", t_or_none)
        raft = panel_values("raft", t_or_none)
        x_grid = np.linspace(0, 100, 500)
        reip_d = kde_1d(reip, x_grid)
        raft_d = kde_1d(raft, x_grid)
        global_ymax = max(global_ymax, float(np.max(reip_d)), float(np.max(raft_d)))
        panel_data.append((reip, raft))
    global_ymax *= 1.10

    fig, axes = plt.subplots(1, len(PANELS), figsize=(8.8, 3.0), sharey=True)
    for ax, (title, t_or_none, _) in zip(axes, PANELS):
        draw_panel(ax, title, t_or_none, global_ymax)
    axes[0].legend(loc="upper left", frameon=False)
    fig.suptitle("High-Coverage Density Under Bad-Leader Faults",
                 fontsize=13, fontweight="bold", y=1.03)
    fig.tight_layout(rect=[0, 0, 1, 0.95], w_pad=0.85)

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    for ext in ("png", "pdf"):
        out = OUTPUT_DIR / f"fault_metric_kde.{ext}"
        fig.savefig(out, facecolor="white", bbox_inches="tight")
        print(f"Saved: {out}")
    plt.close(fig)


if __name__ == "__main__":
    main()
