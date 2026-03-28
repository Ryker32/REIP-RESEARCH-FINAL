#!/usr/bin/env python3
"""
Early-window bad-leader comparison: REIP vs Raft, 20-40s.
"""

import json
import os
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


BASE_DIR = Path("experiments/run_20260228_014625_multiroom_100trials_all")
OUTPUT_DIR = Path("paper_docs/Ryker_Kollmyer___UPDATED_PAPER")
T_MIN = 20.0
T_MAX = 40.0
DT = 0.5
TIME_AXIS = np.arange(T_MIN, T_MAX + DT, DT)

CLR_REIP = "#0015B7"
CLR_RAFT = "#E91D2F"


plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["Times New Roman", "Times", "DejaVu Serif"],
    "font.size": 11,
    "axes.labelsize": 12,
    "axes.titlesize": 13,
    "legend.fontsize": 10,
    "xtick.labelsize": 10,
    "ytick.labelsize": 10,
    "axes.linewidth": 1.2,
    "figure.dpi": 300,
    "savefig.dpi": 600,
})


def coverage_at_time(timeline, target_t):
    idx = 0
    while idx < len(timeline) - 1 and timeline[idx + 1][0] <= target_t:
        idx += 1
    return timeline[idx][1] if timeline else 0.0


def resample(timeline):
    return np.array([coverage_at_time(timeline, t) for t in TIME_AXIS], dtype=float)


def load_timelines(controller):
    logs_dir = BASE_DIR / "logs"
    timelines = []
    prefix = f"multiroom_{controller}_BadLeader_"
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


def main():
    reip = np.array([resample(tl) for tl in load_timelines("reip")])
    raft = np.array([resample(tl) for tl in load_timelines("raft")])

    fig, ax = plt.subplots(figsize=(5.4, 3.35))

    for arr, color, label, marker in [
        (reip, CLR_REIP, "REIP", "o"),
        (raft, CLR_RAFT, "Raft", "s"),
    ]:
        med = np.median(arr, axis=0)
        q25 = np.percentile(arr, 25, axis=0)
        q75 = np.percentile(arr, 75, axis=0)
        ax.fill_between(TIME_AXIS, q25, q75, color=color, alpha=0.14, linewidth=0, zorder=1)
        ax.plot(
            TIME_AXIS, med, color=color, linewidth=3.0,
            marker=marker, markersize=5.5, markevery=6, label=label, zorder=3
        )

    for ft in [10, 30]:
        ax.axvline(ft, color="#666666", linewidth=1.1, linestyle="--", alpha=0.7, zorder=2)

    ax.set_title("Coverage Divergence After Leader Faults", fontweight="bold", pad=9)
    ax.set_xlabel("Time (s)", fontweight="bold")
    ax.set_ylabel("Coverage (%)", fontweight="bold")
    ax.set_xlim(20, 40)
    ax.set_ylim(0, 100)
    ax.grid(True, alpha=0.22, linewidth=0.4)
    ax.legend(loc="upper left", frameon=False)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.spines["left"].set_linewidth(1.2)
    ax.spines["bottom"].set_linewidth(1.2)
    ax.tick_params(axis="both", width=1.1, length=4)

    fig.tight_layout(pad=0.6)
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    for ext in ("png", "pdf"):
        out = OUTPUT_DIR / f"fault_early_window_compare.{ext}"
        fig.savefig(out, facecolor="white", bbox_inches="tight")
        print(f"Saved: {out}")
    plt.close(fig)


if __name__ == "__main__":
    main()
