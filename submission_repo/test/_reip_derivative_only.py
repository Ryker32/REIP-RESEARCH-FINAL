#!/usr/bin/env python3
"""
REIP-only derivative figure showing all three conditions on one panel.
"""

import json
import os
import sys
from collections import defaultdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["Times New Roman", "Times", "DejaVu Serif"],
    "font.size": 10,
    "axes.labelsize": 11,
    "axes.titlesize": 12,
    "legend.fontsize": 9,
    "xtick.labelsize": 9,
    "ytick.labelsize": 9,
    "axes.linewidth": 0.7,
    "figure.dpi": 300,
    "savefig.dpi": 600,
})


BASES = sys.argv[1:] if len(sys.argv) > 1 else ["experiments/run_20260228_014625_multiroom_100trials_all"]
BASE = BASES[0]
T_MAX = 120.0
DT = 0.5
TIME_AXIS = np.arange(0, T_MAX + DT, DT)
MARKER_EVERY = 20

FAULT_COLORS = {
    "none": "#1a1a1a",
    "bad_leader": "#e74c3c",
    "freeze_leader": "#3498db",
}
FAULT_MARKERS = {
    "none": "o",
    "bad_leader": "x",
    "freeze_leader": "D",
}
FAULT_LABELS = {
    "none": "Clean",
    "bad_leader": "Bad Leader",
    "freeze_leader": "Freeze Leader",
}


def resample(timeline, time_axis):
    result = np.zeros(len(time_axis))
    idx = 0
    for i, t_target in enumerate(time_axis):
        while idx < len(timeline) - 1 and timeline[idx + 1][0] <= t_target:
            idx += 1
        if idx < len(timeline):
            result[i] = timeline[idx][1]
        elif timeline:
            result[i] = timeline[-1][1]
    return result


def load_condition_data():
    timelines = defaultdict(list)
    for b in BASES:
        logs_dir = os.path.join(b, "logs")
        if not os.path.isdir(logs_dir):
            continue
        for exp_dir in sorted(os.listdir(logs_dir)):
            tl_file = os.path.join(logs_dir, exp_dir, "coverage_timeline.json")
            if not os.path.exists(tl_file):
                continue
            with open(tl_file) as f:
                data = json.load(f)
            ctrl = data["controller"]
            fault = data.get("fault_type") or "none"
            if fault == "oscillate_leader":
                continue
            timelines[(ctrl, fault)].append(data["timeline"])

    condition_data = {}
    for (ctrl, fault), tls in timelines.items():
        resampled = np.array([resample(tl, TIME_AXIS) for tl in tls])
        condition_data[(ctrl, fault)] = resampled
    return condition_data


def main():
    condition_data = load_condition_data()
    fig, ax = plt.subplots(figsize=(5.3, 3.2))

    for fault in ["none", "bad_leader", "freeze_leader"]:
        key = ("reip", fault)
        if key not in condition_data:
            continue
        resampled = condition_data[key]
        med = np.median(resampled, axis=0)
        derivative = np.gradient(med, DT)
        derivative = np.convolve(derivative, np.ones(5) / 5, mode="same")

        ax.plot(
            TIME_AXIS, derivative, linestyle="-", color=FAULT_COLORS[fault], linewidth=2.8,
            marker=FAULT_MARKERS[fault], markevery=MARKER_EVERY, markersize=5.5,
            label=FAULT_LABELS[fault], zorder=3
        )

    for ft, label in [(10, "Fault 1"), (30, "Fault 2")]:
        ax.axvline(x=ft, color="#666666", linewidth=1.3, linestyle="--", alpha=0.7, zorder=2)
        ax.text(ft + 0.8, ax.get_ylim()[1] * 0.96, label, ha="left", va="top",
                fontsize=8.5, fontweight="bold", color="#333333")

    ax.axhline(y=0, color="black", linewidth=0.8, linestyle="-", alpha=0.3, zorder=1)
    ax.set_xlim(0, 70)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Coverage Rate (%/s)")
    ax.set_title("REIP Search Velocity Across Fault Conditions", fontweight="bold", pad=10)
    ax.grid(True, alpha=0.28, linewidth=0.45)
    ax.legend(loc="upper right", frameon=False)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)

    fig.tight_layout(pad=0.6)
    out_dir = os.path.join("paper_docs", "Ryker_Kollmyer___UPDATED_PAPER")
    os.makedirs(out_dir, exist_ok=True)
    for ext in ("png", "pdf"):
        out = os.path.join(out_dir, f"reip_derivative_only.{ext}")
        fig.savefig(out, bbox_inches="tight", facecolor="white")
        print(f"Saved: {out}")
    plt.close(fig)


if __name__ == "__main__":
    main()
