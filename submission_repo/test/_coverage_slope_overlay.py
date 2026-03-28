#!/usr/bin/env python3
"""
Coverage slope overlay after fault injection.

Compares REIP, Raft, and decentralized controllers on the same axes using
coverage-rate (%/s) curves aligned to the first injected fault.
"""

import argparse
import json
import os
from collections import defaultdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


DEFAULT_BASES = [
    "experiments/run_20260228_014625_multiroom_100trials_all",
]
CONTROLLERS = ["reip", "raft", "decentralized"]
CTRL_LABELS = {
    "reip": "REIP",
    "raft": "Raft",
    "decentralized": "Decentralized",
}
CTRL_COLORS = {
    "reip": "#1f4fff",
    "raft": "#de6464",
    "decentralized": "#1f9d55",
}


plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["Times New Roman", "Times", "DejaVu Serif"],
    "font.size": 10,
    "axes.labelsize": 11,
    "axes.titlesize": 12,
    "legend.fontsize": 9,
    "xtick.labelsize": 9,
    "ytick.labelsize": 9,
    "axes.linewidth": 0.8,
    "figure.dpi": 300,
    "savefig.dpi": 600,
})


def resample(timeline, time_axis):
    out = np.zeros(len(time_axis))
    idx = 0
    for i, t_target in enumerate(time_axis):
        while idx < len(timeline) - 1 and timeline[idx + 1][0] <= t_target:
            idx += 1
        out[i] = timeline[idx][1] if timeline else 0.0
    return out


def smooth(y, window):
    if window <= 1:
        return y
    kernel = np.ones(window, dtype=float) / window
    return np.convolve(y, kernel, mode="same")


def load_timelines(base_dirs, fault_type, fault_schedule_name):
    grouped = defaultdict(list)
    meta = {}

    for base in base_dirs:
        logs_dir = os.path.join(base, "logs")
        if not os.path.isdir(logs_dir):
            continue
        for exp_dir in os.listdir(logs_dir):
            tl_path = os.path.join(logs_dir, exp_dir, "coverage_timeline.json")
            if not os.path.exists(tl_path):
                continue
            with open(tl_path) as f:
                data = json.load(f)

            ctrl = data.get("controller")
            if ctrl not in CONTROLLERS:
                continue
            if (data.get("fault_type") or "none") != fault_type:
                continue

            schedule_name = data.get("fault_schedule_name") or "default"
            if schedule_name != fault_schedule_name:
                continue

            timeline = data.get("timeline") or []
            if not timeline:
                continue

            grouped[ctrl].append(timeline)
            if ctrl not in meta:
                meta[ctrl] = data

    return grouped, meta


def plot_overlay(base_dirs, fault_type, fault_schedule_name, output_name, max_time, dt, smooth_window, output_dir=None):
    grouped, meta = load_timelines(base_dirs, fault_type, fault_schedule_name)
    available = [ctrl for ctrl in CONTROLLERS if grouped.get(ctrl)]
    if len(available) < 2:
        raise RuntimeError("Need at least two controllers with matching timelines to plot slope overlay.")

    output_base = base_dirs[0]
    out_dir = output_dir or os.path.join(output_base, "plots")
    os.makedirs(out_dir, exist_ok=True)

    first_fault_time = min(
        (m.get("fault_time_1") for m in meta.values() if m.get("fault_time_1") is not None),
        default=10.0,
    )
    second_fault_time = min(
        (m.get("fault_time_2") for m in meta.values() if m.get("fault_time_2") is not None),
        default=None,
    )

    t_abs = np.arange(0.0, max_time + dt, dt)
    t_rel = t_abs - first_fault_time
    mask = t_rel >= 0.0

    fig, ax = plt.subplots(figsize=(6.8, 3.2))

    for ctrl in CONTROLLERS:
        tls = grouped.get(ctrl)
        if not tls:
            continue
        resampled = np.array([resample(tl, t_abs) for tl in tls])
        med = np.median(resampled, axis=0)
        q25 = np.percentile(resampled, 25, axis=0)
        q75 = np.percentile(resampled, 75, axis=0)

        slope = smooth(np.gradient(med, dt), smooth_window)
        slope_lo = smooth(np.gradient(q25, dt), smooth_window)
        slope_hi = smooth(np.gradient(q75, dt), smooth_window)

        ax.plot(
            t_rel[mask],
            slope[mask],
            color=CTRL_COLORS[ctrl],
            linewidth=2.7,
            label=CTRL_LABELS[ctrl],
            zorder=3,
        )
        ax.fill_between(
            t_rel[mask],
            slope_lo[mask],
            slope_hi[mask],
            color=CTRL_COLORS[ctrl],
            alpha=0.10,
            linewidth=0,
            zorder=1,
        )

    ax.axvline(0.0, color="#555555", linewidth=1.0, linestyle="--", alpha=0.8, zorder=2)
    ax.text(0.0, ax.get_ylim()[1] * 0.96, "Fault injected", ha="left", va="top",
            fontsize=8.8, color="#333333")
    if second_fault_time is not None:
        rel_second = second_fault_time - first_fault_time
        ax.axvline(rel_second, color="#888888", linewidth=0.95, linestyle=":", alpha=0.8, zorder=2)
        ax.text(rel_second, ax.get_ylim()[1] * 0.88, "Second hit", ha="left", va="top",
                fontsize=8.3, color="#555555")

    ax.axhline(0.0, color="black", linewidth=0.8, alpha=0.28, zorder=1)
    ax.set_xlim(0, max(5.0, max_time - first_fault_time))
    ax.set_xlabel("Seconds Since First Fault")
    ax.set_ylabel("Coverage Rate (%/s)")
    ax.set_title("Coverage Slope Recovery After Fault Injection")
    ax.grid(True, alpha=0.22, linewidth=0.4)
    ax.legend(loc="upper right", frameon=False)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)

    fig.tight_layout(pad=0.6)
    for ext in ("png", "pdf"):
        out = os.path.join(out_dir, f"{output_name}.{ext}")
        fig.savefig(out, bbox_inches="tight", facecolor="white")
        print(f"Saved: {out}")
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description="Coverage slope overlay after fault injection.")
    parser.add_argument("bases", nargs="*", default=DEFAULT_BASES,
                        help="Experiment run directories whose logs/ folders contain coverage_timeline.json files")
    parser.add_argument("--fault", default="bad_leader",
                        help="Fault type to plot (default: bad_leader)")
    parser.add_argument("--schedule", default="default",
                        help="Fault schedule name to filter on (default: default)")
    parser.add_argument("--max-time", type=float, default=70.0,
                        help="Absolute max experiment time to include before alignment (default: 70s)")
    parser.add_argument("--dt", type=float, default=0.5,
                        help="Resample resolution in seconds (default: 0.5)")
    parser.add_argument("--smooth-window", type=int, default=5,
                        help="Moving-average smoothing window for derivatives (default: 5)")
    parser.add_argument("--output-name", default="coverage_slope_overlay",
                        help="Output filename stem inside <base>/plots/")
    parser.add_argument("--output-dir", default=None,
                        help="Directory to save figures (default: <base>/plots)")
    args = parser.parse_args()

    plot_overlay(
        base_dirs=args.bases,
        fault_type=args.fault,
        fault_schedule_name=args.schedule,
        output_name=args.output_name,
        max_time=args.max_time,
        dt=args.dt,
        smooth_window=args.smooth_window,
        output_dir=args.output_dir,
    )


if __name__ == "__main__":
    main()
