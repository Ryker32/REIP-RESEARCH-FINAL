#!/usr/bin/env python3
"""
Generate polished figures for simulated communication-blackout / cyberattack runs.

These figures are for runs where the system is under severe network loss rather
than explicit leader-fault injection.
"""

import argparse
import glob
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


DEFAULT_RUN_DIR = Path("experiments/run_20260319_193103_multiroom_30trials_NoFault")
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
T_MAX = 120.0
DT = 0.5
CHECKPOINTS = [30, 60, 90, 120]


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


def load_timelines(run_dir: Path):
    grouped = {ctrl: [] for ctrl in CONTROLLERS}
    logs_dir = run_dir / "logs"
    for ctrl in CONTROLLERS:
        prefix = f"multiroom_{ctrl}_NoFault_"
        for exp_dir in sorted(logs_dir.iterdir()):
            if not exp_dir.is_dir() or not exp_dir.name.startswith(prefix):
                continue
            tl_path = exp_dir / "coverage_timeline.json"
            if not tl_path.exists():
                continue
            with tl_path.open() as f:
                data = json.load(f)
            if (data.get("fault_type") or "none") != "none":
                continue
            timeline = data.get("timeline") or []
            if timeline:
                grouped[ctrl].append(timeline)
    return grouped


def resample(timeline, time_axis):
    out = np.zeros(len(time_axis))
    idx = 0
    for i, t_target in enumerate(time_axis):
        while idx < len(timeline) - 1 and timeline[idx + 1][0] <= t_target:
            idx += 1
        out[i] = timeline[idx][1] if timeline else 0.0
    return out


def smooth(y, window=5):
    if window <= 1:
        return y
    kernel = np.ones(window) / window
    return np.convolve(y, kernel, mode="same")


def save(fig, output_dir: Path, stem: str):
    output_dir.mkdir(parents=True, exist_ok=True)
    for ext in ("png", "pdf"):
        out = output_dir / f"{stem}.{ext}"
        fig.savefig(out, facecolor="white", bbox_inches="tight")
        print(f"Saved: {out}")
    plt.close(fig)


def build_resampled_arrays(grouped):
    time_axis = np.arange(0.0, T_MAX + DT, DT)
    arrays = {}
    for ctrl, tls in grouped.items():
        if tls:
            arrays[ctrl] = np.array([resample(tl, time_axis) for tl in tls])
    return time_axis, arrays


def plot_convergence(time_axis, arrays, output_dir: Path):
    fig, ax = plt.subplots(figsize=(6.6, 3.3))
    for ctrl in CONTROLLERS:
        if ctrl not in arrays:
            continue
        arr = arrays[ctrl]
        med = np.median(arr, axis=0)
        q25 = np.percentile(arr, 25, axis=0)
        q75 = np.percentile(arr, 75, axis=0)
        ax.plot(time_axis, med, color=CTRL_COLORS[ctrl], linewidth=2.8,
                label=CTRL_LABELS[ctrl], zorder=3)
        ax.fill_between(time_axis, q25, q75, color=CTRL_COLORS[ctrl],
                        alpha=0.11, linewidth=0, zorder=1)
        ax.text(time_axis[-1] + 1.0, med[-1], f"{med[-1]:.1f}%", color=CTRL_COLORS[ctrl],
                fontsize=8.5, va="center")

    ax.set_title("Search Coverage Under Simulated Communication Blackout")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Coverage (%)")
    ax.set_xlim(0, T_MAX)
    ax.set_ylim(0, max(12, max(np.max(a) for a in arrays.values()) + 2))
    ax.grid(True, alpha=0.22, linewidth=0.4)
    ax.legend(loc="upper left", frameon=False)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    fig.tight_layout(pad=0.6)
    save(fig, output_dir, "blackout_convergence")


def plot_velocity(time_axis, arrays, output_dir: Path):
    fig, ax = plt.subplots(figsize=(6.6, 3.3))
    for ctrl in CONTROLLERS:
        if ctrl not in arrays:
            continue
        arr = arrays[ctrl]
        med = np.median(arr, axis=0)
        q25 = np.percentile(arr, 25, axis=0)
        q75 = np.percentile(arr, 75, axis=0)
        slope = smooth(np.gradient(med, DT), 5)
        lo = smooth(np.gradient(q25, DT), 5)
        hi = smooth(np.gradient(q75, DT), 5)
        ax.plot(time_axis, slope, color=CTRL_COLORS[ctrl], linewidth=2.6,
                label=CTRL_LABELS[ctrl], zorder=3)
        ax.fill_between(time_axis, lo, hi, color=CTRL_COLORS[ctrl],
                        alpha=0.10, linewidth=0, zorder=1)

    ax.axhline(0.0, color="black", linewidth=0.8, alpha=0.3, zorder=1)
    ax.set_title("Search Velocity Under Simulated Cyberattack Blackout")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Coverage Rate (%/s)")
    ax.set_xlim(0, T_MAX)
    ax.grid(True, alpha=0.22, linewidth=0.4)
    ax.legend(loc="upper right", frameon=False)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    fig.tight_layout(pad=0.6)
    save(fig, output_dir, "blackout_velocity")


def plot_checkpoints(time_axis, arrays, output_dir: Path):
    fig, ax = plt.subplots(figsize=(6.2, 3.2))
    x = np.arange(len(CHECKPOINTS))
    width = 0.24

    for i, ctrl in enumerate(CONTROLLERS):
        if ctrl not in arrays:
            continue
        arr = arrays[ctrl]
        medians = []
        iqrs = []
        for t in CHECKPOINTS:
            idx = int(round(t / DT))
            vals = arr[:, idx]
            medians.append(np.median(vals))
            iqrs.append((np.percentile(vals, 75) - np.percentile(vals, 25)) / 2.0)
        ax.bar(x + (i - 1) * width, medians, width=width, color=CTRL_COLORS[ctrl],
               alpha=0.90, edgecolor="white", linewidth=0.8, label=CTRL_LABELS[ctrl], zorder=3)
        ax.errorbar(x + (i - 1) * width, medians, yerr=iqrs, fmt="none",
                    ecolor="#333333", elinewidth=0.9, capsize=2.5, zorder=4)

    ax.set_title("Coverage Preserved at Critical Deadlines During Blackout")
    ax.set_xlabel("Mission Time")
    ax.set_ylabel("Coverage (%)")
    ax.set_xticks(x)
    ax.set_xticklabels([f"{t}s" for t in CHECKPOINTS])
    ax.grid(axis="y", alpha=0.22, linewidth=0.4)
    ax.legend(loc="upper left", frameon=False)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    fig.tight_layout(pad=0.6)
    save(fig, output_dir, "blackout_checkpoints")


def plot_final_coverage(results, output_dir: Path):
    grouped = {ctrl: [] for ctrl in CONTROLLERS}
    for r in results:
        if (r.get("fault_type") or "none") != "none":
            continue
        ctrl = r.get("controller")
        if ctrl in grouped:
            grouped[ctrl].append(float(r.get("final_coverage") or 0.0))

    fig, ax = plt.subplots(figsize=(4.4, 3.1))
    x = np.arange(len(CONTROLLERS))
    medians = []
    iqrs = []
    for ctrl in CONTROLLERS:
        vals = np.array(grouped[ctrl], dtype=float)
        medians.append(np.median(vals))
        iqrs.append((np.percentile(vals, 75) - np.percentile(vals, 25)) / 2.0)
    ax.bar(x, medians, color=[CTRL_COLORS[c] for c in CONTROLLERS], alpha=0.90,
           edgecolor="white", linewidth=0.8, zorder=3)
    ax.errorbar(x, medians, yerr=iqrs, fmt="none", ecolor="#333333",
                elinewidth=0.9, capsize=3, zorder=4)
    ax.set_title("End-of-Mission Coverage Under Communication Blackout")
    ax.set_ylabel("Final Coverage (%)")
    ax.set_xticks(x)
    ax.set_xticklabels([CTRL_LABELS[c] for c in CONTROLLERS], fontweight="bold")
    ax.grid(axis="y", alpha=0.22, linewidth=0.4)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    fig.tight_layout(pad=0.6)
    save(fig, output_dir, "blackout_final_coverage")


def main():
    parser = argparse.ArgumentParser(description="Generate polished blackout/cyberattack figures for a run.")
    parser.add_argument("run_dir", nargs="?", default=str(DEFAULT_RUN_DIR),
                        help="Experiment run directory or explicit results_final JSON path")
    parser.add_argument("--output-dir", default=None,
                        help="Directory to save figures (default: <run_dir>/curated_figures)")
    args = parser.parse_args()

    run_path = Path(args.run_dir)
    results_file = resolve_results_file(run_path)
    run_dir = run_path if run_path.is_dir() else results_file.parent
    output_dir = Path(args.output_dir) if args.output_dir else run_dir / "curated_figures"

    results = load_results(results_file)
    grouped = load_timelines(run_dir)
    available = [ctrl for ctrl, tls in grouped.items() if tls]
    if len(available) < 2:
        raise RuntimeError("Need at least two clean-controller timeline sets to plot blackout figures.")

    time_axis, arrays = build_resampled_arrays(grouped)
    plot_convergence(time_axis, arrays, output_dir)
    plot_velocity(time_axis, arrays, output_dir)
    plot_checkpoints(time_axis, arrays, output_dir)
    plot_final_coverage(results, output_dir)


if __name__ == "__main__":
    main()
