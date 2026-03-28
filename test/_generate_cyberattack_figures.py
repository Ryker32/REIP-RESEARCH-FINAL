#!/usr/bin/env python3
"""
Generate polished figures for the sustained repeated-fault cyberattack run.

These plots are designed for the `cyber_burst` schedule where bad-leader faults
are injected repeatedly to simulate an ongoing adversarial attack.
"""

import argparse
import glob
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


DEFAULT_RUN_DIR = Path("experiments/run_20260319_193101_multiroom_30trials_BadLeader")
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
CHECKPOINTS = [30, 45, 60, 90, 120]
ATTACK_START = 10.0


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
        prefix = f"multiroom_{ctrl}_BadLeader_"
        for exp_dir in sorted(logs_dir.iterdir()):
            if not exp_dir.is_dir() or not exp_dir.name.startswith(prefix):
                continue
            tl_path = exp_dir / "coverage_timeline.json"
            if not tl_path.exists():
                continue
            with tl_path.open() as f:
                data = json.load(f)
            if (data.get("fault_type") or "none") != "bad_leader":
                continue
            if (data.get("fault_schedule_name") or "default") != "cyber_burst":
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


def save(fig, output_dir: Path, stem: str):
    output_dir.mkdir(parents=True, exist_ok=True)
    for ext in ("png", "pdf"):
        out = output_dir / f"{stem}.{ext}"
        fig.savefig(out, facecolor="white", bbox_inches="tight")
        print(f"Saved: {out}")
    plt.close(fig)


def kde2d(points_t, points_y, x_grid, y_grid, bw_t=6.0, bw_y=7.5):
    """Simple separable Gaussian KDE without scipy."""
    tx = np.asarray(points_t, dtype=float)[:, None, None]
    ty = np.asarray(points_y, dtype=float)[:, None, None]
    X = x_grid[None, :, :]
    Y = y_grid[None, :, :]
    z_t = (X - tx) / bw_t
    z_y = (Y - ty) / bw_y
    dens = np.exp(-0.5 * (z_t * z_t + z_y * z_y)).sum(axis=0)
    if np.max(dens) > 0:
        dens = dens / np.max(dens)
    return dens


def build_arrays(grouped):
    time_axis = np.arange(0.0, T_MAX + DT, DT)
    arrays = {}
    for ctrl, tls in grouped.items():
        if tls:
            arrays[ctrl] = np.array([resample(tl, time_axis) for tl in tls])
    return time_axis, arrays


def plot_convergence(time_axis, arrays, output_dir: Path):
    fig, ax = plt.subplots(figsize=(6.6, 3.25))
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
        ax.text(time_axis[-1] + 1.0, med[-1], f"{med[-1]:.0f}%", color=CTRL_COLORS[ctrl],
                fontsize=8.5, va="center")

    for ft, label in [(10, "Attack begins"), (13, "Burst starts")]:
        ax.axvline(ft, color="#666666", linewidth=0.9, linestyle="--", alpha=0.7, zorder=2)
        ax.text(ft + 0.6, ax.get_ylim()[1] * 0.95, label, fontsize=8.2, color="#444444",
                ha="left", va="top")

    ax.set_title("Coverage Under Sustained Cyberattack")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Coverage (%)")
    ax.set_xlim(0, T_MAX)
    ax.grid(True, alpha=0.22, linewidth=0.4)
    ax.legend(loc="upper left", frameon=False)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    fig.tight_layout(pad=0.6)
    save(fig, output_dir, "cyberattack_convergence")


def plot_checkpoints(time_axis, arrays, output_dir: Path):
    fig, ax = plt.subplots(figsize=(6.5, 3.2))
    x = np.arange(len(CHECKPOINTS))
    width = 0.24

    for i, ctrl in enumerate(CONTROLLERS):
        if ctrl not in arrays:
            continue
        arr = arrays[ctrl]
        meds = []
        iqrs = []
        for t in CHECKPOINTS:
            idx = int(round(t / DT))
            vals = arr[:, idx]
            meds.append(np.median(vals))
            iqrs.append((np.percentile(vals, 75) - np.percentile(vals, 25)) / 2.0)
        ax.bar(x + (i - 1) * width, meds, width=width, color=CTRL_COLORS[ctrl],
               alpha=0.90, edgecolor="white", linewidth=0.8, label=CTRL_LABELS[ctrl], zorder=3)
        ax.errorbar(x + (i - 1) * width, meds, yerr=iqrs, fmt="none", ecolor="#333333",
                    elinewidth=0.9, capsize=2.5, zorder=4)

    ax.set_title("Coverage Preserved During Repeated Cyberattack Bursts")
    ax.set_xlabel("Mission Time")
    ax.set_ylabel("Coverage (%)")
    ax.set_xticks(x)
    ax.set_xticklabels([f"{t}s" for t in CHECKPOINTS])
    ax.grid(axis="y", alpha=0.22, linewidth=0.4)
    ax.legend(loc="upper left", frameon=False)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    fig.tight_layout(pad=0.6)
    save(fig, output_dir, "cyberattack_checkpoints")


def plot_final_coverage(results, output_dir: Path):
    grouped = {ctrl: [] for ctrl in CONTROLLERS}
    for r in results:
        if (r.get("fault_type") or "none") != "bad_leader":
            continue
        if (r.get("fault_schedule_name") or "default") != "cyber_burst":
            continue
        ctrl = r.get("controller")
        if ctrl in grouped:
            grouped[ctrl].append(float(r.get("final_coverage") or 0.0))

    fig, ax = plt.subplots(figsize=(4.4, 3.1))
    x = np.arange(len(CONTROLLERS))
    meds = []
    iqrs = []
    for ctrl in CONTROLLERS:
        vals = np.array(grouped[ctrl], dtype=float)
        meds.append(np.median(vals))
        iqrs.append((np.percentile(vals, 75) - np.percentile(vals, 25)) / 2.0)
    ax.bar(x, meds, color=[CTRL_COLORS[c] for c in CONTROLLERS], alpha=0.90,
           edgecolor="white", linewidth=0.8, zorder=3)
    ax.errorbar(x, meds, yerr=iqrs, fmt="none", ecolor="#333333",
                elinewidth=0.9, capsize=3, zorder=4)
    ax.set_title("End-of-Mission Coverage After Sustained Cyberattack")
    ax.set_ylabel("Final Coverage (%)")
    ax.set_xticks(x)
    ax.set_xticklabels([CTRL_LABELS[c] for c in CONTROLLERS], fontweight="bold")
    ax.grid(axis="y", alpha=0.22, linewidth=0.4)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    fig.tight_layout(pad=0.6)
    save(fig, output_dir, "cyberattack_final_coverage")


def plot_time_coverage_kde(time_axis, arrays, output_dir: Path):
    rel_time = time_axis - ATTACK_START
    mask = rel_time >= 0
    x_vals = rel_time[mask]
    x_mesh = np.linspace(0, max(x_vals), 220)
    y_mesh = np.linspace(0, 100, 220)
    X, Y = np.meshgrid(x_mesh, y_mesh)

    fig, axes = plt.subplots(1, 3, figsize=(9.0, 3.2), sharey=True)
    for ax, ctrl in zip(axes, CONTROLLERS):
        if ctrl not in arrays:
            ax.set_visible(False)
            continue

        arr = arrays[ctrl][:, mask]
        pts_t = np.tile(x_vals, arr.shape[0])
        pts_y = arr.reshape(-1)
        dens = kde2d(pts_t, pts_y, X, Y)

        levels = [0.08, 0.18, 0.32, 0.50, 0.72, 0.90]
        base = CTRL_COLORS[ctrl]
        ax.contourf(X, Y, dens, levels=levels + [1.01], cmap=None,
                    colors=[
                        matplotlib.colors.to_rgba(base, 0.10),
                        matplotlib.colors.to_rgba(base, 0.18),
                        matplotlib.colors.to_rgba(base, 0.28),
                        matplotlib.colors.to_rgba(base, 0.40),
                        matplotlib.colors.to_rgba(base, 0.56),
                        matplotlib.colors.to_rgba(base, 0.76),
                    ])
        med = np.median(arr, axis=0)
        ax.plot(x_vals, med, color=base, linewidth=1.6, zorder=3)

        ax.set_title(CTRL_LABELS[ctrl], color=base, fontweight="bold", pad=6)
        ax.set_xlim(0, max(x_vals))
        ax.set_ylim(0, 100)
        ax.grid(True, alpha=0.16, linewidth=0.35)
        ax.spines["top"].set_visible(False)
        ax.spines["right"].set_visible(False)
        ax.set_xlabel("Seconds Since Attack Start")

    axes[0].set_ylabel("Coverage (%)")
    fig.suptitle("Time-Coverage KDE Under Sustained Cyberattack", fontsize=13, fontweight="bold", y=1.02)
    fig.tight_layout(rect=[0, 0, 1, 0.95], w_pad=0.7)
    save(fig, output_dir, "cyberattack_kde")


def main():
    parser = argparse.ArgumentParser(description="Generate sustained cyberattack figures for a run.")
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
        raise RuntimeError("Need at least two controller timeline sets for cyber_burst plots.")

    time_axis, arrays = build_arrays(grouped)
    plot_convergence(time_axis, arrays, output_dir)
    plot_checkpoints(time_axis, arrays, output_dir)
    plot_final_coverage(results, output_dir)
    plot_time_coverage_kde(time_axis, arrays, output_dir)


if __name__ == "__main__":
    main()
