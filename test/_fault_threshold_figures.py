#!/usr/bin/env python3
"""
Poster-style fault threshold figures from a bad-leader dataset.

Outputs:
  - fault_attainment_curve.(png|pdf)
  - fault_trial_scatter.(png|pdf)
"""

import argparse
import glob
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


DEFAULT_RUN_DIR = Path("experiments/run_20260228_014625_multiroom_100trials_all")
SUCCESS_THRESHOLD = 80.0
MISSION_CUTOFF = 120.0

CLR_REIP = "#1f4fff"
CLR_RAFT = "#de6464"


plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["Times New Roman", "Times", "DejaVu Serif"],
    "font.size": 10,
    "axes.labelsize": 11,
    "axes.titlesize": 12,
    "xtick.labelsize": 10,
    "ytick.labelsize": 10,
    "axes.linewidth": 0.7,
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


def subset(results, controller: str):
    return [
        r for r in results
        if r.get("controller") == controller
        and (r.get("fault_type") or "none") == "bad_leader"
    ]


def build_attainment_curve(trials, cutoff=MISSION_CUTOFF):
    times = np.array(sorted({
        0.0,
        *[r["time_to_80"] for r in trials if r.get("time_to_80") is not None and r["time_to_80"] <= cutoff],
        cutoff,
    }))
    attained = []
    total = len(trials)
    for t in times:
        count = sum(1 for r in trials if r.get("time_to_80") is not None and r["time_to_80"] <= t)
        attained.append(100.0 * count / total if total else 0.0)
    return times, np.array(attained)


def save(fig, output_dir: Path, stem: str):
    output_dir.mkdir(parents=True, exist_ok=True)
    for ext in ("png", "pdf"):
        out = output_dir / f"{stem}.{ext}"
        fig.savefig(out, facecolor="white", bbox_inches="tight")
        print(f"Saved: {out}")
    plt.close(fig)


def plot_attainment_curve(results, output_dir: Path):
    reip = subset(results, "reip")
    raft = subset(results, "raft")
    if not reip or not raft:
        raise RuntimeError("Need bad_leader trials for both REIP and Raft to plot attainment.")
    reip_t, reip_y = build_attainment_curve(reip)
    raft_t, raft_y = build_attainment_curve(raft)

    fig, ax = plt.subplots(figsize=(4.2, 3.1))
    ax.step(reip_t, reip_y, where="post", color=CLR_REIP, linewidth=3.0, label="REIP")
    ax.step(raft_t, raft_y, where="post", color=CLR_RAFT, linewidth=3.0, label="Raft")

    ax.scatter([reip_t[-1], raft_t[-1]], [reip_y[-1], raft_y[-1]],
               color=[CLR_REIP, CLR_RAFT], s=55, zorder=4)
    ax.text(reip_t[-1] - 24, reip_y[-1] + 3, f"{reip_y[-1]:.0f}%", color=CLR_REIP,
            fontsize=12, fontweight="bold")
    ax.text(raft_t[-1] - 24, raft_y[-1] + 3, f"{raft_y[-1]:.0f}%", color=CLR_RAFT,
            fontsize=12, fontweight="bold")

    ax.text(0.5, 1.03, "Trials reaching 80% coverage over time",
            transform=ax.transAxes, ha="center", fontsize=12, fontweight="bold")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Trials Reaching 80% (%)")
    ax.set_xlim(0, MISSION_CUTOFF)
    ax.set_ylim(0, 100)
    ax.set_yticks([0, 20, 40, 60, 80, 100])
    ax.grid(True, alpha=0.25, linestyle="--", linewidth=0.6)
    ax.legend(loc="lower right", framealpha=0.95, edgecolor="#999999")
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)

    save(fig, output_dir, "fault_attainment_curve")


def plot_trial_scatter(results, output_dir: Path):
    rng = np.random.default_rng(7)
    reip = subset(results, "reip")
    raft = subset(results, "raft")
    if not reip or not raft:
        raise RuntimeError("Need bad_leader trials for both REIP and Raft to plot per-trial scatter.")

    def points(trials):
        xs, ys = [], []
        for r in trials:
            t80 = r.get("time_to_80")
            x = t80 if t80 is not None else MISSION_CUTOFF
            x = x + rng.normal(0, 1.2)
            y = (r.get("final_coverage") or 0) + rng.normal(0, 0.8)
            xs.append(x)
            ys.append(y)
        return np.array(xs), np.array(ys)

    xr, yr = points(reip)
    xb, yb = points(raft)

    fig, ax = plt.subplots(figsize=(4.2, 3.3))
    ax.scatter(xr, yr, s=28, color=CLR_REIP, alpha=0.78, edgecolors="white",
               linewidth=0.4, label="REIP")
    ax.scatter(xb, yb, s=28, color=CLR_RAFT, alpha=0.78, edgecolors="white",
               linewidth=0.4, label="Raft")
    ax.axvline(MISSION_CUTOFF, color="gray", linestyle="--", linewidth=0.8, alpha=0.5)
    ax.text(MISSION_CUTOFF - 3, 6, "not reached by end", rotation=90,
            ha="right", va="bottom", fontsize=7, color="gray")

    ax.set_title("Per-trial outcome cloud", fontweight="bold", pad=8)
    ax.set_xlabel("Time to 80% coverage (s)\n(unreached trials placed at mission end)")
    ax.set_ylabel("Final Coverage (%)")
    ax.set_xlim(0, MISSION_CUTOFF + 6)
    ax.set_ylim(0, 105)
    ax.grid(True, alpha=0.25, linestyle="--", linewidth=0.6)
    ax.legend(loc="lower right", framealpha=0.95, edgecolor="#999999")
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)

    save(fig, output_dir, "fault_trial_scatter")


def main():
    parser = argparse.ArgumentParser(description="Generate threshold attainment and per-trial scatter figures.")
    parser.add_argument("run_dir", nargs="?", default=str(DEFAULT_RUN_DIR),
                        help="Experiment run directory or explicit results_final JSON path")
    parser.add_argument("--output-dir", default=None,
                        help="Directory to save figures (default: <run_dir>/curated_figures)")
    args = parser.parse_args()

    run_path = Path(args.run_dir)
    results_file = resolve_results_file(run_path)
    base_dir = run_path if run_path.is_dir() else results_file.parent
    output_dir = Path(args.output_dir) if args.output_dir else base_dir / "curated_figures"

    results = load_results(results_file)
    plot_attainment_curve(results, output_dir)
    plot_trial_scatter(results, output_dir)


if __name__ == "__main__":
    main()
