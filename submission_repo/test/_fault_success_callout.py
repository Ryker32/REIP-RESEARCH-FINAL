#!/usr/bin/env python3
"""
IEEE-style poster callout: fault-tolerant search success.

Computes the percentage of trials reaching 80% coverage under bad-leader faults
for REIP and Raft, then renders a clean two-bar figure.
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


def success_rate(results, controller: str) -> tuple[float, int, int]:
    subset = [
        r for r in results
        if r.get("controller") == controller
        and (r.get("fault_type") or "none") == "bad_leader"
    ]
    successes = sum(1 for r in subset if (r.get("final_coverage") or 0) >= SUCCESS_THRESHOLD)
    total = len(subset)
    rate = 100.0 * successes / total if total else 0.0
    return rate, successes, total


def main():
    parser = argparse.ArgumentParser(description="Generate the bad-leader success-rate callout figure.")
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
    reip_rate, reip_successes, reip_total = success_rate(results, "reip")
    raft_rate, raft_successes, raft_total = success_rate(results, "raft")
    if reip_total == 0 or raft_total == 0:
        raise RuntimeError("Need bad_leader trials for both REIP and Raft to plot success callout.")

    labels = ["REIP", "Raft"]
    values = [reip_rate, raft_rate]
    colors = ["#1f4fff", "#de6464"]

    fig, ax = plt.subplots(figsize=(3.6, 3.0))
    x = np.arange(len(labels))

    bars = ax.bar(
        x, values, width=0.62, color=colors,
        edgecolor="white", linewidth=1.2
    )

    for bar, value in zip(bars, values):
        ax.text(
            bar.get_x() + bar.get_width() / 2,
            value + 2.2,
            f"{value:.0f}%",
            ha="center",
            va="bottom",
            fontsize=14,
            fontweight="bold",
        )

    ax.annotate(
        "Nearly 2x higher",
        xy=(x[0], values[0] + 1),
        xytext=(0.5, 102),
        textcoords="data",
        ha="center",
        va="bottom",
        fontsize=11,
        fontweight="bold",
        arrowprops=dict(arrowstyle="-", lw=1.0, color="black"),
    )

    ax.text(
        0.5,
        -0.22,
        f"Bad-leader fault, N={reip_total} trials/controller",
        transform=ax.transAxes,
        ha="center",
        va="top",
        fontsize=8,
    )

    ax.set_title("Trials Reaching 80% Coverage", fontweight="bold", pad=10)
    ax.set_ylabel("Success Rate (%)")
    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontweight="bold")
    ax.set_ylim(0, 110)
    ax.set_yticks([0, 20, 40, 60, 80, 100])
    ax.grid(axis="y", alpha=0.25, linestyle="--", linewidth=0.6)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)

    fig.tight_layout(rect=[0, 0.06, 1, 1])
    output_dir.mkdir(parents=True, exist_ok=True)
    for ext in ("png", "pdf"):
        out = output_dir / f"fault_success_callout.{ext}"
        fig.savefig(out, facecolor="white")
        print(f"Saved: {out}")
    plt.close(fig)

    print(
        f"REIP: {reip_successes}/{reip_total} = {reip_rate:.1f}% | "
        f"Raft: {raft_successes}/{raft_total} = {raft_rate:.1f}%"
    )


if __name__ == "__main__":
    main()
