"""
Generate a cleaned-up coverage comparison bar chart for the paper.

This script reads a summary CSV produced by `compare_reip_vs_baseline.py`
and saves a publication-ready bar figure showing final coverage for
Baseline / REIP under clean and adversarial conditions.

Example:
    python -m scripts.make_paper_fig_coverage_bars ^
        --csv results\\reip_vs_baseline_roundrobin_000.csv ^
        --output paper_figures\\coverage_comparison_bars.png
"""

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np
import pandas as pd


def make_coverage_bars(csv_path: Path, output_path: Path, runs: int = 2000) -> None:
    df = pd.read_csv(csv_path)

    # Expect four rows: baseline_clean, baseline_faulted, reip_clean, reip_faulted
    df = df.set_index("scenario")

    # Means
    baseline_clean = df.loc["baseline_clean", "coverage_mean"]
    baseline_faulted = df.loc["baseline_faulted", "coverage_mean"]
    reip_clean = df.loc["reip_clean", "coverage_mean"]
    reip_faulted = df.loc["reip_faulted", "coverage_mean"]

    # Convert std to standard error of the mean (SEM)
    sem_bc = df.loc["baseline_clean", "coverage_std"] / np.sqrt(runs)
    sem_bf = df.loc["baseline_faulted", "coverage_std"] / np.sqrt(runs)
    sem_rc = df.loc["reip_clean", "coverage_std"] / np.sqrt(runs)
    sem_rf = df.loc["reip_faulted", "coverage_std"] / np.sqrt(runs)

    means = np.array([baseline_clean, reip_clean, baseline_faulted, reip_faulted])
    sems = np.array([sem_bc, sem_rc, sem_bf, sem_rf])

    labels = [
        "Baseline\nClean",
        "REIP\nClean",
        "Baseline\nAdversarial",
        "REIP\nAdversarial",
    ]

    # Configure an IEEE-friendly style: serif font, small labels, thin lines.
    mpl.rcParams.update(
        {
            "font.family": "serif",
            "font.serif": ["Times New Roman", "Times", "DejaVu Serif"],
            "font.size": 8,
            "axes.labelsize": 8,
            "xtick.labelsize": 7,
            "ytick.labelsize": 7,
            "axes.linewidth": 0.5,
        }
    )

    # ~3.5 inches wide: single-column IEEE figure.
    fig, ax = plt.subplots(figsize=(3.5, 2.4))

    # Remove the surrounding box and keep only left/bottom axes, per IEEE style.
    for spine in ["top", "right"]:
        ax.spines[spine].set_visible(False)
    for spine in ["left", "bottom"]:
        ax.spines[spine].set_linewidth(0.5)

    x = np.arange(len(labels))
    colors = ["#e57373", "#4db6ac", "#e57373", "#4db6ac"]  # red for baseline, teal for REIP

    bars = ax.bar(
        x,
        means,
        yerr=sems,
        color=colors,
        edgecolor="black",
        linewidth=0.8,
        capsize=3,
    )

    # Add headroom so percentage labels sit comfortably inside the axes box.
    ax.set_ylim(0.0, 1.08)
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_ylabel("Final Coverage")

    # Annotate percentages above bars
    for rect, mean in zip(bars, means):
        height = rect.get_height()
        ax.text(
            rect.get_x() + rect.get_width() / 2.0,
            height + 0.015,
            f"{100 * mean:.1f}%",
            ha="center",
            va="bottom",
            fontsize=7,
            fontweight="bold",
        )

    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    # High-resolution output suitable for print.
    fig.savefig(output_path, dpi=600, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate coverage comparison bar chart for the paper."
    )
    parser.add_argument(
        "--csv",
        type=Path,
        required=True,
        help="Path to summary CSV from compare_reip_vs_baseline.py",
    )
    parser.add_argument(
        "--output",
        type=Path,
        required=True,
        help="Output path for the PNG figure.",
    )
    parser.add_argument(
        "--runs",
        type=int,
        default=2000,
        help="Number of trials used to compute SEM (default: 2000).",
    )
    args = parser.parse_args()
    make_coverage_bars(args.csv, args.output, runs=args.runs)


if __name__ == "__main__":
    main()


