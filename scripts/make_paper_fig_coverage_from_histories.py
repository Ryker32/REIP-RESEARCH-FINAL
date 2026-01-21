"""
Generate IEEE-style coverage-vs-time plots directly from histories CSVs.

This is useful when you have already exported run histories with the
`--export_histories` flag in `compare_reip_vs_baseline.py` and want to
recreate clean/adversarial coverage panels without rerunning simulations.

Example (clean conditions):
    python scripts\\make_paper_fig_coverage_from_histories.py ^
        --baseline_csv results\\histories\\baseline_clean_histories.csv ^
        --reip_csv results\\histories\\reip_clean_histories.csv ^
        --title "Clean Conditions (No Faults)" ^
        --output paper_figures\\coverage_clean_from_histories.png

Example (adversarial conditions):
    python scripts\\make_paper_fig_coverage_from_histories.py ^
        --baseline_csv results\\histories\\baseline_faulted_histories.csv ^
        --reip_csv results\\histories\\reip_faulted_histories.csv ^
        --title "Adversarial Conditions (Faults)" ^
        --output paper_figures\\coverage_adversarial_from_histories.png
"""

import argparse
from pathlib import Path

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def _histories_from_csv(csv_path: Path) -> np.ndarray:
    """Load histories CSV (run_id, timestep, coverage) into [n_runs, T] array."""
    df = pd.read_csv(csv_path)
    if not {"run_id", "timestep", "coverage"}.issubset(df.columns):
        raise ValueError(f"{csv_path} does not have required columns run_id,timestep,coverage")

    n_runs = int(df["run_id"].max()) + 1
    T = int(df["timestep"].max()) + 1
    histories = np.zeros((n_runs, T), dtype=float)

    # Fill array
    for _, row in df.iterrows():
        run = int(row["run_id"])
        t = int(row["timestep"])
        histories[run, t] = float(row["coverage"])

    return histories


def _plot_histories_with_mean(ax, histories, color, label, max_traces: int = 80):
    """Plot a subset of coverage histories plus their mean curve."""
    histories_arr = np.asarray(histories)
    n_runs, _ = histories_arr.shape
    rng = np.random.default_rng(0)  # deterministic subset for reproducibility
    if n_runs > max_traces:
        idx = rng.choice(n_runs, size=max_traces, replace=False)
        subset = histories_arr[idx]
    else:
        subset = histories_arr
    for hist in subset:
        ax.plot(hist, color=color, alpha=0.15, linewidth=0.7)
    ax.plot(
        np.mean(histories_arr, axis=0),
        color=color,
        linewidth=1.2,
        label=label,
    )


def make_coverage_panel(
    baseline_csv: Path, reip_csv: Path, title: str, output_path: Path, max_traces: int = 80
) -> None:
    """Create a single IEEE-style coverage panel from histories CSVs."""
    baseline_hist = _histories_from_csv(baseline_csv)
    reip_hist = _histories_from_csv(reip_csv)

    # IEEE-friendly style
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

    fig, ax = plt.subplots(figsize=(3.5, 2.4))

    _plot_histories_with_mean(ax, baseline_hist, color="#E63946", label="Baseline", max_traces=max_traces)
    _plot_histories_with_mean(ax, reip_hist, color="#2A9D8F", label="REIP", max_traces=max_traces)

    ax.set_xlabel("Timestep")
    ax.set_ylabel("Coverage")
    ax.set_title(title, fontsize=9)
    ax.set_ylim(0.0, 1.0)
    ax.grid(True, alpha=0.2, axis="y")

    for spine in ["top", "right"]:
        ax.spines[spine].set_visible(False)
    for spine in ["left", "bottom"]:
        ax.spines[spine].set_linewidth(0.5)

    ax.legend(fontsize=7, frameon=False)

    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=600, bbox_inches="tight")
    fig.savefig(output_path.with_suffix(".pdf"), bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Create IEEE-style coverage-vs-time panel from histories CSVs."
    )
    parser.add_argument("--baseline_csv", type=Path, required=True, help="Baseline histories CSV.")
    parser.add_argument("--reip_csv", type=Path, required=True, help="REIP histories CSV.")
    parser.add_argument("--title", type=str, required=True, help="Plot title.")
    parser.add_argument("--output", type=Path, required=True, help="Output PNG path.")
    parser.add_argument(
        "--max_traces",
        type=int,
        default=80,
        help="Maximum number of individual trajectories to plot for each policy.",
    )
    args = parser.parse_args()
    make_coverage_panel(args.baseline_csv, args.reip_csv, args.title, args.output, args.max_traces)


if __name__ == "__main__":
    main()



