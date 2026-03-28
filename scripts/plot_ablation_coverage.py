"""Plot coverage-over-time curves from ablation history CSVs.

For each variant, the ablation runners save histories in:
    results/<...>/histories_<timestamp>/<variant>_histories.csv

This script reads those CSVs and produces:
    - One plot per variant (individual trajectories + mean)
    - A combined plot overlaying all variants for quick comparison
"""

from __future__ import annotations

import argparse
import csv
import os
from typing import Dict, List

import matplotlib.pyplot as plt
import numpy as np

DEFAULT_COLORS = [
    "#1b9e77",
    "#d95f02",
    "#7570b3",
    "#e7298a",
    "#66a61e",
    "#e6ab02",
]


def load_histories(csv_path: str) -> np.ndarray:
    with open(csv_path, "r", newline="") as f:
        reader = csv.DictReader(f)
        t_cols = sorted([c for c in reader.fieldnames if c.startswith("t_")],
                        key=lambda x: int(x.split("_")[1]))
        histories: List[np.ndarray] = []
        for row in reader:
            traj = [float(row[c]) if row[c] != "" else np.nan for c in t_cols]
            histories.append(np.array(traj, dtype=float))
    return np.array(histories)


def plot_variant(variant: str, histories: np.ndarray, output_path: str, color: str) -> None:
    plt.figure(figsize=(4.5, 3.0))
    timesteps = np.arange(histories.shape[1])
    for traj in histories:
        plt.plot(timesteps, traj, color=color, alpha=0.15, linewidth=1)
    mean_traj = np.nanmean(histories, axis=0)
    plt.plot(timesteps, mean_traj, color=color, linewidth=2.0, label=variant)
    plt.ylim(0.0, 1.0)
    plt.xlabel("Timestep")
    plt.ylabel("Coverage")
    plt.title(f"{variant} Coverage over Time")
    plt.grid(alpha=0.2, linestyle="--", linewidth=0.5)
    plt.tight_layout()
    plt.savefig(output_path, dpi=300, bbox_inches="tight")
    plt.close()


def plot_combined(histories_map: Dict[str, np.ndarray], output_path: str) -> None:
    plt.figure(figsize=(5.5, 3.5))
    for idx, (variant, histories) in enumerate(histories_map.items()):
        color = DEFAULT_COLORS[idx % len(DEFAULT_COLORS)]
        mean_traj = np.nanmean(histories, axis=0)
        timesteps = np.arange(histories.shape[1])
        for traj in histories:
            plt.plot(timesteps, traj, color=color, alpha=0.05)
        plt.plot(timesteps, mean_traj, color=color, linewidth=2.0, label=variant)
    plt.ylim(0.0, 1.0)
    plt.xlabel("Timestep")
    plt.ylabel("Coverage")
    plt.title("Coverage over Time (All Variants)")
    plt.grid(alpha=0.2, linestyle="--", linewidth=0.5)
    plt.legend()
    plt.tight_layout()
    plt.savefig(output_path, dpi=300, bbox_inches="tight")
    plt.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot coverage-over-time from ablation histories")
    parser.add_argument("--histories_dir", type=str, required=True,
                        help="Directory containing *_histories.csv files")
    parser.add_argument("--output_dir", type=str, required=True,
                        help="Directory to store generated plots")
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)

    histories_files = [
        f for f in os.listdir(args.histories_dir) if f.endswith("_histories.csv")
    ]
    if not histories_files:
        raise FileNotFoundError("No *_histories.csv files found in histories_dir")

    histories_map: Dict[str, np.ndarray] = {}
    for idx, filename in enumerate(sorted(histories_files)):
        variant = filename.replace("_histories.csv", "")
        csv_path = os.path.join(args.histories_dir, filename)
        histories = load_histories(csv_path)
        histories_map[variant] = histories
        color = DEFAULT_COLORS[idx % len(DEFAULT_COLORS)]
        plot_path = os.path.join(args.output_dir, f"{variant}_coverage.png")
        plot_variant(variant, histories, plot_path, color)

    combined_path = os.path.join(args.output_dir, "coverage_combined.png")
    plot_combined(histories_map, combined_path)


if __name__ == "__main__":
    main()

