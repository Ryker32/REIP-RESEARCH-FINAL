#!/usr/bin/env python3
"""
Threshold ladder figure: number of trials reaching progressively stricter
coverage thresholds under bad-leader faults.
"""

import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


RESULTS_FILE = Path("experiments/run_20260228_014625_multiroom_100trials_all/results_final_20260228_035139.json")
OUTPUT_DIR = Path("paper_docs/Ryker_Kollmyer___UPDATED_PAPER")
THRESHOLDS = [80, 90, 95, 99, 100]
THRESHOLD_LABELS = ["80+", "90+", "95+", "99+", "100"]
CLR_REIP = "#0015B7"
CLR_RAFT = "#E91D2F"


plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["Times New Roman", "Times", "DejaVu Serif"],
    "font.size": 11,
    "axes.labelsize": 12,
    "axes.titlesize": 13,
    "xtick.labelsize": 11,
    "ytick.labelsize": 11,
    "axes.linewidth": 0.7,
    "figure.dpi": 300,
    "savefig.dpi": 600,
})


def load_results():
    with RESULTS_FILE.open() as f:
        return json.load(f)


def final_coverages(results, controller: str):
    return [
        float(r.get("final_coverage") or 0.0)
        for r in results
        if r.get("controller") == controller and (r.get("fault_type") or "none") == "bad_leader"
    ]


def counts_at_thresholds(values):
    return [sum(1 for v in values if v >= th) for th in THRESHOLDS]


def main():
    results = load_results()
    reip = final_coverages(results, "reip")
    raft = final_coverages(results, "raft")

    reip_counts = counts_at_thresholds(reip)
    raft_counts = counts_at_thresholds(raft)

    fig, ax = plt.subplots(figsize=(5.6, 3.6))
    y = np.arange(len(THRESHOLDS))
    height = 0.34

    bars1 = ax.barh(y - height / 2, reip_counts, height=height, color=CLR_REIP,
                    alpha=0.90, edgecolor="white", linewidth=0.8, label="REIP")
    bars2 = ax.barh(y + height / 2, raft_counts, height=height, color=CLR_RAFT,
                    alpha=0.90, edgecolor="white", linewidth=0.8, label="Raft")

    for bars in (bars1, bars2):
        for bar in bars:
            ax.text(bar.get_width() + 1.3, bar.get_y() + bar.get_height() / 2,
                    f"{int(bar.get_width())}", ha="left", va="center",
                    fontsize=9.0, fontweight="bold")

    ax.set_title("Success Under Stricter Coverage Thresholds",
                 fontweight="bold", pad=14)
    ax.text(0.0, 1.015, "100 bad-leader trials", transform=ax.transAxes,
            fontsize=9.5, color="#444444", ha="left", va="bottom")
    ax.set_xlabel("Successful Trials")
    ax.set_ylabel("Coverage Threshold")
    ax.set_yticks(y)
    ax.set_yticklabels(THRESHOLD_LABELS)
    ax.set_xlim(0, 100)
    ax.invert_yaxis()
    ax.grid(axis="x", alpha=0.22, linestyle="--", linewidth=0.5)
    ax.legend(loc="lower right", frameon=False)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)

    fig.tight_layout(pad=0.6)
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    for ext in ("png", "pdf"):
        out = OUTPUT_DIR / f"fault_threshold_ladder.{ext}"
        fig.savefig(out, facecolor="white", bbox_inches="tight")
        print(f"Saved: {out}")
    plt.close(fig)


if __name__ == "__main__":
    main()
