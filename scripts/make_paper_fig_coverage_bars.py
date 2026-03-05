"""
Generate a publication-quality grouped bar chart for final coverage.

Three clusters (Clean, Byzantine Assignment, Freeze Leader), three bars
per cluster (REIP first, then Raft, then Decentralized), IQR whiskers.

Data from Table II of the paper (N=100 per condition).

Usage:
    python scripts/make_paper_fig_coverage_bars.py
"""

from pathlib import Path

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

# ── Data from Table II ──────────────────────────────────────────────
CONDITIONS  = ["Clean", "Byzantine\nAssignment", "Freeze\nLeader"]
CONTROLLERS = ["REIP", "Raft", "Decentralized"]

MEAN = {
    "REIP":          [91.2, 90.9, 96.1],
    "Raft":          [86.4, 66.4, 63.9],
    "Decentralized": [90.6, 91.5, 92.2],
}
IQR_VAL = {
    "REIP":          [ 0.5,  1.2,  1.6],
    "Raft":          [12.0, 46.6, 33.1],
    "Decentralized": [ 0.0,  0.0,  2.0],
}

COLORS = {
    "REIP":          "#2171B5",
    "Raft":          "#CB4335",
    "Decentralized": "#27874A",
}
LABELS = {
    "REIP":          "REIP (Proposed)",
    "Raft":          "Raft (Baseline)",
    "Decentralized": "Decentr. (Ref.)",
}

CATASTROPHIC = 70.0
OUTPUT_DIR = Path("paper_docs/Ryker_Kollmyer___UPDATED_PAPER")


def main() -> None:
    mpl.rcParams.update({
        "font.family": "serif",
        "font.serif": ["Times New Roman", "Times", "DejaVu Serif"],
        "font.size": 8,
        "axes.labelsize": 9,
        "xtick.labelsize": 8,
        "ytick.labelsize": 7,
        "axes.linewidth": 0.5,
        "mathtext.fontset": "dejavuserif",
    })

    n_cond = len(CONDITIONS)
    n_ctrl = len(CONTROLLERS)
    bar_w  = 0.23
    x_base = np.arange(n_cond)

    fig, ax = plt.subplots(figsize=(3.5, 2.5))

    # Subtle horizontal grid (behind everything)
    for g in [50, 60, 70, 80, 90, 100]:
        ax.axhline(g, color="#F0F0F0", linewidth=0.35, zorder=0)

    # Catastrophic threshold
    ax.axhline(CATASTROPHIC, color="#E74C3C", linewidth=0.6,
               linestyle="--", alpha=0.4, zorder=1)
    ax.text(-0.42, CATASTROPHIC, "70%",
            fontsize=5.5, color="#C0392B", ha="right", va="center",
            fontweight="bold")

    for ci, ctrl in enumerate(CONTROLLERS):
        means = np.array(MEAN[ctrl])
        iqrs  = np.array(IQR_VAL[ctrl])
        col   = COLORS[ctrl]

        # Symmetric IQR/2 whiskers centered on the mean
        yerr = iqrs / 2.0

        offset = (ci - (n_ctrl - 1) / 2) * bar_w
        xpos   = x_base + offset

        ax.bar(xpos, means, width=bar_w, color=col, edgecolor="white",
               linewidth=0.5, zorder=3, label=LABELS[ctrl])

        # IQR whiskers (only draw if IQR > 1 to avoid invisible stubs)
        visible = iqrs > 1.0
        if visible.any():
            ax.errorbar(xpos[visible], means[visible],
                        yerr=yerr[visible],
                        fmt="none", ecolor=col, elinewidth=0.7,
                        capsize=2.0, capthick=0.5, alpha=0.6, zorder=4)

        # Value annotations
        for xi, yi, half_iqr in zip(xpos, means, yerr):
            label_y = yi + half_iqr + 1.0
            if label_y < yi + 1.5:
                label_y = yi + 1.5
            ax.text(xi, label_y, f"{yi:.1f}",
                    fontsize=5.5, fontweight="bold", color=col,
                    ha="center", va="bottom")

    # Axes
    ax.set_xticks(x_base)
    ax.set_xticklabels(CONDITIONS, linespacing=1.1)
    ax.set_ylabel("Final Coverage (%)")
    ax.set_ylim(38, 106)
    ax.set_xlim(-0.5, n_cond - 0.5)
    ax.set_yticks([40, 50, 60, 70, 80, 90, 100])

    for sp in ("top", "right"):
        ax.spines[sp].set_visible(False)
    ax.spines["left"].set_linewidth(0.4)
    ax.spines["bottom"].set_linewidth(0.4)
    ax.tick_params(axis="x", length=0, pad=4)
    ax.tick_params(axis="y", length=2.5, width=0.4)
    ax.set_axisbelow(True)

    # Legend
    ax.legend(fontsize=5.5, loc="lower left",
              frameon=True, fancybox=False, edgecolor="#DDD",
              framealpha=0.95, handlelength=1.0, handletextpad=0.5,
              borderpad=0.35, labelspacing=0.3)

    fig.tight_layout()

    out = OUTPUT_DIR / "coverage_bar.png"
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=600, bbox_inches="tight")
    fig.savefig(out.with_suffix(".pdf"), bbox_inches="tight")
    print(f"Saved: {out}  +  .pdf")
    plt.close(fig)


if __name__ == "__main__":
    main()
