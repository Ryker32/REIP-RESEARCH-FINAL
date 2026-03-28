#!/usr/bin/env python3
"""
Paper Figure Generator

Generates standalone publication-ready figures from experiment outputs.
Each figure is exported independently so it can be placed and sized in
Overleaf without relying on stitched multi-panel layouts.

Usage:
  python test/generate_poster_graphs.py <results_dir> [output_dir]

<results_dir> is the timestamped experiment output folder, e.g.
  experiments/run_20260226_163015_open_10trials_all
"""

import json
import sys
import os
import glob
import math
from collections import defaultdict

try:
    import matplotlib
    matplotlib.use('Agg')  # non-interactive backend
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    from matplotlib.ticker import MultipleLocator
    import numpy as np
except ImportError:
    print("matplotlib required: pip install matplotlib numpy")
    sys.exit(1)

# IEEE-style defaults: compact, legible, and conservative.
plt.rcParams.update({
    'font.size': 8,
    'font.family': 'serif',
    'font.serif': ['Times New Roman', 'Times', 'DejaVu Serif'],
    'axes.labelsize': 8,
    'axes.titlesize': 8,
    'axes.linewidth': 0.8,
    'legend.fontsize': 7,
    'xtick.labelsize': 7,
    'ytick.labelsize': 7,
    'lines.linewidth': 1.6,
    'lines.markersize': 4,
    'figure.dpi': 200,
    'savefig.dpi': 300,
    'savefig.bbox': 'tight',
    'pdf.fonttype': 42,
    'ps.fonttype': 42,
    'axes.spines.top': False,
    'axes.spines.right': False,
})

IEEE_SINGLE_COL_W = 3.5
IEEE_DOUBLE_COL_W = 7.16
IEEE_SHORT_H = 2.35
IEEE_MED_H = 2.8
IEEE_TALL_H = 3.2

# Color palette (colorblind-friendly)
COLORS = {
    'reip':          '#2E86AB',   # Blue
    'raft':          '#A23B72',   # Magenta
    'decentralized': '#F18F01',   # Orange
    'fault_line':    '#C73E1D',   # Red
    'success':       '#3A7D44',   # Green
}
LABELS = {
    'reip': 'REIP',
    'raft': 'RAFT',
    'decentralized': 'Decentralized',
}
FAULT_LABELS = {
    'none': 'clean',
    'bad_leader': 'bad_leader',
    'freeze_leader': 'freeze_leader',
}
ABLATION_LABELS = {
    'no_trust': 'No Trust',
    'no_causality': 'No Causality',
    'no_direction': 'No Direction',
}

# --------------------------------------------------
# Data loaders
# --------------------------------------------------

def load_results_json(results_dir: str) -> list:
    """Load the main results JSON (final metrics per experiment)."""
    pattern = os.path.join(results_dir, "results_final_*.json")
    files = sorted(glob.glob(pattern))
    if not files:
        print(f"No results_final_*.json found in {results_dir}")
        return []
    with open(files[-1], 'r') as f:
        return json.load(f)


def load_timelines(results_dir: str) -> dict:
    """Load per-experiment coverage timelines from log subdirectories.

    Returns: { experiment_name: [(time, coverage), ...], ... }
    """
    logs_dir = os.path.join(results_dir, "logs")
    timelines = {}
    if not os.path.isdir(logs_dir):
        return timelines
    for exp_dir in os.listdir(logs_dir):
        tl_path = os.path.join(logs_dir, exp_dir, "coverage_timeline.json")
        if os.path.exists(tl_path):
            with open(tl_path, 'r') as f:
                data = json.load(f)
                timelines[exp_dir] = data
    return timelines


def group_results(results: list) -> dict:
    """Group results by (controller, fault_type)."""
    groups = defaultdict(list)
    for r in results:
        key = (r['controller'], r.get('fault_type') or 'none')
        groups[key].append(r)
    return dict(groups)


def group_timelines(timelines: dict) -> dict:
    """Group timeline data by (controller, fault_type).

    Returns: { (controller, fault): [ [(t,c), ...], ...], ... }
    """
    groups = defaultdict(list)
    for exp_name, data in timelines.items():
        ctrl = data.get('controller', '')
        fault = data.get('fault_type') or 'none'
        tl = data.get('timeline', [])
        if tl:
            groups[(ctrl, fault)].append(tl)
    return dict(groups)


def save_figure(fig, output_dir: str, stem: str):
    """Save each figure in both vector and raster form."""
    pdf_path = os.path.join(output_dir, f"{stem}.pdf")
    png_path = os.path.join(output_dir, f"{stem}.png")
    fig.savefig(pdf_path)
    fig.savefig(png_path)
    print(f"  Saved: {pdf_path}")
    print(f"  Saved: {png_path}")
    plt.close(fig)


def style_axis(ax, xlabel: str, ylabel: str, xlim=None, ylim=None):
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    if xlim is not None:
        ax.set_xlim(*xlim)
    if ylim is not None:
        ax.set_ylim(*ylim)
    ax.grid(True, axis='both', linewidth=0.4, alpha=0.3)
    ax.tick_params(direction='out', length=3, width=0.8)


# --------------------------------------------------
# Graph 1: Convergence plot (HERO FIGURE)
# --------------------------------------------------

def graph_convergence(timelines: dict, output_dir: str):
    """Coverage-over-time with one standalone figure per fault condition."""
    fault_panels = [
        ('none', 'Clean'),
        ('bad_leader', 'Bad Leader'),
        ('freeze_leader', 'Freeze Leader'),
    ]
    controllers = ['reip', 'raft', 'decentralized']
    tl_groups = group_timelines(timelines)
    for fault_key, panel_title in fault_panels:
        has_any_traces = any(tl_groups.get((ctrl, fault_key), []) for ctrl in controllers)
        if not has_any_traces:
            print(f"  [SKIP] convergence_{FAULT_LABELS[fault_key]} - no timeline data")
            continue

        fig, ax = plt.subplots(figsize=(IEEE_SINGLE_COL_W, IEEE_MED_H))
        for ctrl in controllers:
            key = (ctrl, fault_key)
            traces = tl_groups.get(key, [])
            color = COLORS[ctrl]
            label = LABELS[ctrl]

            for tl in traces:
                ts = [p[0] for p in tl]
                cs = [p[1] for p in tl]
                ax.plot(ts, cs, color=color, alpha=0.15, linewidth=0.7)

            if traces:
                max_t = max(p[0] for tl in traces for p in tl) if traces else 120
                t_grid = np.linspace(0, min(max_t, 120), 500)
                interp_vals = []
                for tl in traces:
                    ts = np.array([p[0] for p in tl])
                    cs = np.array([p[1] for p in tl])
                    if len(ts) > 1:
                        interp = np.interp(t_grid, ts, cs)
                        interp_vals.append(interp)
                if interp_vals:
                    mean_c = np.mean(interp_vals, axis=0)
                    ax.plot(t_grid, mean_c, color=color, linewidth=2.5,
                            label=label, zorder=5)

        if fault_key != 'none':
            fault_time_1 = 10.0
            fault_time_2 = None
            for exp_name, exp_data in timelines.items():
                if (exp_data.get('controller') == controllers[0] and
                    (exp_data.get('fault_type') or 'none') == fault_key):
                    fault_time_1 = exp_data.get('fault_time_1', 10.0)
                    fault_time_2 = exp_data.get('fault_time_2')
                    break

            ax.axvline(x=fault_time_1, color=COLORS['fault_line'], linestyle='--',
                       linewidth=1.5, alpha=0.7, zorder=6)
            ax.annotate('Fault 1', xy=(fault_time_1 + 1, 8), fontsize=7,
                        color=COLORS['fault_line'])
            if fault_time_2:
                ax.axvline(x=fault_time_2, color=COLORS['fault_line'], linestyle=':',
                           linewidth=1.0, alpha=0.5, zorder=6)
                ax.annotate('Fault 2', xy=(fault_time_2 + 1, 18), fontsize=7,
                            color=COLORS['fault_line'])

        style_axis(ax, 'Time (s)', 'Coverage (%)', xlim=(0, 120), ylim=(0, 105))
        handles, labels = ax.get_legend_handles_labels()
        if handles:
            ax.legend(loc='lower right', frameon=False, ncol=1)
        fig.tight_layout()
        save_figure(fig, output_dir, f"convergence_{FAULT_LABELS[fault_key]}")


# --------------------------------------------------
# Graph 2: Time-to-50% box plot
# --------------------------------------------------

def graph_time_to_50(results: list, output_dir: str):
    """Standalone box plots comparing time-to-50% for each fault."""
    groups = group_results(results)
    faults = [('none', 'Clean'), ('bad_leader', 'Bad Leader'), ('freeze_leader', 'Freeze Leader')]
    controllers = ['reip', 'raft', 'decentralized']
    for fkey, flabel in faults:
        has_any_trials = any(groups.get((ctrl, fkey), []) for ctrl in controllers)
        if not has_any_trials:
            print(f"  [SKIP] time_to_50_{FAULT_LABELS[fkey]} - no results")
            continue

        fig, ax = plt.subplots(figsize=(IEEE_SINGLE_COL_W, IEEE_MED_H))
        data = []
        labels = []
        colors = []
        for ctrl in controllers:
            key = (ctrl, fkey)
            trials = groups.get(key, [])
            t50s = [t['time_to_50'] for t in trials if t.get('time_to_50') is not None]
            data.append(t50s)
            labels.append(LABELS[ctrl])
            colors.append(COLORS[ctrl])

        bp = ax.boxplot(data, tick_labels=labels, patch_artist=True, widths=0.6)
        for patch, c in zip(bp['boxes'], colors):
            patch.set_facecolor(c)
            patch.set_alpha(0.7)
        for median in bp['medians']:
            median.set_color('black')
            median.set_linewidth(2)

        style_axis(ax, '', 'Time to 50% coverage (s)')
        fig.tight_layout()
        save_figure(fig, output_dir, f"time_to_50_{FAULT_LABELS[fkey]}")


# --------------------------------------------------
# Graph 3: Detection latency
# --------------------------------------------------

def graph_detection_latency(results: list, output_dir: str):
    """Bar chart: REIP detection latency under bad_leader."""
    groups = group_results(results)

    reip_bl = groups.get(('reip', 'bad_leader'), [])
    if not reip_bl:
        print("  [SKIP] detection_latency_bad_leader - no REIP bad-leader data")
        return

    suspicion_times = [r['time_to_first_suspicion']
                       for r in reip_bl if r.get('time_to_first_suspicion')]
    impeach_times = [r['time_to_detection']
                     for r in reip_bl if r.get('time_to_detection')]

    fig, ax = plt.subplots(figsize=(IEEE_SINGLE_COL_W, IEEE_MED_H))

    # Theoretical bound
    ax.axhline(y=0.4, color='gray', linestyle=':', linewidth=1.5,
               label='Theoretical T1 bound (0.4s)', zorder=1)

    categories = []
    means = []
    stds = []
    cols = []

    if suspicion_times:
        categories.append('First\nSuspicion')
        means.append(np.mean(suspicion_times))
        stds.append(np.std(suspicion_times))
        cols.append(COLORS['reip'])

    if impeach_times:
        categories.append('Full\nImpeachment')
        means.append(np.mean(impeach_times))
        stds.append(np.std(impeach_times))
        cols.append(COLORS['fault_line'])

    if means:
        bars = ax.bar(categories, means, yerr=stds, capsize=8,
                      color=cols, edgecolor='black', alpha=0.85, zorder=3)
        for bar, m, s in zip(bars, means, stds):
            ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + s + 0.1,
                    f'{m:.2f} s', ha='center', fontsize=7)

    style_axis(ax, '', 'Time after fault injection (s)')
    ax.legend(loc='upper right', frameon=False)
    ax.set_ylim(0, max(means + [1]) * 1.45 if means else 5)

    n = len(reip_bl)
    detected = len([r for r in reip_bl if r.get('time_to_detection')])
    fps = sum(r.get('false_positives', 0) for r in reip_bl)
    detection_rate = (100.0 * detected / n) if n else 0.0
    ax.annotate(f'Detection rate: {detection_rate:.0f}%   False positives: {fps}',
                xy=(0.5, 0.02), xycoords='axes fraction', fontsize=7,
                ha='center',
                bbox=dict(boxstyle='round', facecolor='white', alpha=0.9, linewidth=0.6))

    fig.tight_layout()
    save_figure(fig, output_dir, 'detection_latency_bad_leader')


# --------------------------------------------------
# Graph 4: Final coverage comparison (bar chart)
# --------------------------------------------------

def graph_coverage_bars(results: list, output_dir: str):
    """Grouped bar chart: final coverage by controller and fault."""
    groups = group_results(results)
    controllers = ['reip', 'raft', 'decentralized']
    all_faults = [('none', 'Clean'), ('bad_leader', 'Bad Leader'), ('freeze_leader', 'Freeze Leader')]
    faults = [(fkey, flabel) for fkey, flabel in all_faults
              if any(groups.get((ctrl, fkey), []) for ctrl in controllers)]
    if not faults:
        print("  [SKIP] coverage_bars - no coverage results")
        return

    fig, ax = plt.subplots(figsize=(IEEE_DOUBLE_COL_W, IEEE_MED_H))
    x = np.arange(len(faults))
    width = 0.25

    for i, ctrl in enumerate(controllers):
        means = []
        stds_ = []
        for fkey, _ in faults:
            trials = groups.get((ctrl, fkey), [])
            covs = [t['final_coverage'] for t in trials]
            means.append(np.mean(covs) if covs else np.nan)
            stds_.append(np.std(covs) if covs else 0.0)
        bars = ax.bar(x + i * width, means, width, yerr=stds_, capsize=4,
                      label=LABELS[ctrl], color=COLORS[ctrl], edgecolor='black',
                      alpha=0.85)

    style_axis(ax, '', 'Final coverage at 120 s (%)', ylim=(0, 105))
    ax.set_xticks(x + width)
    ax.set_xticklabels([fl for _, fl in faults])
    ax.legend(frameon=False, ncol=3, loc='upper center')

    fig.tight_layout()
    save_figure(fig, output_dir, 'coverage_bars')


# --------------------------------------------------
# Graph 5: Three-tier trust model diagram
# --------------------------------------------------

def graph_three_tier_trust(output_dir: str):
    """Visual explanation of three-tier trust model."""
    fig, ax = plt.subplots(figsize=(IEEE_DOUBLE_COL_W, IEEE_TALL_H))

    # Pyramid tiers (bottom-up: Peer, Sensor, Personal)
    tier_data = [
        ([(1.5, 7), (8.5, 7), (7.5, 5)], (2.5, 5), COLORS['decentralized'],
         'Tier 3: Peer Reports', '"They said so"', 'Weight: 0.3',
         'May be stale or inaccurate'),
        ([(2.5, 5), (7.5, 5), (6.5, 3)], (3.5, 3), COLORS['reip'],
         'Tier 2: Sensor Data (ToF)', '"I can see it"', 'Weight: 1.0',
         'Direct obstacle verification'),
        ([(3.5, 3), (6.5, 3), (5, 1)], (3.5, 1), COLORS['success'],
         'Tier 1: Personal Experience', '"I was there"', 'Weight: 1.0',
         'Ground truth - physically visited'),
    ]

    for verts, _, color, title, quote, weight, note in tier_data:
        poly = plt.Polygon(verts, facecolor=color, alpha=0.7,
                           edgecolor='black', linewidth=1.5)
        ax.add_patch(poly)
        cx = np.mean([v[0] for v in verts])
        cy = np.mean([v[1] for v in verts])
        ax.text(cx, cy + 0.15, title, ha='center', va='center',
                fontsize=13, fontweight='bold')
        ax.text(cx, cy - 0.35, f'{quote}  -  {weight}',
                ha='center', va='center', fontsize=11, style='italic')

    ax.set_xlim(0, 10)
    ax.set_ylim(0, 8)
    ax.axis('off')
    ax.text(5, 7.6,
            'Higher tiers = faster detection  |  Lower tiers = more coverage but slower',
            ha='center', fontsize=7,
            bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8, linewidth=0.6))

    fig.tight_layout()
    save_figure(fig, output_dir, 'three_tier_trust')


def graph_ablation(results: list, output_dir: str):
    """Standalone ablation figures for bad-leader coverage and false positives."""
    ablation_results = [r for r in results if r.get('ablation')]
    if not ablation_results:
        print("  [SKIP] ablation figures - no ablation data")
        return

    baseline_bl = [
        r for r in results
        if r.get('controller') == 'reip'
        and (r.get('fault_type') or 'none') == 'bad_leader'
        and not r.get('ablation')
    ]

    labels = ['Baseline']
    coverage_means = [np.mean([r['final_coverage'] for r in baseline_bl])] if baseline_bl else [0.0]
    fp_means = [np.mean([r.get('false_positives', 0) for r in baseline_bl])] if baseline_bl else [0.0]

    for ablation_key in ['no_trust', 'no_causality', 'no_direction']:
        subset = [r for r in ablation_results if r.get('ablation') == ablation_key]
        if not subset:
            continue
        labels.append(ABLATION_LABELS[ablation_key])
        coverage_means.append(np.mean([r['final_coverage'] for r in subset]))
        fp_means.append(np.mean([r.get('false_positives', 0) for r in subset]))

    x = np.arange(len(labels))

    fig_cov, ax_cov = plt.subplots(figsize=(IEEE_SINGLE_COL_W, IEEE_MED_H))
    ax_cov.bar(x, coverage_means, color=COLORS['reip'], edgecolor='black', alpha=0.85)
    ax_cov.set_xticks(x)
    ax_cov.set_xticklabels(labels, rotation=20, ha='right')
    style_axis(ax_cov, '', 'Final coverage (%)', ylim=(0, 105))
    fig_cov.tight_layout()
    save_figure(fig_cov, output_dir, 'ablation_final_coverage')

    fig_fp, ax_fp = plt.subplots(figsize=(IEEE_SINGLE_COL_W, IEEE_MED_H))
    ax_fp.bar(x, fp_means, color=COLORS['fault_line'], edgecolor='black', alpha=0.85)
    ax_fp.set_xticks(x)
    ax_fp.set_xticklabels(labels, rotation=20, ha='right')
    style_axis(ax_fp, '', 'False positives per trial')
    fig_fp.tight_layout()
    save_figure(fig_fp, output_dir, 'ablation_false_positives')


# --------------------------------------------------
# Main
# --------------------------------------------------

def main():
    if len(sys.argv) < 2:
        print("Usage: python test/generate_poster_graphs.py <results_dir> [output_dir]")
        print("  <results_dir> = experiments/run_YYYYMMDD_HHMMSS_*/")
        sys.exit(1)

    results_dir = sys.argv[1]
    output_dir = sys.argv[2] if len(sys.argv) >= 3 else os.path.join(results_dir, "paper_figures")
    os.makedirs(output_dir, exist_ok=True)
    print(f"Loading data from: {results_dir}")

    results = load_results_json(results_dir)
    timelines = load_timelines(results_dir)

    print(f"  {len(results)} experiment results")
    print(f"  {len(timelines)} coverage timelines")

    print("\nGenerating standalone paper figures...")

    if timelines:
        graph_convergence(timelines, output_dir)
    else:
        print("  [SKIP] convergence_plot - no timeline data (re-run experiments)")

    if results:
        graph_time_to_50(results, output_dir)
        graph_detection_latency(results, output_dir)
        graph_coverage_bars(results, output_dir)
        graph_ablation(results, output_dir)

    graph_three_tier_trust(output_dir)

    print(f"\nAll graphs saved to: {output_dir}/")


if __name__ == "__main__":
    main()
