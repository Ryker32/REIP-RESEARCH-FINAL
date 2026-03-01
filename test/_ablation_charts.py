#!/usr/bin/env python3
"""
Ablation Study: REIP Component Analysis (3-Panel Bar Chart)

Generates an IEEE-style figure with:
  Panel (a): Final coverage bar chart
  Panel (b): Success rate (% reaching 80% coverage) bar chart
  Panel (c): Median time-to-80% (on successful runs)

Also generates:
  - ablation_convergence.png  — coverage vs time overlay
  - ablation_traces.png       — per-trial traces
"""

import json, os, sys, glob, math
import numpy as np

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib as mpl
from collections import defaultdict

# ---- IEEE styling ----
mpl.rcParams.update({
    'font.family': 'serif',
    'font.serif': ['Times New Roman', 'Times', 'DejaVu Serif'],
    'font.size': 8,
    'axes.labelsize': 8,
    'axes.titlesize': 9,
    'xtick.labelsize': 7,
    'ytick.labelsize': 7,
    'axes.linewidth': 0.5,
    'legend.fontsize': 7,
    'figure.dpi': 300,
    'savefig.dpi': 600,
    'savefig.bbox': 'tight',
})

COL_WIDTH = 3.5   # IEEE single column (inches)
FULL_WIDTH = 7.16 # IEEE double column

# ---- Variant styling ----
VARIANT_STYLES = {
    'REIP-Full':     {'color': '#1f77b4', 'marker': 'o', 'ls': '-',  'lw': 2.0},
    'REIP-NoTrust':  {'color': '#d62728', 'marker': 's', 'ls': '--', 'lw': 1.8},
    'REIP-NoDir':    {'color': '#ff7f0e', 'marker': '^', 'ls': '-.', 'lw': 1.8},
    'REIP-NoCaus':   {'color': '#2ca02c', 'marker': 'D', 'ls': ':',  'lw': 1.8},
    'REIP-NoCaus\n(Clean)': {'color': '#9467bd', 'marker': 'v', 'ls': ':', 'lw': 1.5},
}

# Map directory name patterns -> display name
ABLATION_MAP = {
    'Ablation_NoTrust_BL':        'REIP-NoTrust',
    'Ablation_NoDirection_BL':    'REIP-NoDir',
    'Ablation_NoCausality_BL':    'REIP-NoCaus',
    'Ablation_NoCausality_Clean': 'REIP-NoCaus\n(Clean)',
}

# Threshold for "success"
SUCCESS_THRESHOLD = 80.0  # percent


def load_results_json(results_file):
    """Load experiment results from JSON."""
    with open(results_file) as f:
        return json.load(f)


def load_timelines(logs_dir, name_map):
    """Load coverage timelines grouped by variant name."""
    timelines = defaultdict(list)
    for exp_dir in sorted(os.listdir(logs_dir)):
        tl_file = os.path.join(logs_dir, exp_dir, 'coverage_timeline.json')
        if not os.path.exists(tl_file):
            continue
        with open(tl_file) as f:
            data = json.load(f)
        tl = data.get('timeline', [])
        if not tl:
            continue
        variant = None
        for pattern, name in name_map.items():
            if pattern in exp_dir:
                variant = name
                break
        if variant is None:
            continue
        timelines[variant].append(tl)
    return timelines


def load_full_reip_timelines(n100_logs_dir):
    """Load Full REIP (bad_leader) timelines from the N=100 run."""
    timelines = []
    for exp_dir in sorted(os.listdir(n100_logs_dir)):
        if 'reip' not in exp_dir or 'BadLeader' not in exp_dir:
            continue
        tl_file = os.path.join(n100_logs_dir, exp_dir, 'coverage_timeline.json')
        if not os.path.exists(tl_file):
            continue
        with open(tl_file) as f:
            data = json.load(f)
        tl = data.get('timeline', [])
        if tl:
            timelines.append(tl)
    return timelines


def resample_timeline(tl, dt=1.0, t_max=120.0):
    """Resample irregular timeline to uniform grid."""
    ts = np.arange(0, t_max + dt, dt)
    covs = np.zeros_like(ts)
    idx = 0
    for i, t in enumerate(ts):
        while idx < len(tl) - 1 and tl[idx + 1][0] <= t:
            idx += 1
        covs[i] = tl[idx][1] if idx < len(tl) else (tl[-1][1] if tl else 0)
    return ts, covs


# ===========================================================================
# PLOT 1: Three-panel bar chart (main ablation figure)
# ===========================================================================
def plot_ablation_bars(ablation_results, n100_results, output_dir):
    """Generate 3-panel ablation bar chart matching the reference style."""

    # --- Gather per-variant stats ---
    # Full REIP baseline from N=100 bad_leader
    reip_bl = [r for r in n100_results
               if r.get('controller') == 'reip'
               and r.get('fault_type') == 'bad_leader']

    # Ablation groups
    groups = defaultdict(list)
    for r in ablation_results:
        abl = r.get('ablation', 'none')
        fault = r.get('fault_type', 'none') or 'none'
        groups[(abl, fault)].append(r)

    # Build ordered data
    # (label, results_list)
    variant_data = [
        ('REIP-\nFull',    reip_bl),
        ('REIP-\nNoTrust', groups.get(('no_trust', 'bad_leader'), [])),
        ('REIP-\nNoDir',   groups.get(('no_direction', 'bad_leader'), [])),
        ('REIP-\nNoCaus',  groups.get(('no_causality', 'bad_leader'), [])),
    ]

    labels = [d[0] for d in variant_data]
    n_variants = len(labels)
    x_pos = np.arange(n_variants)

    # Compute stats
    mean_covs, sem_covs = [], []
    success_rates, success_sems = [], []
    median_t80, iqr_t80 = [], []

    for label, trials in variant_data:
        covs = [r['final_coverage'] for r in trials]
        n = len(covs)
        mean_covs.append(np.mean(covs))
        sem_covs.append(np.std(covs) / np.sqrt(n) if n > 1 else 0)

        successes = [1 if c >= SUCCESS_THRESHOLD else 0 for c in covs]
        sr = np.mean(successes)
        success_rates.append(sr * 100)
        success_sems.append(np.std(successes) / np.sqrt(n) * 100 if n > 1 else 0)

        t80s = [r['time_to_80'] for r in trials
                if r.get('time_to_80') is not None]
        if t80s:
            median_t80.append(np.median(t80s))
            q75, q25 = np.percentile(t80s, [75, 25])
            iqr_t80.append((q75 - q25) / 2)
        else:
            median_t80.append(None)
            iqr_t80.append(0)

    # --- Create figure ---
    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(FULL_WIDTH, 2.5))

    # ---- Panel (a): Final Coverage ----
    colors_a = ['steelblue'] * n_variants
    bars1 = ax1.bar(x_pos, mean_covs, yerr=sem_covs, capsize=3, alpha=0.8,
                    color=colors_a, edgecolor='black', linewidth=0.5)
    for i, (m, s) in enumerate(zip(mean_covs, sem_covs)):
        ax1.text(i, m + s + 1.5, f'{m:.1f}%', ha='center', va='bottom',
                 fontsize=7, fontweight='bold')

    ax1.set_ylabel('Final Coverage (%)')
    ax1.set_title('(a) Final Coverage', fontweight='bold')
    ax1.set_xticks(x_pos)
    ax1.set_xticklabels(labels, ha='center')
    ax1.set_xlabel('Controller Mode')
    ax1.set_ylim(0, max(mean_covs) * 1.18)
    ax1.grid(axis='y', alpha=0.3, ls='--', lw=0.5)
    for sp in ('top', 'right'):
        ax1.spines[sp].set_visible(False)

    # ---- Panel (b): Success Rate ----
    colors_b = ['coral'] * n_variants
    bars2 = ax2.bar(x_pos, success_rates, yerr=success_sems, capsize=3,
                    alpha=0.8, color=colors_b, edgecolor='black', linewidth=0.5)
    for i, (sr, se) in enumerate(zip(success_rates, success_sems)):
        ax2.text(i, sr + se + 2, f'{sr:.1f}%', ha='center', va='bottom',
                 fontsize=7, fontweight='bold')

    ax2.set_ylabel('Success Rate (%)')
    ax2.set_title(f'(b) Success Rate ({SUCCESS_THRESHOLD:.0f}% Coverage)',
                  fontweight='bold')
    ax2.set_xticks(x_pos)
    ax2.set_xticklabels(labels, ha='center')
    ax2.set_xlabel('Controller Mode')
    ax2.set_ylim(0, 110)
    ax2.grid(axis='y', alpha=0.3, ls='--', lw=0.5)
    for sp in ('top', 'right'):
        ax2.spines[sp].set_visible(False)

    # ---- Panel (c): Median Time to 80% ----
    valid_idx = [i for i, t in enumerate(median_t80) if t is not None]
    if valid_idx:
        vx = np.arange(len(valid_idx))
        vt = [median_t80[i] for i in valid_idx]
        ve = [iqr_t80[i] for i in valid_idx]
        vl = [labels[i] for i in valid_idx]

        bars3 = ax3.bar(vx, vt, yerr=ve, capsize=3, alpha=0.8,
                        color='mediumseagreen', edgecolor='black', linewidth=0.5)
        for j, (t, e) in enumerate(zip(vt, ve)):
            ax3.text(j, t + e + 1, f'{t:.0f}s', ha='center', va='bottom',
                     fontsize=7, fontweight='bold')
        ax3.set_xticks(vx)
        ax3.set_xticklabels(vl, ha='center')
    else:
        ax3.text(0.5, 0.5, 'No runs\nreached 80%',
                 ha='center', va='center', transform=ax3.transAxes, fontsize=8)
        ax3.set_xticks([])

    ax3.set_ylabel('Median Time-to-80% (s)')
    title_c = '(c) Time to 80% Coverage'
    if any(t is None for t in median_t80):
        title_c += '\n(successful runs only)'
    ax3.set_title(title_c, fontweight='bold')
    ax3.set_xlabel('Controller Mode')
    ax3.grid(axis='y', alpha=0.3, ls='--', lw=0.5)
    for sp in ('top', 'right'):
        ax3.spines[sp].set_visible(False)

    fig.tight_layout()
    for fmt in ('png', 'pdf'):
        fig.savefig(os.path.join(output_dir, f'ablation_figure.{fmt}'),
                    facecolor='white')
    plt.close(fig)
    print('Saved: ablation_figure.png/pdf')


# ===========================================================================
# PLOT 2: Convergence curves (mean ± std)
# ===========================================================================
def plot_convergence(timelines, output_dir):
    fig, ax = plt.subplots(figsize=(COL_WIDTH, 2.8))
    dt, t_max = 1.0, 120.0
    ts = np.arange(0, t_max + dt, dt)

    order = ['REIP-Full', 'REIP-NoTrust', 'REIP-NoDir', 'REIP-NoCaus',
             'REIP-NoCaus\n(Clean)']

    for variant in order:
        if variant not in timelines:
            continue
        style = VARIANT_STYLES[variant]
        trials = timelines[variant]

        all_covs = []
        for tl in trials:
            _, covs = resample_timeline(tl, dt, t_max)
            all_covs.append(covs)
        all_covs = np.array(all_covs)
        mean = np.nanmean(all_covs, axis=0)
        std = np.nanstd(all_covs, axis=0)

        ax.fill_between(ts, mean - std, mean + std, alpha=0.12,
                        color=style['color'])
        marker_every = max(1, len(ts) // 10)
        ax.plot(ts, mean, color=style['color'], linestyle=style['ls'],
                linewidth=style['lw'], marker=style['marker'],
                markersize=4, markevery=marker_every, label=variant)

    ax.axvline(10, color='gray', ls='--', lw=0.7, alpha=0.5)
    ax.axvline(30, color='gray', ls='--', lw=0.7, alpha=0.5)
    ax.text(10.5, 2, 'F1', fontsize=7, color='gray')
    ax.text(30.5, 2, 'F2', fontsize=7, color='gray')

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Coverage (%)')
    ax.set_xlim(0, 120)
    ax.set_ylim(0, 105)
    ax.legend(loc='lower right', framealpha=0.9, edgecolor='0.8')

    for fmt in ('png', 'pdf'):
        fig.savefig(os.path.join(output_dir, f'ablation_convergence.{fmt}'))
    plt.close(fig)
    print('Saved: ablation_convergence.png/pdf')


# ===========================================================================
# PLOT 3: Per-trial trace panels
# ===========================================================================
def plot_traces(timelines, output_dir):
    variants = ['REIP-Full', 'REIP-NoTrust', 'REIP-NoDir', 'REIP-NoCaus']
    present = [v for v in variants if v in timelines]
    if not present:
        print('No trace data available')
        return

    n = len(present)
    fig, axes = plt.subplots(1, n, figsize=(FULL_WIDTH, 2.5), sharey=True)
    if n == 1:
        axes = [axes]

    dt, t_max = 1.0, 120.0
    ts = np.arange(0, t_max + dt, dt)

    for ax, variant in zip(axes, present):
        style = VARIANT_STYLES[variant]
        trials = timelines[variant]

        all_covs = []
        for tl in trials:
            _, covs = resample_timeline(tl, dt, t_max)
            all_covs.append(covs)
            ax.plot(ts, covs, color=style['color'], alpha=0.2, lw=0.5)

        all_covs = np.array(all_covs)
        mean = np.nanmean(all_covs, axis=0)
        ax.plot(ts, mean, color=style['color'], lw=2.0, label='Mean')

        ax.axvline(10, color='gray', ls='--', lw=0.5, alpha=0.4)
        ax.axvline(30, color='gray', ls='--', lw=0.5, alpha=0.4)
        ax.set_title(variant, fontsize=9, fontweight='bold')
        ax.set_xlabel('Time (s)')
        ax.set_xlim(0, 120)
        ax.set_ylim(0, 105)

    axes[0].set_ylabel('Coverage (%)')
    fig.tight_layout()
    for fmt in ('png', 'pdf'):
        fig.savefig(os.path.join(output_dir, f'ablation_traces.{fmt}'))
    plt.close(fig)
    print('Saved: ablation_traces.png/pdf')


# ===========================================================================
# Main
# ===========================================================================
def main():
    ablation_dir = sys.argv[1] if len(sys.argv) > 1 else None
    n100_dir = sys.argv[2] if len(sys.argv) > 2 else None

    if not ablation_dir:
        exp_root = 'experiments'
        candidates = sorted(glob.glob(os.path.join(exp_root, 'run_*_30trials_*')))
        if candidates:
            ablation_dir = candidates[-1]
        else:
            print("Usage: python _ablation_charts.py <ablation_dir> [n100_dir]")
            sys.exit(1)

    if not n100_dir:
        exp_root = 'experiments'
        candidates = sorted(glob.glob(os.path.join(exp_root, 'run_*_100trials_*')))
        if candidates:
            n100_dir = candidates[-1]

    print(f'Ablation data: {ablation_dir}')
    if n100_dir:
        print(f'Full REIP data: {n100_dir}')

    # ---- Load JSON results ----
    abl_json = glob.glob(os.path.join(ablation_dir, 'results_final_*.json'))
    ablation_results = load_results_json(abl_json[0]) if abl_json else []

    n100_results = []
    if n100_dir:
        n100_json = glob.glob(os.path.join(n100_dir, 'results_final_*.json'))
        if n100_json:
            n100_results = load_results_json(n100_json[0])

    print(f'  Ablation results: {len(ablation_results)}')
    print(f'  N=100 results: {len(n100_results)}')

    # ---- Load timelines ----
    timelines = load_timelines(os.path.join(ablation_dir, 'logs'), ABLATION_MAP)
    if n100_dir:
        full_tls = load_full_reip_timelines(os.path.join(n100_dir, 'logs'))
        if full_tls:
            timelines['REIP-Full'] = full_tls
            print(f'  Added {len(full_tls)} REIP-Full baseline timelines')

    total = sum(len(v) for v in timelines.values())
    print(f'  {total} total timelines across {len(timelines)} variants')

    output_dir = ablation_dir

    # Generate all three figures
    plot_ablation_bars(ablation_results, n100_results, output_dir)
    plot_convergence(timelines, output_dir)
    plot_traces(timelines, output_dir)

    print(f'\nAll figures saved to: {output_dir}/')


if __name__ == '__main__':
    main()
