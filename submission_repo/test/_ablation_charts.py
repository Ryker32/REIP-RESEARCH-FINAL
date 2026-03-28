#!/usr/bin/env python3
"""
Ablation Study: REIP Component Analysis (3-Panel Bar Chart)

Generates an IEEE-style figure with:
  Panel (a): Final coverage bar chart
  Panel (b): Success rate (% reaching 80% coverage) bar chart
  Panel (c): Median time-to-80% (on successful runs)

Also generates:
  - ablation_convergence.png  - coverage vs time overlay
  - ablation_traces.png       - per-trial traces
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
    'REIP-NoTrust':  {'color': '#d62728', 'marker': 's', 'ls': '-', 'lw': 1.8},
    'REIP-NoDir':    {'color': '#ff7f0e', 'marker': '^', 'ls': '-', 'lw': 1.8},
    'REIP-NoCaus':   {'color': '#2ca02c', 'marker': 'D', 'ls': '-', 'lw': 1.8},
    'REIP-NoTrust\n(Clean)': {'color': '#d62728', 'marker': 's', 'ls': '-', 'lw': 1.5},
    'REIP-NoDir\n(Clean)':   {'color': '#ff7f0e', 'marker': '^', 'ls': '-', 'lw': 1.5},
    'REIP-NoCaus\n(Clean)': {'color': '#9467bd', 'marker': 'v', 'ls': '-', 'lw': 1.5},
}

# Map directory name patterns -> display name
ABLATION_MAP = {
    'Ablation_NoTrust_BL':        'REIP-NoTrust',
    'Ablation_NoTrust_Clean':     'REIP-NoTrust\n(Clean)',
    'Ablation_NoDirection_BL':    'REIP-NoDir',
    'Ablation_NoDirection_Clean': 'REIP-NoDir\n(Clean)',
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


def merge_timeline_dicts(*timeline_dicts):
    merged = defaultdict(list)
    for tl_dict in timeline_dicts:
        for key, vals in tl_dict.items():
            merged[key].extend(vals)
    return merged


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


def build_variant_data(ablation_results, n100_results):
    """Collect bad-leader and clean trials for each ablation variant."""
    reip_full_bad = [
        r for r in n100_results
        if r.get('controller') == 'reip'
        and (r.get('fault_type') or 'none') == 'bad_leader'
        and not r.get('ablation')
    ]
    reip_full_clean = [
        r for r in n100_results
        if r.get('controller') == 'reip'
        and (r.get('fault_type') or 'none') == 'none'
        and not r.get('ablation')
    ]

    groups = defaultdict(list)
    for r in ablation_results:
        abl = r.get('ablation', 'none')
        fault = r.get('fault_type', 'none') or 'none'
        groups[(abl, fault)].append(r)

    return [
        {
            'key': 'REIP-Full',
            'label': 'Full\nREIP',
            'short': 'Full',
            'color': VARIANT_STYLES['REIP-Full']['color'],
            'bad_trials': reip_full_bad,
            'clean_trials': reip_full_clean,
        },
        {
            'key': 'REIP-NoTrust',
            'label': 'No\nTrust',
            'short': 'No Trust',
            'color': VARIANT_STYLES['REIP-NoTrust']['color'],
            'bad_trials': groups.get(('no_trust', 'bad_leader'), []),
            'clean_trials': groups.get(('no_trust', 'none'), []),
        },
        {
            'key': 'REIP-NoDir',
            'label': 'No\nDirection',
            'short': 'No Direction',
            'color': VARIANT_STYLES['REIP-NoDir']['color'],
            'bad_trials': groups.get(('no_direction', 'bad_leader'), []),
            'clean_trials': groups.get(('no_direction', 'none'), []),
        },
        {
            'key': 'REIP-NoCaus',
            'label': 'No\nCausality',
            'short': 'No Causality',
            'color': VARIANT_STYLES['REIP-NoCaus']['color'],
            'bad_trials': groups.get(('no_causality', 'bad_leader'), []),
            'clean_trials': groups.get(('no_causality', 'none'), []),
        },
    ]


def mean_metric(trials, key):
    vals = [r.get(key) for r in trials if r.get(key) is not None]
    return float(np.mean(vals)) if vals else None


def sem_metric(trials, key):
    vals = [r.get(key) for r in trials if r.get(key) is not None]
    if len(vals) <= 1:
        return 0.0
    return float(np.std(vals) / np.sqrt(len(vals)))


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
        ('Full\nREIP',      reip_bl),
        ('No\nTrust',       groups.get(('no_trust', 'bad_leader'), [])),
        ('No\nDirection',   groups.get(('no_direction', 'bad_leader'), [])),
        ('No\nCausality',   groups.get(('no_causality', 'bad_leader'), [])),
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

    fig.text(0.5, 0.01,
             'Error bars: bars show SEM; time-to-80 bars show median with IQR/2.',
             ha='center', va='bottom', fontsize=7)
    fig.tight_layout(rect=[0, 0.04, 1, 1])
    for fmt in ('png', 'pdf'):
        fig.savefig(os.path.join(output_dir, f'ablation_figure.{fmt}'),
                    facecolor='white')
    plt.close(fig)
    print('Saved: ablation_figure.png/pdf')


def plot_ablation_tradeoffs(ablation_results, n100_results, output_dir):
    """Sharper ablation story: robustness, degradation, and clean precision."""
    variants = build_variant_data(ablation_results, n100_results)

    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(FULL_WIDTH, 2.7))

    # ---- Panel (a): bad-leader success rate for all variants ----
    x = np.arange(len(variants))
    success_rates = []
    success_sems = []
    colors = []
    for variant in variants:
        bad = variant['bad_trials']
        covs = [r['final_coverage'] for r in bad]
        successes = [1 if c >= SUCCESS_THRESHOLD else 0 for c in covs]
        success_rates.append(np.mean(successes) * 100 if successes else 0)
        success_sems.append(
            np.std(successes) / np.sqrt(len(successes)) * 100
            if len(successes) > 1 else 0
        )
        colors.append(variant['color'])

    bars = ax1.bar(
        x, success_rates, yerr=success_sems, capsize=3,
        color=colors, edgecolor='black', linewidth=0.5, alpha=0.85
    )
    for i, (bar, sr) in enumerate(zip(bars, success_rates)):
        ax1.text(
            bar.get_x() + bar.get_width() / 2,
            sr + success_sems[i] + 2,
            f'{sr:.0f}%',
            ha='center', va='bottom', fontsize=7, fontweight='bold'
        )
    ax1.set_ylabel(f'Success Rate (>= {SUCCESS_THRESHOLD:.0f}%)')
    ax1.set_title('(a) Robust Success Under Bad Leader', fontweight='bold')
    ax1.set_xticks(x)
    ax1.set_xticklabels([v['label'] for v in variants], ha='center')
    ax1.set_ylim(0, 110)
    ax1.grid(axis='y', alpha=0.3, ls='--', lw=0.5)
    for sp in ('top', 'right'):
        ax1.spines[sp].set_visible(False)

    # ---- Panel (b): degradation score where clean baseline exists ----
    degradable = [v for v in variants if v['clean_trials']]
    x2 = np.arange(len(degradable))
    degradations = []
    for variant in degradable:
        clean_mean = mean_metric(variant['clean_trials'], 'final_coverage')
        bad_mean = mean_metric(variant['bad_trials'], 'final_coverage')
        degr = ((bad_mean - clean_mean) / clean_mean) * 100 if clean_mean else 0
        degradations.append(degr)

    bars = ax2.bar(
        x2, degradations,
        color=[v['color'] for v in degradable],
        edgecolor='black', linewidth=0.5, alpha=0.85
    )
    ax2.axhline(0, color='black', linewidth=0.8, alpha=0.6)
    for bar, degr in zip(bars, degradations):
        ax2.text(
            bar.get_x() + bar.get_width() / 2,
            degr + (1.5 if degr >= 0 else -1.5),
            f'{degr:.1f}%',
            ha='center', va='bottom' if degr >= 0 else 'top',
            fontsize=7, fontweight='bold'
        )
    ax2.set_ylabel('Degradation vs Clean (%)')
    ax2.set_title('(b) Fault Degradation\n(clean baseline available)', fontweight='bold')
    ax2.set_xticks(x2)
    ax2.set_xticklabels([v['label'] for v in degradable], ha='center')
    ax2.grid(axis='y', alpha=0.3, ls='--', lw=0.5)
    for sp in ('top', 'right'):
        ax2.spines[sp].set_visible(False)

    # ---- Panel (c): clean false positives for precision tradeoff ----
    clean_fp_means = [mean_metric(v['clean_trials'], 'false_positives') for v in degradable]
    clean_fp_sems = [sem_metric(v['clean_trials'], 'false_positives') for v in degradable]
    bars = ax3.bar(
        x2, clean_fp_means, yerr=clean_fp_sems, capsize=3,
        color=[v['color'] for v in degradable],
        edgecolor='black', linewidth=0.5, alpha=0.85
    )
    for i, (bar, fp) in enumerate(zip(bars, clean_fp_means)):
        ax3.text(
            bar.get_x() + bar.get_width() / 2,
            fp + clean_fp_sems[i] + 0.2,
            f'{fp:.1f}',
            ha='center', va='bottom', fontsize=7, fontweight='bold'
        )
    ax3.set_ylabel('False Positives (clean)')
    ax3.set_title('(c) Clean Precision Cost', fontweight='bold')
    ax3.set_xticks(x2)
    ax3.set_xticklabels([v['label'] for v in degradable], ha='center')
    ax3.grid(axis='y', alpha=0.3, ls='--', lw=0.5)
    for sp in ('top', 'right'):
        ax3.spines[sp].set_visible(False)

    fig.text(0.5, 0.01,
             'Error bars: success and clean-precision panels show SEM.',
             ha='center', va='bottom', fontsize=7)
    fig.tight_layout(rect=[0, 0.04, 1, 1])
    for fmt in ('png', 'pdf'):
        fig.savefig(os.path.join(output_dir, f'ablation_tradeoffs.{fmt}'),
                    facecolor='white')
    plt.close(fig)
    print('Saved: ablation_tradeoffs.png/pdf')


def plot_ablation_pareto(ablation_results, n100_results, output_dir):
    """Precision vs robustness tradeoff for variants with both clean and fault data."""
    variants = [v for v in build_variant_data(ablation_results, n100_results)
                if v['clean_trials']]
    if not variants:
        print('No clean/fault paired ablation data available for Pareto plot')
        return

    fig, ax = plt.subplots(figsize=(COL_WIDTH, 2.9))

    for variant in variants:
        bad_covs = [r['final_coverage'] for r in variant['bad_trials']]
        success_rate = (
            np.mean([1 if c >= SUCCESS_THRESHOLD else 0 for c in bad_covs]) * 100
            if bad_covs else 0
        )
        false_pos = mean_metric(variant['clean_trials'], 'false_positives') or 0
        clean_mean = mean_metric(variant['clean_trials'], 'final_coverage') or 0
        bad_mean = mean_metric(variant['bad_trials'], 'final_coverage') or 0
        degradation = ((bad_mean - clean_mean) / clean_mean) * 100 if clean_mean else 0

        ax.scatter(
            success_rate, false_pos,
            s=140, color=variant['color'], edgecolor='black',
            linewidth=0.8, zorder=3
        )
        ax.text(
            success_rate + 1.5, false_pos + 0.15,
            f"{variant['short']}\n({degradation:.1f}%)",
            fontsize=7, fontweight='bold', ha='left', va='bottom'
        )

    # Ideal region: high success, low false positives
    ax.annotate(
        'Better',
        xy=(98, 0.2), xytext=(84, 2.5),
        arrowprops=dict(arrowstyle='->', lw=1.0, color='black'),
        fontsize=8, fontweight='bold'
    )
    ax.set_xlabel(f'Bad-Leader Success Rate (>= {SUCCESS_THRESHOLD:.0f}%)')
    ax.set_ylabel('False Positives (clean)')
    ax.set_title('Ablation Pareto Tradeoff', fontweight='bold')
    ax.set_xlim(0, 105)
    ymax = max(mean_metric(v['clean_trials'], 'false_positives') or 0 for v in variants)
    ax.set_ylim(0, max(3.0, ymax * 1.25))
    ax.grid(True, alpha=0.3, ls='--', lw=0.5)
    for sp in ('top', 'right'):
        ax.spines[sp].set_visible(False)

    fig.tight_layout()
    for fmt in ('png', 'pdf'):
        fig.savefig(os.path.join(output_dir, f'ablation_pareto.{fmt}'),
                    facecolor='white')
    plt.close(fig)
    print('Saved: ablation_pareto.png/pdf')


# ===========================================================================
# PLOT 2: Convergence curves (mean +/- std)
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


def plot_ablation_lifecycle(ablation_results, n100_results, timelines, output_dir):
    """Poster-clean ablation figure: whole-lifespan fault convergence + clean error cost."""
    variants = build_variant_data(ablation_results, n100_results)

    fig = plt.figure(figsize=(FULL_WIDTH, 2.9))
    gs = fig.add_gridspec(1, 2, width_ratios=[1.85, 1.0], wspace=0.32)
    ax1 = fig.add_subplot(gs[0, 0])
    ax2 = fig.add_subplot(gs[0, 1])

    dt, t_max = 1.0, 120.0
    ts = np.arange(0, t_max + dt, dt)
    order = ['REIP-Full', 'REIP-NoTrust', 'REIP-NoDir', 'REIP-NoCaus']

    # ---- Panel (a): full lifespan under bad leader ----
    for variant in order:
        variant_cfg = next((v for v in variants if v['key'] == variant), None)
        if variant_cfg is None or not variant_cfg['bad_trials']:
            continue

        if variant not in timelines:
            continue
        style = VARIANT_STYLES[variant]
        all_covs = []
        for tl in timelines[variant]:
            _, covs = resample_timeline(tl, dt, t_max)
            all_covs.append(covs)
        if not all_covs:
            continue

        all_covs = np.asarray(all_covs)
        med = np.median(all_covs, axis=0)
        q25 = np.percentile(all_covs, 25, axis=0)
        q75 = np.percentile(all_covs, 75, axis=0)
        marker_every = max(1, len(ts) // 10)

        ax1.fill_between(ts, q25, q75, color=style['color'], alpha=0.12, linewidth=0, zorder=1)
        ax1.plot(
            ts, med,
            color=style['color'], linestyle=style['ls'], linewidth=style['lw'],
            marker=style['marker'], markersize=3.8, markevery=marker_every,
            label=variant_cfg['short'], zorder=3
        )
    for ft, label in [(10, 'Fault 1'), (30, 'Fault 2')]:
        ax1.axvline(ft, color='0.45', ls='--', lw=0.8, alpha=0.65, zorder=2)
        ax1.text(ft + 1.2, 4.5, label, fontsize=7, color='0.35', ha='left', va='bottom')

    ax1.set_title('(a) Ablation Convergence Under Bad-Leader Faults', fontweight='bold')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Coverage (%)')
    ax1.set_xlim(0, 120)
    ax1.set_ylim(0, 105)
    ax1.grid(axis='y', alpha=0.24, ls='--', lw=0.45)
    ax1.legend(loc='lower right', framealpha=0.95, edgecolor='0.82', ncol=2)
    for sp in ('top', 'right'):
        ax1.spines[sp].set_visible(False)

    # ---- Panel (b): clean false positives / error cost ----
    clean_variants = [v for v in variants if v['clean_trials']]
    x = np.arange(len(clean_variants))
    means = [mean_metric(v['clean_trials'], 'false_positives') or 0.0 for v in clean_variants]
    sems = [sem_metric(v['clean_trials'], 'false_positives') for v in clean_variants]
    plot_vals = [max(0.1, m) for m in means]
    bars = ax2.bar(
        x, plot_vals, yerr=sems, capsize=3,
        color=[v['color'] for v in clean_variants],
        edgecolor='black', linewidth=0.5, alpha=0.86
    )
    for i, (bar, val, pv) in enumerate(zip(bars, means, plot_vals)):
        ax2.text(
            bar.get_x() + bar.get_width() / 2,
            pv * 1.12 + sems[i],
            f'{val:.1f}',
            ha='center', va='bottom', fontsize=7, fontweight='bold'
        )
    ax2.set_title('(b) Clean Error Burden', fontweight='bold')
    ax2.set_ylabel('False Positives / Errors (clean, log scale)')
    ax2.set_xticks(x)
    ax2.set_xticklabels([v['label'] for v in clean_variants], ha='center')
    ax2.set_yscale('log')
    ax2.set_ylim(0.1, max(plot_vals) * 1.7)
    ax2.grid(axis='y', alpha=0.24, ls='--', lw=0.45)
    for sp in ('top', 'right'):
        ax2.spines[sp].set_visible(False)

    fig.text(
        0.5, 0.02,
        'Shading shows IQR under bad-leader faults; false-positive bars show clean-condition SEM.',
        ha='center', va='bottom', fontsize=7
    )
    fig.subplots_adjust(left=0.065, right=0.985, top=0.89, bottom=0.20, wspace=0.32)
    for fmt in ('png', 'pdf'):
        fig.savefig(os.path.join(output_dir, f'ablation_lifecycle.{fmt}'), facecolor='white')
    plt.close(fig)
    print('Saved: ablation_lifecycle.png/pdf')


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
            print("Usage: python _ablation_charts.py <ablation_dir> [extra_ablation_dir ...] [n100_dir]")
            sys.exit(1)

    ablation_dirs = [ablation_dir]
    if len(sys.argv) > 3:
        ablation_dirs = sys.argv[1:-1]
        n100_dir = sys.argv[-1]

    if not n100_dir:
        exp_root = 'experiments'
        candidates = sorted(glob.glob(os.path.join(exp_root, 'run_*_100trials_*')))
        if candidates:
            n100_dir = candidates[-1]

    print(f'Ablation data: {ablation_dirs}')
    if n100_dir:
        print(f'Full REIP data: {n100_dir}')

    # ---- Load JSON results ----
    ablation_results = []
    for abl_dir in ablation_dirs:
        abl_json = glob.glob(os.path.join(abl_dir, 'results_final_*.json'))
        if abl_json:
            ablation_results.extend(load_results_json(abl_json[0]))

    n100_results = []
    if n100_dir:
        n100_json = glob.glob(os.path.join(n100_dir, 'results_final_*.json'))
        if n100_json:
            n100_results = load_results_json(n100_json[0])

    print(f'  Ablation results: {len(ablation_results)}')
    print(f'  N=100 results: {len(n100_results)}')

    # ---- Load timelines ----
    timeline_sets = []
    for abl_dir in ablation_dirs:
        timeline_sets.append(load_timelines(os.path.join(abl_dir, 'logs'), ABLATION_MAP))
    timelines = merge_timeline_dicts(*timeline_sets)
    if n100_dir:
        full_tls = load_full_reip_timelines(os.path.join(n100_dir, 'logs'))
        if full_tls:
            timelines['REIP-Full'] = full_tls
            print(f'  Added {len(full_tls)} REIP-Full baseline timelines')

    total = sum(len(v) for v in timelines.values())
    print(f'  {total} total timelines across {len(timelines)} variants')

    output_dir = ablation_dirs[0]

    # Generate all three figures
    plot_ablation_bars(ablation_results, n100_results, output_dir)
    plot_ablation_tradeoffs(ablation_results, n100_results, output_dir)
    plot_ablation_pareto(ablation_results, n100_results, output_dir)
    plot_convergence(timelines, output_dir)
    plot_traces(timelines, output_dir)
    plot_ablation_lifecycle(ablation_results, n100_results, timelines, output_dir)

    print(f'\nAll figures saved to: {output_dir}/')


if __name__ == '__main__':
    main()
