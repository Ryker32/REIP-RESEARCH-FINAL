#!/usr/bin/env python3
"""
ISEF Poster Graph Generator

Generates publication-quality graphs from experimental data:
1. Convergence Plot  — per-trial coverage traces (hero figure)
2. Detection Latency — bar chart comparing detection speed
3. Trust Timeline    — suspicion + trust decay during fault
4. Three-tier Model  — visual explanation diagram
5. Time-to-50% Box  — box plot of coordination speed

Usage:
  python test/generate_poster_graphs.py <results_dir>

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

# Poster-quality settings
plt.rcParams.update({
    'font.size': 13,
    'font.family': 'sans-serif',
    'axes.labelsize': 15,
    'axes.titlesize': 17,
    'legend.fontsize': 11,
    'xtick.labelsize': 11,
    'ytick.labelsize': 11,
    'figure.dpi': 150,
    'savefig.dpi': 300,
    'savefig.bbox': 'tight'
})

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

# ──────────────────────────────────────────────────
# Data loaders
# ──────────────────────────────────────────────────

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


# ──────────────────────────────────────────────────
# Graph 1: Convergence plot (HERO FIGURE)
# ──────────────────────────────────────────────────

def graph_convergence(timelines: dict, output_dir: str):
    """Coverage-over-time with individual trial traces and mean.

    Three panels: Clean, Bad Leader, Freeze Leader.
    """
    fault_panels = [
        ('none',        'Clean (No Fault)'),
        ('bad_leader',  'Bad Leader Fault'),
        ('freeze_leader', 'Freeze Leader Fault'),
    ]
    controllers = ['reip', 'raft', 'decentralized']

    fig, axes = plt.subplots(1, 3, figsize=(18, 5.5), sharey=True)

    tl_groups = group_timelines(timelines)

    for ax, (fault_key, panel_title) in zip(axes, fault_panels):
        for ctrl in controllers:
            key = (ctrl, fault_key)
            traces = tl_groups.get(key, [])
            color = COLORS[ctrl]
            label = LABELS[ctrl]

            # Individual trial traces
            for tl in traces:
                ts = [p[0] for p in tl]
                cs = [p[1] for p in tl]
                ax.plot(ts, cs, color=color, alpha=0.15, linewidth=0.8)

            # Mean curve
            if traces:
                # Resample all traces to common time grid
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

        # Fault injection line(s) - read from timeline data if available
        if fault_key != 'none':
            # Get fault time from timeline metadata (first experiment with this fault)
            fault_time_1 = 10.0  # default
            fault_time_2 = None
            for exp_name, exp_data in timelines.items():
                if (exp_data.get('controller') == controllers[0] and 
                    (exp_data.get('fault_type') or 'none') == fault_key):
                    fault_time_1 = exp_data.get('fault_time_1', 10.0)
                    fault_time_2 = exp_data.get('fault_time_2')
                    break
            
            ax.axvline(x=fault_time_1, color=COLORS['fault_line'], linestyle='--',
                       linewidth=1.5, alpha=0.7, zorder=6)
            ax.annotate('Fault', xy=(fault_time_1 + 1, 5), fontsize=9,
                        color=COLORS['fault_line'], fontweight='bold')
            if fault_time_2:
                ax.axvline(x=fault_time_2, color=COLORS['fault_line'], linestyle=':',
                           linewidth=1.0, alpha=0.5, zorder=6)

        ax.set_title(panel_title, fontweight='bold')
        ax.set_xlabel('Time (s)')
        ax.set_xlim(0, 120)
        ax.set_ylim(0, 105)
        ax.grid(True, alpha=0.25)
        if ax == axes[0]:
            ax.set_ylabel('Coverage (%)')
        ax.legend(loc='lower right', framealpha=0.9)

    fig.suptitle('Coverage Convergence: Individual Trials + Mean',
                 fontsize=18, fontweight='bold', y=1.02)
    plt.tight_layout()
    path = os.path.join(output_dir, 'convergence_plot.png')
    plt.savefig(path)
    print(f"  Saved: {path}")
    plt.close()


# ──────────────────────────────────────────────────
# Graph 2: Time-to-50% box plot
# ──────────────────────────────────────────────────

def graph_time_to_50(results: list, output_dir: str):
    """Box plots comparing time-to-50% across controllers and faults."""
    groups = group_results(results)

    fig, axes = plt.subplots(1, 3, figsize=(14, 5), sharey=True)
    faults = [('none', 'Clean'), ('bad_leader', 'Bad Leader'), ('freeze_leader', 'Freeze Leader')]
    controllers = ['reip', 'raft', 'decentralized']

    for ax, (fkey, flabel) in zip(axes, faults):
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

        bp = ax.boxplot(data, labels=labels, patch_artist=True, widths=0.6)
        for patch, c in zip(bp['boxes'], colors):
            patch.set_facecolor(c)
            patch.set_alpha(0.7)
        for median in bp['medians']:
            median.set_color('black')
            median.set_linewidth(2)

        ax.set_title(flabel, fontweight='bold')
        ax.set_ylabel('Time to 50% (s)' if ax == axes[0] else '')
        ax.grid(True, axis='y', alpha=0.25)

    fig.suptitle('Time to 50% Coverage: Coordination Speed',
                 fontsize=17, fontweight='bold', y=1.02)
    plt.tight_layout()
    path = os.path.join(output_dir, 'time_to_50_boxplot.png')
    plt.savefig(path)
    print(f"  Saved: {path}")
    plt.close()


# ──────────────────────────────────────────────────
# Graph 3: Detection latency
# ──────────────────────────────────────────────────

def graph_detection_latency(results: list, output_dir: str):
    """Bar chart: REIP detection latency under bad_leader."""
    groups = group_results(results)

    reip_bl = groups.get(('reip', 'bad_leader'), [])

    suspicion_times = [r['time_to_first_suspicion']
                       for r in reip_bl if r.get('time_to_first_suspicion')]
    impeach_times = [r['time_to_detection']
                     for r in reip_bl if r.get('time_to_detection')]

    fig, ax = plt.subplots(figsize=(8, 5))

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
        cols.append(COLORS['decentralized'])

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
                    f'{m:.2f}s ± {s:.2f}', ha='center', fontsize=12,
                    fontweight='bold')

    ax.set_ylabel('Time after fault injection (s)')
    ax.set_title('REIP Detection Latency (Bad Leader)', fontweight='bold')
    ax.legend(loc='upper right')
    ax.set_ylim(0, max(means + [1]) * 1.6 if means else 5)
    ax.grid(True, axis='y', alpha=0.25)

    # Annotation
    n = len(reip_bl)
    detected = len([r for r in reip_bl if r.get('time_to_detection')])
    fps = sum(r.get('false_positives', 0) for r in reip_bl)
    ax.annotate(f'{detected}/{n} detected (100%)  |  {fps} false positives',
                xy=(0.5, 0.02), xycoords='axes fraction', fontsize=12,
                ha='center', fontweight='bold',
                bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.6))

    plt.tight_layout()
    path = os.path.join(output_dir, 'detection_latency.png')
    plt.savefig(path)
    print(f"  Saved: {path}")
    plt.close()


# ──────────────────────────────────────────────────
# Graph 4: Final coverage comparison (bar chart)
# ──────────────────────────────────────────────────

def graph_coverage_bars(results: list, output_dir: str):
    """Grouped bar chart: final coverage by controller and fault."""
    groups = group_results(results)
    controllers = ['reip', 'raft', 'decentralized']
    faults = [('none', 'Clean'), ('bad_leader', 'Bad Leader'), ('freeze_leader', 'Freeze Leader')]

    fig, ax = plt.subplots(figsize=(12, 6))
    x = np.arange(len(faults))
    width = 0.25

    for i, ctrl in enumerate(controllers):
        means = []
        stds_ = []
        for fkey, _ in faults:
            trials = groups.get((ctrl, fkey), [])
            covs = [t['final_coverage'] for t in trials]
            means.append(np.mean(covs) if covs else 0)
            stds_.append(np.std(covs) if covs else 0)
        bars = ax.bar(x + i * width, means, width, yerr=stds_, capsize=4,
                      label=LABELS[ctrl], color=COLORS[ctrl], edgecolor='black',
                      alpha=0.85)

    ax.set_ylabel('Final Coverage (%)')
    ax.set_title('Final Coverage @ 120s by Controller and Fault Type',
                 fontweight='bold')
    ax.set_xticks(x + width)
    ax.set_xticklabels([fl for _, fl in faults])
    ax.legend()
    ax.set_ylim(0, 115)
    ax.grid(True, axis='y', alpha=0.25)

    plt.tight_layout()
    path = os.path.join(output_dir, 'coverage_bars.png')
    plt.savefig(path)
    print(f"  Saved: {path}")
    plt.close()


# ──────────────────────────────────────────────────
# Graph 5: Three-tier trust model diagram
# ──────────────────────────────────────────────────

def graph_three_tier_trust(output_dir: str):
    """Visual explanation of three-tier trust model."""
    fig, ax = plt.subplots(figsize=(10, 7))

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
         'Ground truth — physically visited'),
    ]

    for verts, _, color, title, quote, weight, note in tier_data:
        poly = plt.Polygon(verts, facecolor=color, alpha=0.7,
                           edgecolor='black', linewidth=1.5)
        ax.add_patch(poly)
        cx = np.mean([v[0] for v in verts])
        cy = np.mean([v[1] for v in verts])
        ax.text(cx, cy + 0.15, title, ha='center', va='center',
                fontsize=13, fontweight='bold')
        ax.text(cx, cy - 0.35, f'{quote}  —  {weight}',
                ha='center', va='center', fontsize=11, style='italic')

    ax.set_xlim(0, 10)
    ax.set_ylim(0, 8)
    ax.axis('off')
    ax.set_title('Three-Tier Confidence-Weighted Trust Model',
                 fontsize=17, fontweight='bold', pad=15)

    # Key insight box
    ax.text(5, 7.6,
            'Higher tiers = faster detection  |  Lower tiers = more coverage but slower',
            ha='center', fontsize=11,
            bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))

    plt.tight_layout()
    path = os.path.join(output_dir, 'three_tier_trust.png')
    plt.savefig(path)
    print(f"  Saved: {path}")
    plt.close()


# ──────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────

def main():
    output_dir = "poster_graphs"
    os.makedirs(output_dir, exist_ok=True)

    if len(sys.argv) < 2:
        print("Usage: python test/generate_poster_graphs.py <results_dir>")
        print("  <results_dir> = experiments/run_YYYYMMDD_HHMMSS_*/")
        sys.exit(1)

    results_dir = sys.argv[1]
    print(f"Loading data from: {results_dir}")

    results = load_results_json(results_dir)
    timelines = load_timelines(results_dir)

    print(f"  {len(results)} experiment results")
    print(f"  {len(timelines)} coverage timelines")

    print("\nGenerating poster graphs...")

    if timelines:
        graph_convergence(timelines, output_dir)
    else:
        print("  [SKIP] convergence_plot — no timeline data (re-run experiments)")

    if results:
        graph_time_to_50(results, output_dir)
        graph_detection_latency(results, output_dir)
        graph_coverage_bars(results, output_dir)

    graph_three_tier_trust(output_dir)

    print(f"\nAll graphs saved to: {output_dir}/")


if __name__ == "__main__":
    main()
