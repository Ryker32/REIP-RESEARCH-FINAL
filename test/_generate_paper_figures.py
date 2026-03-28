#!/usr/bin/env python3
"""
Generate all IEEE-quality figures for the REIP paper.

IEEE formatting rules (IEEEtran compliance):
  - Column width:  3.5 in (single), 7.16 in (double)
  - Font:          Serif (Times), 8 pt base
  - Axes:          Thin (0.5 pt), ticks inward
  - Grid:          Horizontal only, very faint
  - Colors:        Muted, B&W-distinguishable (hatching required)
  - No embedded figure titles - use LaTeX \\caption{}
  - 300 DPI minimum for raster output

Produces:
  - Standalone single-column figures (one per controller / condition)
  - Combined multi-panel figures (backward compat with existing LaTeX)
"""

import argparse
import json, os, glob
import numpy as np
from collections import defaultdict

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import matplotlib.patches as mpatches

# ====================================================================
# IEEE rcParams
# ====================================================================
plt.rcParams.update({
    'font.family': 'serif',
    'font.serif': ['Times New Roman', 'Times', 'DejaVu Serif'],
    'font.size': 8,
    'axes.labelsize': 9,
    'axes.titlesize': 9,
    'legend.fontsize': 7,
    'xtick.labelsize': 8,
    'ytick.labelsize': 8,
    'axes.linewidth': 0.5,
    'xtick.major.width': 0.4,
    'ytick.major.width': 0.4,
    'xtick.minor.width': 0.3,
    'ytick.minor.width': 0.3,
    'xtick.direction': 'in',
    'ytick.direction': 'in',
    'xtick.major.pad': 3,
    'ytick.major.pad': 3,
    'xtick.top': True,
    'ytick.right': True,
    'axes.grid': False,
    'grid.linewidth': 0.3,
    'grid.alpha': 0.35,
    'grid.color': '#b0b0b0',
    'grid.linestyle': '-',
    'lines.linewidth': 1.0,
    'lines.markersize': 3.5,
    'patch.linewidth': 0.4,
    'figure.dpi': 300,
    'savefig.dpi': 300,
    'savefig.bbox': 'tight',
    'savefig.pad_inches': 0.02,
    'mathtext.fontset': 'stix',
    'text.usetex': False,
})

# ====================================================================
# Colour palette - muted, harmonious, B&W-safe via hatching/markers
# ====================================================================
CLR_CLEAN  = '#4a4a4a'   # dark grey
CLR_BAD    = '#c44e52'   # muted red
CLR_FREEZE = '#2a7b9b'   # teal blue

CLR_REIP   = '#1f4e79'   # navy
CLR_RAFT   = '#a63d40'   # muted red
CLR_DEC    = '#3a7d44'   # muted green

FAULT_COLORS  = {'none': CLR_CLEAN, 'bad_leader': CLR_BAD,
                 'freeze_leader': CLR_FREEZE}
FAULT_HATCHES = {'none': '',  'bad_leader': '///',  'freeze_leader': '...'}
FAULT_LABELS  = {'none': 'Clean', 'bad_leader': 'Bad Leader',
                 'freeze_leader': 'Freeze Leader'}
FAULT_STYLES  = {'none': '-', 'bad_leader': '--', 'freeze_leader': '-.'}
FAULT_MARKERS = {'none': 'o', 'bad_leader': 'x',  'freeze_leader': 'D'}

CTRL_COLORS  = {'reip': CLR_REIP, 'raft': CLR_RAFT,
                'decentralized': CLR_DEC}
CTRL_LABELS  = {'reip': 'REIP (Proposed)', 'raft': 'Raft (Baseline)',
                'decentralized': 'Decentr. (Ref.)'}
CTRL_MARKERS = {'reip': 'o',  'raft': 's',  'decentralized': '^'}
MARKER_EVERY = 20

IEEE_SINGLE = 3.5
IEEE_DOUBLE = 7.16
FAULT_INJECT_TIMES = [10, 30]
RESILIENCE_FAULT_COLORS = {
    'none': '#1a1a1a',
    'bad_leader': '#e74c3c',
    'freeze_leader': '#3498db',
}

DEFAULT_MAIN_DIR     = 'experiments/run_20260228_014625_multiroom_100trials_all'
DEFAULT_FREEZE_DIR   = 'experiments/run_20260228_135632_multiroom_100trials_FreezeLeader'
DEFAULT_ABLATION_DIR = 'experiments/run_20260228_102956_multiroom_30trials_all'
DEFAULT_OUTPUT_DIR   = 'paper_docs/Ryker_Kollmyer___UPDATED_PAPER'


# ====================================================================
# Helpers
# ====================================================================
def ieee_grid(ax, axis='y'):
    ax.grid(True, axis=axis, linewidth=0.3, alpha=0.35, color='#b0b0b0')
    ax.set_axisbelow(True)


def save_fig(fig, output_dir, name):
    for ext in ('png', 'pdf'):
        fig.savefig(os.path.join(output_dir, f'{name}.{ext}'))
    plt.close(fig)
    print(f'  {name}')


def _fault_vlines(ax, add_labels=False, label_size=6.5):
    labels = ['Fault 1', 'Fault 2']
    for i, ft in enumerate(FAULT_INJECT_TIMES):
        ax.axvline(ft, color='#888', lw=0.5, ls='-.', alpha=0.45)
        if add_labels:
            ax.text(ft, 102.0, labels[i], ha='center', va='bottom',
                    fontsize=label_size, fontweight='bold', color='#444444',
                    bbox=dict(boxstyle='round,pad=0.18', facecolor='white',
                              edgecolor='#aaaaaa', linewidth=0.4, alpha=0.92))


# ====================================================================
# Data Loading
# ====================================================================
def load_results(path):
    pattern = os.path.join(path, 'results_final_*.json')
    files = sorted(glob.glob(pattern))
    if not files:
        print(f"  WARNING: No results_final_*.json in {path}")
        return []
    with open(files[-1]) as f:
        return json.load(f)


def load_timelines(dirs):
    timelines = defaultdict(list)
    for d in dirs:
        logs_dir = os.path.join(d, 'logs')
        if not os.path.isdir(logs_dir):
            continue
        for exp_dir in sorted(os.listdir(logs_dir)):
            tl_file = os.path.join(logs_dir, exp_dir, 'coverage_timeline.json')
            if not os.path.exists(tl_file):
                continue
            with open(tl_file) as f:
                data = json.load(f)
            ctrl  = data['controller']
            fault = data.get('fault_type') or 'none'
            if fault == 'oscillate_leader':
                continue
            timelines[(ctrl, fault)].append(data['timeline'])
    return timelines


def group_results(results_list):
    groups = defaultdict(list)
    for r in results_list:
        ctrl  = r.get('controller', '?')
        fault = r.get('fault_type') or 'none'
        if fault == 'oscillate_leader':
            continue
        groups[(ctrl, fault)].append(r)
    return groups


def resample(timeline, time_axis):
    result = np.zeros(len(time_axis))
    idx = 0
    for i, t in enumerate(time_axis):
        while idx < len(timeline) - 1 and timeline[idx + 1][0] <= t:
            idx += 1
        if idx < len(timeline):
            result[i] = timeline[idx][1]
        elif len(timeline) > 0:
            result[i] = timeline[-1][1]
    return result


# ====================================================================
# Panel drawers - reused by both standalone and combined figures
# ====================================================================
T_MAX = 120.0
DT = 0.5

FAN_BANDS = [(5, 95, 0.07), (10, 90, 0.09), (25, 75, 0.13)]
FAN_LEGEND = [
    Line2D([0], [0], color='gray', lw=1.4, label='Median'),
    Line2D([0], [0], color='gray', lw=0.5, ls='--', label='Q25 / Q75'),
    mpatches.Patch(facecolor='gray', alpha=0.13, label='IQR'),
    mpatches.Patch(facecolor='gray', alpha=0.07, label='5\u201395 %'),
]


def _draw_convergence_panel(ax, ctrl, cond, time_axis, title=None):
    """Mean +/- 1sigma convergence for one controller across fault conditions."""
    color = CTRL_COLORS[ctrl]
    for fault in ['none', 'bad_leader', 'freeze_leader']:
        if (ctrl, fault) not in cond:
            continue
        m, s, _ = cond[(ctrl, fault)]
        ax.plot(time_axis, m, ls=FAULT_STYLES[fault], color=color,
                lw=1.1, marker=FAULT_MARKERS[fault],
                markevery=MARKER_EVERY, markersize=3,
                label=FAULT_LABELS[fault])
        ax.fill_between(time_axis, m - s, m + s, alpha=0.06,
                        color=color, linewidth=0)
    _fault_vlines(ax)
    ax.set_xlim(0, 120)
    ax.set_ylim(0, 105)
    ax.set_xlabel('Time (s)')
    if title:
        ax.set_title(title, fontsize=8, fontweight='bold')
    ax.legend(fontsize=5.5, loc='lower right', framealpha=0.92,
              edgecolor='#999')
    ieee_grid(ax, 'both')


def _draw_resilience_panel(ax, ctrl, cond, time_axis, title=None,
                           show_legend=True, add_fault_labels=False):
    """Median + IQR shading for one controller across fault conditions."""
    for fault in ['none', 'bad_leader', 'freeze_leader']:
        if (ctrl, fault) not in cond:
            continue
        arr = cond[(ctrl, fault)]
        med = np.median(arr, axis=0)
        q25 = np.percentile(arr, 25, axis=0)
        q75 = np.percentile(arr, 75, axis=0)
        color = RESILIENCE_FAULT_COLORS[fault]
        ax.plot(time_axis, med, ls='-', color=color,
                lw=1.5, marker=FAULT_MARKERS[fault],
                markevery=MARKER_EVERY, markersize=3.4,
                label=FAULT_LABELS[fault], zorder=3)
        ax.fill_between(time_axis, q25, q75, alpha=0.14, color=color,
                        linewidth=0)
    _fault_vlines(ax, add_labels=add_fault_labels, label_size=6.0)
    ax.set_xlim(10, 50)
    ax.set_ylim(0, 105)
    ax.set_xlabel('Time (s)')
    if title:
        ax.set_title(title, fontsize=9, fontweight='bold')
    if show_legend:
        ax.legend(fontsize=6, loc='lower right', framealpha=0.92,
                  edgecolor='#999')
    ieee_grid(ax, 'both')


def _draw_fan_panel(ax, ctrl, resampled, time_axis, title=None):
    """Per-trial fan chart for one controller under bad_leader."""
    color = CTRL_COLORS[ctrl]
    for lo, hi, alpha in FAN_BANDS:
        ax.fill_between(time_axis,
                        np.percentile(resampled, lo, axis=0),
                        np.percentile(resampled, hi, axis=0),
                        alpha=alpha, color=color, linewidth=0)
    ax.plot(time_axis, np.median(resampled, axis=0),
            color=color, linewidth=1.4, zorder=5)
    q25 = np.percentile(resampled, 25, axis=0)
    q75 = np.percentile(resampled, 75, axis=0)
    ax.plot(time_axis, q25, color=color, lw=0.5, ls='--', alpha=0.45)
    ax.plot(time_axis, q75, color=color, lw=0.5, ls='--', alpha=0.45)

    n_cat = sum(1 for tr in resampled if tr[-1] < 70)
    cat_c = '#a63d40' if n_cat else '#3a7d44'
    ax.text(0.97, 0.06, f'{n_cat}/{len(resampled)} cat.',
            transform=ax.transAxes, fontsize=6, ha='right',
            va='bottom', color=cat_c, fontweight='bold')

    _fault_vlines(ax)
    ax.set_xlim(0, 120)
    ax.set_ylim(0, 105)
    ax.set_xlabel('Time (s)')
    if title:
        ax.set_title(title, fontsize=8, fontweight='bold')
    ieee_grid(ax, 'both')


def _draw_detection_panel(ax, data_list, f_labels, colors, hatches, xlim):
    """Horizontal bar for suspicion or impeachment timing."""
    for i, (vals, fl, c, h) in enumerate(
            zip(data_list, f_labels, colors, hatches)):
        if not vals:
            continue
        med = np.median(vals)
        q25 = np.percentile(vals, 25)
        q75 = np.percentile(vals, 75)
        xerr_lo = med - q25
        xerr_hi = q75 - med
        ax.barh(i, med, height=0.50, color=c, alpha=0.80,
                hatch=h, edgecolor='black', linewidth=0.5,
                xerr=[[xerr_lo], [xerr_hi]], capsize=4,
                error_kw={'linewidth': 0.7, 'capthick': 0.7})
        ax.text(q75 + xlim * 0.03, i,
                f'{med:.2f} s', va='center', ha='left',
                fontsize=8, fontweight='bold')
    y_pos = np.arange(len(f_labels))
    ax.set_yticks(y_pos)
    ax.set_yticklabels(f_labels, fontsize=8)
    ax.set_xlabel('Time After Fault Injection (s)', fontsize=8)
    ax.set_xlim(0, xlim)
    ax.invert_yaxis()
    ieee_grid(ax, 'x')


# ====================================================================
# Figure: Coverage Bar Chart  (single-column)
# ====================================================================
def fig_coverage_bars(groups, output_dir):
    fig, ax = plt.subplots(figsize=(IEEE_SINGLE, 2.5))

    controllers = ['reip', 'raft', 'decentralized']
    faults      = ['none', 'bad_leader', 'freeze_leader']
    bar_w = 0.23
    x = np.arange(len(controllers))

    for j, fault in enumerate(faults):
        means, stds = [], []
        for ctrl in controllers:
            covs = [t['final_coverage'] for t in groups.get((ctrl, fault), [])]
            means.append(np.mean(covs) if covs else 0)
            stds.append(np.std(covs)  if covs else 0)

        offset = (j - 1) * bar_w
        ax.bar(x + offset, means, bar_w,
               yerr=stds, capsize=2,
               color=FAULT_COLORS[fault], alpha=0.82,
               hatch=FAULT_HATCHES[fault],
               edgecolor='black', linewidth=0.4,
               label=FAULT_LABELS[fault],
               error_kw={'linewidth': 0.5, 'capthick': 0.5})

    ax.set_xticks(x)
    ax.set_xticklabels([CTRL_LABELS[c] for c in controllers])
    ax.set_ylabel('Final Coverage (%)')
    ax.set_ylim(0, 105)
    ax.set_yticks([0, 20, 40, 60, 80, 100])
    ax.legend(loc='upper right', framealpha=0.9, edgecolor='#999',
              handlelength=1.4, handletextpad=0.4)
    ieee_grid(ax)

    fig.tight_layout(pad=0.3)
    save_fig(fig, output_dir, 'coverage_bar')


# ====================================================================
# Figure: Detection Timing
# ====================================================================
def fig_detection_timing(groups, output_dir):
    faults   = ['bad_leader', 'freeze_leader']
    f_labels = [FAULT_LABELS[f] for f in faults]
    colors   = ['#1f4e79', '#8fb0cc']   # navy + light steel - clean two-tone
    hatches  = ['', '']                  # no hatching for detection bars

    sus_data, imp_data = [], []
    for fault in faults:
        trials = groups.get(('reip', fault), [])
        sus = [t['time_to_first_suspicion'] for t in trials
               if t.get('time_to_first_suspicion')
               and t['time_to_first_suspicion'] > 0]
        imp = [t['time_to_detection'] for t in trials
               if t.get('time_to_detection')
               and t['time_to_detection'] > 0]
        sus_data.append(sus)
        imp_data.append(imp)

    sus_max = max((np.percentile(s, 75) for s in sus_data if s), default=0.5)
    imp_max = max((np.percentile(s, 75) for s in imp_data if s), default=2.5)
    sus_xlim = round(sus_max * 1.6 + 0.05, 1)
    imp_xlim = round(imp_max * 1.3 + 0.2, 1)

    # --- Standalone: suspicion ---
    fig, ax = plt.subplots(figsize=(IEEE_SINGLE, 1.4))
    _draw_detection_panel(ax, sus_data, f_labels, colors, hatches, sus_xlim)
    fig.tight_layout(pad=0.3)
    save_fig(fig, output_dir, 'detection_suspicion')

    # --- Standalone: impeachment ---
    fig, ax = plt.subplots(figsize=(IEEE_SINGLE, 1.4))
    _draw_detection_panel(ax, imp_data, f_labels, colors, hatches, imp_xlim)
    fig.tight_layout(pad=0.3)
    save_fig(fig, output_dir, 'detection_impeach')

    # --- Combined (backward compat) ---
    fig, (ax_sus, ax_imp) = plt.subplots(1, 2, figsize=(IEEE_DOUBLE, 1.6),
                                          gridspec_kw={'wspace': 0.45})
    _draw_detection_panel(ax_sus, sus_data, f_labels, colors, hatches, sus_xlim)
    ax_sus.set_title('(a)  First Suspicion', fontsize=9, fontweight='bold')
    _draw_detection_panel(ax_imp, imp_data, f_labels, colors, hatches, imp_xlim)
    ax_imp.set_title('(b)  Impeachment', fontsize=9, fontweight='bold')
    fig.tight_layout(pad=0.4)
    save_fig(fig, output_dir, 'detection_timing')


# ====================================================================
# Figure: Per-Trial Fan Chart
# ====================================================================
def fig_per_trial_traces(timelines, output_dir):
    time_axis = np.arange(0, T_MAX + DT, DT)
    controllers = ['reip', 'raft', 'decentralized']
    labels = ['(a)', '(b)', '(c)']

    resampled_cache = {}
    for ctrl in controllers:
        tls = timelines.get((ctrl, 'bad_leader'), [])
        if tls:
            resampled_cache[ctrl] = np.array(
                [resample(tl, time_axis) for tl in tls])

    # --- Standalone files ---
    for ctrl in controllers:
        if ctrl not in resampled_cache:
            continue
        fig, ax = plt.subplots(figsize=(IEEE_SINGLE, 2.3))
        _draw_fan_panel(ax, ctrl, resampled_cache[ctrl], time_axis)
        ax.set_ylabel('Coverage (%)')
        ax.legend(handles=FAN_LEGEND, fontsize=5.5, loc='center right',
                  framealpha=0.92, edgecolor='#999')
        fig.tight_layout(pad=0.3)
        save_fig(fig, output_dir, f'per_trial_traces_{ctrl}')

    # --- Combined (backward compat) ---
    fig, axes = plt.subplots(1, 3, figsize=(IEEE_DOUBLE, 2.3), sharey=True)
    for ai, ctrl in enumerate(controllers):
        ax = axes[ai]
        if ctrl in resampled_cache:
            _draw_fan_panel(ax, ctrl, resampled_cache[ctrl], time_axis,
                            title=f'{labels[ai]} {CTRL_LABELS[ctrl]}')
        else:
            ax.set_title(f'{labels[ai]} {CTRL_LABELS[ctrl]}',
                         fontsize=8, fontweight='bold')
        ax.legend(handles=FAN_LEGEND, fontsize=5.5, loc='center right',
                  framealpha=0.92, edgecolor='#999')
    axes[0].set_ylabel('Coverage (%)')
    fig.tight_layout(pad=0.4)
    save_fig(fig, output_dir, 'per_trial_traces')


# ====================================================================
# Figure: Convergence by Controller
# ====================================================================
def fig_convergence_by_controller(timelines, output_dir):
    time_axis = np.arange(0, T_MAX + DT, DT)
    controllers = ['reip', 'raft', 'decentralized']
    labels = ['(a)', '(b)', '(c)']

    cond = {}
    for (ctrl, fault), tls in timelines.items():
        arr = np.array([resample(tl, time_axis) for tl in tls])
        cond[(ctrl, fault)] = (np.mean(arr, 0), np.std(arr, 0), arr)

    # --- Standalone files ---
    for ctrl in controllers:
        has_data = any((ctrl, f) in cond
                       for f in ['none', 'bad_leader', 'freeze_leader'])
        if not has_data:
            continue
        fig, ax = plt.subplots(figsize=(IEEE_SINGLE, 2.2))
        _draw_convergence_panel(ax, ctrl, cond, time_axis)
        ax.set_ylabel('Coverage (%)')
        fig.tight_layout(pad=0.3)
        save_fig(fig, output_dir, f'convergence_{ctrl}')

    # --- Combined (backward compat) ---
    fig, axes = plt.subplots(1, 3, figsize=(IEEE_DOUBLE, 2.2), sharey=True)
    for ai, ctrl in enumerate(controllers):
        _draw_convergence_panel(axes[ai], ctrl, cond, time_axis,
                                title=f'{labels[ai]} {CTRL_LABELS[ctrl]}')
    axes[0].set_ylabel('Coverage (%)')
    fig.tight_layout(pad=0.4)
    save_fig(fig, output_dir, 'convergence_by_controller')


# ====================================================================
# Figure: Resilience Contrast
# ====================================================================
def fig_resilience_contrast(timelines, output_dir):
    time_axis = np.arange(0, T_MAX + DT, DT)

    cond = {}
    for (ctrl, fault), tls in timelines.items():
        arr = np.array([resample(tl, time_axis) for tl in tls])
        cond[(ctrl, fault)] = arr

    # --- Standalone files ---
    for ctrl in ['reip', 'raft']:
        has_data = any((ctrl, f) in cond
                       for f in ['none', 'bad_leader', 'freeze_leader'])
        if not has_data:
            continue
        fig, ax = plt.subplots(figsize=(IEEE_SINGLE, 2.4))
        _draw_resilience_panel(ax, ctrl, cond, time_axis, add_fault_labels=False)
        ax.set_ylabel('Coverage (%)')
        fig.tight_layout(pad=0.3)
        save_fig(fig, output_dir, f'resilience_{ctrl}')

    # --- Combined (backward compat) ---
    fig, axes = plt.subplots(1, 2, figsize=(IEEE_DOUBLE, 2.4), sharey=True)
    panel = ['(a)', '(b)']
    for ai, ctrl in enumerate(['reip', 'raft']):
        _draw_resilience_panel(axes[ai], ctrl, cond, time_axis,
                               title=f'{panel[ai]} {CTRL_LABELS[ctrl]}',
                               show_legend=True, add_fault_labels=False)
    axes[0].set_ylabel('Coverage (%)')
    axes[1].set_ylabel('Coverage (%)')
    axes[1].tick_params(axis='y', labelleft=True)
    fig.tight_layout(pad=0.4)
    save_fig(fig, output_dir, 'resilience_contrast')


# ====================================================================
# Figure: Ablation
# ====================================================================
def fig_ablation(ablation_results, n100_results, output_dir):
    reip_bl = [r for r in n100_results
               if r.get('controller') == 'reip'
               and (r.get('fault_type') or 'none') == 'bad_leader'
               and not r.get('ablation')]

    abl_groups = defaultdict(list)
    for r in ablation_results:
        abl   = r.get('ablation', 'none')
        fault = r.get('fault_type', 'none') or 'none'
        abl_groups[(abl, fault)].append(r)

    nocaus_clean = abl_groups.get(('no_causality', 'none'), [])

    variant_data = [
        ('REIP-Full',    reip_bl,
         [r for r in n100_results if r.get('controller') == 'reip'
          and (r.get('fault_type') or 'none') == 'none'
          and not r.get('ablation')]),
        ('No Trust',     abl_groups.get(('no_trust', 'bad_leader'), []), []),
        ('No Direction', abl_groups.get(('no_direction', 'bad_leader'), []), []),
        ('No Causality', abl_groups.get(('no_causality', 'bad_leader'), []),
         nocaus_clean),
    ]

    colors = [CLR_REIP, CLR_BAD, '#c4973b', CLR_FREEZE]
    x = np.arange(len(variant_data))

    # ---------- helper: one ablation bar panel ----------
    def _abl_coverage(ax):
        means = [np.mean([t['final_coverage'] for t in v[1]])
                 if v[1] else 0 for v in variant_data]
        stds  = [np.std([t['final_coverage']  for t in v[1]])
                 if v[1] else 0 for v in variant_data]
        bars = ax.bar(x, means, yerr=stds, capsize=2.5, color=colors,
                      edgecolor='black', linewidth=0.4, alpha=0.82,
                      error_kw={'linewidth': 0.5, 'capthick': 0.5})
        for bar, m in zip(bars, means):
            ax.text(bar.get_x() + bar.get_width() / 2,
                    bar.get_height() + 2,
                    f'{m:.1f}%', ha='center', fontsize=6, fontweight='bold')
        ax.set_xticks(x)
        ax.set_xticklabels([v[0] for v in variant_data], fontsize=7)
        ax.set_ylabel('Coverage (%)')
        ax.set_ylim(0, 110)
        ieee_grid(ax)

    def _abl_success(ax):
        rates = []
        for _, trials, _ in variant_data:
            if trials:
                rates.append(
                    sum(1 for t in trials if t['final_coverage'] >= 80)
                    / len(trials) * 100)
            else:
                rates.append(0)
        bars = ax.bar(x, rates, color=colors, edgecolor='black',
                      linewidth=0.4, alpha=0.82)
        for bar, r in zip(bars, rates):
            ax.text(bar.get_x() + bar.get_width() / 2,
                    bar.get_height() + 2,
                    f'{r:.0f}%', ha='center', fontsize=6, fontweight='bold')
        ax.set_xticks(x)
        ax.set_xticklabels([v[0] for v in variant_data], fontsize=7)
        ax.set_ylabel('Success Rate (%)')
        ax.set_ylim(0, 110)
        ieee_grid(ax)

    def _abl_fp(ax):
        fp_means, fp_labels_list, fp_colors = [], [], []
        for i, (label, _, clean_trials) in enumerate(variant_data):
            if clean_trials:
                fp_means.append(np.mean(
                    [t.get('false_positives', 0) for t in clean_trials]))
                fp_labels_list.append(label)
                fp_colors.append(colors[i])
        if fp_means:
            fp_x = np.arange(len(fp_means))
            bars = ax.bar(fp_x, fp_means, width=0.45, color=fp_colors,
                          edgecolor='black', linewidth=0.4, alpha=0.82)
            for bar, fpm in zip(bars, fp_means):
                ax.text(bar.get_x() + bar.get_width() / 2,
                        bar.get_height() + 0.15,
                        f'{fpm:.1f}', ha='center', fontsize=6,
                        fontweight='bold')
            ax.set_xticks(fp_x)
            ax.set_xticklabels(fp_labels_list, fontsize=7)
        ax.set_ylabel('False Positives')
        ieee_grid(ax)

    # --- Standalone files ---
    fig, ax = plt.subplots(figsize=(IEEE_SINGLE, 2.2))
    _abl_coverage(ax)
    fig.tight_layout(pad=0.3)
    save_fig(fig, output_dir, 'ablation_coverage')

    fig, ax = plt.subplots(figsize=(IEEE_SINGLE, 2.2))
    _abl_success(ax)
    fig.tight_layout(pad=0.3)
    save_fig(fig, output_dir, 'ablation_success')

    fig, ax = plt.subplots(figsize=(IEEE_SINGLE, 2.2))
    _abl_fp(ax)
    fig.tight_layout(pad=0.3)
    save_fig(fig, output_dir, 'ablation_false_positives')

    # --- Combined (backward compat) ---
    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(IEEE_DOUBLE, 2.2))
    _abl_coverage(ax1)
    ax1.set_title('(a) Coverage', fontsize=8, fontweight='bold')
    _abl_success(ax2)
    ax2.set_title('(b) Success (\u226580 %)', fontsize=8, fontweight='bold')
    _abl_fp(ax3)
    ax3.set_title('(c) FP (clean)', fontsize=8, fontweight='bold')
    fig.tight_layout(pad=0.4)
    save_fig(fig, output_dir, 'ablation_figure')


# ====================================================================
# Main
# ====================================================================
def parse_args():
    parser = argparse.ArgumentParser(
        description="Generate IEEE paper figures from experiment outputs.")
    parser.add_argument("--main-dir",     default=DEFAULT_MAIN_DIR)
    parser.add_argument("--freeze-dir",   default=DEFAULT_FREEZE_DIR)
    parser.add_argument("--ablation-dir", default=DEFAULT_ABLATION_DIR)
    parser.add_argument("--output-dir",   default=DEFAULT_OUTPUT_DIR)
    return parser.parse_args()


def main():
    args = parse_args()
    output_dir = args.output_dir
    os.makedirs(output_dir, exist_ok=True)
    main_dir     = os.path.normpath(args.main_dir)
    freeze_dir   = os.path.normpath(args.freeze_dir)
    ablation_dir = os.path.normpath(args.ablation_dir)

    print("Loading data \u2026")
    print(f"  main:     {main_dir}")
    print(f"  freeze:   {freeze_dir}")
    print(f"  ablation: {ablation_dir}")
    print(f"  output:   {output_dir}")

    wallfixed = load_results(main_dir)
    freeze = [] if freeze_dir == main_dir else load_results(freeze_dir)
    ablation = load_results(ablation_dir)

    combined = [r for r in wallfixed
                if (r.get('fault_type') or 'none') != 'oscillate_leader']
    combined.extend(freeze)

    groups = group_results(combined)
    print(f"  {len(combined)} trials total")
    for k in sorted(groups.keys()):
        print(f"    {k[0]:>15} | {k[1]:<15} : {len(groups[k])}")

    timeline_dirs = [main_dir]
    if freeze_dir != main_dir:
        timeline_dirs.append(freeze_dir)
    timelines = load_timelines(timeline_dirs)
    print(f"  {sum(len(v) for v in timelines.values())} timelines")

    print("\nGenerating IEEE figures \u2026")
    fig_coverage_bars(groups, output_dir)
    fig_detection_timing(groups, output_dir)
    fig_per_trial_traces(timelines, output_dir)
    fig_convergence_by_controller(timelines, output_dir)
    fig_resilience_contrast(timelines, output_dir)
    fig_ablation(ablation, wallfixed, output_dir)

    print(f"\nAll figures saved to {output_dir}/")


if __name__ == '__main__':
    main()
