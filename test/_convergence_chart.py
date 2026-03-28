#!/usr/bin/env python3
"""
IEEE-style Convergence Chart: Coverage vs Time
Generates publication-quality figures with mean +/- std shading.

IEEE formatting rules applied:
  - No embedded figure titles (use LaTeX \\caption{} instead)
  - Serif font (Times), >=8pt at column width
  - 300 DPI minimum
  - Distinct markers for B&W readability
  - Minimal annotations; descriptive content belongs in the caption
"""

import json, os, sys, glob, math
import numpy as np

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from collections import defaultdict

# ---- IEEE styling (matches IEEEtran column width & font conventions) ----
plt.rcParams.update({
    'font.family': 'serif',
    'font.serif': ['Times New Roman', 'Times', 'DejaVu Serif'],
    'font.size': 9,
    'axes.labelsize': 10,
    'axes.titlesize': 10,
    'legend.fontsize': 7.5,
    'xtick.labelsize': 8,
    'ytick.labelsize': 8,
    'figure.dpi': 300,
    'savefig.dpi': 300,
    'savefig.format': 'pdf',
    'axes.linewidth': 0.6,
    'grid.linewidth': 0.3,
    'grid.alpha': 0.25,
    'lines.linewidth': 1.2,
    'lines.markersize': 4,
    'patch.linewidth': 0.5,
    'xtick.major.width': 0.5,
    'ytick.major.width': 0.5,
    'xtick.direction': 'in',
    'ytick.direction': 'in',
    'xtick.major.pad': 3,
    'ytick.major.pad': 3,
    'text.usetex': False,
    'mathtext.fontset': 'stix',
})

# ---- Load data ----
# Accept multiple base directories: first is primary (output goes here),
# rest are merged in (e.g. freeze_leader from a separate run).
bases = sys.argv[1:] if len(sys.argv) > 1 else ['experiments/run_20260226_192129_multiroom_10trials_all']
base = bases[0]  # primary directory for output

timelines = defaultdict(list)  # (controller, fault_type) -> [timeline, ...]

for b in bases:
    logs_dir = os.path.join(b, 'logs')
    if not os.path.isdir(logs_dir):
        print(f"WARNING: {logs_dir} not found, skipping")
        continue
    for exp_dir in sorted(os.listdir(logs_dir)):
        tl_file = os.path.join(logs_dir, exp_dir, 'coverage_timeline.json')
        if not os.path.exists(tl_file):
            continue
        with open(tl_file) as f:
            data = json.load(f)
        ctrl = data['controller']
        fault = data.get('fault_type') or 'none'
        # Skip oscillate_leader (replaced by freeze_leader)
        if fault == 'oscillate_leader':
            continue
        timelines[(ctrl, fault)].append(data['timeline'])
    print(f"Loaded from {b}")

print(f"Total: {sum(len(v) for v in timelines.values())} timelines")
for k, v in sorted(timelines.items()):
    print(f"  {k[0]:>15} | {k[1]:<12} : {len(v)} trials")

# ---- Resample timelines to uniform time axis ----
T_MAX = 120.0
DT = 0.5  # 0.5s resolution
time_axis = np.arange(0, T_MAX + DT, DT)

def resample(timeline, time_axis):
    """Resample a (t, cov) timeline to uniform time axis."""
    result = np.zeros(len(time_axis))
    idx = 0
    for i, t_target in enumerate(time_axis):
        while idx < len(timeline) - 1 and timeline[idx + 1][0] <= t_target:
            idx += 1
        if idx < len(timeline):
            result[i] = timeline[idx][1]
        elif len(timeline) > 0:
            result[i] = timeline[-1][1]
    return result

# ---- Compute per-condition mean/std ----
condition_data = {}
for (ctrl, fault), tls in timelines.items():
    resampled = np.array([resample(tl, time_axis) for tl in tls])
    m = np.mean(resampled, axis=0)
    s = np.std(resampled, axis=0)
    condition_data[(ctrl, fault)] = (m, s, resampled)

# ---- Color & style definitions ----
# Colors chosen for both screen and B&W print distinguishability
CTRL_COLORS = {
    'reip': '#1a5276',          # dark blue
    'raft': '#c0392b',          # dark red
    'decentralized': '#27ae60', # green
}
# Markers enable B&W readability (placed every N points via markevery)
CTRL_MARKERS = {
    'reip': 'o',            # circle
    'raft': 's',            # square
    'decentralized': '^',   # triangle
}
MARKER_EVERY = 20  # place a marker every N data points on the time axis
FAULT_STYLES = {
    'none': '-',              # solid
    'bad_leader': '--',       # dashed
    'freeze_leader': '-.',    # dash-dot
    'spin': ':',              # dotted (legacy)
    'oscillate_leader': ':',  # dotted
}
FAULT_LABELS = {
    'none': 'Clean',
    'bad_leader': 'Bad Leader',
    'freeze_leader': 'Freeze Leader',
    'spin': 'Spin Fault',
    'oscillate_leader': 'Oscillate Leader',
}
CTRL_LABELS = {
    'reip': 'REIP (Proposed)',
    'raft': 'Raft (Baseline)',
    'decentralized': 'Decentralized',
}

# ============================================================
# FIGURE 1: All 9 conditions on one plot
# ============================================================
fig, ax = plt.subplots(figsize=(3.5, 2.6))  # IEEE single-column width

for (ctrl, fault), (m, s, _) in sorted(condition_data.items()):
    color = CTRL_COLORS[ctrl]
    style = FAULT_STYLES[fault]
    marker = CTRL_MARKERS[ctrl]
    alpha_fill = 0.08 if fault != 'none' else 0.12

    ax.plot(time_axis, m, linestyle=style, color=color, marker=marker,
            markevery=MARKER_EVERY, markersize=3, zorder=3)
    ax.fill_between(time_axis, m - s, m + s, alpha=alpha_fill,
                    color=color, linewidth=0)

# Fault injection lines (no text - described in caption)
for ft in [10, 30]:
    ax.axvline(x=ft, color='#888888', linewidth=0.6, linestyle='-.',
               alpha=0.55, zorder=1)

ax.set_xlabel('Time (s)')
ax.set_ylabel('Coverage (%)')
ax.set_xlim(0, 120)
ax.set_ylim(0, 105)
ax.set_yticks([0, 20, 40, 60, 80, 100])
ax.grid(True, alpha=0.25, linewidth=0.3)

# Build legend - controllers (color+marker) and faults (linestyle)
legend_elements = []
for ctrl in ['reip', 'raft', 'decentralized']:
    legend_elements.append(Line2D([0], [0], color=CTRL_COLORS[ctrl], linewidth=1.5,
                                  marker=CTRL_MARKERS[ctrl], markersize=4,
                                  label=CTRL_LABELS[ctrl]))
legend_elements.append(Line2D([0], [0], color='gray', linestyle='-', linewidth=0.8, label='Clean'))
legend_elements.append(Line2D([0], [0], color='gray', linestyle='--', linewidth=0.8, label='Bad Leader'))
legend_elements.append(Line2D([0], [0], color='gray', linestyle='-.', linewidth=0.8, label='Freeze Leader'))

ax.legend(handles=legend_elements, loc='lower right', framealpha=0.9,
          edgecolor='#cccccc', ncol=2, columnspacing=1.0)

fig.tight_layout(pad=0.3)
for ext in ['png', 'pdf']:
    out = os.path.join(base, f'convergence_all.{ext}')
    fig.savefig(out, bbox_inches='tight')
    print(f"Saved: {out}")
plt.close(fig)

# ============================================================
# FIGURE 2: Bad Leader only - clean baselines faded, fault bold
# ============================================================
fig, ax = plt.subplots(figsize=(3.5, 2.6))

for ctrl in ['reip', 'raft', 'decentralized']:
    marker = CTRL_MARKERS[ctrl]
    # Clean baseline (thin, low alpha)
    if (ctrl, 'none') in condition_data:
        m_clean, _, _ = condition_data[(ctrl, 'none')]
        ax.plot(time_axis, m_clean, linestyle='-', color=CTRL_COLORS[ctrl],
                alpha=0.25, linewidth=0.7, zorder=2)

    # Bad leader (bold, with markers)
    if (ctrl, 'bad_leader') in condition_data:
        m, s, _ = condition_data[(ctrl, 'bad_leader')]
        ax.plot(time_axis, m, linestyle='-', color=CTRL_COLORS[ctrl],
                marker=marker, markevery=MARKER_EVERY, markersize=3.5,
                linewidth=1.5, zorder=3)
        ax.fill_between(time_axis, m - s, m + s, alpha=0.12,
                        color=CTRL_COLORS[ctrl], linewidth=0)

# Fault injection lines - clean, no text
for ft in [10, 30]:
    ax.axvline(x=ft, color='#888888', linewidth=0.6, linestyle='-.',
               alpha=0.55, zorder=1)

ax.set_xlabel('Time (s)')
ax.set_ylabel('Coverage (%)')
ax.set_xlim(0, 120)
ax.set_ylim(0, 105)
ax.set_yticks([0, 20, 40, 60, 80, 100])
ax.grid(True, alpha=0.25, linewidth=0.3)

legend_elements = [
    Line2D([0], [0], color=CTRL_COLORS['reip'], linewidth=1.5,
           marker=CTRL_MARKERS['reip'], markersize=4, label='REIP'),
    Line2D([0], [0], color=CTRL_COLORS['raft'], linewidth=1.5,
           marker=CTRL_MARKERS['raft'], markersize=4, label='Raft'),
    Line2D([0], [0], color=CTRL_COLORS['decentralized'], linewidth=1.5,
           marker=CTRL_MARKERS['decentralized'], markersize=4, label='Decentralized'),
    Line2D([0], [0], color='gray', linewidth=0.7, alpha=0.4, label='Clean (faded)'),
]
ax.legend(handles=legend_elements, loc='lower right', framealpha=0.9,
          edgecolor='#cccccc', fontsize=7)

fig.tight_layout(pad=0.3)
for ext in ['png', 'pdf']:
    out = os.path.join(base, f'convergence_badleader.{ext}')
    fig.savefig(out, bbox_inches='tight')
    print(f"Saved: {out}")
plt.close(fig)

# ============================================================
# FIGURE 3: Per-trial fan chart (percentile bands - replaces spaghetti)
# ============================================================
fig, axes = plt.subplots(1, 3, figsize=(7.16, 2.6), sharey=True)  # IEEE double-column

PANEL_LABELS = ['(a)', '(b)', '(c)']
FAN_BANDS = [
    (5,  95, 0.08),   # lightest outer band
    (10, 90, 0.10),
    (25, 75, 0.14),   # IQR
]

for ax_idx, ctrl in enumerate(['reip', 'raft', 'decentralized']):
    ax = axes[ax_idx]
    color = CTRL_COLORS[ctrl]

    if (ctrl, 'bad_leader') in condition_data:
        _, _, resampled = condition_data[(ctrl, 'bad_leader')]

        # Fan chart: percentile bands (outer -> inner)
        for lo, hi, alpha in FAN_BANDS:
            q_lo = np.percentile(resampled, lo, axis=0)
            q_hi = np.percentile(resampled, hi, axis=0)
            ax.fill_between(time_axis, q_lo, q_hi,
                            alpha=alpha, color=color, linewidth=0)

        # Median line (bold)
        med = np.median(resampled, axis=0)
        ax.plot(time_axis, med, color=color, linewidth=1.8,
                zorder=5, label='Median')

        # 25th / 75th dashed for clarity
        q25 = np.percentile(resampled, 25, axis=0)
        q75 = np.percentile(resampled, 75, axis=0)
        ax.plot(time_axis, q25, color=color, linewidth=0.6,
                linestyle='--', alpha=0.5, zorder=4)
        ax.plot(time_axis, q75, color=color, linewidth=0.6,
                linestyle='--', alpha=0.5, zorder=4)

    for ft in [10, 30]:
        ax.axvline(x=ft, color='#888888', linewidth=0.5, linestyle='-.',
                   alpha=0.5)
    ax.set_xlim(0, 120)
    ax.set_ylim(0, 105)
    ax.set_xlabel('Time (s)')
    ax.set_title(f'{PANEL_LABELS[ax_idx]} {CTRL_LABELS[ctrl]}',
                 fontsize=9, fontweight='bold')
    ax.grid(True, alpha=0.2, linewidth=0.3)

    # Catastrophic count annotation
    if (ctrl, 'bad_leader') in condition_data:
        _, _, resampled = condition_data[(ctrl, 'bad_leader')]
        n_cat = sum(1 for trace in resampled if trace[-1] < 70)
        n_total = len(resampled)
        if n_cat > 0:
            ax.text(0.97, 0.06, f'{n_cat}/{n_total} cat.',
                    transform=ax.transAxes, fontsize=6.5,
                    ha='right', va='bottom', color='#c0392b',
                    fontweight='bold')
        else:
            ax.text(0.97, 0.06, f'0/{n_total} cat.',
                    transform=ax.transAxes, fontsize=6.5,
                    ha='right', va='bottom', color='#27ae60',
                    fontweight='bold')

axes[0].set_ylabel('Coverage (%)')

# Shared legend for band meaning
from matplotlib.patches import Patch
band_legend = [
    Line2D([0], [0], color='gray', linewidth=1.6, label='Median'),
    Line2D([0], [0], color='gray', linewidth=0.6, linestyle='--', label='Q25 / Q75'),
    Patch(facecolor='gray', alpha=0.14, label='IQR (25-75%)'),
    Patch(facecolor='gray', alpha=0.08, label='5-95%'),
]
axes[0].legend(handles=band_legend, fontsize=6, loc='center right',
               framealpha=0.9, edgecolor='#cccccc')

fig.tight_layout(pad=0.5)

for ext in ['png', 'pdf']:
    out = os.path.join(base, f'convergence_traces.{ext}')
    fig.savefig(out, bbox_inches='tight')
    print(f"Saved: {out}")
plt.close(fig)

# ============================================================
# FIGURE 4: Clean vs Fault side-by-side for each controller
# ============================================================
fig, axes = plt.subplots(1, 3, figsize=(7.16, 2.4), sharey=True)

FAULT_MARKERS = {'none': 'o', 'bad_leader': 'x', 'freeze_leader': 'D', 'spin': 'D', 'oscillate_leader': 'D'}
for ax_idx, ctrl in enumerate(['reip', 'raft', 'decentralized']):
    ax = axes[ax_idx]
    color = CTRL_COLORS[ctrl]

    for fault, style, label_suffix in [('none', '-', 'Clean'),
                                        ('bad_leader', '--', 'Bad Leader'),
                                        ('freeze_leader', '-.', 'Freeze Leader'),
                                        ('oscillate_leader', ':', 'Oscillate'),
                                        ('spin', ':', 'Spin')]:
        if (ctrl, fault) in condition_data:
            m, s, _ = condition_data[(ctrl, fault)]
            ax.plot(time_axis, m, linestyle=style, color=color, linewidth=1.3,
                    marker=FAULT_MARKERS[fault], markevery=MARKER_EVERY,
                    markersize=3, label=label_suffix)
            ax.fill_between(time_axis, m - s, m + s, alpha=0.08,
                            color=color, linewidth=0)

    for ft in [10, 30]:
        ax.axvline(x=ft, color='#888888', linewidth=0.5, linestyle='-.',
                   alpha=0.45)
    ax.set_xlim(0, 120)
    ax.set_ylim(0, 105)
    ax.set_xlabel('Time (s)')
    ax.set_title(f'{PANEL_LABELS[ax_idx]} {CTRL_LABELS[ctrl]}',
                 fontsize=9, fontweight='bold')
    ax.grid(True, alpha=0.2, linewidth=0.3)
    ax.legend(fontsize=6.5, loc='lower right', framealpha=0.9,
              edgecolor='#cccccc')

axes[0].set_ylabel('Coverage (%)')
fig.tight_layout(pad=0.5)

for ext in ['png', 'pdf']:
    out = os.path.join(base, f'convergence_by_controller.{ext}')
    fig.savefig(out, bbox_inches='tight')
    print(f"Saved: {out}")
plt.close(fig)

# ============================================================
# FIGURE 5: RESILIENCE CONTRAST - the key result figure
#   Panel (a): REIP - all 3 conditions nearly overlap = resilient
#   Panel (b): RAFT - conditions diverge = not resilient
#   Uses median + IQR instead of mean + std for robustness
# ============================================================
fig, axes = plt.subplots(1, 2, figsize=(10, 4), sharey=True)

FAULT_COLORS = {
    'none': '#1a1a1a',       # darker gray/black for better poster visibility
    'bad_leader': '#e74c3c', # red
    'freeze_leader': '#3498db',  # blue (changed from purple for better contrast)
    'spin': '#f39c12',       # orange (legacy)
    'oscillate_leader': '#f39c12',  # orange
}
FAULT_MARKERS_RC = {
    'none': 'o',
    'bad_leader': 'x',
    'freeze_leader': 'D',
    'spin': 'D',
    'oscillate_leader': 'D',
}

RC_PANEL_LABELS = ['(a)', '(b)']
shared_handles = None
shared_labels = None
for ax_idx, ctrl in enumerate(['reip', 'raft']):
    ax = axes[ax_idx]

    for fault in ['none', 'bad_leader', 'freeze_leader']:
        if (ctrl, fault) not in condition_data:
            continue
        _, _, resampled = condition_data[(ctrl, fault)]
        med = np.median(resampled, axis=0)
        color = FAULT_COLORS[fault]
        fmarker = FAULT_MARKERS_RC[fault]

        line, = ax.plot(time_axis, med, linestyle='-', color=color, linewidth=3.0,
                marker=fmarker, markevery=MARKER_EVERY, markersize=7,
                label=FAULT_LABELS[fault], zorder=3, markeredgewidth=2.0)
        # IQR shading (25th-75th percentile)
        q25 = np.percentile(resampled, 25, axis=0)
        q75 = np.percentile(resampled, 75, axis=0)
        ax.fill_between(time_axis, q25, q75, alpha=0.18, color=color,
                        linewidth=0)
        if ax_idx == 0:
            if shared_handles is None:
                shared_handles = []
                shared_labels = []
            shared_handles.append(line)
            shared_labels.append(FAULT_LABELS[fault])

    # Fault injection lines only; labels are added later in poster layout if needed.
    fault_times = [10, 30]
    for ft in fault_times:
        ax.axvline(x=ft, color='#666666', linewidth=1.8, linestyle='-.',
                   alpha=0.7, zorder=2)

    ax.set_xlim(10, 50)
    ax.set_ylim(0, 110)  # More room at top for labels
    ax.set_xlabel('Time (s)', fontsize=13, fontweight='bold')
    if ax_idx == 0:
        ax.set_ylabel('Coverage (%)', fontsize=13, fontweight='bold')
    else:
        ax.set_ylabel('')
        ax.tick_params(axis='y', labelleft=False)
    ax.set_title(f'{RC_PANEL_LABELS[ax_idx]} {CTRL_LABELS[ctrl]}',
                 fontsize=14, fontweight='bold', pad=15)
    ax.grid(True, alpha=0.35, linewidth=0.6)

axes[0].set_ylabel('Coverage (%)', fontsize=13, fontweight='bold')
fig.legend(shared_handles, shared_labels, fontsize=10, loc='lower center',
           ncol=3, framealpha=0.95, edgecolor='#999999', frameon=True,
           borderpad=0.8, columnspacing=1.2, handlelength=2.0,
           bbox_to_anchor=(0.5, -0.01))
# Increase spacing between subplots and add more padding
fig.subplots_adjust(wspace=0.16, left=0.1, right=0.98, top=0.90, bottom=0.20)
fig.tight_layout(rect=[0, 0.10, 1, 1], pad=1.2)

for ext in ['png', 'pdf']:
    out = os.path.join(base, f'resilience_contrast.{ext}')
    fig.savefig(out, bbox_inches='tight')
    print(f"Saved: {out}")
plt.close(fig)

# ============================================================
# FIGURE 6: DERIVATIVE (Rate of Change) - Coverage Velocity
#   Shows how fast coverage is increasing at each time point
# ============================================================
fig, axes = plt.subplots(1, 2, figsize=(10, 4), sharey=True)

for ax_idx, ctrl in enumerate(['reip', 'raft']):
    ax = axes[ax_idx]

    for fault in ['none', 'bad_leader', 'freeze_leader']:
        if (ctrl, fault) not in condition_data:
            continue
        _, _, resampled = condition_data[(ctrl, fault)]
        med = np.median(resampled, axis=0)
        color = FAULT_COLORS[fault]
        style = FAULT_STYLES[fault]
        fmarker = FAULT_MARKERS_RC[fault]

        # Compute derivative (rate of change) using numpy gradient
        # This gives %/s (percentage points per second)
        derivative = np.gradient(med, DT)  # DT is 0.5 seconds
        
        # Smoothing options (uncomment one):
        # Option 1: Moving average (simple, no dependencies)
        window = 5
        derivative = np.convolve(derivative, np.ones(window)/window, mode='same')
        
        # Option 2: Exponential moving average (more weight on recent values)
        # alpha = 0.3  # smoothing factor (0-1, lower = more smoothing)
        # smoothed = np.zeros_like(derivative)
        # smoothed[0] = derivative[0]
        # for i in range(1, len(derivative)):
        #     smoothed[i] = alpha * derivative[i] + (1 - alpha) * smoothed[i-1]
        # derivative = smoothed
        
        # Option 3: No smoothing (raw derivative)
        # (keep derivative as-is)
        
        ax.plot(time_axis, derivative, linestyle='-', color=color, linewidth=3.0,
                marker=fmarker, markevery=MARKER_EVERY, markersize=7,
                label=FAULT_LABELS[fault], zorder=3, markeredgewidth=2.0)
        

    # Fault injection lines with labels
    fault_times = [10, 30]
    fault_labels = ['Fault 1 injected', 'Fault 2 injected']
    for i, ft in enumerate(fault_times):
        ax.axvline(x=ft, color='#666666', linewidth=1.8, linestyle='-.',
                   alpha=0.7, zorder=2)
        ax.text(ft, ax.get_ylim()[1] * 0.95, fault_labels[i], ha='center', va='top',
                fontsize=10, fontweight='bold', color='#333333',
                bbox=dict(boxstyle='round,pad=0.5', facecolor='white',
                         edgecolor='#999999', linewidth=1.0, alpha=0.95),
                zorder=4)

    ax.set_xlim(0, 70)
    ax.set_xlabel('Time (s)', fontsize=13, fontweight='bold')
    ax.set_ylabel('Coverage Rate (%/s)', fontsize=13, fontweight='bold')
    ax.set_title(f'{RC_PANEL_LABELS[ax_idx]} {CTRL_LABELS[ctrl]}',
                 fontsize=14, fontweight='bold', pad=15)
    ax.grid(True, alpha=0.35, linewidth=0.6)
    ax.legend(fontsize=10, loc='upper right', framealpha=0.95,
              edgecolor='#999999', frameon=True, borderpad=1.2, 
              columnspacing=1.0, handlelength=2.0)
    ax.axhline(y=0, color='black', linewidth=0.8, linestyle='-', alpha=0.3, zorder=1)

axes[0].set_ylabel('Coverage Rate (%/s)', fontsize=13, fontweight='bold')
fig.subplots_adjust(wspace=0.4, left=0.1, right=0.95, top=0.92, bottom=0.12)
fig.tight_layout(pad=2.0)

for ext in ['png', 'pdf']:
    out = os.path.join(base, f'resilience_contrast_derivative.{ext}')
    fig.savefig(out, bbox_inches='tight')
    print(f"Saved: {out}")
plt.close(fig)

# ============================================================
# FIGURE 7: NORMALIZED COVERAGE RATE - Exploration Intensity
#   Shows coverage rate normalized by remaining coverage
#   This reveals exploration "intensity" relative to unexplored area
# ============================================================
fig, axes = plt.subplots(1, 2, figsize=(10, 4), sharey=True)

for ax_idx, ctrl in enumerate(['reip', 'raft']):
    ax = axes[ax_idx]

    for fault in ['none', 'bad_leader', 'freeze_leader']:
        if (ctrl, fault) not in condition_data:
            continue
        _, _, resampled = condition_data[(ctrl, fault)]
        med = np.median(resampled, axis=0)
        color = FAULT_COLORS[fault]
        fmarker = FAULT_MARKERS_RC[fault]

        # Compute derivative (rate of change)
        derivative = np.gradient(med, DT)
        
        # Normalize by remaining coverage: rate / sqrt(100 - current_coverage)
        # Square root scaling reduces extreme spikes near completion while preserving comparison
        # This shows exploration intensity relative to unexplored area
        remaining = 100.0 - med
        # Avoid division by zero (when coverage = 100%)
        remaining = np.maximum(remaining, 0.1)  # minimum 0.1% to avoid division issues
        normalized_rate = derivative / np.sqrt(remaining)
        
        # Smoothing
        window = 5
        normalized_rate = np.convolve(normalized_rate, np.ones(window)/window, mode='same')
        
        ax.plot(time_axis, normalized_rate, linestyle='-', color=color, linewidth=3.0,
                marker=fmarker, markevery=MARKER_EVERY, markersize=7,
                label=FAULT_LABELS[fault], zorder=3, markeredgewidth=2.0)

    # Fault injection lines with labels
    fault_times = [10, 30]
    fault_labels = ['Fault 1 injected', 'Fault 2 injected']
    for i, ft in enumerate(fault_times):
        ax.axvline(x=ft, color='#666666', linewidth=1.8, linestyle='-.',
                   alpha=0.7, zorder=2)
        ax.text(ft, ax.get_ylim()[1] * 0.95, fault_labels[i], ha='center', va='top',
                fontsize=10, fontweight='bold', color='#333333',
                bbox=dict(boxstyle='round,pad=0.5', facecolor='white',
                         edgecolor='#999999', linewidth=1.0, alpha=0.95),
                zorder=4)

    ax.set_xlim(0, 70)
    ax.set_xlabel('Time (s)', fontsize=13, fontweight='bold')
    ax.set_ylabel('Normalized Rate (1/s)', fontsize=13, fontweight='bold')
    ax.set_title(f'{RC_PANEL_LABELS[ax_idx]} {CTRL_LABELS[ctrl]}',
                 fontsize=14, fontweight='bold', pad=15)
    ax.grid(True, alpha=0.35, linewidth=0.6)
    ax.legend(fontsize=10, loc='upper right', framealpha=0.95,
              edgecolor='#999999', frameon=True, borderpad=1.2, 
              columnspacing=1.0, handlelength=2.0)
    ax.axhline(y=0, color='black', linewidth=0.8, linestyle='-', alpha=0.3, zorder=1)

axes[0].set_ylabel('Normalized Rate (1/s)', fontsize=13, fontweight='bold')
fig.subplots_adjust(wspace=0.4, left=0.1, right=0.95, top=0.92, bottom=0.12)
fig.tight_layout(pad=2.0)

for ext in ['png', 'pdf']:
    out = os.path.join(base, f'resilience_contrast_normalized_rate.{ext}')
    fig.savefig(out, bbox_inches='tight')
    print(f"Saved: {out}")
plt.close(fig)

# ============================================================
# FIGURE 8: RESILIENCE METRICS - Quantitative Comparison
#   Three panels: Degradation Score, Efficiency by Range, Slope Analysis
# ============================================================
fig = plt.figure(figsize=(14, 4.5))
gs = fig.add_gridspec(1, 3, hspace=0.3, wspace=0.4)

# Panel 1: Resilience Degradation Score
ax1 = fig.add_subplot(gs[0, 0])
degradation_data = []
labels = []
colors_bar = []

for ctrl in ['reip', 'raft']:
    if (ctrl, 'none') not in condition_data:
        continue
    _, _, clean_resampled = condition_data[(ctrl, 'none')]
    clean_med = np.median(clean_resampled, axis=0)
    # AUC using trapezoidal rule manually
    clean_auc = np.sum((clean_med[1:] + clean_med[:-1]) / 2 * DT)
    
    for fault in ['bad_leader', 'freeze_leader']:
        if (ctrl, fault) not in condition_data:
            continue
        _, _, fault_resampled = condition_data[(ctrl, fault)]
        fault_med = np.median(fault_resampled, axis=0)
        fault_auc = np.sum((fault_med[1:] + fault_med[:-1]) / 2 * DT)
        
        # Degradation = (fault - clean) / clean * 100%
        degradation = ((fault_auc - clean_auc) / clean_auc) * 100
        
        degradation_data.append(degradation)
        labels.append(f'{CTRL_LABELS[ctrl][:4]}\n{FAULT_LABELS[fault]}')
        colors_bar.append(FAULT_COLORS[fault] if ctrl == 'reip' else '#999999')

bars = ax1.bar(range(len(degradation_data)), degradation_data, color=colors_bar, 
               edgecolor='black', linewidth=1.5, alpha=0.8)
ax1.axhline(y=0, color='black', linewidth=1, linestyle='-', alpha=0.5)
ax1.set_xticks(range(len(labels)))
ax1.set_xticklabels(labels, fontsize=9, rotation=0, ha='center')
ax1.set_ylabel('Degradation (%)', fontsize=12, fontweight='bold')
ax1.set_title('(a) Resilience Degradation Score', fontsize=13, fontweight='bold', pad=10)
ax1.grid(True, alpha=0.3, axis='y')
for i, (bar, val) in enumerate(zip(bars, degradation_data)):
    ax1.text(bar.get_x() + bar.get_width()/2, val + (2 if val >= 0 else -5),
             f'{val:.1f}%', ha='center', va='bottom' if val >= 0 else 'top',
             fontsize=9, fontweight='bold')

# Panel 2: Exploration Efficiency by Coverage Range
ax2 = fig.add_subplot(gs[0, 1])
efficiency_data = {'REIP': [], 'Raft': []}
fault_types = ['Clean', 'Bad Leader', 'Freeze Leader']
fault_keys = ['none', 'bad_leader', 'freeze_leader']
coverage_ranges = ['0-50%', '50-80%', '80-100%']

for ctrl in ['reip', 'raft']:
    ctrl_name = 'REIP' if ctrl == 'reip' else 'Raft'
    for fault_key, fault_label in zip(fault_keys, fault_types):
        if (ctrl, fault_key) not in condition_data:
            continue
        _, _, resampled = condition_data[(ctrl, fault_key)]
        med = np.median(resampled, axis=0)
        
        # Find time indices for coverage ranges
        idx_50 = np.where(med >= 50)[0]
        idx_80 = np.where(med >= 80)[0]
        idx_100 = np.where(med >= 100)[0]
        
        t_start = 0
        t_50 = time_axis[idx_50[0]] if len(idx_50) > 0 else 70
        t_80 = time_axis[idx_80[0]] if len(idx_80) > 0 else 70
        t_100 = time_axis[idx_100[0]] if len(idx_100) > 0 else 70
        
        # Average rate in each range
        ranges = [
            (t_start, min(t_50, 70)),
            (min(t_50, 70), min(t_80, 70)),
            (min(t_80, 70), min(t_100, 70))
        ]
        
        rates = []
        for t0, t1 in ranges:
            if t1 > t0:
                mask = (time_axis >= t0) & (time_axis <= t1)
                if np.any(mask):
                    coverage_in_range = med[mask]
                    if len(coverage_in_range) > 1:
                        rate = (coverage_in_range[-1] - coverage_in_range[0]) / (t1 - t0)
                        rates.append(rate)
                    else:
                        rates.append(0)
                else:
                    rates.append(0)
            else:
                rates.append(0)
        
        efficiency_data[ctrl_name].append(rates)

# Plot as grouped bars
x = np.arange(len(fault_types))
width = 0.35
for i, range_name in enumerate(coverage_ranges):
    reip_vals = [efficiency_data['REIP'][j][i] if i < len(efficiency_data['REIP'][j]) else 0 
                 for j in range(len(fault_types))]
    raft_vals = [efficiency_data['Raft'][j][i] if i < len(efficiency_data['Raft'][j]) else 0 
                 for j in range(len(fault_types))]
    
    offset = (i - 1) * width * 1.2
    ax2.bar(x + offset - width/2, reip_vals, width, label=f'REIP {range_name}' if i == 0 else '',
            color=['#1a1a1a', '#e74c3c', '#3498db'][i], alpha=0.7, edgecolor='black')
    ax2.bar(x + offset + width/2, raft_vals, width, label=f'Raft {range_name}' if i == 0 else '',
            color=['#666666', '#c0392b', '#2980b9'][i], alpha=0.7, edgecolor='black', hatch='///')

ax2.set_xlabel('Fault Condition', fontsize=11, fontweight='bold')
ax2.set_ylabel('Avg Rate (%/s)', fontsize=11, fontweight='bold')
ax2.set_title('(b) Efficiency by Coverage Range', fontsize=13, fontweight='bold', pad=10)
ax2.set_xticks(x)
ax2.set_xticklabels(fault_types, fontsize=10)
ax2.legend(fontsize=8, loc='upper right', ncol=2)
ax2.grid(True, alpha=0.3, axis='y')

# Panel 3: Slope Analysis from Normalized Rate
ax3 = fig.add_subplot(gs[0, 2])
slope_data = []

for ctrl in ['reip', 'raft']:
    for fault_key, fault_label in zip(fault_keys, fault_types):
        if (ctrl, fault_key) not in condition_data:
            continue
        _, _, resampled = condition_data[(ctrl, fault_key)]
        med = np.median(resampled, axis=0)
        
        # Compute normalized rate
        derivative = np.gradient(med, DT)
        remaining = np.maximum(100.0 - med, 0.1)
        normalized_rate = derivative / np.sqrt(remaining)
        window = 5
        normalized_rate = np.convolve(normalized_rate, np.ones(window)/window, mode='same')
        
        # Fit linear trend after fault 2 (t=30s onwards)
        mask = time_axis >= 30
        if np.sum(mask) > 10:  # Need enough points
            x_fit = time_axis[mask]
            y_fit = normalized_rate[mask]
            
            # Fit polynomial (degree 1 = linear)
            coeffs = np.polyfit(x_fit, y_fit, deg=1)
            slope = coeffs[0]  # slope of linear fit
            
            slope_data.append({
                'ctrl': CTRL_LABELS[ctrl][:4],
                'fault': fault_label,
                'slope': slope,
                'color': FAULT_COLORS[fault_key] if ctrl == 'reip' else '#999999'
            })

# Plot slopes
if slope_data:
    x_pos = np.arange(len(slope_data))
    slopes = [d['slope'] for d in slope_data]
    colors_slope = [d['color'] for d in slope_data]
    labels_slope = [f"{d['ctrl']}\n{d['fault']}" for d in slope_data]
    
    bars = ax3.bar(x_pos, slopes, color=colors_slope, edgecolor='black', 
                   linewidth=1.5, alpha=0.8)
    ax3.axhline(y=0, color='black', linewidth=1, linestyle='-', alpha=0.5)
    ax3.set_xticks(x_pos)
    ax3.set_xticklabels(labels_slope, fontsize=9, rotation=0, ha='center')
    ax3.set_ylabel('Slope (1/s^2)', fontsize=12, fontweight='bold')
    ax3.set_title('(c) Normalized Rate Slope\n(post-fault 2)', fontsize=13, fontweight='bold', pad=10)
    ax3.grid(True, alpha=0.3, axis='y')
    for bar, val in zip(bars, slopes):
        ax3.text(bar.get_x() + bar.get_width()/2, val + (0.002 if val >= 0 else -0.005),
                 f'{val:.4f}', ha='center', va='bottom' if val >= 0 else 'top',
                 fontsize=8, fontweight='bold')

fig.tight_layout(pad=2.0)
for ext in ['png', 'pdf']:
    out = os.path.join(base, f'resilience_metrics.{ext}')
    fig.savefig(out, bbox_inches='tight')
    print(f"Saved: {out}")
plt.close(fig)

print(f"\nAll figures saved to: {base}/")
print("  convergence_all          - all 9 conditions overlaid")
print("  convergence_badleader    - bad leader focus with clean baselines faded")
print("  convergence_traces       - per-trial traces showing reliability")
print("  convergence_by_controller- clean vs fault per controller")
print("  resilience_contrast      - REIP tight bundle vs Raft divergence")
print("  resilience_contrast_derivative - rate of change (coverage velocity)")
print("  resilience_contrast_normalized_rate - exploration intensity (rate/remaining)")
print("  resilience_metrics       - degradation, efficiency, slope analysis")