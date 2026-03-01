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
    'reip': 'REIP',
    'raft': 'Raft',
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

# Fault injection lines (no text — described in caption)
for ft in [10, 30]:
    ax.axvline(x=ft, color='#888888', linewidth=0.6, linestyle='-.',
               alpha=0.55, zorder=1)

ax.set_xlabel('Time (s)')
ax.set_ylabel('Coverage (%)')
ax.set_xlim(0, 120)
ax.set_ylim(0, 105)
ax.set_yticks([0, 20, 40, 60, 80, 100])
ax.grid(True, alpha=0.25, linewidth=0.3)

# Build legend — controllers (color+marker) and faults (linestyle)
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
# FIGURE 2: Bad Leader only — clean baselines faded, fault bold
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

# Fault injection lines — clean, no text
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
# FIGURE 3: Per-trial fan chart (percentile bands — replaces spaghetti)
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

        # Fan chart: percentile bands (outer → inner)
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
    Patch(facecolor='gray', alpha=0.14, label='IQR (25–75%)'),
    Patch(facecolor='gray', alpha=0.08, label='5–95%'),
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
# FIGURE 5: RESILIENCE CONTRAST — the key result figure
#   Panel (a): REIP — all 3 conditions nearly overlap = resilient
#   Panel (b): RAFT — conditions diverge = not resilient
#   Uses median + IQR instead of mean + std for robustness
# ============================================================
fig, axes = plt.subplots(1, 2, figsize=(7.16, 2.6), sharey=True)

FAULT_COLORS = {
    'none': '#2c3e50',       # dark gray-blue
    'bad_leader': '#e74c3c', # red
    'freeze_leader': '#8e44ad',  # purple
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
for ax_idx, ctrl in enumerate(['reip', 'raft']):
    ax = axes[ax_idx]

    for fault in ['none', 'bad_leader', 'freeze_leader', 'oscillate_leader', 'spin']:
        if (ctrl, fault) not in condition_data:
            continue
        _, _, resampled = condition_data[(ctrl, fault)]
        med = np.median(resampled, axis=0)
        color = FAULT_COLORS[fault]
        style = FAULT_STYLES[fault]
        fmarker = FAULT_MARKERS_RC[fault]

        ax.plot(time_axis, med, linestyle=style, color=color, linewidth=1.4,
                marker=fmarker, markevery=MARKER_EVERY, markersize=3,
                label=FAULT_LABELS[fault], zorder=3)
        # IQR shading (25th-75th percentile)
        q25 = np.percentile(resampled, 25, axis=0)
        q75 = np.percentile(resampled, 75, axis=0)
        ax.fill_between(time_axis, q25, q75, alpha=0.12, color=color,
                        linewidth=0)

    for ft in [10, 30]:
        ax.axvline(x=ft, color='#888888', linewidth=0.5, linestyle='-.',
                   alpha=0.45)

    ax.set_xlim(0, 120)
    ax.set_ylim(0, 105)
    ax.set_xlabel('Time (s)')
    ax.set_title(f'{RC_PANEL_LABELS[ax_idx]} {CTRL_LABELS[ctrl]}',
                 fontsize=9.5, fontweight='bold')
    ax.grid(True, alpha=0.2, linewidth=0.3)
    ax.legend(fontsize=7, loc='lower right', framealpha=0.9,
              edgecolor='#cccccc')

axes[0].set_ylabel('Coverage (%)')
fig.tight_layout(pad=0.5)

for ext in ['png', 'pdf']:
    out = os.path.join(base, f'resilience_contrast.{ext}')
    fig.savefig(out, bbox_inches='tight')
    print(f"Saved: {out}")
plt.close(fig)

print(f"\nAll figures saved to: {base}/")
print("  convergence_all          — all 9 conditions overlaid")
print("  convergence_badleader    — bad leader focus with clean baselines faded")
print("  convergence_traces       — per-trial traces showing reliability")
print("  convergence_by_controller— clean vs fault per controller")
print("  resilience_contrast      — REIP tight bundle vs Raft divergence")
