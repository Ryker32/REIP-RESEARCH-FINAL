#!/usr/bin/env python3
"""
Gridworld Snapshot Figure Generator
====================================
Renders filmstrip-style figures showing robot positions, explored cells,
walls, targets, and leader roles at key timestamps during a simulation.

Key visual: each robot shows TWO vectors:
  GREEN arrow = robot's predicted movement (from its own local map belief)
  RED arrow   = leader's commanded direction (the assignment from the leader)
When these diverge, the robot is receiving a bad command.

Trust contour:
  Yellow halo = trust wavering (suspicion > 0.3)
  Red halo    = trust critical / compromised (suspicion > 1.0 or motor fault)

Usage:
    python test/_gridworld_snapshots.py <snapshot_dir_or_file> [--times 0,10,30,60,90,120]
    python test/_gridworld_snapshots.py experiments/run_xxx/logs/multiroom_reip_BadLeader_t1
    python test/_gridworld_snapshots.py path1 path2 --compare --times 5,11,13,32,60,120
"""

import json
import math
import os
import sys
import argparse
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from pathlib import Path

# ---- IEEE styling (consistent with convergence charts) ----
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
    'axes.linewidth': 0.6,
    'text.usetex': False,
    'mathtext.fontset': 'stix',
})


# ==================== Style Constants ====================
ROBOT_COLORS = {
    1: '#1f77b4',   # blue
    2: '#ff7f0e',   # orange
    3: '#2ca02c',   # green
    4: '#d62728',   # red
    5: '#9467bd',   # purple
}
WALL_COLOR = '#2c3e50'
WALL_LINEWIDTH = 4
EXPLORED_COLOR = '#d5e8d4'       # light green
UNEXPLORED_COLOR = '#f5f5f5'     # light gray
GRID_COLOR = '#e0e0e0'
LEADER_MARKER = '*'
FOLLOWER_MARKER = 'o'
LEADER_SIZE = 350
FOLLOWER_SIZE = 150
FONT_SIZE = 7

# Vector arrow styles
CMD_VECTOR_COLOR = '#c0392b'      # dark red - leader command
PRED_VECTOR_COLOR = '#27ae60'     # green - robot's own belief
VECTOR_LW = 2.8
VECTOR_ALPHA = 0.9
ARROW_SCALE = 2.5                 # fraction of cell_size for arrow length

# Trust contour styles
TRUST_OK_ALPHA = 0.0             # no contour when trust is fine
TRUST_WAVER_COLOR = '#f39c12'    # yellow - trust wavering
TRUST_WAVER_ALPHA = 0.35
TRUST_CRITICAL_COLOR = '#e74c3c' # red - trust critical
TRUST_CRITICAL_ALPHA = 0.45
CONTOUR_RADIUS_MULT = 1.8        # radius multiplier vs robot marker


def load_snapshots(path):
    """Load snapshots from a directory or file."""
    if os.path.isdir(path):
        snap_file = os.path.join(path, "snapshots.json")
    else:
        snap_file = path

    if not os.path.exists(snap_file):
        print(f"ERROR: No snapshots.json found at {snap_file}")
        print("  Run a trial with --snapshots flag to generate position data.")
        sys.exit(1)

    with open(snap_file, 'r') as f:
        data = json.load(f)
    return data


def find_closest_frame(frames, target_time):
    """Find the frame closest to target_time."""
    best = None
    best_diff = float('inf')
    for frame in frames:
        diff = abs(frame['t'] - target_time)
        if diff < best_diff:
            best_diff = diff
            best = frame
    return best


def _point_to_segment_dist(px, py, x1, y1, x2, y2):
    """Distance from point (px,py) to line segment (x1,y1)-(x2,y2)."""
    dx, dy = x2 - x1, y2 - y1
    if dx == 0 and dy == 0:
        return ((px - x1)**2 + (py - y1)**2) ** 0.5
    t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)))
    proj_x = x1 + t * dx
    proj_y = y1 + t * dy
    return ((px - proj_x)**2 + (py - proj_y)**2) ** 0.5


def _compute_vector(rx, ry, target, cell_size):
    """Compute unit-direction arrow from robot pos to target.
    Returns (dx, dy) scaled to ARROW_SCALE * cell_size, or None."""
    if not target or len(target) < 2:
        return None
    tx, ty = target[0], target[1]
    dx, dy = tx - rx, ty - ry
    mag = math.hypot(dx, dy)
    if mag < 1:
        return None
    arrow_len = cell_size * ARROW_SCALE
    return (dx / mag * arrow_len, dy / mag * arrow_len)


def draw_arena(ax, arena_w, arena_h, cell_size, walls, visited_cells,
               frame, fault_times, title_prefix="", annotation=""):
    """Draw one snapshot panel with dual vectors and trust contours."""
    n_cols = arena_w // cell_size
    n_rows = arena_h // cell_size

    ax.set_xlim(-10, arena_w + 10)
    ax.set_ylim(-10, arena_h + 10)
    ax.set_aspect('equal')
    ax.set_facecolor('#fafafa')

    # ---- Grid cells ----
    visited_set = set(tuple(c) for c in visited_cells)
    for cx in range(n_cols):
        for cy in range(n_rows):
            x0 = cx * cell_size
            y0 = cy * cell_size
            color = EXPLORED_COLOR if (cx, cy) in visited_set else UNEXPLORED_COLOR
            rect = mpatches.FancyBboxPatch(
                (x0 + 1, y0 + 1), cell_size - 2, cell_size - 2,
                boxstyle="round,pad=2", facecolor=color, edgecolor=GRID_COLOR,
                linewidth=0.5)
            ax.add_patch(rect)

    # ---- Wall-blocked cells (hatched overlay) ----
    for cx in range(n_cols):
        for cy in range(n_rows):
            ccx = (cx + 0.5) * cell_size
            ccy = (cy + 0.5) * cell_size
            for wx1, wy1, wx2, wy2 in walls:
                if _point_to_segment_dist(ccx, ccy, wx1, wy1, wx2, wy2) < cell_size * 0.4:
                    x0 = cx * cell_size
                    y0 = cy * cell_size
                    rect = mpatches.FancyBboxPatch(
                        (x0 + 1, y0 + 1), cell_size - 2, cell_size - 2,
                        boxstyle="round,pad=2", facecolor='#dcdde1',
                        edgecolor=GRID_COLOR, linewidth=0.5, alpha=0.6)
                    ax.add_patch(rect)
                    break

    # ---- Walls ----
    for wx1, wy1, wx2, wy2 in walls:
        ax.plot([wx1, wx2], [wy1, wy2], color=WALL_COLOR,
                linewidth=WALL_LINEWIDTH, solid_capstyle='round', zorder=5)

    # ---- Arena border ----
    ax.plot([0, arena_w, arena_w, 0, 0], [0, 0, arena_h, arena_h, 0],
            color=WALL_COLOR, linewidth=2, zorder=5)

    # ---- Draw robots with dual vectors ----
    robots = frame.get('robots', {})
    for rid_str, rdata in robots.items():
        rid = int(rid_str)
        rx, ry = rdata['x'], rdata['y']
        state = rdata.get('state', 'follower')
        leader_id = rdata.get('leader_id')
        is_leader = (state == 'leader') or (leader_id == rid)
        motor_fault = rdata.get('motor_fault')
        trust = rdata.get('trust', 1.0)
        suspicion = rdata.get('suspicion', 0.0)
        predicted_target = rdata.get('predicted_target')
        commanded_target = rdata.get('commanded_target')

        agent_color = ROBOT_COLORS.get(rid, '#888888')
        marker = LEADER_MARKER if is_leader else FOLLOWER_MARKER
        msize = LEADER_SIZE if is_leader else FOLLOWER_SIZE

        # ---- Trust contour (halo around robot) ----
        contour_r = cell_size * CONTOUR_RADIUS_MULT * 0.5
        if motor_fault:
            halo = plt.Circle((rx, ry), contour_r,
                              color=TRUST_CRITICAL_COLOR,
                              alpha=TRUST_CRITICAL_ALPHA,
                              linewidth=0, zorder=7)
            ax.add_patch(halo)
        elif suspicion > 1.0:
            halo = plt.Circle((rx, ry), contour_r,
                              color=TRUST_CRITICAL_COLOR,
                              alpha=TRUST_CRITICAL_ALPHA,
                              linewidth=0, zorder=7)
            ax.add_patch(halo)
        elif suspicion > 0.3:
            halo = plt.Circle((rx, ry), contour_r,
                              color=TRUST_WAVER_COLOR,
                              alpha=TRUST_WAVER_ALPHA,
                              linewidth=0, zorder=7)
            ax.add_patch(halo)

        # ---- Edge color ----
        if motor_fault:
            edgecolor = '#e74c3c'
            elw = 3
        elif suspicion > 0.5:
            edgecolor = '#f39c12'
            elw = 2.5
        else:
            edgecolor = 'black'
            elw = 1

        # ---- Robot marker ----
        ax.scatter(rx, ry, c=[agent_color], s=msize, marker=marker,
                   edgecolors=edgecolor, linewidths=elw, zorder=10)

        # ---- COMMAND VECTOR (RED) - what the leader told this robot ----
        cmd_vec = _compute_vector(rx, ry, commanded_target, cell_size)
        if cmd_vec and not is_leader:
            ax.annotate('', xy=(rx + cmd_vec[0], ry + cmd_vec[1]),
                        xytext=(rx, ry),
                        arrowprops=dict(arrowstyle='->', color=CMD_VECTOR_COLOR,
                                        lw=VECTOR_LW, alpha=VECTOR_ALPHA),
                        zorder=9)

        # ---- PREDICTED VECTOR (GREEN) - robot's own belief ----
        pred_vec = _compute_vector(rx, ry, predicted_target, cell_size)
        if pred_vec and not is_leader:
            ax.annotate('', xy=(rx + pred_vec[0], ry + pred_vec[1]),
                        xytext=(rx, ry),
                        arrowprops=dict(arrowstyle='->', color=PRED_VECTOR_COLOR,
                                        lw=VECTOR_LW, alpha=VECTOR_ALPHA),
                        zorder=9)

        # ---- Leader gets a single heading arrow (its own color) ----
        if is_leader:
            theta = rdata.get('theta', 0)
            arrow_len = cell_size * ARROW_SCALE
            dx_a = arrow_len * math.cos(theta)
            dy_a = arrow_len * math.sin(theta)
            ax.annotate('', xy=(rx + dx_a, ry + dy_a), xytext=(rx, ry),
                        arrowprops=dict(arrowstyle='->', color=agent_color,
                                        lw=1.5, alpha=0.7), zorder=9)

        # ---- Robot ID label ----
        ax.annotate(str(rid), (rx + 25, ry + 25), fontsize=FONT_SIZE,
                    fontweight='bold', color=agent_color, zorder=11,
                    bbox=dict(facecolor='white', alpha=0.7,
                              edgecolor='none', pad=0.5))

    # ---- Title ----
    t = frame['t']
    cov = frame.get('coverage', 0)
    fault_label = ""
    if fault_times:
        for i, ft in enumerate(fault_times):
            if ft is not None and abs(t - ft) < 1.5:
                fault_label = "  [FAULT]"
    title = f"{title_prefix}t = {t:.0f}s  |  {cov:.0f}%"
    if fault_label:
        title += fault_label
    if annotation:
        title += f"\n{annotation}"
    ax.set_title(title, fontsize=8, fontweight='bold', pad=4)
    ax.set_xticks([])
    ax.set_yticks([])


# ==================== Gallery Generator ====================
def generate_gallery(reip_data, raft_data, output_path,
                     reip_times=None, raft_times=None, reip_annotations=None,
                     raft_annotations=None):
    """Generate the 2-row x 5-column gallery: WITH REIP (top) vs WITHOUT REIP (bottom).

    This matches the poster concept:
      Top row (REIP):    fault -> suspicion -> impeach -> election -> recovery
      Bottom row (RAFT): fault -> stuck -> stuck -> stuck -> stuck
    """
    if reip_times is None:
        reip_times = [9, 11, 13, 32, 60]
    if raft_times is None:
        raft_times = [9, 11, 20, 40, 60]
    if reip_annotations is None:
        reip_annotations = [
            "Pre-fault: vectors aligned",
            "Fault! Vectors diverge",
            "Trust wavering -> impeach",
            "New leader elected",
            "Recovery: full coverage",
        ]
    if raft_annotations is None:
        raft_annotations = [
            "Pre-fault: vectors aligned",
            "Fault! Vectors diverge",
            "No detection, still following",
            "Stuck on explored cells",
            "Permanently stalled",
        ]

    n_cols = max(len(reip_times), len(raft_times))
    fig_w = 3.8 * n_cols
    fig_h = 7.2
    fig, axes = plt.subplots(2, n_cols, figsize=(fig_w, fig_h))

    # ---- Top row: WITH REIP ----
    r_frames = reip_data['frames']
    r_aw, r_ah = reip_data['arena_width'], reip_data['arena_height']
    r_cs = reip_data['cell_size']
    r_walls = reip_data.get('walls', [])
    r_faults = [reip_data.get('fault_time_1'), reip_data.get('fault_time_2')]

    for col in range(n_cols):
        if col < len(reip_times):
            frame = find_closest_frame(r_frames, reip_times[col])
            ann = reip_annotations[col] if col < len(reip_annotations) else ""
            draw_arena(axes[0][col], r_aw, r_ah, r_cs, r_walls,
                       frame.get('visited_cells', []), frame, r_faults,
                       annotation=ann)
        else:
            axes[0][col].set_visible(False)

    # ---- Bottom row: WITHOUT REIP (RAFT) ----
    f_frames = raft_data['frames']
    f_aw, f_ah = raft_data['arena_width'], raft_data['arena_height']
    f_cs = raft_data['cell_size']
    f_walls = raft_data.get('walls', [])
    f_faults = [raft_data.get('fault_time_1'), raft_data.get('fault_time_2')]

    for col in range(n_cols):
        if col < len(raft_times):
            frame = find_closest_frame(f_frames, raft_times[col])
            ann = raft_annotations[col] if col < len(raft_annotations) else ""
            draw_arena(axes[1][col], f_aw, f_ah, f_cs, f_walls,
                       frame.get('visited_cells', []), frame, f_faults,
                       annotation=ann)
        else:
            axes[1][col].set_visible(False)

    # ---- Row labels ----
    axes[0][0].set_ylabel("WITH REIP", fontsize=12, fontweight='bold',
                           color='#27ae60', rotation=90, labelpad=10)
    axes[1][0].set_ylabel("WITHOUT REIP\n(RAFT)", fontsize=12, fontweight='bold',
                           color='#c0392b', rotation=90, labelpad=10)

    # ---- Legend ----
    legend_elements = [
        mpatches.Patch(facecolor=EXPLORED_COLOR, edgecolor=GRID_COLOR, label='Explored'),
        mpatches.Patch(facecolor=UNEXPLORED_COLOR, edgecolor=GRID_COLOR, label='Unexplored'),
        plt.Line2D([0], [0], marker='*', color='w', markerfacecolor='#1f77b4',
                   markeredgecolor='black', markersize=12, label='Leader'),
        plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='#ff7f0e',
                   markeredgecolor='black', markersize=9, label='Follower'),
        plt.Line2D([0], [0], color=PRED_VECTOR_COLOR, lw=2.5,
                   label='Predicted (robot belief)'),
        plt.Line2D([0], [0], color=CMD_VECTOR_COLOR, lw=2.5,
                   label='Commanded (leader order)'),
        mpatches.Patch(facecolor=TRUST_WAVER_COLOR, alpha=0.5,
                       edgecolor='none', label='Trust wavering'),
        mpatches.Patch(facecolor=TRUST_CRITICAL_COLOR, alpha=0.5,
                       edgecolor='none', label='Trust critical'),
    ]
    fig.legend(handles=legend_elements, loc='lower center', ncol=4,
               fontsize=8, frameon=True, edgecolor='gray',
               bbox_to_anchor=(0.5, -0.02))

    fig.suptitle("Fault Resilience Gallery: REIP vs RAFT under Byzantine Leader Attack",
                 fontsize=13, fontweight='bold', y=1.01)

    plt.tight_layout(h_pad=1.5)
    for ext in ['.png', '.pdf']:
        out = output_path.replace('.png', ext).replace('.pdf', ext)
        if not out.endswith(ext):
            out = output_path + ext
        fig.savefig(out, dpi=300, bbox_inches='tight',
                    facecolor='white', edgecolor='none')
        print(f"  Saved: {out}")
    plt.close(fig)


# ==================== Filmstrip ====================
def generate_filmstrip(data, snapshot_times, output_path, title_prefix=""):
    """Generate a filmstrip figure with one panel per timestamp."""
    frames_data = data['frames']
    arena_w = data['arena_width']
    arena_h = data['arena_height']
    cell_size = data['cell_size']
    walls = data.get('walls', [])
    ft1 = data.get('fault_time_1')
    ft2 = data.get('fault_time_2')
    fault_times = [ft1, ft2]

    n_panels = len(snapshot_times)
    fig_width = 3.5 * n_panels
    fig, axes = plt.subplots(1, n_panels, figsize=(fig_width, 3.2))
    if n_panels == 1:
        axes = [axes]

    for i, target_t in enumerate(snapshot_times):
        frame = find_closest_frame(frames_data, target_t)
        if frame is None:
            print(f"  Warning: no frame near t={target_t}s")
            continue
        draw_arena(axes[i], arena_w, arena_h, cell_size, walls,
                   frame.get('visited_cells', []), frame, fault_times,
                   title_prefix=title_prefix)

    # Legend
    legend_elements = [
        mpatches.Patch(facecolor=EXPLORED_COLOR, edgecolor=GRID_COLOR, label='Explored'),
        mpatches.Patch(facecolor=UNEXPLORED_COLOR, edgecolor=GRID_COLOR, label='Unexplored'),
        plt.Line2D([0], [0], marker='*', color='w', markerfacecolor='#1f77b4',
                   markeredgecolor='black', markersize=12, label='Leader'),
        plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='#ff7f0e',
                   markeredgecolor='black', markersize=9, label='Follower'),
        plt.Line2D([0], [0], color=PRED_VECTOR_COLOR, lw=2.5,
                   label='Predicted (belief)'),
        plt.Line2D([0], [0], color=CMD_VECTOR_COLOR, lw=2.5,
                   label='Commanded (leader)'),
        mpatches.Patch(facecolor=TRUST_WAVER_COLOR, alpha=0.5,
                       edgecolor='none', label='Trust wavering'),
    ]
    fig.legend(handles=legend_elements, loc='lower center', ncol=len(legend_elements),
               fontsize=7, frameon=True, edgecolor='gray',
               bbox_to_anchor=(0.5, -0.02))

    controller = data.get('controller', '?').upper()
    fault = data.get('fault_type', 'clean') or 'clean'
    fig.suptitle(f"{controller} - {fault.replace('_', ' ').title()}  (seed {data.get('seed', '?')})",
                 fontsize=11, fontweight='bold', y=1.02)

    plt.tight_layout()
    fig.savefig(output_path, dpi=300, bbox_inches='tight',
                facecolor='white', edgecolor='none')
    print(f"  Saved: {output_path}")
    plt.close(fig)


# ==================== Comparison ====================
def generate_comparison(snapshot_files, snapshot_times, output_path):
    """Generate multi-row comparison: one row per condition, one col per timestamp."""
    n_rows = len(snapshot_files)
    n_cols = len(snapshot_times)
    fig_width = 3.2 * n_cols
    fig_height = 2.8 * n_rows
    fig, axes = plt.subplots(n_rows, n_cols, figsize=(fig_width, fig_height))
    if n_rows == 1:
        axes = [axes]

    for row, (label, data) in enumerate(snapshot_files):
        frames_data = data['frames']
        arena_w = data['arena_width']
        arena_h = data['arena_height']
        cell_size = data['cell_size']
        walls = data.get('walls', [])
        ft1 = data.get('fault_time_1')
        ft2 = data.get('fault_time_2')
        fault_times = [ft1, ft2]

        for col, target_t in enumerate(snapshot_times):
            ax = axes[row][col] if n_rows > 1 else axes[col]
            frame = find_closest_frame(frames_data, target_t)
            if frame is None:
                ax.set_visible(False)
                continue
            tp = f"{label}\n" if col == 0 else ""
            draw_arena(ax, arena_w, arena_h, cell_size, walls,
                       frame.get('visited_cells', []), frame, fault_times,
                       title_prefix=tp)

    fig.suptitle("Gridworld Snapshots - REIP vs RAFT",
                 fontsize=12, fontweight='bold', y=1.01)
    plt.tight_layout()
    fig.savefig(output_path, dpi=300, bbox_inches='tight',
                facecolor='white', edgecolor='none')
    print(f"  Saved: {output_path}")
    plt.close(fig)


# ==================== Main ====================
def main():
    parser = argparse.ArgumentParser(description="Gridworld snapshot figures")
    parser.add_argument("paths", nargs='+',
                        help="Snapshot dir(s) or file(s)")
    parser.add_argument("--times", default="0,10,30,60,90,120",
                        help="Comma-separated snapshot timestamps")
    parser.add_argument("--output", default=None,
                        help="Output path (default: next to snapshot file)")
    parser.add_argument("--compare", action="store_true",
                        help="Multi-row comparison figure")
    parser.add_argument("--gallery", action="store_true",
                        help="Generate REIP-vs-RAFT gallery (needs exactly 2 paths: reip, raft)")
    args = parser.parse_args()

    snapshot_times = [float(t) for t in args.times.split(',')]
    print(f"Snapshot times: {snapshot_times}")

    if args.gallery and len(args.paths) >= 2:
        reip_data = load_snapshots(args.paths[0])
        raft_data = load_snapshots(args.paths[1])
        out = args.output or os.path.join(os.path.dirname(args.paths[0]),
                                          "..", "gallery_reip_vs_raft.png")
        generate_gallery(reip_data, raft_data, out,
                         reip_times=snapshot_times, raft_times=snapshot_times)
    elif args.compare and len(args.paths) > 1:
        rows = []
        for p in args.paths:
            data = load_snapshots(p)
            label = f"{data.get('controller', '?').upper()} - " \
                    f"{(data.get('fault_type') or 'clean').replace('_', ' ').title()}"
            rows.append((label, data))
        out = args.output or os.path.join(os.path.dirname(args.paths[0]),
                                          "..", "gridworld_comparison.png")
        generate_comparison(rows, snapshot_times, out)
    else:
        for p in args.paths:
            data = load_snapshots(p)
            name = data.get('name', 'snapshot')
            out_dir = os.path.dirname(p) if os.path.isfile(p) else p
            out = args.output or os.path.join(out_dir, f"gridworld_{name}.png")
            generate_filmstrip(data, snapshot_times, out)
            pdf_out = out.replace('.png', '.pdf')
            generate_filmstrip(data, snapshot_times, pdf_out)


if __name__ == '__main__':
    main()
