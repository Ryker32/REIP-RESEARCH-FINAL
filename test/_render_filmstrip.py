#!/usr/bin/env python3
"""
Render a filmstrip visualization from experiment snapshot data.

Produces a multi-panel figure showing exploration state at key moments:
  Pre-fault | Fault active | Post-impeachment | Near-complete

Adapted from the GridWorld plot_env_enhanced style for multiroom layout.
"""

import json, os, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, FancyArrowPatch
from matplotlib.collections import PatchCollection
import matplotlib.patheffects as pe

# ---- IEEE styling ----
plt.rcParams.update({
    'font.family': 'serif',
    'font.serif': ['Times New Roman', 'Times', 'DejaVu Serif'],
    'font.size': 9,
    'figure.dpi': 300,
    'savefig.dpi': 300,
    'text.usetex': False,
    'mathtext.fontset': 'stix',
})

ROBOT_COLORS = {
    '1': '#e74c3c',  # red
    '2': '#2980b9',  # blue
    '3': '#27ae60',  # green
    '4': '#f39c12',  # orange
    '5': '#8e44ad',  # purple
}

COL_SENSE   = '#7f8c8d'
CMD_COL     = '#c0392b'       # dark red - leader command
PRED_COL    = '#27ae60'       # green - agent belief / navigation
COL_LEADER  = 'gold'
VEC_LW      = 2.2
VEC_ALP     = 0.88
MUT_SC      = 12
MIN_ARROW_DIST = 20

def find_frame_at_time(frames, target_t):
    """Find the frame closest to target_t."""
    best = None
    best_dt = float('inf')
    for f in frames:
        dt = abs(f['t'] - target_t)
        if dt < best_dt:
            best_dt = dt
            best = f
    return best


def _draw_arrow(ax, rx, ry, tx, ty, arrow_len, color, ls, zorder,
                walls=None):
    """Draw directional arrow, suppressing if it crosses a wall."""
    import math
    dist = math.hypot(tx - rx, ty - ry)
    if dist < MIN_ARROW_DIST:
        return
    dx = (tx - rx) / dist * arrow_len
    dy = (ty - ry) / dist * arrow_len
    tip_x, tip_y = rx + dx, ry + dy
    if walls:
        for x1, y1, x2, y2 in walls:
            if _segments_intersect(rx, ry, tip_x, tip_y, x1, y1, x2, y2):
                return
    ax.annotate(
        '', xy=(tip_x, tip_y), xytext=(rx, ry),
        arrowprops=dict(arrowstyle='-|>', color=color, lw=VEC_LW,
                        alpha=VEC_ALP, linestyle=ls, mutation_scale=MUT_SC),
        zorder=zorder)


def render_frame(ax, frame, arena_w, arena_h, cell_size, walls,
                 fault_time_1=None, fault_time_2=None, show_legend=False):
    """Render a single frame with sim_environment-style vector annotations."""
    import math
    cols = arena_w // cell_size
    rows = arena_h // cell_size
    t = frame['t']
    coverage = frame['coverage']
    visited = set(tuple(c) for c in frame.get('visited_cells', []))
    robots = frame.get('robots', {})
    arrow_len = cell_size * 1.8

    is_fault_active = fault_time_1 and t >= fault_time_1

    # Grid cells
    for c in range(cols):
        for r in range(rows):
            x0 = c * cell_size
            y0 = r * cell_size
            color = '#ffffff' if (c, r) in visited else '#c8ddf0'
            rect = Rectangle((x0, y0), cell_size, cell_size,
                            facecolor=color, edgecolor='#e0e0e0',
                            linewidth=0.3, zorder=1)
            ax.add_patch(rect)

    # Grid lines (faint)
    for i in range(cols + 1):
        ax.axvline(i * cell_size, color='#cccccc', lw=0.3, alpha=0.3, zorder=1)
    for j in range(rows + 1):
        ax.axhline(j * cell_size, color='#cccccc', lw=0.3, alpha=0.3, zorder=1)

    # Walls
    for wall in walls:
        x1, y1, x2, y2 = wall
        ax.plot([x1, x2], [y1, y2], color='#222222', linewidth=4,
                solid_capstyle='butt', zorder=5)

    # Arena border
    ax.plot([0, arena_w, arena_w, 0, 0],
            [0, 0, arena_h, arena_h, 0],
            color='#333333', linewidth=1.5, zorder=10)

    # Frontiers
    frontier_cells = set()
    for (c, r) in visited:
        for dc, dr in [(-1,0),(1,0),(0,-1),(0,1)]:
            nc, nr = c+dc, r+dr
            if 0 <= nc < cols and 0 <= nr < rows and (nc, nr) not in visited:
                cx1 = c * cell_size + cell_size // 2
                cy1 = r * cell_size + cell_size // 2
                cx2 = nc * cell_size + cell_size // 2
                cy2 = nr * cell_size + cell_size // 2
                blocked = False
                for wall in walls:
                    wx1, wy1, wx2, wy2 = wall
                    if _segments_intersect(cx1, cy1, cx2, cy2, wx1, wy1, wx2, wy2):
                        blocked = True
                        break
                if not blocked:
                    frontier_cells.add((nc, nr))

    if frontier_cells:
        fx = [c * cell_size + cell_size / 2 for c, r in frontier_cells]
        fy = [r * cell_size + cell_size / 2 for c, r in frontier_cells]
        ax.scatter(fx, fy, marker='s', s=28, c='#f1c40f',
                   edgecolors='k', linewidths=0.4, zorder=4)

    # Find leader
    leader_id = None
    for rid, rdata in robots.items():
        if rdata.get('state') == 'leader':
            leader_id = rid
            break

    # Draw robots with sim_environment-style annotations
    sense_r = 170
    for rid in sorted(robots.keys()):
        rdata = robots[rid]
        rx, ry = rdata['x'], rdata['y']
        color = ROBOT_COLORS.get(rid, '#888888')
        is_leader = (rid == leader_id)
        zorder = 20 if is_leader else 15

        # Sensor sweep circle
        ax.add_patch(Circle((rx, ry), sense_r,
                            color=(0.2, 0.9, 0.2, 0.06), zorder=2))
        ax.add_patch(Circle((rx, ry), sense_r, color=COL_SENSE,
                            fill=False, ls='dashed', lw=0.6, alpha=0.4,
                            zorder=6))

        # Arrows - same logic as sim_environment.png
        cmd  = rdata.get('commanded_target') or rdata.get('target')
        pred = rdata.get('predicted_target')
        wp   = rdata.get('next_waypoint')

        if is_leader:
            src = pred or wp
            if src:
                _draw_arrow(ax, rx, ry, src[0], src[1],
                            arrow_len, PRED_COL, 'solid', 12,
                            walls=walls if not is_fault_active else None)
        else:
            if is_fault_active:
                if cmd:
                    _draw_arrow(ax, rx, ry, cmd[0], cmd[1],
                                arrow_len, CMD_COL, 'dashed', 11)
                if pred:
                    _draw_arrow(ax, rx, ry, pred[0], pred[1],
                                arrow_len, PRED_COL, 'solid', 13)
            else:
                drawn = False
                if pred:
                    d = math.hypot(pred[0] - rx, pred[1] - ry)
                    if d >= MIN_ARROW_DIST:
                        dx = (pred[0] - rx) / d * arrow_len
                        dy = (pred[1] - ry) / d * arrow_len
                        if not any(_segments_intersect(rx, ry, rx+dx, ry+dy,
                                   w[0], w[1], w[2], w[3]) for w in walls):
                            _draw_arrow(ax, rx, ry, pred[0], pred[1],
                                        arrow_len, PRED_COL, 'solid', 13)
                            drawn = True
                if not drawn and wp:
                    _draw_arrow(ax, rx, ry, wp[0], wp[1],
                                arrow_len, PRED_COL, 'solid', 13, walls=walls)

        # Robot marker
        if is_leader:
            ax.plot(rx, ry, 's', ms=10, color=COL_LEADER, mec='k',
                    mew=1.2, zorder=zorder)
            ax.add_patch(Circle((rx, ry), cell_size * 0.8, color=COL_LEADER,
                                fill=False, lw=2, alpha=0.9, zorder=zorder-1))
        else:
            ax.plot(rx, ry, 'o', color=color, markersize=8,
                    markeredgecolor='k', markeredgewidth=0.8, zorder=zorder)

        # Label
        ax.text(rx + cell_size * 0.3, ry + cell_size * 0.3,
                f'R{rid}', fontsize=6, fontweight='bold', color='k',
                bbox=dict(facecolor='white', alpha=0.85, edgecolor='k',
                          boxstyle='round,pad=0.1', linewidth=0.4),
                zorder=25)

        # Suspicion indicator
        suspicion = rdata.get('suspicion', 0)
        if suspicion > 0.1:
            ax.add_patch(Circle((rx, ry), cell_size * 0.5,
                                facecolor='red', alpha=min(0.4, suspicion * 0.5),
                                edgecolor='none', zorder=zorder-3))

    # Room labels
    ax.text(arena_w * 0.25, arena_h * 0.50, 'Room A', fontsize=8,
            ha='center', va='center', color='gray', fontstyle='italic',
            alpha=0.45)
    ax.text(arena_w * 0.75, arena_h * 0.50, 'Room B', fontsize=8,
            ha='center', va='center', color='gray', fontstyle='italic',
            alpha=0.45)
    ax.text(arena_w * 0.5, arena_h * 0.88, 'Passage', fontsize=6,
            ha='center', va='center', color='gray', alpha=0.4)

    # Title with metrics
    status_parts = [f't = {t:.0f}s', f'Cov = {coverage:.0f}%']
    if leader_id:
        status_parts.append(f'Leader = R{leader_id}')
    if fault_time_1 and t >= fault_time_1:
        if fault_time_2 and t >= fault_time_2:
            status_parts.append('FAULT x2')
        else:
            status_parts.append('FAULT')

    ax.set_title('  |  '.join(status_parts), fontsize=7.5, fontweight='bold',
                 pad=3)

    ax.set_xlim(0, arena_w)
    ax.set_ylim(0, arena_h)
    ax.set_aspect('equal')
    ax.set_xticks([])
    ax.set_yticks([])

    if show_legend:
        from matplotlib.lines import Line2D
        import matplotlib.patches as mp
        legend_elements = [
            mp.Patch(facecolor=COL_LEADER, edgecolor='k', label='Leader'),
            Line2D([0], [0], marker='o', color='w',
                   markerfacecolor='#2980b9', markeredgecolor='k',
                   markersize=6, label='Follower'),
            Line2D([0], [0], marker='s', color='w',
                   markerfacecolor='#f1c40f', markeredgecolor='k',
                   markersize=4, label='Frontier'),
            Rectangle((0,0), 1, 1, facecolor='#ffffff', edgecolor='#ccc',
                      label='Explored'),
            Rectangle((0,0), 1, 1, facecolor='#c8ddf0', edgecolor='#ccc',
                      label='Unexplored'),
            Line2D([0], [0], color=PRED_COL, lw=2, label='Agent belief'),
            Line2D([0], [0], color=CMD_COL, lw=2, ls='dashed',
                   label='Leader command'),
        ]
        ax.legend(handles=legend_elements, loc='lower right', fontsize=5,
                 framealpha=0.9, edgecolor='#ccc', handletextpad=0.3,
                 borderpad=0.3, labelspacing=0.2)


def _segments_intersect(ax1, ay1, ax2, ay2, bx1, by1, bx2, by2):
    """Check if line segment (ax1,ay1)-(ax2,ay2) crosses (bx1,by1)-(bx2,by2)."""
    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    
    d1 = cross((bx1, by1), (bx2, by2), (ax1, ay1))
    d2 = cross((bx1, by1), (bx2, by2), (ax2, ay2))
    d3 = cross((ax1, ay1), (ax2, ay2), (bx1, by1))
    d4 = cross((ax1, ay1), (ax2, ay2), (bx2, by2))
    
    if ((d1 > 0 and d2 < 0) or (d1 < 0 and d2 > 0)) and \
       ((d3 > 0 and d4 < 0) or (d3 < 0 and d4 > 0)):
        return True
    return False


def main():
    # Load snapshot data
    snap_dir = sys.argv[1] if len(sys.argv) > 1 else \
        'experiments/run_20260228_155656_multiroom_1trials_BadLeader/logs/multiroom_reip_BadLeader_t1'
    output_dir = sys.argv[2] if len(sys.argv) > 2 else \
        'paper_docs/Ryker_Kollmyer___UPDATED_PAPER'
    
    snap_path = os.path.join(snap_dir, 'snapshots.json')
    with open(snap_path) as f:
        data = json.load(f)
    
    frames = data['frames']
    arena_w = data['arena_width']
    arena_h = data['arena_height']
    cell_size = data['cell_size']
    walls = data.get('walls', [])
    fault_time_1 = data.get('fault_time_1')
    fault_time_2 = data.get('fault_time_2')
    
    print(f"Loaded {len(frames)} frames from {snap_dir}")
    print(f"Arena: {arena_w}x{arena_h}, cell: {cell_size}")
    print(f"Fault times: {fault_time_1}, {fault_time_2}")
    
    # Key moments for filmstrip
    target_times = [
        8.0,    # Pre-fault: normal exploration
        12.0,   # Just after first fault injection
        18.0,   # Post-impeachment, new leader elected
        60.0,   # Near-complete coverage
    ]
    panel_labels = [
        '(a) Pre-fault',
        '(b) Fault injected',
        '(c) Post-impeachment',
        '(d) Recovery complete',
    ]
    
    key_frames = [find_frame_at_time(frames, t) for t in target_times]
    
    # Create filmstrip figure - 2x2 grid for better readability
    fig, axes = plt.subplots(2, 2, figsize=(7.16, 5.4))
    axes = axes.flatten()
    
    for i, (frame, label) in enumerate(zip(key_frames, panel_labels)):
        show_legend = (i == 3)  # Legend on last panel
        render_frame(axes[i], frame, arena_w, arena_h, cell_size, walls,
                    fault_time_1=fault_time_1, fault_time_2=fault_time_2,
                    show_legend=show_legend)
        # Panel label in top-left corner
        axes[i].text(0.02, 0.95, label, transform=axes[i].transAxes,
                    fontsize=8, fontweight='bold', va='top',
                    bbox=dict(facecolor='white', alpha=0.85, edgecolor='#999',
                              boxstyle='round,pad=0.2'),
                    zorder=30)
    
    fig.tight_layout(pad=0.4, h_pad=0.6, w_pad=0.5)
    
    for ext in ['png', 'pdf']:
        out = os.path.join(output_dir, f'exploration_filmstrip.{ext}')
        fig.savefig(out, bbox_inches='tight')
        print(f"Saved: {out}")
    plt.close(fig)


def comparison():
    """Generate REIP vs Raft comparison filmstrip - the key paper figure."""
    # Find output dir: last arg that is not --compare
    args = [a for a in sys.argv[1:] if a != '--compare']
    output_dir = args[0] if args else 'paper_docs/Ryker_Kollmyer___UPDATED_PAPER'

    # REIP bad_leader snapshots (newer run first, then older)
    reip_paths = [
        'experiments/run_20260228_155656_multiroom_1trials_BadLeader/logs/multiroom_reip_BadLeader_t1/snapshots.json',
        'experiments/run_20260226_221118_multiroom_1trials_all/logs/multiroom_reip_BadLeader_t1/snapshots.json',
    ]
    # Raft bad_leader (use older run - representative failure with 57% plateau)
    raft_paths = [
        'experiments/run_20260226_221118_multiroom_1trials_all/logs/multiroom_raft_BadLeader_t1/snapshots.json',
        'experiments/run_20260228_155656_multiroom_1trials_BadLeader/logs/multiroom_raft_BadLeader_t1/snapshots.json',
    ]

    def load_first(paths):
        for p in paths:
            if os.path.exists(p):
                with open(p) as f:
                    return json.load(f), p
        raise FileNotFoundError(f"None of {paths} found")

    reip_data, reip_src = load_first(reip_paths)
    raft_data, raft_src = load_first(raft_paths)
    print(f"REIP: {len(reip_data['frames'])} frames from {reip_src}")
    print(f"Raft: {len(raft_data['frames'])} frames from {raft_src}")

    arena_w = reip_data['arena_width']
    arena_h = reip_data['arena_height']
    cell_size = reip_data['cell_size']
    walls = reip_data.get('walls', [])
    ft1 = reip_data.get('fault_time_1')
    ft2 = reip_data.get('fault_time_2')

    # Timepoints: pre-fault, fault active, post-recovery, final
    times = [8.0, 15.0, 60.0]
    col_labels = ['Pre-fault\n(t = 8 s)', 'Fault active\n(t = 15 s)',
                  'Late trial\n(t = 60 s)']

    # Layout: 2 rows (REIP, Raft) x 3 columns (timepoints)
    fig, axes = plt.subplots(2, 3, figsize=(7.16, 4.4))

    for col, t_target in enumerate(times):
        for row, (data, ctrl_label) in enumerate([
            (reip_data, 'REIP'),
            (raft_data, 'Raft'),
        ]):
            ax = axes[row][col]
            frame = find_frame_at_time(data['frames'], t_target)
            show_legend = (row == 1 and col == 2)
            render_frame(ax, frame, arena_w, arena_h, cell_size, walls,
                         fault_time_1=ft1, fault_time_2=ft2,
                         show_legend=show_legend)

            # Row label on leftmost column
            if col == 0:
                ax.set_ylabel(ctrl_label, fontsize=10, fontweight='bold',
                              labelpad=5)

    # Column labels at top
    for col, label in enumerate(col_labels):
        axes[0][col].set_title(label, fontsize=8.5, fontweight='bold', pad=8)

    fig.tight_layout(pad=0.3, h_pad=0.5, w_pad=0.3)

    for ext in ['png', 'pdf']:
        out = os.path.join(output_dir, f'reip_vs_raft_filmstrip.{ext}')
        fig.savefig(out, bbox_inches='tight')
        print(f"Saved: {out}")
    plt.close(fig)


if __name__ == '__main__':
    if '--compare' in sys.argv:
        comparison()
    else:
        main()
