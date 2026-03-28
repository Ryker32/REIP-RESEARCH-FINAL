"""Generate sim_environment.png - IEEE-quality two-panel comparison.

Left:  Clean operation - coordinated exploration, aligned arrows
Right: Byzantine fault active - leader commands vs agent beliefs diverge

Arrow logic:
  Clean panel:
    GREEN solid = next_waypoint (A*-routed, NEVER crosses walls)
    No red arrows - everything is aligned in clean mode

  Fault panel:
    RED  dashed = commanded_target  (bad leader's order -> explored cells)
    GREEN solid = predicted_target  (agent's own frontier belief -> unexplored)
    The visible divergence between these IS how REIP detects the fault.
"""

import json, math, os, glob
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.colors import ListedColormap

# ---- IEEE style ----
plt.rcParams.update({
    'font.family': 'serif',
    'font.serif': ['Times New Roman', 'Times', 'DejaVu Serif'],
    'font.size': 9,
    'axes.linewidth': 0.5,
})

# ---- Colours ----
COL_OBSTACLE = '#222222'
COL_UNKNOWN  = '#9ec3e6'
COL_FREE     = '#ffffff'
COL_FRONTIER = '#f1c40f'
COL_LEADER   = 'gold'
FOL_COLS     = ['#3498db', '#e74c3c', '#2ecc71', '#9b59b6', '#f39c12']
COL_SENSE    = '#7f8c8d'
CMD_COL      = '#c0392b'       # dark red  - leader command
PRED_COL     = '#27ae60'       # green     - agent belief / navigation
GRID_COL     = '#cccccc'

# Arrow/marker sizes
VEC_LW   = 3.0
VEC_ALP  = 0.92
ARROW_CELLS = 1.8
MS_LEAD  = 12
MS_FOL   = 11
MUT_SC   = 14

cmap_grid = ListedColormap([COL_OBSTACLE, COL_UNKNOWN, COL_FREE])

MIN_ARROW_DIST = 20  # px - suppress arrows when agent is almost at target


# ============================================================
# Geometry helpers
# ============================================================

def _vec(rx, ry, tx, ty, length):
    dx, dy = tx - rx, ty - ry
    mag = math.hypot(dx, dy)
    if mag < MIN_ARROW_DIST:
        return None
    return (dx / mag * length, dy / mag * length)


def _seg_intersect(ax, ay, bx, by, cx, cy, dx, dy):
    def cross(ox, oy, px, py, qx, qy):
        return (px - ox) * (qy - oy) - (py - oy) * (qx - ox)
    d1 = cross(cx, cy, dx, dy, ax, ay)
    d2 = cross(cx, cy, dx, dy, bx, by)
    d3 = cross(ax, ay, bx, by, cx, cy)
    d4 = cross(ax, ay, bx, by, dx, dy)
    if ((d1 > 0 and d2 < 0) or (d1 < 0 and d2 > 0)) and \
       ((d3 > 0 and d4 < 0) or (d3 < 0 and d4 > 0)):
        return True
    return False


def _crosses_wall(rx, ry, tx, ty, walls):
    for x1, y1, x2, y2 in walls:
        if _seg_intersect(rx, ry, tx, ty, x1, y1, x2, y2):
            return True
    return False


# ============================================================
# Scoring / frame picking
# ============================================================

def _min_pairwise(robots):
    pts = [(r['x'], r['y']) for r in robots.values()]
    dmin = float('inf')
    for i in range(len(pts)):
        for j in range(i + 1, len(pts)):
            d = math.hypot(pts[i][0] - pts[j][0], pts[i][1] - pts[j][1])
            dmin = min(dmin, d)
    return dmin


def _bb_spread(robots):
    xs = [r['x'] for r in robots.values()]
    ys = [r['y'] for r in robots.values()]
    return (max(xs) - min(xs)) * (max(ys) - min(ys))


def pick_clean(data, t_min=3.0, t_max=8.0):
    """Best spread + agents well separated + ALL agents have visible arrows.

    Arrow visibility uses predicted_target distance (the arrow we actually
    draw), falling back to next_waypoint.
    """
    best, best_sc = None, -1
    for fr in data['frames']:
        if not (t_min <= fr['t'] <= t_max):
            continue
        r = fr['robots']
        md = _min_pairwise(r)
        if md < 80:                         # reject frames with piled-up agents
            continue
        sp = _bb_spread(r)

        # Count agents with arrows visible - predicted_target or next_waypoint
        n_with_arrow = 0
        for rd in r.values():
            rx, ry = rd['x'], rd['y']
            pred = rd.get('predicted_target')
            wp   = rd.get('next_waypoint')
            has = False
            if pred and math.hypot(pred[0] - rx, pred[1] - ry) > MIN_ARROW_DIST:
                has = True
            elif wp and math.hypot(wp[0] - rx, wp[1] - ry) > MIN_ARROW_DIST:
                has = True
            if has:
                n_with_arrow += 1

        n_total = len(r)
        # Strongly prefer frames where ALL agents have visible arrows
        arrow_bonus = 10000 if n_with_arrow == n_total else n_with_arrow * 100
        sc = arrow_bonus * sp * md
        if sc > best_sc:
            best_sc = sc
            best = fr
    return best


def pick_fault(data, t_min=10.5, t_max=13.0):
    """During ACTIVE fault: most bad commands + most pred-divergence + spread.

    We want frames where:
      - The original bad leader (R1) is still active
      - All followers are commanded to EXPLORED cells
      - Followers' predicted_target is in UNEXPLORED cells (max divergence)
      - Agents are reasonably spread out
    """
    cell = data['cell_size']
    best, best_sc = None, -1
    for fr in data['frames']:
        if not (t_min <= fr['t'] <= t_max):
            continue
        r = fr['robots']
        md = _min_pairwise(r)
        if md < 50:
            continue
        sp = _bb_spread(r)
        visited = set(tuple(c) for c in fr.get('visited_cells', []))

        n_bad = 0
        n_pred_unex = 0
        for rd in r.values():
            if rd.get('state') == 'leader':
                continue
            cmd = rd.get('commanded_target') or rd.get('target')
            pred = rd.get('predicted_target')
            if cmd:
                tcx, tcy = int(cmd[0] // cell), int(cmd[1] // cell)
                if (tcx, tcy) in visited:
                    n_bad += 1
            if pred:
                pcx, pcy = int(pred[0] // cell), int(pred[1] // cell)
                if (pcx, pcy) not in visited:
                    n_pred_unex += 1

        # Score: heavily reward bad commands + pred-to-unexplored divergence
        sc = (n_bad * 200 + n_pred_unex * 300) * (1 + sp * 0.0001) * (1 + md * 0.01)
        if sc > best_sc:
            best_sc = sc
            best = fr
    return best


# ============================================================
# Rendering
# ============================================================

def _draw_arrow(ax, rx, ry, tx, ty, arrow_len, color, ls, zorder,
                walls=None):
    """Draw directional arrow at standard length.

    If *walls* is provided, suppress arrows whose tip crosses a wall segment.
    """
    dist = math.hypot(tx - rx, ty - ry)
    if dist < MIN_ARROW_DIST:
        return
    dx, dy = (tx - rx) / dist * arrow_len, (ty - ry) / dist * arrow_len
    tip_x, tip_y = rx + dx, ry + dy

    if walls and _crosses_wall(rx, ry, tip_x, tip_y, walls):
        return  # suppress arrow that would visually cross a wall

    ax.annotate(
        '', xy=(tip_x, tip_y), xytext=(rx, ry),
        arrowprops=dict(arrowstyle='-|>', color=color, lw=VEC_LW,
                        alpha=VEC_ALP, linestyle=ls, mutation_scale=MUT_SC),
        zorder=zorder)


def render_panel(ax, snap, frame, title, subtitle=None,
                 mode='clean'):
    """
    mode='clean': show only green arrows (next_waypoint, wall-safe)
    mode='fault': show red (commanded) + green (predicted) arrows
    """
    aw = snap['arena_width']
    ah = snap['arena_height']
    cell = snap['cell_size']
    walls = snap.get('walls', [])
    cols, rows = aw // cell, ah // cell
    arrow_len = cell * ARROW_CELLS

    # Grid
    visited = set(tuple(c) for c in frame.get('visited_cells', []))
    grid = np.full((cols, rows), -1, dtype=int)
    for cx, cy in visited:
        if 0 <= cx < cols and 0 <= cy < rows:
            grid[cx, cy] = 0

    frontiers = []
    for cx in range(cols):
        for cy in range(rows):
            if grid[cx, cy] != -1:
                continue
            for ddx, ddy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                nx, ny = cx + ddx, cy + ddy
                if 0 <= nx < cols and 0 <= ny < rows and grid[nx, ny] == 0:
                    frontiers.append(((cx + 0.5) * cell, (cy + 0.5) * cell))
                    break

    ax.imshow(grid.T, cmap=cmap_grid, origin='lower', vmin=-2.5, vmax=0.5,
              extent=[0, aw, 0, ah], interpolation='nearest')

    for i in range(cols + 1):
        ax.axvline(i * cell, color=GRID_COL, lw=0.4, alpha=0.35, zorder=1)
    for j in range(rows + 1):
        ax.axhline(j * cell, color=GRID_COL, lw=0.4, alpha=0.35, zorder=1)

    for x1, y1, x2, y2 in walls:
        ax.plot([x1, x2], [y1, y2], color=COL_OBSTACLE, lw=4.0,
                solid_capstyle='butt', zorder=5)

    if frontiers:
        fx, fy = zip(*frontiers)
        ax.scatter(fx, fy, marker='s', s=40, c=COL_FRONTIER,
                   edgecolors='k', linewidths=0.4, zorder=4)

    # --- robots ---
    robots = frame['robots']
    leader_rid = None
    for rid_s, rd in robots.items():
        if rd.get('state') == 'leader':
            leader_rid = int(rid_s)

    sense_r = 200
    for rid_s in sorted(robots.keys()):
        rd = robots[rid_s]
        rid = int(rid_s)
        rx, ry = rd['x'], rd['y']
        is_leader = (rid == leader_rid)

        # sensing overlay
        ax.add_patch(plt.Circle((rx, ry), sense_r,
                                color=(0.2, 0.9, 0.2, 0.08), zorder=2))
        ax.add_patch(plt.Circle((rx, ry), sense_r, color=COL_SENSE,
                                fill=False, ls='dashed', lw=0.8, alpha=0.45,
                                zorder=6))

        # --- arrows ---
        wp   = rd.get('next_waypoint')
        cmd  = rd.get('commanded_target') or rd.get('target')
        pred = rd.get('predicted_target')

        if is_leader:
            # Leader navigation direction only (green)
            src = pred or wp
            if src:
                _draw_arrow(ax, rx, ry, src[0], src[1],
                            arrow_len, PRED_COL, 'solid', 12,
                            walls=walls if mode == 'clean' else None)
        else:
            if mode == 'clean':
                # Clean: prefer predicted_target (long, full-length arrow
                # showing where agent wants to go).  Fall back to
                # next_waypoint if the predicted arrow would cross a wall.
                drawn = False
                if pred:
                    d = math.hypot(pred[0] - rx, pred[1] - ry)
                    if d >= MIN_ARROW_DIST:
                        dx = (pred[0] - rx) / d * arrow_len
                        dy = (pred[1] - ry) / d * arrow_len
                        if not _crosses_wall(rx, ry, rx + dx, ry + dy, walls):
                            _draw_arrow(ax, rx, ry, pred[0], pred[1],
                                        arrow_len, PRED_COL, 'solid', 13)
                            drawn = True
                if not drawn and wp:
                    # Fallback: next_waypoint (always wall-safe direction)
                    _draw_arrow(ax, rx, ry, wp[0], wp[1],
                                arrow_len, PRED_COL, 'solid', 13,
                                walls=walls)
            elif mode == 'fault':
                # Fault: red = commanded target, green = predicted target
                # Red dashed - leader's order (to explored cell)
                if cmd:
                    _draw_arrow(ax, rx, ry, cmd[0], cmd[1],
                                arrow_len, CMD_COL, 'dashed', 11)
                # Green solid - agent's own belief (to frontier)
                if pred:
                    _draw_arrow(ax, rx, ry, pred[0], pred[1],
                                arrow_len, PRED_COL, 'solid', 13)

        # robot marker
        if is_leader:
            ax.plot(rx, ry, 's', ms=MS_LEAD, color=COL_LEADER, mec='k',
                    mew=1.4, zorder=15)
            ax.add_patch(plt.Circle((rx, ry), cell * 0.9, color=COL_LEADER,
                                    fill=False, lw=2.5, alpha=0.9, zorder=14))
        else:
            ci = (rid - 1) % len(FOL_COLS)
            ax.plot(rx, ry, 'o', ms=MS_FOL, color=FOL_COLS[ci],
                    mec='k', mew=1.0, zorder=15)

        # label
        ax.text(rx + cell * 0.3, ry + cell * 0.3, f'R{rid}',
                fontsize=7.5, fontweight='bold', color='k',
                bbox=dict(facecolor='white', alpha=0.9, edgecolor='k',
                          boxstyle='round,pad=0.12', linewidth=0.5),
                zorder=16)

    # room labels
    ax.text(aw * 0.25, ah * 0.50, 'Room A', fontsize=11, ha='center',
            va='center', color='gray', fontstyle='italic', alpha=0.5)
    ax.text(aw * 0.75, ah * 0.50, 'Room B', fontsize=11, ha='center',
            va='center', color='gray', fontstyle='italic', alpha=0.5)
    ax.text(aw * 0.5, ah * 0.88, 'Passage', fontsize=8, ha='center',
            va='center', color='gray', alpha=0.45)

    ax.set_title(title, fontsize=10, fontweight='bold', pad=14)
    if subtitle:
        ax.text(0.5, 1.01, subtitle, transform=ax.transAxes,
                fontsize=6.5, ha='center', va='bottom', color='#444',
                fontstyle='italic')

    ax.set_xlim(0, aw)
    ax.set_ylim(0, ah)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_aspect('equal')


# ============================================================
# Main
# ============================================================

def _find_best_snapshot(pattern, picker_fn, **picker_kw):
    """Pick the best seed from multiple snapshot files."""
    paths = sorted(glob.glob(pattern))
    if not paths:
        return None, None
    best_path, best_frame, best_score = None, None, -1
    for p in paths:
        d = json.load(open(p))
        fr = picker_fn(d, **picker_kw)
        if fr is None:
            continue
        sp = _bb_spread(fr['robots'])
        md = _min_pairwise(fr['robots'])
        sc = sp * md
        if sc > best_score:
            best_score = sc
            best_path = p
            best_frame = fr
    return best_path, best_frame


def main():
    out_dir = 'paper_docs/Ryker_Kollmyer___UPDATED_PAPER/'
    os.makedirs(out_dir, exist_ok=True)

    fresh = 'experiments/viz_fresh/logs'

    # ---- Find best clean snapshot ----
    clean_path, clean_frame = _find_best_snapshot(
        os.path.join(fresh, 'viz_clean_*/snapshots.json'), pick_clean)
    if clean_path is None:
        clean_path = 'experiments/viz_trial_fixed/logs/viz_reip_clean/snapshots.json'
        d = json.load(open(clean_path))
        clean_frame = pick_clean(d)
    print(f'Clean: {clean_path}')
    with open(clean_path) as f:
        clean_data = json.load(f)

    # ---- Find best fault snapshot ----
    fault_path, fault_frame = _find_best_snapshot(
        os.path.join(fresh, 'viz_bad_*/snapshots.json'), pick_fault)
    if fault_path is None:
        fault_path = ('experiments/run_20260228_155656_multiroom_1trials_BadLeader'
                      '/logs/multiroom_reip_BadLeader_t1/snapshots.json')
        d = json.load(open(fault_path))
        fault_frame = pick_fault(d)
    print(f'Fault: {fault_path}')
    with open(fault_path) as f:
        fault_data = json.load(f)

    if clean_frame is None:
        clean_frame = clean_data['frames'][len(clean_data['frames']) // 4]
    if fault_frame is None:
        fault_frame = fault_data['frames'][len(fault_data['frames']) // 3]

    has_wp = any(
        r.get('next_waypoint') is not None
        for fr in clean_data['frames'][:5]
        for r in fr['robots'].values()
    )
    print(f'next_waypoint data: {"YES" if has_wp else "NO (fallback to raw target)"}')

    ct, cc = clean_frame['t'], clean_frame['coverage']
    ft, fc = fault_frame['t'], fault_frame['coverage']
    cell = fault_data['cell_size']
    visited_f = set(tuple(c) for c in fault_frame.get('visited_cells', []))

    n_bad = 0
    n_pred_unex = 0
    for rd in fault_frame['robots'].values():
        if rd.get('state') == 'leader':
            continue
        cmd = rd.get('commanded_target') or rd.get('target')
        pred = rd.get('predicted_target')
        if cmd:
            tcx, tcy = int(cmd[0] // cell), int(cmd[1] // cell)
            if (tcx, tcy) in visited_f:
                n_bad += 1
        if pred:
            pcx, pcy = int(pred[0] // cell), int(pred[1] // cell)
            if (pcx, pcy) not in visited_f:
                n_pred_unex += 1

    print(f'  Clean: t={ct:.1f}s  cov={cc:.0f}%  '
          f'min_d={_min_pairwise(clean_frame["robots"]):.0f}  '
          f'spread={_bb_spread(clean_frame["robots"]):.0f}')
    print(f'  Fault: t={ft:.1f}s  cov={fc:.0f}%  '
          f'min_d={_min_pairwise(fault_frame["robots"]):.0f}  '
          f'spread={_bb_spread(fault_frame["robots"]):.0f}  '
          f'bad_cmd={n_bad}/4  pred_unex={n_pred_unex}/4')

    # ---- Build figure ----
    fig, (ax_c, ax_f) = plt.subplots(1, 2, figsize=(7.16, 3.0))

    render_panel(ax_c, clean_data, clean_frame,
                 '(a) Clean Operation',
                 subtitle=(f't = {ct:.0f}s  |  N = 5  |  '
                           f'Coverage = {cc:.0f}%  |  Coordinated exploration'),
                 mode='clean')

    render_panel(ax_f, fault_data, fault_frame,
                 '(b) Byzantine Leader Fault Active',
                 subtitle=(f't = {ft:.0f}s  |  Fault at t = 10 s  |  '
                           f'Coverage = {fc:.0f}%  |  Arrows diverge'),
                 mode='fault')

    legend_elements = [
        mpatches.Patch(facecolor=COL_LEADER, edgecolor='k', label='Leader'),
        plt.Line2D([0], [0], marker='o', color='w',
                   markerfacecolor=FOL_COLS[0], markeredgecolor='k',
                   markersize=8, label='Follower'),
        mpatches.Patch(facecolor=COL_FREE, edgecolor='k', label='Explored'),
        mpatches.Patch(facecolor=COL_UNKNOWN, edgecolor='k', label='Unexplored'),
        mpatches.Patch(facecolor=COL_FRONTIER, edgecolor='k', label='Frontier'),
        plt.Line2D([0], [0], color=PRED_COL, lw=2.5, label='Agent belief'),
        plt.Line2D([0], [0], color=CMD_COL, lw=2.5, ls='dashed',
                   label='Leader command'),
    ]
    fig.legend(handles=legend_elements, loc='lower center', ncol=4,
               fontsize=7, framealpha=0.95, edgecolor='k',
               bbox_to_anchor=(0.5, -0.04))

    fig.tight_layout(rect=[0, 0.10, 1, 1])

    for ext in ('.png', '.pdf'):
        p = os.path.join(out_dir, f'sim_environment{ext}')
        fig.savefig(p, dpi=300, bbox_inches='tight', facecolor='white')
        print(f'Saved: {p}')
    plt.close(fig)


if __name__ == '__main__':
    main()
