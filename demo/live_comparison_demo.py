#!/usr/bin/env python3
"""
Live side-by-side REIP vs Raft comparison demo.

Uses ExperimentRunner from isef_experiments.py DIRECTLY -- the exact same
physics, impairment model, message scheduling, peer relay, contact
resolution, sensor simulation, and coverage tracking that produced every
result in the paper.  This script adds ONLY a Pygame rendering layer on
top.  Zero reimplementation.

Usage: python demo/live_comparison_demo.py [num_robots] [layout]

Controls:
  F           Inject bad_leader on current leader (both sides)
  SHIFT+F     Inject freeze_leader on current leader (both sides)
  C           Clear all faults on both sides
  SPACE       Pause / resume simulation
  R           Reset (kill processes, respawn, start fresh)
  Q / ESC     Quit
"""

import sys
import os
import math
import time
import json
import socket

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

try:
    import pygame
except ImportError:
    print("pygame not installed. Install with: pip install pygame-ce")
    sys.exit(1)

# Import the REAL experiment runner -- all physics, networking, coverage,
# impairment, contact resolution, divider queue, message scheduling etc.
from test.isef_experiments import (
    ExperimentRunner,
    ARENA_LAYOUTS,
    ARENA_WIDTH, ARENA_HEIGHT,
    CELL_SIZE,
    NUM_ROBOTS,
    REIP_SCRIPT,
    RAFT_SCRIPT,
    START_PRESETS,
)
from hardware_fidelity import DEFAULT_ARENA

# ====================== Constants ======================
HEADER_H = 48
PLOT_H = 140
GAP = 10
MARGIN = 10
SIM_RATE = 20  # Match isef_experiments tick rate
SPEED_MULTIPLIER = 5.0  # 5x -- ExperimentRunner sub-steps handle this safely
REIP_COLOR = (0, 52, 204)
RAFT_COLOR = (227, 30, 52)

# Drawing colors (matching visual_sim OG style)
COLOR_UNKNOWN = (158, 195, 230)
COLOR_EXPLORED = (255, 255, 255)
COLOR_WALL = (50, 50, 55)
COLOR_GRID_LINE = (200, 208, 216)
COLOR_FRONTIER = (255, 230, 80)
AGENT_COLORS = [
    (31, 119, 180), (255, 127, 14), (44, 160, 44),
    (214, 39, 40), (148, 103, 189), (140, 86, 75),
    (227, 119, 194), (127, 127, 127),
]
COLOR_LEADER_GOLD = (255, 215, 0)


# ====================== Panel drawing ======================
def draw_panel(surface, runner, wall_segs, fonts, is_reip, label, color,
               fault_active_rids):
    """Render one ExperimentRunner's state onto a Pygame surface."""
    w, h = surface.get_size()
    surface.fill(COLOR_UNKNOWN)

    # Grid dimensions
    cols = ARENA_WIDTH // CELL_SIZE
    rows = ARENA_HEIGHT // CELL_SIZE
    # Scale: fit arena into surface
    scale_x = w / ARENA_WIDTH
    scale_y = h / ARENA_HEIGHT
    scale = min(scale_x, scale_y)
    off_x = (w - ARENA_WIDTH * scale) / 2
    off_y = (h - ARENA_HEIGHT * scale) / 2

    # Draw explored cells
    for (cx, cy) in runner.visited_cells:
        rx = off_x + cx * CELL_SIZE * scale
        ry = off_y + cy * CELL_SIZE * scale
        cw = CELL_SIZE * scale
        ch = CELL_SIZE * scale
        pygame.draw.rect(surface, COLOR_EXPLORED,
                         (int(rx), int(ry), int(cw) + 1, int(ch) + 1))

    # Grid lines
    for col in range(cols + 1):
        x = int(off_x + col * CELL_SIZE * scale)
        pygame.draw.line(surface, COLOR_GRID_LINE,
                         (x, int(off_y)), (x, int(off_y + rows * CELL_SIZE * scale)), 1)
    for row in range(rows + 1):
        y = int(off_y + row * CELL_SIZE * scale)
        pygame.draw.line(surface, COLOR_GRID_LINE,
                         (int(off_x), y), (int(off_x + cols * CELL_SIZE * scale), y), 1)

    # Walls (filled rectangle for divider)
    for seg in wall_segs:
        x1, y1, x2, y2 = seg
        sx1 = int(off_x + x1 * scale)
        sy1 = int(off_y + y1 * scale)
        sx2 = int(off_x + x2 * scale)
        sy2 = int(off_y + y2 * scale)
        pygame.draw.line(surface, COLOR_WALL, (sx1, sy1), (sx2, sy2), 3)

    # Fill between the two divider faces (if multiroom)
    if len(wall_segs) >= 2:
        x_left = wall_segs[0][0]
        x_right = wall_segs[1][0]
        y_top = wall_segs[0][1]
        y_bot = wall_segs[0][3]
        rx = int(off_x + x_left * scale)
        ry = int(off_y + y_top * scale)
        rw = int((x_right - x_left) * scale)
        rh = int((y_bot - y_top) * scale)
        if rw > 0 and rh > 0:
            pygame.draw.rect(surface, COLOR_WALL, (rx, ry, rw, rh))

    # Robots
    for rid in range(1, NUM_ROBOTS + 1):
        if runner.sim_mode == "hardware_clone" and rid in runner.robot_plants:
            plant = runner.robot_plants[rid]
            rx, ry, rtheta = plant.x, plant.y, plant.theta
        elif rid in runner.sim_positions:
            rx, ry, rtheta = runner.sim_positions[rid]
        else:
            continue

        sx = int(off_x + rx * scale)
        sy = int(off_y + ry * scale)
        rc = AGENT_COLORS[(rid - 1) % len(AGENT_COLORS)]
        robot_r = max(6, int(77 * scale))  # Body radius ~77mm

        # Check if leader
        st = runner.last_states_by_robot.get(rid, {})
        is_leader = (st.get('state') == 'leader')

        # Gold ring for leader
        if is_leader:
            pygame.draw.circle(surface, COLOR_LEADER_GOLD, (sx, sy), robot_r + 3, 3)

        # Robot body
        pygame.draw.circle(surface, rc, (sx, sy), robot_r)

        # Heading line
        hx = sx + int(robot_r * 1.3 * math.cos(rtheta))
        hy = sy + int(robot_r * 1.3 * math.sin(rtheta))
        pygame.draw.line(surface, (255, 255, 255), (sx, sy), (hx, hy), 2)

        # ID label
        id_surf = fonts['tiny'].render(str(rid), True, (255, 255, 255))
        surface.blit(id_surf, (sx - id_surf.get_width() // 2,
                                sy - id_surf.get_height() // 2))

        # Fault badge
        if rid in fault_active_rids:
            badge = fonts['tiny'].render("BAD", True, (255, 60, 60))
            surface.blit(badge, (sx - badge.get_width() // 2, sy - robot_r - 14))


def draw_hud(screen, x, y, w, h, runner, label, color, fonts, is_reip):
    """Draw HUD overlay on top of a panel."""
    total_cells = DEFAULT_ARENA.explorable_cell_count()
    cov_pct = len(runner.visited_cells) / total_cells * 100 if total_cells else 0

    # Panel label badge
    tag_surf = fonts['tag'].render(label, True, (255, 255, 255))
    tw, th = tag_surf.get_size()
    badge_x = x + w // 2 - tw // 2 - 10
    badge_y = y + 4
    badge_bg = pygame.Surface((tw + 20, th + 6), pygame.SRCALPHA)
    badge_bg.fill((*color, 210))
    screen.blit(badge_bg, (badge_x, badge_y))
    screen.blit(tag_surf, (badge_x + 10, badge_y + 3))

    # Coverage % (top-left)
    cov_surf = fonts['cov'].render(f"{cov_pct:.0f}%", True, (255, 255, 255))
    cov_bg = pygame.Surface((cov_surf.get_width() + 10, cov_surf.get_height() + 4),
                            pygame.SRCALPHA)
    cov_bg.fill((0, 0, 0, 150))
    screen.blit(cov_bg, (x + 6, y + 6))
    screen.blit(cov_surf, (x + 11, y + 8))

    # Leader + Elections (top-right)
    leader = runner._get_current_leader()
    elec_count = sum(1 for _ in runner.tracked_leaders.values())  # approximate
    info_str = f"L={leader}"
    info_surf = fonts['small'].render(info_str, True, (255, 255, 255))
    info_bg = pygame.Surface((info_surf.get_width() + 10, info_surf.get_height() + 4),
                             pygame.SRCALPHA)
    info_bg.fill((0, 0, 0, 150))
    screen.blit(info_bg, (x + w - info_surf.get_width() - 16, y + 6))
    screen.blit(info_surf, (x + w - info_surf.get_width() - 11, y + 8))

    # Bottom bar: trust (REIP) or "No Trust Layer" (Raft)
    bar_h = 22
    bar_bg = pygame.Surface((w, bar_h), pygame.SRCALPHA)
    bar_bg.fill((0, 0, 0, 140))
    screen.blit(bar_bg, (x, y + h - bar_h))

    if is_reip:
        trusts = [st.get('trust_in_leader', 1.0)
                  for st in runner.last_states_by_robot.values()
                  if st.get('state') != 'leader']
        avg_trust = sum(trusts) / len(trusts) if trusts else 1.0
        tc = (100, 220, 100) if avg_trust > 0.6 else \
             (255, 180, 50) if avg_trust > 0.3 else (255, 80, 80)
        trust_str = f"Avg Trust: {avg_trust:.2f}"
        trust_surf = fonts['small'].render(trust_str, True, tc)
        screen.blit(trust_surf, (x + 8, y + h - bar_h + 4))

        sus_robots = [(rid, st.get('suspicion', 0.0))
                      for rid, st in runner.last_states_by_robot.items()
                      if st.get('suspicion', 0.0) > 0.05]
        if sus_robots:
            sus_parts = [f"R{rid}:{s:.1f}" for rid, s in sorted(sus_robots)]
            sus_str = "Sus: " + " ".join(sus_parts)
            sus_surf = fonts['small'].render(sus_str, True, (255, 200, 100))
            screen.blit(sus_surf, (x + w // 2, y + h - bar_h + 4))
    else:
        raft_surf = fonts['small'].render("No Trust Layer", True, (180, 180, 190))
        screen.blit(raft_surf, (x + 8, y + h - bar_h + 4))


# ====================== Coverage plot ======================
def draw_coverage_plot(screen, rect, runners, labels, colors, elapsed,
                       max_time, fonts, cov_histories, fault_times):
    """Coverage vs time plot (dark theme)."""
    bg = (40, 42, 50)
    pygame.draw.rect(screen, bg, rect, border_radius=6)
    pygame.draw.rect(screen, (70, 72, 80), rect, width=1, border_radius=6)

    inner = rect.inflate(-28, -38)
    inner.y += 16
    inner.h -= 4

    ts = fonts['body'].render("Coverage vs Time", True, (200, 200, 210))
    screen.blit(ts, (rect.x + 14, rect.y + 6))

    # Axes
    pygame.draw.line(screen, (100, 102, 110),
                     (inner.x, inner.bottom), (inner.right, inner.bottom), 1)
    pygame.draw.line(screen, (100, 102, 110),
                     (inner.x, inner.y), (inner.x, inner.bottom), 1)

    # Grid
    for pct in (0, 25, 50, 75, 100):
        py = inner.bottom - int((pct / 100.0) * inner.h)
        pygame.draw.line(screen, (55, 57, 65), (inner.x, py), (inner.right, py), 1)
        lbl = fonts['small'].render(f"{pct}%", True, (120, 120, 130))
        screen.blit(lbl, (inner.x - lbl.get_width() - 4, py - 8))

    # Fault markers
    for ft in fault_times:
        if ft <= elapsed:
            fx = inner.x + int((ft / max_time) * inner.w)
            for dy in range(inner.y, inner.bottom, 6):
                pygame.draw.line(screen, (200, 60, 60),
                                 (fx, dy), (fx, min(dy + 3, inner.bottom)), 1)
            fl = fonts['small'].render(f"F@{ft:.0f}s", True, (200, 80, 80))
            screen.blit(fl, (fx + 3, inner.y - 2))

    # Curves
    for hist, clr in zip(cov_histories, colors):
        if len(hist) < 2:
            continue
        pts = []
        for idx, (t, cov) in enumerate(hist):
            px = inner.x + int((t / max_time) * inner.w)
            py = inner.bottom - int((min(100.0, cov) / 100.0) * inner.h)
            pts.append((px, py))
        if len(pts) >= 2:
            pygame.draw.lines(screen, clr, False, pts, 3)

    # Legend
    lx = rect.right - 240
    ly = rect.y + 8
    for lbl_text, clr in zip(labels, colors):
        pygame.draw.line(screen, clr, (lx, ly + 8), (lx + 20, ly + 8), 3)
        lbl = fonts['small'].render(lbl_text, True, (190, 190, 200))
        screen.blit(lbl, (lx + 26, ly))
        lx += 100

    # Controls
    ctrl = "R: restart   F: fault   C: clear   Space: pause   Q: quit"
    cs = fonts['small'].render(ctrl, True, (90, 90, 100))
    screen.blit(cs, (inner.x, rect.bottom - 18))


# ====================== Overlay ======================
def draw_overlay(screen, fonts, message, sub=""):
    w, h = screen.get_size()
    screen.fill((30, 32, 38))
    big = fonts.get('big') or fonts['tag']
    surf = big.render(message, True, (220, 220, 230))
    screen.blit(surf, (w // 2 - surf.get_width() // 2,
                        h // 2 - surf.get_height() // 2 - 16))
    if sub:
        ss = fonts['body'].render(sub, True, (140, 140, 150))
        screen.blit(ss, (w // 2 - ss.get_width() // 2, h // 2 + 18))
    pygame.display.flip()


# ====================== Bootstrap ======================
def bootstrap_and_wait(runners, timeout=10.0):
    """Mirror isef_experiments.py startup: pump positions + sensor data,
    wait for leader consensus, then send start."""
    for r in runners:
        r._trial_start_wall_time = time.time()
        r._next_position_publish = time.time()

    # Phase 1: pump for 3s
    pump_until = time.time() + 3.0
    while time.time() < pump_until:
        for r in runners:
            r.receive_motor_intents()
            r.receive_states()
            r._flush_scheduled_messages()
            r.send_sensor_feedback()
            r._emit_position_packets()
            r._flush_scheduled_messages()
        time.sleep(0.05)

    # Phase 2: wait for leader consensus
    deadline = time.time() + timeout - 3.0
    leaders_ok = [False] * len(runners)
    while time.time() < deadline and not all(leaders_ok):
        for i, r in enumerate(runners):
            r.receive_motor_intents()
            states = r.receive_states()
            r._flush_scheduled_messages()
            r.send_sensor_feedback()
            r._emit_position_packets()
            r._flush_scheduled_messages()
            if not leaders_ok[i]:
                from collections import Counter
                reports = [st.get('leader_id') for st in
                           (list(states.values()) or list(r.last_states_by_robot.values()))
                           if st.get('leader_id') is not None]
                if reports:
                    counts = Counter(reports)
                    lid, cnt = counts.most_common(1)[0]
                    if lid is not None and cnt >= max(1, NUM_ROBOTS // 2 + 1):
                        leaders_ok[i] = True
                        print(f"  [{'REIP' if i == 0 else 'RAFT'}] Leader consensus: Robot {lid}")
        time.sleep(0.05)

    for i, ok in enumerate(leaders_ok):
        if not ok:
            lbl = "REIP" if i == 0 else "RAFT"
            print(f"  [{lbl}] WARNING: no leader consensus before start")

    # Phase 3: send start
    for r in runners:
        r._send_bootstrap_packets()
    time.sleep(0.2)
    for r in runners:
        r._send_bootstrap_packets()


# ====================== Create / reset runner ======================
def create_runner(port_offset, script, layout="multiroom"):
    """Create an ExperimentRunner wired up for the demo."""
    r = ExperimentRunner(
        output_dir=os.path.join(PROJECT_ROOT, "demo_logs"),
        port_offset=port_offset,
        sim_mode="hardware_clone",
        impairment_preset="hardware_clone_clean",
    )
    r.set_layout(layout)
    return r


def start_runner(runner, script, label):
    """Start robot processes on a runner."""
    fixed = START_PRESETS.get("hardware_clone_20260308")
    print(f"  Starting {label} nodes (port_offset={runner.port_offset})...")
    runner.start_robots(
        script=script,
        extra_args=[],
        seed=42,
        experiment_name=f"demo_{label.lower()}",
        fixed_positions=fixed,
    )


# ====================== Main ======================
def main():
    num_robots_arg = int(sys.argv[1]) if len(sys.argv) > 1 else NUM_ROBOTS
    layout = sys.argv[2] if len(sys.argv) > 2 else "multiroom"

    if layout not in ARENA_LAYOUTS:
        print(f"Unknown layout '{layout}'. Available: {', '.join(ARENA_LAYOUTS.keys())}")
        sys.exit(1)

    print("REIP vs Raft - Live Comparison Demo")
    print(f"  Robots: {num_robots_arg}  Layout: {layout}")
    print(f"  Uses ExperimentRunner directly (identical physics to paper)")
    print(f"  Speed: {SPEED_MULTIPLIER}x  Faults: t=10s R1 + t=30s leader (paper schedule)")
    last_cov_print = 0
    print("=" * 60)

    # ---- Pygame init ----
    pygame.init()

    # Auto-fit window to screen
    disp_info = pygame.display.Info()
    screen_w, screen_h = disp_info.current_w, disp_info.current_h
    usable_w = int(screen_w * 0.95)
    usable_h = int(screen_h * 0.88)

    aspect = ARENA_WIDTH / ARENA_HEIGHT
    max_panel_w = (usable_w - 2 * MARGIN - GAP) // 2
    max_panel_h = usable_h - HEADER_H - MARGIN - PLOT_H - MARGIN
    panel_h = min(max_panel_h, int(max_panel_w / aspect))
    panel_w = int(panel_h * aspect)
    total_w = 2 * panel_w + GAP + 2 * MARGIN
    total_h = HEADER_H + panel_h + MARGIN + PLOT_H + MARGIN
    print(f"  Window: {total_w}x{total_h}  (screen {screen_w}x{screen_h})")

    screen = pygame.display.set_mode((total_w, total_h))
    pygame.display.set_caption("REIP vs Raft - Live Comparison")
    clock = pygame.time.Clock()

    fonts = {
        'big':   pygame.font.SysFont('consolas', 32, bold=True),
        'tag':   pygame.font.SysFont('consolas', 18, bold=True),
        'cov':   pygame.font.SysFont('consolas', 14, bold=True),
        'title': pygame.font.SysFont('consolas', 16, bold=True),
        'body':  pygame.font.SysFont('consolas', 14, bold=True),
        'small': pygame.font.SysFont('consolas', 12),
        'tiny':  pygame.font.SysFont('consolas', 10, bold=True),
    }

    # Panel surfaces
    reip_surface = pygame.Surface((panel_w, panel_h))
    raft_surface = pygame.Surface((panel_w, panel_h))

    left_x = MARGIN
    right_x = MARGIN + panel_w + GAP
    panels_y = HEADER_H
    plot_rect = pygame.Rect(MARGIN, HEADER_H + panel_h + MARGIN,
                            total_w - 2 * MARGIN, PLOT_H)

    wall_segs = list(ARENA_LAYOUTS.get(layout, []))

    # ---- Create runners ----
    draw_overlay(screen, fonts, "CREATING RUNNERS...", "Setting up ExperimentRunner instances")
    reip_runner = create_runner(port_offset=0, script=REIP_SCRIPT, layout=layout)
    raft_runner = create_runner(port_offset=2000, script=RAFT_SCRIPT, layout=layout)
    runners = [reip_runner, raft_runner]
    runner_labels = ["REIP", "RAFT"]
    runner_colors = [REIP_COLOR, RAFT_COLOR]
    runner_scripts = [REIP_SCRIPT, RAFT_SCRIPT]
    runner_surfaces = [reip_surface, raft_surface]
    runner_x_positions = [left_x, right_x]

    # ---- Start processes ----
    draw_overlay(screen, fonts, "STARTING ROBOT NODES...", "Spawning REIP and Raft processes")
    for r, script, label in zip(runners, runner_scripts, runner_labels):
        start_runner(r, script, label)

    # ---- Bootstrap ----
    draw_overlay(screen, fonts, "BOOTSTRAPPING...", "Waiting for leader consensus")
    bootstrap_and_wait(runners, timeout=10.0)
    print("  Ready!\n")

    # ---- State ----
    start_time = time.time()
    paused = False
    run_number = 1
    max_time = 40  # Enough to see full divergence after both faults
    fault_schedule = [
        {"time": 10.0, "target": "leader", "fired": False},   # bad_leader on CURRENT leader at t=10
        {"time": 30.0, "target": "leader", "fired": False},   # bad_leader on CURRENT leader at t=30
    ]
    cov_histories = [[], []]  # [(elapsed, coverage%), ...]
    fault_active_rids = [{}, {}]  # Which robots have active faults per runner

    # Track leader changes for each runner
    prev_leaders = [{}, {}]  # {rid: last_known_leader_id}

    try:
        running = True
        while running:
            # ---- Events ----
            needs_reset = False
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key in (pygame.K_q, pygame.K_ESCAPE):
                        running = False
                    elif event.key == pygame.K_r:
                        needs_reset = True
                    elif event.key == pygame.K_SPACE:
                        paused = not paused
                    elif event.key == pygame.K_f:
                        mods = pygame.key.get_mods()
                        ft = 'freeze_leader' if (mods & pygame.KMOD_SHIFT) else 'bad_leader'
                        for i, r in enumerate(runners):
                            target = r._get_current_leader()
                            r.inject_fault(target, ft)
                            r._flush_scheduled_messages()
                            fault_active_rids[i][target] = True
                            print(f"  [MANUAL] {ft} on [{runner_labels[i]}] R{target}")
                    elif event.key == pygame.K_c:
                        for i, r in enumerate(runners):
                            for rid in range(1, NUM_ROBOTS + 1):
                                r.inject_fault(rid, 'clear')
                            r._flush_scheduled_messages()
                            fault_active_rids[i].clear()
                            print(f"  [MANUAL] Clear all faults on [{runner_labels[i]}]")

            # ---- Reset ----
            if needs_reset:
                run_number += 1
                print(f"\n{'='*60}")
                print(f"  RESET -- starting run #{run_number}")
                print(f"{'='*60}")
                draw_overlay(screen, fonts, "RESETTING...",
                             "Killing old processes and respawning")

                # Stop old processes and close sockets
                for r in runners:
                    r.stop_robots()
                time.sleep(1.0)

                # Close old sockets
                for r in runners:
                    try:
                        r.state_socket.close()
                        r.motor_socket.close()
                        r.pos_socket.close()
                        r.sensor_socket.close()
                        r.fault_socket.close()
                    except Exception:
                        pass

                # Recreate runners fresh
                reip_runner = create_runner(port_offset=0, script=REIP_SCRIPT, layout=layout)
                raft_runner = create_runner(port_offset=2000, script=RAFT_SCRIPT, layout=layout)
                runners = [reip_runner, raft_runner]

                # Start new processes
                draw_overlay(screen, fonts, "STARTING ROBOT NODES...",
                             "Spawning fresh REIP and Raft processes")
                for r, script, label in zip(runners, runner_scripts, runner_labels):
                    start_runner(r, script, label)

                draw_overlay(screen, fonts, "BOOTSTRAPPING...",
                             "Waiting for leader consensus")
                bootstrap_and_wait(runners, timeout=10.0)

                start_time = time.time()
                paused = False
                cov_histories = [[], []]
                fault_active_rids = [{}, {}]
                prev_leaders = [{}, {}]
                for fe in fault_schedule:
                    fe["fired"] = False
                print("  Ready!\n")
                continue

            elapsed = time.time() - start_time

            # ---- Auto fault schedule (matches paper: t=10 on R1, t=30 on leader) ----
            for fe in fault_schedule:
                if not fe["fired"] and elapsed >= fe["time"]:
                    fe["fired"] = True
                    for i, r in enumerate(runners):
                        if fe["target"] == "robot1":
                            rid = 1  # Fixed target: Robot 1
                        else:
                            rid = r._get_current_leader()  # Current leader
                        r.inject_fault(rid, 'bad_leader')
                        r._flush_scheduled_messages()
                        fault_active_rids[i][rid] = True
                        mode = "R1 (fixed)" if fe["target"] == "robot1" else f"R{rid} (leader)"
                        print(f"  [AUTO] bad_leader on [{runner_labels[i]}] {mode} "
                              f"at t={elapsed:.1f}s")

            # ---- Simulation tick (EXACT same as isef_experiments) ----
            if not paused:
                now = time.time()
                for i, r in enumerate(runners):
                    dt = now - (r._last_tick_time if hasattr(r, '_last_tick_time') else now)
                    dt = min(dt, 0.15)
                    dt *= SPEED_MULTIPLIER
                    r._last_tick_time = now

                    r.receive_motor_intents()
                    states = r.receive_states()
                    r._flush_scheduled_messages()
                    r.update_positions(states, dt)
                    r.send_sensor_feedback()
                    r._emit_position_packets()
                    r._flush_scheduled_messages()

                    # Track coverage
                    cov = r.get_coverage()
                    cov_histories[i].append((elapsed, cov))

                    # Track leader changes for display
                    for rid, st in states.items():
                        reported_leader = st.get('leader_id')
                        prev_leader = r.tracked_leaders.get(rid)
                        if prev_leader is not None and reported_leader != prev_leader:
                            pass  # Could count elections here
                        r.tracked_leaders[rid] = reported_leader

            # ---- Draw ----
            screen.fill((30, 32, 38))

            # Header
            hdr = fonts['title'].render(
                "REIP vs Raft  -  Synchronized Fault Comparison",
                True, (220, 220, 230))
            screen.blit(hdr, (total_w // 2 - hdr.get_width() // 2, 10))
            status = "PAUSED" if paused else ""
            info_parts = [f"t={elapsed:.0f}s", f"run #{run_number}"]
            if status:
                info_parts.insert(0, status)
            info = "  |  ".join(info_parts)
            st_surf = fonts['body'].render(info, True, (160, 160, 170))
            screen.blit(st_surf, (total_w - st_surf.get_width() - MARGIN, 30))

            # Draw each panel
            for i, (r, surf, lbl, clr, px) in enumerate(zip(
                    runners, runner_surfaces, runner_labels, runner_colors,
                    runner_x_positions)):
                draw_panel(surf, r, wall_segs, fonts,
                           is_reip=(i == 0), label=lbl, color=clr,
                           fault_active_rids=fault_active_rids[i])
                screen.blit(surf, (px, panels_y))

                # Border
                pygame.draw.rect(screen, clr,
                                 (px - 2, panels_y - 2, panel_w + 4, panel_h + 4),
                                 2, border_radius=3)

                # HUD overlay
                draw_hud(screen, px, panels_y, panel_w, panel_h,
                         r, lbl, clr, fonts, is_reip=(i == 0))

            # Coverage plot
            ft = tuple(fe["time"] for fe in fault_schedule)
            draw_coverage_plot(screen, plot_rect, runners, runner_labels,
                               runner_colors, elapsed, max_time, fonts,
                               cov_histories, ft)

            # Restart hint
            # Log coverage every 5s for debugging
            if int(elapsed) % 5 == 0 and int(elapsed) != last_cov_print and int(elapsed) > 0:
                last_cov_print = int(elapsed)
                c0 = runners[0].get_coverage()
                c1 = runners[1].get_coverage()
                print(f"  t={elapsed:.0f}s  REIP={c0:.1f}%  RAFT={c1:.1f}%  gap={c0-c1:+.1f}%")

            if elapsed > 30:
                hint = fonts['body'].render(
                    "Press R to restart", True, (120, 120, 130))
                screen.blit(hint, (total_w // 2 - hint.get_width() // 2,
                                    total_h - 20))

            pygame.display.flip()
            clock.tick(SIM_RATE)

    finally:
        print("\nStopping robot processes...")
        for r in runners:
            r.stop_robots()
            try:
                r.state_socket.close()
                r.motor_socket.close()
            except Exception:
                pass
        pygame.quit()
        print("Done.")


if __name__ == "__main__":
    main()
