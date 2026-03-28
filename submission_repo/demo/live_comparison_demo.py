#!/usr/bin/env python3
"""
Live side-by-side REIP vs Raft comparison demo.

Imports VisualSimulation from test/visual_sim.py -- the SAME code that
runs the regular sim -- and creates two embedded instances on separate
UDP port ranges (one for REIP nodes, one for Raft nodes).

Each panel is a real VisualSimulation running real robot node
subprocesses.  All physics, coverage tracking, trust computation, and
frontier assignment come from the actual reip_node.py / raft_node.py
code, NOT re-implemented here.

Usage: python demo/live_comparison_demo.py [num_robots] [layout]

Controls:
  F           Inject bad_leader on current leader (both sides)
  SHIFT+F     Inject freeze_leader on current leader (both sides)
  C           Clear all faults on both sides
  SPACE       Pause / resume simulation
  UP / DOWN   Speed up / slow down
  R           Reset (kill processes, respawn, start fresh)
  Q / ESC     Quit
"""

import sys
import os
import time

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

try:
    import pygame
except ImportError:
    print("pygame not installed. Install with: pip install pygame-ce")
    sys.exit(1)

# Import the real simulation class -- all physics, networking, coverage,
# drawing, A* pathfinding, stuck detection etc. live here.
from test.visual_sim import (
    VisualSimulation,
    ARENA_LAYOUTS, SIM_RATE,
    WINDOW_WIDTH as VS_W,
    WINDOW_HEIGHT as VS_H,
)

# ====================== Hardware-captured starting positions ======================
# All robots in Room A (left of divider), from 2026-03-08 hardware trial
HARDWARE_STARTS = {
    1: (185.9, 382.1, 0.188),
    2: (792.9, 709.2, 2.255),
    3: (318.8, 147.4, 0.736),
    4: (768.7, 219.2, 1.577),
    5: (182.4, 709.9, 0.410),
}

# ====================== Demo window constants ======================
HEADER_H = 48
PLOT_H = 170
GAP = 12
MARGIN = 12
NUM_ROBOTS = 5
REIP_COLOR = (0, 52, 204)
RAFT_COLOR = (227, 30, 52)


# ====================== Coverage history plot ======================
def _draw_coverage_plot(screen, rect, panels, labels, colors, elapsed, max_time, fonts):
    """Draw a coverage-vs-time plot at the bottom of the window."""
    bg = (245, 242, 235)
    pygame.draw.rect(screen, bg, rect, border_radius=6)
    pygame.draw.rect(screen, (80, 80, 80), rect, width=2, border_radius=6)

    inner = rect.inflate(-28, -38)
    inner.y += 16
    inner.h -= 4

    ts = fonts['body'].render("Coverage History", True, (30, 30, 30))
    screen.blit(ts, (rect.x + 14, rect.y + 6))

    # Axes
    pygame.draw.line(screen, (80, 80, 80),
                     (inner.x, inner.bottom), (inner.right, inner.bottom), 2)
    pygame.draw.line(screen, (80, 80, 80),
                     (inner.x, inner.y), (inner.x, inner.bottom), 2)

    # Grid lines
    for pct in (0, 25, 50, 75, 100):
        y = inner.bottom - int((pct / 100.0) * inner.h)
        pygame.draw.line(screen, (215, 219, 224),
                         (inner.x, y), (inner.right, y), 1)
        lbl = fonts['small'].render(f"{pct}%", True, (140, 140, 140))
        screen.blit(lbl, (inner.x - lbl.get_width() - 4, y - 8))

    # Curves
    for panel, color in zip(panels, colors):
        hist = panel._cov_history
        if len(hist) < 2:
            continue
        pts = []
        for idx, cov in enumerate(hist):
            x = inner.x + int((idx / max(1, max_time * SIM_RATE)) * inner.w)
            y = inner.bottom - int((min(100.0, cov) / 100.0) * inner.h)
            pts.append((x, y))
        if len(pts) >= 2:
            pygame.draw.lines(screen, color, False, pts, 3)

    # Legend
    lx = rect.right - 260
    ly = rect.y + 8
    for label, color in zip(labels, colors):
        pygame.draw.line(screen, color, (lx, ly + 8), (lx + 24, ly + 8), 4)
        lbl = fonts['small'].render(label, True, (30, 30, 30))
        screen.blit(lbl, (lx + 30, ly))
        lx += 110

    # Controls
    ctrl = "R: restart  F: fault  C: clear  Space: pause  Up/Down: speed  Q: quit"
    cs = fonts['small'].render(ctrl, True, (160, 160, 160))
    screen.blit(cs, (inner.x, rect.bottom - 18))

    tm = fonts['small'].render(f"t={elapsed:.0f}s", True, (120, 120, 120))
    screen.blit(tm, (inner.right - tm.get_width(), rect.bottom - 18))


# ====================== Overlay helpers ======================
def _draw_overlay(screen, fonts, message, sub=""):
    w, h = screen.get_size()
    overlay = pygame.Surface((w, h), pygame.SRCALPHA)
    overlay.fill((20, 22, 28, 190))
    screen.blit(overlay, (0, 0))
    big = pygame.font.SysFont('consolas', 36, bold=True)
    surf = big.render(message, True, (255, 255, 255))
    screen.blit(surf, (w // 2 - surf.get_width() // 2,
                        h // 2 - surf.get_height() // 2 - 20))
    if sub:
        ss = fonts['body'].render(sub, True, (180, 180, 190))
        screen.blit(ss, (w // 2 - ss.get_width() // 2,
                          h // 2 + 20))
    pygame.display.flip()


# ====================== Main ======================
def main():
    num_robots = int(sys.argv[1]) if len(sys.argv) > 1 else NUM_ROBOTS
    layout = sys.argv[2] if len(sys.argv) > 2 else "multiroom"

    if layout not in ARENA_LAYOUTS:
        print(f"Unknown layout '{layout}'. "
              f"Available: {', '.join(ARENA_LAYOUTS.keys())}")
        sys.exit(1)

    # Use real hardware starts for multiroom
    if layout == "multiroom" and num_robots <= len(HARDWARE_STARTS):
        starts = {i: HARDWARE_STARTS[i] for i in range(1, num_robots + 1)}
    else:
        starts = None  # Let VisualSimulation use its defaults

    print("REIP vs Raft - Live Comparison Demo")
    print(f"  Robots: {num_robots}  Layout: {layout}")
    print(f"  Faults: t=10s on R1, t=30s on leader")
    print("=" * 55)

    # ---- Create two real VisualSimulation instances ----
    # REIP on default ports (port_base=0)
    # Raft on offset ports (port_base=2000)
    reip_sim = VisualSimulation(
        num_robots, layout,
        port_base=0,
        robot_script=os.path.join("robot", "reip_node.py"),
        start_positions=starts,
        embedded=True,
    )
    raft_sim = VisualSimulation(
        num_robots, layout,
        port_base=2000,
        robot_script=os.path.join("robot", "baselines", "raft_node.py"),
        start_positions=starts,
        embedded=True,
    )
    panels = [reip_sim, raft_sim]
    labels = ["REIP", "RAFT"]
    colors = [REIP_COLOR, RAFT_COLOR]

    # Coverage history (separate from VisualSimulation's internal state)
    for p in panels:
        p._cov_history = []

    # ---- Pygame window ----
    pygame.init()
    # Each panel renders to VS_W x VS_H.  Scale to fit side-by-side.
    panel_w = (1920 - 2 * MARGIN - GAP) // 2
    panel_h_max = 750
    # Maintain aspect ratio
    aspect = VS_W / VS_H
    panel_h = min(panel_h_max, int(panel_w / aspect))
    panel_w = int(panel_h * aspect)
    total_w = 2 * panel_w + GAP + 2 * MARGIN
    total_h = HEADER_H + panel_h + MARGIN + PLOT_H + MARGIN
    screen = pygame.display.set_mode((total_w, total_h))
    pygame.display.set_caption("REIP vs Raft - Live Comparison")
    clock = pygame.time.Clock()

    fonts = {
        'title': pygame.font.SysFont('consolas', 16, bold=True),
        'body':  pygame.font.SysFont('consolas', 14, bold=True),
        'small': pygame.font.SysFont('consolas', 12),
    }

    # Panel blit positions
    left_x = MARGIN
    right_x = MARGIN + panel_w + GAP
    panels_y = HEADER_H
    plot_rect = pygame.Rect(MARGIN, HEADER_H + panel_h + MARGIN,
                            total_w - 2 * MARGIN, PLOT_H)

    # ---- Startup ----
    screen.fill((235, 237, 240))
    _draw_overlay(screen, fonts, "STARTING ROBOT NODES...",
                  "Spawning REIP and Raft processes")
    print("\n  Starting robot nodes...")
    for p in panels:
        p.start_robot_processes()
    time.sleep(1.5)
    _draw_overlay(screen, fonts, "BOOTSTRAPPING...",
                  "Sending initial positions")
    # Send a few rounds of positions so nodes initialise
    for _ in range(10):
        for p in panels:
            p.send_positions()
        time.sleep(0.05)
    print("  Ready!\n")

    # ---- State ----
    sim_rate = SIM_RATE
    start_time = time.time()
    paused = False
    run_number = 1
    max_time = 120
    fault_schedule = [
        {"time": 10.0, "target": "R1",     "fired": False},
        {"time": 30.0, "target": "leader", "fired": False},
    ]

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
                        for p in panels:
                            target = p.current_leader_id or 1
                            p.inject_fault(target, ft)
                    elif event.key == pygame.K_c:
                        for p in panels:
                            p.clear_all_faults()
                    elif event.key == pygame.K_UP:
                        sim_rate = min(60, sim_rate + 5)
                    elif event.key == pygame.K_DOWN:
                        sim_rate = max(5, sim_rate - 5)

            # ---- Reset ----
            if needs_reset:
                run_number += 1
                print(f"\n{'='*55}")
                print(f"  RESET -- starting run #{run_number}")
                print(f"{'='*55}")
                _draw_overlay(screen, fonts, "RESETTING...",
                              "Killing old processes and respawning")
                for p in panels:
                    p.reset(start_positions=starts)
                    p._cov_history = []
                time.sleep(1.0)
                _draw_overlay(screen, fonts, "BOOTSTRAPPING...",
                              "Sending initial positions")
                for _ in range(10):
                    for p in panels:
                        p.send_positions()
                    time.sleep(0.05)
                start_time = time.time()
                paused = False
                for fe in fault_schedule:
                    fe["fired"] = False
                print("  Ready!\n")
                continue

            elapsed = time.time() - start_time

            # ---- Auto fault schedule ----
            for fe in fault_schedule:
                if not fe["fired"] and elapsed >= fe["time"]:
                    fe["fired"] = True
                    for p, lbl in zip(panels, labels):
                        if fe["target"] == "leader":
                            rid = p.current_leader_id or 1
                        else:
                            rid = 1
                        p.inject_fault(rid, 'bad_leader')
                        print(f"  [AUTO] bad_leader on [{lbl}] R{rid} "
                              f"at t={elapsed:.1f}s")

            # ---- Simulation step ----
            if not paused:
                for p in panels:
                    p.update_sim_robots()
                    p.send_positions()
                    p.receive_states()
                    # Record coverage for the bottom plot
                    total = ((2000 // 125) * (1500 // 125))
                    cov_pct = len(p.visited_cells) / total * 100 if total else 0
                    p._cov_history.append(cov_pct)

            # ---- Draw ----
            screen.fill((235, 237, 240))

            # Header
            hdr = fonts['title'].render(
                "REIP vs Raft - Synchronized Fault Comparison",
                True, (30, 30, 30))
            screen.blit(hdr, (total_w // 2 - hdr.get_width() // 2, 14))
            status = "PAUSED" if paused else "RUNNING"
            info = (f"{status}  |  t={elapsed:.0f}s  "
                    f"|  rate={sim_rate} Hz  |  run #{run_number}")
            st_surf = fonts['body'].render(info, True, (120, 120, 120))
            screen.blit(st_surf, (total_w - st_surf.get_width() - MARGIN, 16))

            # Each VisualSimulation draws to its own surface
            for p in panels:
                p.draw()

            # Scale and blit each panel surface into the window
            scaled_reip = pygame.transform.smoothscale(
                reip_sim.screen, (panel_w, panel_h))
            scaled_raft = pygame.transform.smoothscale(
                raft_sim.screen, (panel_w, panel_h))
            screen.blit(scaled_reip, (left_x, panels_y))
            screen.blit(scaled_raft, (right_x, panels_y))

            # Panel labels overlaid on top
            for lbl, color, x in [("REIP", REIP_COLOR, left_x),
                                    ("RAFT", RAFT_COLOR, right_x)]:
                tag = fonts['title'].render(lbl, True, color)
                screen.blit(tag, (x + panel_w // 2 - tag.get_width() // 2,
                                  panels_y + 4))

            # Coverage plot
            _draw_coverage_plot(screen, plot_rect, panels, labels, colors,
                                elapsed, max_time, fonts)

            # Restart hint
            if elapsed > 45:
                hint = fonts['body'].render(
                    "Press R to restart demo", True, (170, 170, 180))
                screen.blit(hint, (total_w // 2 - hint.get_width() // 2,
                                    total_h - 22))

            pygame.display.flip()
            clock.tick(sim_rate)

    finally:
        print("\nStopping robot processes...")
        for p in panels:
            p.stop_robot_processes()
            p.close_sockets()
        pygame.quit()
        print("Done.")


if __name__ == "__main__":
    main()
