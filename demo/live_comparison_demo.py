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
PLOT_H = 140
GAP = 10
MARGIN = 10
NUM_ROBOTS = 5
SPEED_MULTIPLIER = 5.0   # 5x real-time so judges see results in ~15s
REIP_COLOR = (0, 52, 204)
RAFT_COLOR = (227, 30, 52)


# ====================== Per-panel HUD overlay ======================
_hud_fonts = {}  # Cached fonts (created once after pygame.init)


def _get_hud_fonts():
    """Lazy-init fonts for the HUD (called after pygame.init)."""
    if not _hud_fonts:
        _hud_fonts['tag'] = pygame.font.SysFont('consolas', 18, bold=True)
        _hud_fonts['cov'] = pygame.font.SysFont('consolas', 14, bold=True)
    return _hud_fonts


def _draw_panel_hud(screen, x, y, w, h, panel, label, color, fonts, is_reip):
    """Draw a clean info overlay on top of a panel."""
    hf = _get_hud_fonts()
    total_cells = (2000 // 125) * (1500 // 125)
    cov_pct = len(panel.visited_cells) / total_cells * 100 if total_cells else 0

    # ---- Panel label badge (top center) ----
    tag_surf = hf['tag'].render(label, True, (255, 255, 255))
    tw, th = tag_surf.get_size()
    badge_x = x + w // 2 - tw // 2 - 10
    badge_y = y + 4
    badge_rect = pygame.Rect(badge_x, badge_y, tw + 20, th + 6)
    badge_bg = pygame.Surface((badge_rect.w, badge_rect.h), pygame.SRCALPHA)
    badge_bg.fill((*color, 210))
    screen.blit(badge_bg, badge_rect.topleft)
    screen.blit(tag_surf, (badge_x + 10, badge_y + 3))

    # ---- Coverage % (top-left) ----
    cov_surf = hf['cov'].render(f"{cov_pct:.0f}%", True, (255, 255, 255))
    cov_bg = pygame.Surface((cov_surf.get_width() + 10, cov_surf.get_height() + 4),
                            pygame.SRCALPHA)
    cov_bg.fill((0, 0, 0, 150))
    screen.blit(cov_bg, (x + 6, y + 6))
    screen.blit(cov_surf, (x + 11, y + 8))

    # ---- Leader + Elections (top-right) ----
    leader = panel.current_leader_id or "?"
    elec = panel.election_count
    info_str = f"L={leader}  E={elec}"
    info_surf = fonts['small'].render(info_str, True, (255, 255, 255))
    info_bg = pygame.Surface((info_surf.get_width() + 10, info_surf.get_height() + 4),
                             pygame.SRCALPHA)
    info_bg.fill((0, 0, 0, 150))
    screen.blit(info_bg, (x + w - info_surf.get_width() - 16, y + 6))
    screen.blit(info_surf, (x + w - info_surf.get_width() - 11, y + 8))

    # ---- Bottom bar: trust (REIP) or "No Trust Layer" (Raft) ----
    bar_h = 22
    bar_bg = pygame.Surface((w, bar_h), pygame.SRCALPHA)
    bar_bg.fill((0, 0, 0, 140))
    screen.blit(bar_bg, (x, y + h - bar_h))

    if is_reip:
        avg_trust = getattr(panel, '_avg_trust', 1.0)
        if avg_trust > 0.6:
            tc = (100, 220, 100)
        elif avg_trust > 0.3:
            tc = (255, 180, 50)
        else:
            tc = (255, 80, 80)
        trust_str = f"Avg Trust: {avg_trust:.2f}"
        trust_surf = fonts['small'].render(trust_str, True, tc)
        screen.blit(trust_surf, (x + 8, y + h - bar_h + 4))

        sus_robots = [(rid, s.suspicion) for rid, s in panel.robot_states.items()
                      if s.suspicion > 0.05]
        if sus_robots:
            sus_parts = [f"R{rid}:{s:.1f}" for rid, s in sorted(sus_robots)]
            sus_str = "Sus: " + " ".join(sus_parts)
            sus_color = (255, 120, 80) if any(s > 1.0 for _, s in sus_robots) \
                else (255, 200, 100)
            sus_surf = fonts['small'].render(sus_str, True, sus_color)
            screen.blit(sus_surf, (x + w // 2, y + h - bar_h + 4))
    else:
        raft_surf = fonts['small'].render("No Trust Layer", True, (180, 180, 190))
        screen.blit(raft_surf, (x + 8, y + h - bar_h + 4))


# ====================== Coverage history plot ======================
def _draw_coverage_plot(screen, rect, panels, labels, colors, elapsed, max_time,
                        fonts, fault_times=(10, 30)):
    """Draw a coverage-vs-time plot at the bottom of the window (dark theme)."""
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

    # Grid lines
    for pct in (0, 25, 50, 75, 100):
        y = inner.bottom - int((pct / 100.0) * inner.h)
        pygame.draw.line(screen, (55, 57, 65),
                         (inner.x, y), (inner.right, y), 1)
        lbl = fonts['small'].render(f"{pct}%", True, (120, 120, 130))
        screen.blit(lbl, (inner.x - lbl.get_width() - 4, y - 8))

    # Fault markers (red dashed vertical lines)
    for ft in fault_times:
        if ft <= elapsed:
            fx = inner.x + int((ft / max_time) * inner.w)
            for dy in range(inner.y, inner.bottom, 6):
                pygame.draw.line(screen, (200, 60, 60),
                                 (fx, dy), (fx, min(dy + 3, inner.bottom)), 1)
            fl = fonts['small'].render(f"F@{ft}s", True, (200, 80, 80))
            screen.blit(fl, (fx + 3, inner.y - 2))

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

    # Legend (top right of plot)
    lx = rect.right - 240
    ly = rect.y + 8
    for label, color in zip(labels, colors):
        pygame.draw.line(screen, color, (lx, ly + 8), (lx + 20, ly + 8), 3)
        lbl = fonts['small'].render(label, True, (190, 190, 200))
        screen.blit(lbl, (lx + 26, ly))
        lx += 100

    # Controls
    ctrl = "R: restart   F: fault   C: clear   Space: pause   Q: quit"
    cs = fonts['small'].render(ctrl, True, (90, 90, 100))
    screen.blit(cs, (inner.x, rect.bottom - 18))


# ====================== Overlay helpers ======================
_overlay_font = None


def _draw_overlay(screen, fonts, message, sub=""):
    global _overlay_font
    if _overlay_font is None:
        _overlay_font = pygame.font.SysFont('consolas', 32, bold=True)
    w, h = screen.get_size()
    screen.fill((30, 32, 38))
    surf = _overlay_font.render(message, True, (220, 220, 230))
    screen.blit(surf, (w // 2 - surf.get_width() // 2,
                        h // 2 - surf.get_height() // 2 - 16))
    if sub:
        ss = fonts['body'].render(sub, True, (140, 140, 150))
        screen.blit(ss, (w // 2 - ss.get_width() // 2,
                          h // 2 + 18))
    pygame.display.flip()


# ====================== Bootstrap with leader consensus ======================
def _bootstrap_and_wait(panels, fonts, screen, timeout=6.0):
    """Pump positions + relay + sensor feedback, then wait for leader consensus.

    Mirrors isef_experiments.py flow:
      1. Send positions continuously so nodes get real poses before elections
      2. Relay peer broadcasts so nodes can see each other
      3. Wait until each panel reports a stable leader (majority agreement)
      4. Send 'start' to unlock motor output

    Without this, nodes self-elect at (0,0) and the leader may not be the one
    we later fault-inject, making the demo meaningless.
    """
    deadline = time.time() + timeout

    # Phase 1: pump positions for 3s before the nodes even try elections
    pump_until = time.time() + 3.0
    while time.time() < pump_until:
        for p in panels:
            p.send_positions()
            p.send_sensor_feedback()
            p.receive_states()
        time.sleep(0.05)

    # Phase 2: keep pumping and check for leader consensus
    leaders_ok = [False] * len(panels)
    while time.time() < deadline and not all(leaders_ok):
        for i, p in enumerate(panels):
            p.send_positions()
            p.send_sensor_feedback()
            p.receive_states()
            if p.current_leader_id is not None:
                leaders_ok[i] = True
        time.sleep(0.05)

    for i, p in enumerate(panels):
        lbl = "REIP" if i == 0 else "RAFT"
        lid = p.current_leader_id
        if lid:
            print(f"  [{lbl}] Leader consensus: Robot {lid}")
        else:
            print(f"  [{lbl}] WARNING: no leader consensus before start")

    # Phase 3: send 'start' to unlock motor output
    for p in panels:
        p.send_start()
    time.sleep(0.1)
    # Double-send for reliability (matches isef_experiments)
    for p in panels:
        p.send_start()


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
    print(f"  Faults: t=10s on leader, t=30s on leader")
    print("=" * 55)

    # Init pygame BEFORE creating VisualSimulation instances (fonts need it)
    pygame.init()

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

    # Speed up playback so judges don't wait 2 minutes
    for p in panels:
        p.speed_multiplier = SPEED_MULTIPLIER

    # Coverage history (separate from VisualSimulation's internal state)
    for p in panels:
        p._cov_history = []

    # ---- Pygame window (auto-fit to screen) ----
    # Detect screen resolution so it fits on small laptops (e.g. 14" OmniBook)
    disp_info = pygame.display.Info()
    screen_w = disp_info.current_w
    screen_h = disp_info.current_h
    # Leave room for taskbar / title bar
    usable_w = int(screen_w * 0.95)
    usable_h = int(screen_h * 0.88)

    # Each panel renders to VS_W x VS_H internally. Scale to fit side-by-side.
    aspect = VS_W / VS_H
    max_panel_w = (usable_w - 2 * MARGIN - GAP) // 2
    max_panel_h = usable_h - HEADER_H - MARGIN - PLOT_H - MARGIN
    # Maintain aspect ratio -- constrained by width or height
    panel_h = min(max_panel_h, int(max_panel_w / aspect))
    panel_w = int(panel_h * aspect)
    total_w = 2 * panel_w + GAP + 2 * MARGIN
    total_h = HEADER_H + panel_h + MARGIN + PLOT_H + MARGIN
    print(f"  Window: {total_w}x{total_h}  (screen {screen_w}x{screen_h})")
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
    time.sleep(2.0)  # Let processes initialise (matches isef_experiments)

    # Bootstrap: pump positions + sensor data + relay, then wait for leader
    # consensus before sending 'start'. This mirrors what isef_experiments.py
    # does so that robots elect a leader at their real positions instead of
    # self-electing at (0,0).
    _draw_overlay(screen, fonts, "BOOTSTRAPPING...",
                  "Waiting for leader consensus")
    _bootstrap_and_wait(panels, fonts, screen, timeout=10.0)
    print("  Ready!\n")

    # ---- State ----
    sim_rate = SIM_RATE
    start_time = time.time()
    paused = False
    run_number = 1
    max_time = 60
    fault_schedule = [
        {"time": 10.0, "target": "leader", "fired": False},
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
                time.sleep(2.0)  # Let processes init (matches startup)
                _draw_overlay(screen, fonts, "BOOTSTRAPPING...",
                              "Waiting for leader consensus")
                _bootstrap_and_wait(panels, fonts, screen, timeout=10.0)
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
                        rid = p.current_leader_id or 1
                        p.inject_fault(rid, 'bad_leader')
                        print(f"  [AUTO] bad_leader on [{lbl}] R{rid} "
                              f"(leader) at t={elapsed:.1f}s")

            # ---- Simulation step ----
            if not paused:
                for p in panels:
                    p.receive_motor_intents()
                    p.update_sim_robots()
                    p.send_positions()
                    p.send_sensor_feedback()
                    p.receive_states()
                    # Record coverage for the bottom plot
                    total = ((2000 // 125) * (1500 // 125))
                    cov_pct = len(p.visited_cells) / total * 100 if total else 0
                    p._cov_history.append(cov_pct)

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

            # Each VisualSimulation draws to its own surface
            for p in panels:
                p.draw()

            # Scale and blit each panel
            scaled_reip = pygame.transform.smoothscale(
                reip_sim.screen, (panel_w, panel_h))
            scaled_raft = pygame.transform.smoothscale(
                raft_sim.screen, (panel_w, panel_h))
            screen.blit(scaled_reip, (left_x, panels_y))
            screen.blit(scaled_raft, (right_x, panels_y))

            # Thin colored border around each panel
            pygame.draw.rect(screen, REIP_COLOR,
                             (left_x - 2, panels_y - 2, panel_w + 4, panel_h + 4), 2,
                             border_radius=3)
            pygame.draw.rect(screen, RAFT_COLOR,
                             (right_x - 2, panels_y - 2, panel_w + 4, panel_h + 4), 2,
                             border_radius=3)

            # Clean per-panel HUD overlays
            _draw_panel_hud(screen, left_x, panels_y, panel_w, panel_h,
                            reip_sim, "REIP", REIP_COLOR, fonts, is_reip=True)
            _draw_panel_hud(screen, right_x, panels_y, panel_w, panel_h,
                            raft_sim, "RAFT", RAFT_COLOR, fonts, is_reip=False)

            # Coverage plot
            _draw_coverage_plot(screen, plot_rect, panels, labels, colors,
                                elapsed, max_time, fonts)

            # Restart hint
            if elapsed > 35:
                hint = fonts['body'].render(
                    "Press R to restart", True, (120, 120, 130))
                screen.blit(hint, (total_w // 2 - hint.get_width() // 2,
                                    total_h - 20))

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
