#!/usr/bin/env python3
"""
REIP Visual Simulation - All-in-One

Single command runs everything with visual debug:
- Spawns all robot nodes as threads
- Mock position server with simulated movement
- Pygame visualization showing robots, trust, coverage
- Keyboard fault injection

Usage: python test/visual_sim.py [num_robots]

Controls:
  1-5: Inject bad_leader fault on that robot
  SHIFT+1-5: Inject spin fault
  C: Clear all faults
  Q: Quit
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import socket
import json
import time
import math
import threading
import random
import subprocess
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Set

try:
    import pygame
    PYGAME_AVAILABLE = True
except ImportError:
    PYGAME_AVAILABLE = False
    print("pygame not installed. Install with: pip install pygame")
    sys.exit(1)

# ============== Configuration ==============
NUM_ROBOTS = 5
# Network - MUST MATCH robot/reip_node.py
# Robots are spawned with --sim, so they use unique ports: base + robot_id
# Simulator listens on base port (5200) to receive broadcasts and relay
UDP_POSITION_PORT = 5100   # Sim sends to 5100+i
UDP_PEER_PORT = 5200       # Sim listens on 5200, relays to 5200+i
UDP_FAULT_PORT = 5300      # Sim sends to 5300+i

ARENA_WIDTH = 2000
ARENA_HEIGHT = 1500
CELL_SIZE = 125

SIM_RATE = 20
# Moderate speed - fast enough to test, slow enough to be realistic
ROBOT_SPEED = 25  # ~500mm/sec

# ============== Arena Layouts ==============
# Each layout is a list of wall segments: (x1, y1, x2, y2) in mm
# where (x1,y1)-(x2,y2) defines a line segment that blocks robot movement
ARENA_LAYOUTS = {
    "open": [],  # No obstacles - baseline
    "multiroom": [
        # Divider wall at x=1000, from y=0 to y=1200 (leaving 300mm passage at top)
        (1000, 0, 1000, 1200),
    ],
    "multiroom2": [
        # Two-room: wall at x=1000, passage at top
        (1000, 0, 1000, 1200),
        # Small obstacle in left room
        (400, 600, 400, 900),
    ],
    "lshape": [
        # L-shaped: wall blocks direct path from bottom-left to top-right
        (800, 0, 800, 900),
        (800, 900, 2000, 900),
    ],
    "clustered": [
        # Large central obstacle forcing path-finding
        (700, 500, 1300, 500),
        (700, 500, 700, 1000),
        (1300, 500, 1300, 1000),
    ],
}
# Select layout via command line: python test/visual_sim.py 5 multiroom
ARENA_LAYOUT = "open"  # Default

# Display
WINDOW_WIDTH = 1100
WINDOW_HEIGHT = 900
MARGIN = 60
TITLE_HEIGHT = 40
STATS_HEIGHT = 110

# Colors — OG GridWorld matplotlib style
COLOR_UNKNOWN = (158, 195, 230)    # Light blue — unexplored
COLOR_EXPLORED = (255, 255, 255)   # White — explored/free
COLOR_OBSTACLE = (34, 34, 34)      # Dark — obstacles
COLOR_FRONTIER = (255, 230, 80)    # Yellow — frontier cells
COLOR_GRID_LINE = (190, 200, 210)  # Subtle gray grid lines
COLOR_WALL = (50, 50, 55)          # Dark wall segments
COLOR_BG = (158, 195, 230)         # Same as unknown (fills background)
COLOR_TEXT = (30, 30, 30)          # Dark text for light background
COLOR_TEXT_LIGHT = (255, 255, 255) # White text for dark backgrounds
COLOR_PANEL_BG = (245, 242, 235)   # Warm off-white panel

# Agent colors (matplotlib tab colors)
AGENT_COLORS = [
    (31, 119, 180),    # tab:blue
    (255, 127, 14),    # tab:orange
    (44, 160, 44),     # tab:green
    (214, 39, 40),     # tab:red
    (148, 103, 189),   # tab:purple
    (140, 86, 75),     # tab:brown
    (227, 119, 194),   # tab:pink
    (127, 127, 127),   # tab:gray
]
COLOR_LEADER_GOLD = (255, 215, 0)
COLOR_LOCAL_OBS = (51, 255, 51, 90)  # Translucent green for local observation
COLOR_TRUST_LOW = (255, 165, 0)
COLOR_TRUST_CRITICAL = (255, 80, 80)

# ============== Simulated Robot ==============
@dataclass
class SimRobot:
    robot_id: int
    x: float
    y: float
    theta: float
    target_x: float = 0
    target_y: float = 0
    motor_fault: Optional[str] = None  # spin, stop, erratic
    is_bad_leader: bool = False
    # Stuck detection: override unreachable targets
    stuck_counter: int = 0
    prev_x: float = 0.0
    prev_y: float = 0.0
    stuck_override: Optional[Tuple[float, float]] = None  # locked override target

# ============== Received State ==============
@dataclass 
class RobotState:
    robot_id: int
    x: float = 0
    y: float = 0
    theta: float = 0
    state: str = "idle"
    leader_id: Optional[int] = None
    trust_in_leader: float = 1.0
    suspicion: float = 0.0
    coverage_count: int = 0
    visited_cells: Set[tuple] = field(default_factory=set)
    predicted_target: Optional[Tuple[float, float]] = None
    commanded_target: Optional[Tuple[float, float]] = None
    last_update: float = 0

# ============== Visual Simulation ==============
class VisualSimulation:
    def __init__(self, num_robots: int = NUM_ROBOTS, layout: str = "open"):
        self.num_robots = num_robots
        self.walls = ARENA_LAYOUTS.get(layout, [])
        self.layout_name = layout
        
        # Pygame
        pygame.init()
        self.screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
        pygame.display.set_caption(f"REIP GridWorld Simulation — {layout}")
        self.font = pygame.font.SysFont('consolas', 13)
        self.font_id = pygame.font.SysFont('arial', 11, bold=True)
        self.font_large = pygame.font.SysFont('consolas', 16, bold=True)
        self.font_title = pygame.font.SysFont('consolas', 15, bold=True)
        self.clock = pygame.time.Clock()
        self.start_time = time.time()
        self.election_count = 0
        self._last_known_leader = None
        
        # Coordinate transform — leave room for title bar on top and stats at bottom
        draw_area_h = WINDOW_HEIGHT - TITLE_HEIGHT - STATS_HEIGHT
        self.scale = min(
            (WINDOW_WIDTH - 2 * MARGIN) / ARENA_WIDTH,
            (draw_area_h - 2 * 10) / ARENA_HEIGHT
        )
        self.offset_x = MARGIN
        self.offset_y = TITLE_HEIGHT + 10
        
        # Simulated robots (for position generation)
        # Start with NO target - robots wait for first leader assignment
        # This prevents startup race condition where robots explore randomly
        # before the leader can coordinate them
        self.sim_robots: Dict[int, SimRobot] = {}
        for i in range(1, num_robots + 1):
            if layout in ("multiroom", "multiroom2"):
                # Cluster all robots in Room A (left room, x < 1000)
                # This is critical for demonstrating leader coordination advantage
                col = (i - 1) % 2
                row = (i - 1) // 2
                start_x = 200 + col * 250
                start_y = 200 + row * 300
            else:
                start_x = 200 + (i - 1) * 300
                start_y = ARENA_HEIGHT / 2
            self.sim_robots[i] = SimRobot(
                robot_id=i,
                x=start_x,
                y=start_y,
                theta=0,
                target_x=start_x,  # Stay in place until assigned
                target_y=start_y,
                prev_x=start_x,
                prev_y=start_y
            )
        
        # Received states (from reip_node broadcasts)
        self.robot_states: Dict[int, RobotState] = {}
        
        # Coverage tracking
        self.visited_cells: Set[tuple] = set()
        self.current_leader_id = None
        
        # Sockets
        self.pos_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.pos_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        
        self.state_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.state_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.state_socket.bind(('', UDP_PEER_PORT))
        self.state_socket.setblocking(False)
        
        self.fault_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.fault_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        
        # Robot processes
        self.robot_processes: List[subprocess.Popen] = []
        
        self.running = False
    
    def arena_to_screen(self, x: float, y: float) -> Tuple[int, int]:
        sx = int(self.offset_x + x * self.scale)
        sy = int(self.offset_y + (ARENA_HEIGHT - y) * self.scale)
        return (sx, sy)
    
    def _line_circle_collision(self, x1, y1, x2, y2, cx, cy, r):
        """Check if circle (cx,cy,r) overlaps line segment (x1,y1)-(x2,y2).
        Tangent contact (distance == radius) is NOT collision — this prevents
        robots from deadlocking when sliding along a wall."""
        dx, dy = x2 - x1, y2 - y1
        fx, fy = x1 - cx, y1 - cy
        a = dx * dx + dy * dy
        if a < 1e-6:
            return math.hypot(cx - x1, cy - y1) < r
        b = 2 * (fx * dx + fy * dy)
        c = fx * fx + fy * fy - r * r
        disc = b * b - 4 * a * c
        if disc <= 0:  # <= : tangent is NOT collision (prevents wall-follow deadlock)
            return False
        disc = math.sqrt(disc)
        t1 = (-b - disc) / (2 * a)
        t2 = (-b + disc) / (2 * a)
        return (0 <= t1 <= 1) or (0 <= t2 <= 1) or (t1 < 0 and t2 > 1)
    
    def _collides_with_wall(self, x, y, radius=75):
        """Check if position (x,y) with given radius collides with any wall."""
        for wx1, wy1, wx2, wy2 in self.walls:
            if self._line_circle_collision(wx1, wy1, wx2, wy2, x, y, radius):
                return True
        return False
    
    @staticmethod
    def _ccw(ax, ay, bx, by, cx, cy):
        return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
    
    def _path_crosses_wall(self, x1, y1, x2, y2):
        """Check if line segment (x1,y1)-(x2,y2) crosses any wall segment.
        Used to detect if a target is on the other side of a wall."""
        for wx1, wy1, wx2, wy2 in self.walls:
            d1 = self._ccw(x1, y1, x2, y2, wx1, wy1)
            d2 = self._ccw(x1, y1, x2, y2, wx2, wy2)
            d3 = self._ccw(wx1, wy1, wx2, wy2, x1, y1)
            d4 = self._ccw(wx1, wy1, wx2, wy2, x2, y2)
            if ((d1 > 0 and d2 < 0) or (d1 < 0 and d2 > 0)) and \
               ((d3 > 0 and d4 < 0) or (d3 < 0 and d4 > 0)):
                return True
        return False
    
    def start_robot_processes(self):
        """Start reip_node.py as subprocesses"""
        print("Starting robot processes...")
        
        # Create logs directory for robot output
        log_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "logs")
        os.makedirs(log_dir, exist_ok=True)
        
        for i in range(1, self.num_robots + 1):
            # Write robot output to log file so we can see faults/trust
            log_file = open(os.path.join(log_dir, f"robot_{i}_console.log"), 'w')
            proc = subprocess.Popen(
                [sys.executable, "-u", "robot/reip_node.py", str(i), "--sim"],
                stdout=log_file,
                stderr=subprocess.STDOUT,
                cwd=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            )
            self.robot_processes.append(proc)
            print(f"  Robot {i}: PID {proc.pid} (log: logs/robot_{i}_console.log)")
        time.sleep(1)  # Let them initialize
    
    def stop_robot_processes(self):
        """Stop all robot processes"""
        for proc in self.robot_processes:
            proc.terminate()
        for proc in self.robot_processes:
            proc.wait(timeout=2)
        self.robot_processes = []
    
    def send_positions(self):
        """Send mock positions to robots"""
        for rid, robot in self.sim_robots.items():
            msg = {
                'type': 'position',
                'robot_id': rid,
                'x': robot.x,
                'y': robot.y,
                'theta': robot.theta,
                'timestamp': time.time()
            }
            data = json.dumps(msg).encode()
            # Each robot has unique position port: 5003 + robot_id
            robot_pos_port = UDP_POSITION_PORT + rid
            try:
                self.pos_socket.sendto(data, ('127.0.0.1', robot_pos_port))
            except Exception as e:
                print(f"[ERROR] Failed to send position to robot {rid}: {e}")
    
    def _find_reachable_unexplored(self, robot):
        """Find nearest unexplored cell whose center isn't blocked by a wall
        from the robot's current position.  Falls back to any unexplored cell
        if none are reachable (robot may need to route through passage)."""
        best_reachable = None
        best_rdist = float('inf')
        best_any = None
        best_adist = float('inf')
        for cx in range(ARENA_WIDTH // CELL_SIZE):
            for cy in range(ARENA_HEIGHT // CELL_SIZE):
                if (cx, cy) in self.visited_cells:
                    continue
                tx = (cx + 0.5) * CELL_SIZE
                ty = (cy + 0.5) * CELL_SIZE
                # Skip cells whose center is inside wall exclusion zone
                if self._collides_with_wall(tx, ty):
                    continue
                d = math.sqrt((tx - robot.x)**2 + (ty - robot.y)**2)
                if d < best_adist:
                    best_adist = d
                    best_any = (tx, ty)
                if not self._path_crosses_wall(robot.x, robot.y, tx, ty):
                    if d < best_rdist:
                        best_rdist = d
                        best_reachable = (tx, ty)
        return best_reachable or best_any

    def update_sim_robots(self):
        """Update simulated robot positions - follow REIP assignments.
        Includes stuck detection: if a robot makes no progress for
        STUCK_THRESHOLD frames, override its target with the nearest
        reachable unexplored cell."""
        STUCK_THRESHOLD = 20  # frames (~1 sec at 20 FPS)
        STUCK_MOVE_EPS = 15   # mm — less movement than this = stuck
        OVERRIDE_ARRIVE_DIST = 60  # close enough to release override lock

        for rid, robot in self.sim_robots.items():
            if robot.motor_fault == 'stop':
                continue
            elif robot.motor_fault == 'spin':
                robot.theta += 0.2
                continue
            elif robot.motor_fault == 'erratic':
                robot.x += random.uniform(-20, 20)
                robot.y += random.uniform(-20, 20)
                robot.theta += random.uniform(-0.3, 0.3)
            else:
                # ---- Check if override target reached ----
                if robot.stuck_override:
                    od = math.sqrt((robot.x - robot.stuck_override[0])**2 +
                                   (robot.y - robot.stuck_override[1])**2)
                    if od < OVERRIDE_ARRIVE_DIST:
                        robot.stuck_override = None
                        robot.stuck_counter = 0

                # ---- Stuck detection ----
                move_since = math.sqrt((robot.x - robot.prev_x)**2 +
                                       (robot.y - robot.prev_y)**2)
                if move_since < STUCK_MOVE_EPS:
                    robot.stuck_counter += 1
                else:
                    robot.stuck_counter = 0
                    robot.prev_x, robot.prev_y = robot.x, robot.y

                if robot.stuck_counter >= STUCK_THRESHOLD:
                    # Only override if robot is FAR from target (wall wedge).
                    # If robot arrived at target and sits there, that's not
                    # stuck — it's following orders.  Only REIP's trust model
                    # can recover from a bad command, not a physics hack.
                    target_dist = math.sqrt((robot.target_x - robot.x)**2 +
                                            (robot.target_y - robot.y)**2)
                    if target_dist > OVERRIDE_ARRIVE_DIST:
                        alt = self._find_reachable_unexplored(robot)
                        if alt:
                            robot.target_x, robot.target_y = alt
                            robot.stuck_override = alt
                    robot.stuck_counter = 0
                    robot.prev_x, robot.prev_y = robot.x, robot.y

                # ---- Normal movement ----
                dx = robot.target_x - robot.x
                dy = robot.target_y - robot.y
                dist = math.sqrt(dx*dx + dy*dy)
                
                if dist > 20:
                    robot.theta = math.atan2(dy, dx)
                    new_x = robot.x + ROBOT_SPEED * math.cos(robot.theta)
                    new_y = robot.y + ROBOT_SPEED * math.sin(robot.theta)
                    
                    # Wall collision: try to slide, then wall-follow
                    if self._collides_with_wall(new_x, new_y):
                        moved = False
                        # Try sliding along each axis independently
                        if not self._collides_with_wall(new_x, robot.y):
                            robot.x = new_x
                            moved = True
                        if not self._collides_with_wall(robot.x, new_y) and abs(new_y - robot.y) > 0.5:
                            robot.y = new_y
                            moved = True
                        
                        if not moved:
                            # Both axes blocked (target directly through wall).
                            # Wall-follow: try perpendicular directions, pick the
                            # one that moves closer to target.
                            perp1 = robot.theta + math.pi / 2
                            perp2 = robot.theta - math.pi / 2
                            options = []
                            for perp in [perp1, perp2]:
                                tx = robot.x + ROBOT_SPEED * math.cos(perp)
                                ty = robot.y + ROBOT_SPEED * math.sin(perp)
                                if (not self._collides_with_wall(tx, ty) and
                                        75 <= tx <= ARENA_WIDTH - 75 and
                                        75 <= ty <= ARENA_HEIGHT - 75):
                                    d = math.sqrt((tx - robot.target_x)**2 +
                                                  (ty - robot.target_y)**2)
                                    options.append((d, tx, ty))
                            if options:
                                options.sort()
                                _, robot.x, robot.y = options[0]
                    else:
                        robot.x = new_x
                        robot.y = new_y
                else:
                    # Reached target - find nearest unexplored cell as fallback
                    # (Leader should assign new target, but this prevents getting stuck)
                    alt = self._find_reachable_unexplored(robot)
                    if alt:
                        robot.target_x, robot.target_y = alt
            
            # Clamp to arena
            robot.x = max(75, min(ARENA_WIDTH - 75, robot.x))
            robot.y = max(75, min(ARENA_HEIGHT - 75, robot.y))
            
            # Track coverage
            cell = (int(robot.x / CELL_SIZE), int(robot.y / CELL_SIZE))
            self.visited_cells.add(cell)
    
    def receive_states(self):
        """Receive state broadcasts from robots and update targets.
        
        KEY: Each robot broadcasts its navigation_target — the position
        it actually decided to go to.  For a trusting follower this is
        the leader's assignment.  For a distrusting follower this is
        its local fallback frontier.  The visual sim drives physics
        based on this, so the display faithfully reflects REIP decisions.
        """
        try:
            while True:
                data, _ = self.state_socket.recvfrom(8192)
                msg = json.loads(data.decode())
                if msg.get('type') == 'peer_state':
                    rid = msg.get('robot_id')
                    if rid:
                        if rid not in self.robot_states:
                            self.robot_states[rid] = RobotState(robot_id=rid)
                        
                        state = self.robot_states[rid]
                        state.x = msg.get('x', 0)
                        state.y = msg.get('y', 0)
                        state.theta = msg.get('theta', 0)
                        state.state = msg.get('state', 'idle')
                        state.leader_id = msg.get('leader_id')
                        state.trust_in_leader = msg.get('trust_in_leader', 1.0)
                        state.suspicion = msg.get('suspicion', 0.0)
                        state.coverage_count = msg.get('coverage_count', 0)
                        state.last_update = time.time()
                        
                        for cell in msg.get('visited_cells', []):
                            state.visited_cells.add(tuple(cell))
                            self.visited_cells.add(tuple(cell))
                        
                        # Track leader
                        if msg.get('state') == 'leader':
                            self.current_leader_id = rid
                        
                        # Use the robot's ACTUAL navigation target to drive sim physics.
                        # This correctly reflects trust decisions: a follower that
                        # drops trust and goes autonomous will report its local
                        # fallback target, not the leader's bad assignment.
                        # BUT: don't overwrite if the harness has a stuck-override locked.
                        nav = msg.get('navigation_target')
                        if nav and rid in self.sim_robots:
                            robot = self.sim_robots[rid]
                            if not robot.stuck_override:
                                robot.target_x = nav[0]
                                robot.target_y = nav[1]
        except BlockingIOError:
            pass
        
        # NOTE: No relay needed — in sim mode each robot sends directly to every
        # other robot's unique peer port via broadcast_state().  The sim only
        # needs to *observe* messages on the base port (5200) for visualization.
    
    def inject_fault(self, robot_id: int, fault_type: str):
        """Inject fault"""
        # Update local sim
        if robot_id in self.sim_robots:
            if fault_type == 'bad_leader':
                self.sim_robots[robot_id].is_bad_leader = True
                self.sim_robots[robot_id].motor_fault = None
            elif fault_type in ('none', 'clear'):
                self.sim_robots[robot_id].motor_fault = None
                self.sim_robots[robot_id].is_bad_leader = False
            else:
                self.sim_robots[robot_id].motor_fault = fault_type
                self.sim_robots[robot_id].is_bad_leader = False
        
        # Send to reip_node
        msg = {
            'type': 'fault_inject',
            'robot_id': robot_id,
            'fault': fault_type,
            'timestamp': time.time()
        }
        # Each robot has unique fault port: 5005 + robot_id
        robot_fault_port = UDP_FAULT_PORT + robot_id
        self.fault_socket.sendto(json.dumps(msg).encode(), ('127.0.0.1', robot_fault_port))
        print(f"Fault: {fault_type} on Robot {robot_id}")
    
    def clear_all_faults(self):
        """Clear all faults"""
        for rid in self.sim_robots:
            self.inject_fault(rid, 'none')
    
    def _draw_dashed_circle(self, surface, color, center, radius, width=1, dash_len=8):
        """Draw a dashed circle (matplotlib-style dotted sensing radius)."""
        circumference = 2 * math.pi * radius
        n_dashes = max(8, int(circumference / (dash_len * 2)))
        for i in range(n_dashes):
            start_angle = (2 * math.pi * i) / n_dashes
            end_angle = (2 * math.pi * (i + 0.5)) / n_dashes
            # Draw arc segment
            rect = pygame.Rect(center[0] - radius, center[1] - radius,
                              radius * 2, radius * 2)
            if rect.width > 0 and rect.height > 0:
                pygame.draw.arc(surface, color, rect, start_angle, end_angle, width)

    def _get_frontier_cells(self):
        """Find frontier cells: unexplored cells adjacent to at least one explored cell."""
        cells_x = int(ARENA_WIDTH / CELL_SIZE)
        cells_y = int(ARENA_HEIGHT / CELL_SIZE)
        frontiers = set()
        for cx in range(cells_x):
            for cy in range(cells_y):
                if (cx, cy) in self.visited_cells:
                    continue  # Already explored
                # Check 4-neighbors for an explored cell
                for dx, dy in [(1,0),(-1,0),(0,1),(0,-1)]:
                    nx, ny = cx + dx, cy + dy
                    if (nx, ny) in self.visited_cells:
                        frontiers.add((cx, cy))
                        break
        return frontiers

    def draw(self):
        """Draw the GridWorld — OG matplotlib-style visualization."""
        cells_x = int(ARENA_WIDTH / CELL_SIZE)
        cells_y = int(ARENA_HEIGHT / CELL_SIZE)
        elapsed = time.time() - self.start_time

        # ---- Track leader changes for election counter ----
        current_leader = None
        for state in self.robot_states.values():
            if state.state == 'leader':
                current_leader = state.robot_id
                break
            if state.leader_id and current_leader is None:
                current_leader = state.leader_id
        if current_leader is not None and self._last_known_leader is not None \
                and current_leader != self._last_known_leader:
            self.election_count += 1
        if current_leader is not None:
            self._last_known_leader = current_leader

        # ---- Avg trust ----
        trusts = [s.trust_in_leader for s in self.robot_states.values()
                  if s.state != 'leader']
        avg_trust = sum(trusts) / len(trusts) if trusts else 1.0

        # ===================== TITLE BAR =====================
        pygame.draw.rect(self.screen, (245, 245, 245), (0, 0, WINDOW_WIDTH, TITLE_HEIGHT))
        pygame.draw.line(self.screen, (180, 180, 180),
                         (0, TITLE_HEIGHT - 1), (WINDOW_WIDTH, TITLE_HEIGHT - 1), 1)
        title_parts = [
            f"t={elapsed:.0f}",
            f"N={self.num_robots}",
            f"Leader={current_leader or '?'}",
            f"Trust={avg_trust:.2f}",
            f"Elections={self.election_count}",
            f"Layout={self.layout_name}",
        ]
        title_str = "  |  ".join(title_parts)
        title_surf = self.font_title.render(title_str, True, (30, 30, 30))
        self.screen.blit(title_surf,
                         (WINDOW_WIDTH // 2 - title_surf.get_width() // 2,
                          TITLE_HEIGHT // 2 - title_surf.get_height() // 2))

        # ===================== GRID WORLD =====================
        # Fill arena area with unknown color (light blue)
        arena_x1, arena_y1_top = self.arena_to_screen(0, ARENA_HEIGHT)
        arena_x2, arena_y2_bot = self.arena_to_screen(ARENA_WIDTH, 0)
        arena_rect = pygame.Rect(arena_x1, arena_y1_top,
                                 arena_x2 - arena_x1, arena_y2_bot - arena_y1_top)
        # Draw background around arena
        self.screen.fill(COLOR_PANEL_BG)
        # Title bar
        pygame.draw.rect(self.screen, (245, 245, 245), (0, 0, WINDOW_WIDTH, TITLE_HEIGHT))
        pygame.draw.line(self.screen, (180, 180, 180),
                         (0, TITLE_HEIGHT - 1), (WINDOW_WIDTH, TITLE_HEIGHT - 1), 1)
        self.screen.blit(title_surf,
                         (WINDOW_WIDTH // 2 - title_surf.get_width() // 2,
                          TITLE_HEIGHT // 2 - title_surf.get_height() // 2))

        # Arena background: light blue (unknown)
        pygame.draw.rect(self.screen, COLOR_UNKNOWN, arena_rect)

        # ---- Frontier detection ----
        frontier_cells = self._get_frontier_cells()

        # ---- Draw cells ----
        for cx in range(cells_x):
            for cy in range(cells_y):
                sx1, sy1 = self.arena_to_screen(cx * CELL_SIZE, (cy + 1) * CELL_SIZE)
                sx2, sy2 = self.arena_to_screen((cx + 1) * CELL_SIZE, cy * CELL_SIZE)
                w = sx2 - sx1
                h = sy2 - sy1
                rect = pygame.Rect(sx1, sy1, w, h)

                if (cx, cy) in self.visited_cells:
                    pygame.draw.rect(self.screen, COLOR_EXPLORED, rect)
                elif (cx, cy) in frontier_cells:
                    pygame.draw.rect(self.screen, COLOR_FRONTIER, rect)
                # else: stays unknown light blue

                # Grid line
                pygame.draw.rect(self.screen, COLOR_GRID_LINE, rect, 1)

        # ---- Arena border ----
        pygame.draw.rect(self.screen, (80, 80, 80), arena_rect, 2)

        # ---- Draw walls ----
        for wx1, wy1, wx2, wy2 in self.walls:
            p1 = self.arena_to_screen(wx1, wy1)
            p2 = self.arena_to_screen(wx2, wy2)
            pygame.draw.line(self.screen, COLOR_WALL, p1, p2, 5)

        # ===================== LOCAL OBSERVATION OVERLAY =====================
        # Translucent green around each robot's local observation area
        obs_radius_px = int(3 * CELL_SIZE * self.scale)  # ~3 cells sensing radius
        obs_surface = pygame.Surface((WINDOW_WIDTH, WINDOW_HEIGHT), pygame.SRCALPHA)
        for rid in range(1, self.num_robots + 1):
            if rid in self.sim_robots:
                sim = self.sim_robots[rid]
                pos = self.arena_to_screen(sim.x, sim.y)
                color = AGENT_COLORS[(rid - 1) % len(AGENT_COLORS)]
                pygame.draw.circle(obs_surface, (color[0], color[1], color[2], 30),
                                   pos, obs_radius_px)
        self.screen.blit(obs_surface, (0, 0))

        # ===================== AGENTS =====================
        for rid in range(1, self.num_robots + 1):
            state = self.robot_states.get(rid, RobotState(robot_id=rid))

            if rid in self.sim_robots:
                sim = self.sim_robots[rid]
                x, y, theta = sim.x, sim.y, sim.theta
            else:
                x, y, theta = state.x, state.y, state.theta

            pos = self.arena_to_screen(x, y)
            color = AGENT_COLORS[(rid - 1) % len(AGENT_COLORS)]
            is_leader = (state.state == 'leader') or (current_leader == rid)

            # ---- Dashed sensing/comm radius (matplotlib-style) ----
            comm_radius_px = int(5 * CELL_SIZE * self.scale)
            self._draw_dashed_circle(self.screen, (*color, ), pos,
                                     comm_radius_px, width=1, dash_len=6)

            # ---- Green field-of-view circle (solid, thin) ----
            fov_radius_px = int(2 * CELL_SIZE * self.scale)
            pygame.draw.circle(self.screen, (60, 180, 60), pos, fov_radius_px, 1)

            # ---- Gold leader halo ----
            if is_leader:
                pygame.draw.circle(self.screen, COLOR_LEADER_GOLD, pos, 20, 3)
                # Thicker outer ring
                pygame.draw.circle(self.screen, (255, 200, 0, 180), pos, 24, 2)

            # ---- Trust warning ring ----
            if not is_leader and state.trust_in_leader < 0.6:
                if state.trust_in_leader < 0.3:
                    ring_col = COLOR_TRUST_CRITICAL
                else:
                    ring_col = COLOR_TRUST_LOW
                pygame.draw.circle(self.screen, ring_col, pos, 22, 3)

            # ---- Robot body (filled circle + black edge) ----
            marker_size = 14 if is_leader else 11
            marker_color = COLOR_LEADER_GOLD if is_leader else color
            pygame.draw.circle(self.screen, marker_color, pos, marker_size)
            pygame.draw.circle(self.screen, (0, 0, 0), pos, marker_size, 2)

            # ---- Heading line ----
            hx = int(pos[0] + (marker_size + 10) * math.cos(theta))
            hy = int(pos[1] - (marker_size + 10) * math.sin(theta))
            pygame.draw.line(self.screen, (0, 0, 0), pos, (hx, hy), 2)

            # ---- Target line (thin, towards target) ----
            if rid in self.sim_robots:
                sr = self.sim_robots[rid]
                tp = self.arena_to_screen(sr.target_x, sr.target_y)
                target_color = (200, 50, 50) if is_leader else (80, 80, 80)
                pygame.draw.line(self.screen, target_color, pos, tp, 1)

            # ---- ID label (white box behind number, like matplotlib) ----
            id_surf = self.font_id.render(str(rid), True, (0, 0, 0))
            id_w, id_h = id_surf.get_size()
            label_x = pos[0] + marker_size + 2
            label_y = pos[1] - id_h - 2
            # White background box
            bg_rect = pygame.Rect(label_x - 2, label_y - 1, id_w + 4, id_h + 2)
            pygame.draw.rect(self.screen, (255, 255, 255), bg_rect)
            pygame.draw.rect(self.screen, (0, 0, 0), bg_rect, 1)
            self.screen.blit(id_surf, (label_x, label_y))

            # ---- Fault indicator (red badge) ----
            if rid in self.sim_robots:
                sr = self.sim_robots[rid]
                if sr.motor_fault or sr.is_bad_leader:
                    fault_label = "BAD_LEADER" if sr.is_bad_leader else sr.motor_fault.upper()
                    ft = self.font.render(fault_label, True, (255, 255, 255))
                    fw, fh = ft.get_size()
                    badge_rect = pygame.Rect(pos[0] - fw // 2 - 3, pos[1] - 38,
                                            fw + 6, fh + 2)
                    pygame.draw.rect(self.screen, (200, 40, 40), badge_rect, border_radius=3)
                    self.screen.blit(ft, (pos[0] - fw // 2, pos[1] - 37))

        # ===================== STATS PANEL (bottom) =====================
        panel_y = WINDOW_HEIGHT - STATS_HEIGHT
        pygame.draw.rect(self.screen, COLOR_PANEL_BG,
                         (0, panel_y, WINDOW_WIDTH, STATS_HEIGHT))
        pygame.draw.line(self.screen, (180, 180, 180),
                         (0, panel_y), (WINDOW_WIDTH, panel_y), 2)

        y = panel_y + 8

        # Coverage bar
        total_cells = cells_x * cells_y
        cov_pct = len(self.visited_cells) / total_cells * 100
        cov_str = f"Coverage: {len(self.visited_cells)}/{total_cells} ({cov_pct:.1f}%)"
        cov_surf = self.font_large.render(cov_str, True, COLOR_TEXT)
        self.screen.blit(cov_surf, (20, y))

        # Coverage bar visual
        bar_x, bar_w, bar_h = 350, 300, 16
        pygame.draw.rect(self.screen, (200, 200, 200),
                         (bar_x, y + 2, bar_w, bar_h), border_radius=3)
        fill_w = int(bar_w * min(cov_pct / 100.0, 1.0))
        bar_color = (44, 160, 44) if cov_pct > 50 else (255, 165, 0)
        if fill_w > 0:
            pygame.draw.rect(self.screen, bar_color,
                             (bar_x, y + 2, fill_w, bar_h), border_radius=3)
        pygame.draw.rect(self.screen, (100, 100, 100),
                         (bar_x, y + 2, bar_w, bar_h), 1, border_radius=3)

        # Frontiers count
        fr_str = f"Frontiers: {len(frontier_cells)}"
        fr_surf = self.font.render(fr_str, True, (120, 120, 0))
        self.screen.blit(fr_surf, (bar_x + bar_w + 15, y + 2))

        y += 28

        # Per-robot trust status
        x_pos = 20
        for rid in sorted(self.robot_states.keys()):
            st = self.robot_states[rid]
            color = AGENT_COLORS[(rid - 1) % len(AGENT_COLORS)]
            is_ldr = (st.state == 'leader') or (current_leader == rid)

            # Color dot
            pygame.draw.circle(self.screen, color, (x_pos + 6, y + 8), 6)
            pygame.draw.circle(self.screen, (0, 0, 0), (x_pos + 6, y + 8), 6, 1)

            # Trust value
            trust_color = COLOR_TEXT
            if st.trust_in_leader < 0.3:
                trust_color = COLOR_TRUST_CRITICAL
            elif st.trust_in_leader < 0.6:
                trust_color = COLOR_TRUST_LOW
            label = f"R{rid}: T={st.trust_in_leader:.2f}"
            if is_ldr:
                label += " [L]"
            t_surf = self.font.render(label, True, trust_color)
            self.screen.blit(t_surf, (x_pos + 16, y))
            x_pos += 160

        y += 22

        # Suspicion status
        x_pos = 20
        for rid in sorted(self.robot_states.keys()):
            st = self.robot_states[rid]
            if st.suspicion > 0.01:
                sus_color = COLOR_TRUST_CRITICAL if st.suspicion > 1.0 else (180, 120, 0)
                sus_surf = self.font.render(f"R{rid} sus={st.suspicion:.2f}", True, sus_color)
                self.screen.blit(sus_surf, (x_pos, y))
                x_pos += 140

        y += 18
        # Controls
        ctrl_surf = self.font.render(
            "1-5: bad_leader   SHIFT+1-5: spin   C: clear   Q: quit",
            True, (140, 140, 140))
        self.screen.blit(ctrl_surf, (20, y))

        pygame.display.flip()
    
    def handle_events(self) -> bool:
        """Handle keyboard input, return False to quit"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            
            if event.type == pygame.KEYDOWN:
                mods = pygame.key.get_mods()
                shift = mods & pygame.KMOD_SHIFT
                
                if event.key == pygame.K_q:
                    return False
                elif event.key == pygame.K_c:
                    self.clear_all_faults()
                elif event.key in (pygame.K_1, pygame.K_2, pygame.K_3, pygame.K_4, pygame.K_5):
                    rid = event.key - pygame.K_0
                    if rid <= self.num_robots:
                        if shift:
                            self.inject_fault(rid, 'spin')
                        else:
                            self.inject_fault(rid, 'bad_leader')
        
        return True
    
    def run(self):
        """Main loop"""
        self.running = True
        self.start_robot_processes()
        
        print("\nVisual simulation running!")
        print("Keys: 1-5 = DENSE_EXPLORED (send to visited cells)")
        print("      SHIFT+1-5 = SPIN (motor fault)")  
        print("      C = clear all faults, Q = quit\n")
        
        try:
            while self.running:
                # Handle input
                if not self.handle_events():
                    break
                
                # Update simulation
                self.update_sim_robots()
                self.send_positions()
                self.receive_states()
                
                # Draw
                self.draw()
                
                # Rate limit
                self.clock.tick(SIM_RATE)
        
        finally:
            print("\nStopping...")
            self.stop_robot_processes()
            pygame.quit()

# ============== Main ==============
def main():
    num_robots = int(sys.argv[1]) if len(sys.argv) > 1 else NUM_ROBOTS
    layout = sys.argv[2] if len(sys.argv) > 2 else "open"
    
    if layout not in ARENA_LAYOUTS:
        print(f"Unknown layout '{layout}'. Available: {', '.join(ARENA_LAYOUTS.keys())}")
        sys.exit(1)
    
    print(f"REIP Visual Simulation - {num_robots} robots, layout: {layout}")
    print("=" * 50)
    if layout != "open":
        print(f"  Walls: {len(ARENA_LAYOUTS[layout])} segments")
    
    sim = VisualSimulation(num_robots, layout)
    sim.run()

if __name__ == "__main__":
    main()
