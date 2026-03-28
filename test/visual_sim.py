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
import heapq
from collections import deque as _deque
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
# Network - MUST MATCH robot/reip_node.py and robot/baselines/raft_node.py
# Robots are spawned with --sim, so they use unique ports: base + robot_id
# In sim mode, robots send ALL peer messages to SIM_PEER_RELAY_PORT.
# The harness receives them, relays to each robot's peer port, and
# extracts state for the visualization.
UDP_POSITION_PORT = 5100          # Sim sends to base+i
UDP_PEER_PORT = 5200              # Robot i listens on base+i
UDP_FAULT_PORT = 5300             # Sim sends faults to base+i
SIM_MOTOR_PORT = 5400             # Robots send motor intents here (all to same port)
SIM_PEER_RELAY_PORT = 5500        # Robots send peer msgs here; harness relays
SIM_SENSOR_PORT = 5600            # Sim sends ToF/encoder data to base+i

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
        # Real foam divider: 35.7mm thick, two faces from y=0 to y=1200
        # (leaving 300mm passage at top)
        (1000, 0, 1000, 1200),
        (1036, 0, 1036, 1200),
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
# Exact starts captured from the clean hardware REIP no-fault run
# `trials/reip_none_t1_20260308_151155`.
START_PRESETS = {
    "hardware_clone_20260308": {
        1: (185.9, 382.1, 0.188),
        2: (792.9, 709.2, 2.255),
        3: (318.8, 147.4, 0.736),
        4: (768.7, 219.2, 1.577),
        5: (182.4, 709.9, 0.410),
    },
}
# Select layout via command line: python test/visual_sim.py 5 multiroom
ARENA_LAYOUT = "open"  # Default

# Display
WINDOW_WIDTH = 1100
WINDOW_HEIGHT = 900
MARGIN = 60
TITLE_HEIGHT = 40
STATS_HEIGHT = 110

# Colors - OG GridWorld matplotlib style
COLOR_UNKNOWN = (158, 195, 230)    # Light blue - unexplored
COLOR_EXPLORED = (255, 255, 255)   # White - explored/free
COLOR_OBSTACLE = (34, 34, 34)      # Dark - obstacles
COLOR_FRONTIER = (255, 230, 80)    # Yellow - frontier cells
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

# ============== ToF Sensor Simulation ==============
# Must match hardware_fidelity.py TOF_SENSOR_ANGLES_DEG
TOF_SENSOR_ANGLES_DEG = {
    "right": -75.0,
    "front_right": -37.5,
    "front": 0.0,
    "front_left": 37.5,
    "left": 75.0,
}
TOF_SENSOR_OFFSET_MM = 70.0   # Sensor origin distance from robot center
TOF_RANGE_MM = 200.0           # Max sensor range
BODY_RADIUS_MM = 77.0          # Peer body radius for ToF hits

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
    path_waypoints: list = None   # A* waypoints [(x,y), ...]
    path_target_key: tuple = None # (tx,ty) target that generated path

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
    """Simulation harness with Pygame visualization.

    Parameters
    ----------
    num_robots : int
        Number of robot nodes to spawn.
    layout : str
        Arena layout name (key in ARENA_LAYOUTS).
    start_preset : str | None
        Named starting positions from START_PRESETS.
    port_base : int
        Offset added to ALL UDP port numbers.  Allows two independent
        instances on the same machine (e.g. 0 for REIP, 2000 for Raft).
    robot_script : str | None
        Path to robot node script.  Defaults to ``robot/reip_node.py``.
    start_positions : dict | None
        Explicit {robot_id: (x, y, theta)} start positions.  Overrides
        start_preset and layout defaults.
    embedded : bool
        If True, do NOT create a Pygame display or call pygame.init().
        Instead render to an internal Surface of WINDOW_WIDTH x
        WINDOW_HEIGHT.  The caller owns the display and blits the
        surface wherever it likes.
    """

    def __init__(self, num_robots: int = NUM_ROBOTS, layout: str = "open",
                 start_preset: Optional[str] = None, *,
                 port_base: int = 0,
                 robot_script: Optional[str] = None,
                 start_positions: Optional[Dict[int, Tuple[float, float, float]]] = None,
                 embedded: bool = False):
        self.num_robots = num_robots
        self.walls = ARENA_LAYOUTS.get(layout, [])
        self.layout_name = layout
        self.start_preset = start_preset
        self.port_base = port_base
        self.robot_script = robot_script or "robot/reip_node.py"
        self.embedded = embedded

        # Pygame -- always ensure full init so fonts/display are ready
        if not embedded:
            pygame.init()
            self.screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
            pygame.display.set_caption(f"REIP GridWorld Simulation - {layout}")
        else:
            # Caller owns the display, but we NEED font + video subsystems
            if not pygame.get_init():
                pygame.init()
            if not pygame.font.get_init():
                pygame.font.init()
            self.screen = pygame.Surface((WINDOW_WIDTH, WINDOW_HEIGHT))

        self.font = pygame.font.SysFont('consolas', 13)
        self.font_id = pygame.font.SysFont('arial', 11, bold=True)
        self.font_large = pygame.font.SysFont('consolas', 16, bold=True)
        self.font_title = pygame.font.SysFont('consolas', 15, bold=True)
        self.clock = pygame.time.Clock()
        self.start_time = time.time()
        self.election_count = 0
        self._last_known_leader = None

        # Coordinate transform -- in embedded mode skip chrome for bigger grid
        if embedded:
            pad = 8
            draw_area_h = WINDOW_HEIGHT - 2 * pad
            self.scale = min(
                (WINDOW_WIDTH - 2 * pad) / ARENA_WIDTH,
                (draw_area_h) / ARENA_HEIGHT
            )
            self.offset_x = pad
            self.offset_y = pad
        else:
            draw_area_h = WINDOW_HEIGHT - TITLE_HEIGHT - STATS_HEIGHT
            self.scale = min(
                (WINDOW_WIDTH - 2 * MARGIN) / ARENA_WIDTH,
                (draw_area_h - 2 * 10) / ARENA_HEIGHT
            )
            self.offset_x = MARGIN
            self.offset_y = TITLE_HEIGHT + 10

        # Simulated robots
        self.sim_robots: Dict[int, SimRobot] = {}
        explicit = start_positions or START_PRESETS.get(start_preset, {})
        for i in range(1, num_robots + 1):
            if i in explicit:
                start_x, start_y, start_theta = explicit[i]
            elif layout in ("multiroom", "multiroom2"):
                col = (i - 1) % 2
                row = (i - 1) // 2
                start_x = 200 + col * 250
                start_y = 200 + row * 300
                start_theta = 0
            else:
                start_x = 200 + (i - 1) * 300
                start_y = ARENA_HEIGHT / 2
                start_theta = 0
            self.sim_robots[i] = SimRobot(
                robot_id=i,
                x=start_x, y=start_y, theta=start_theta,
                target_x=start_x, target_y=start_y,
                prev_x=start_x, prev_y=start_y
            )

        # Received states (from robot node broadcasts)
        self.robot_states: Dict[int, RobotState] = {}

        # Motor intents from robot nodes (the ACTUAL motor commands)
        self.motor_intents: Dict[int, Tuple[float, float]] = {}
        # Per-robot velocity state for differential drive integration
        self._motor_vel: Dict[int, List[float]] = {}  # rid -> [left_mm_s, right_mm_s]
        self._last_tick_time = time.time()

        # Coverage tracking
        self.visited_cells: Set[tuple] = set()
        self.current_leader_id = None

        # Sockets (port_base offsets every port)
        self.pos_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.pos_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        # Bind to the relay port -- robots send ALL sim-mode peer messages here.
        self.state_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.state_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.state_socket.bind(('127.0.0.1', SIM_PEER_RELAY_PORT + port_base))
        self.state_socket.setblocking(False)

        self.fault_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.fault_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        self.sensor_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sensor_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        # Motor socket -- robots send motor intents to SIM_MOTOR_PORT (not per-robot).
        # This is how isef_experiments.py receives motor commands; we match it exactly.
        self.motor_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.motor_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.motor_socket.bind(('127.0.0.1', SIM_MOTOR_PORT + port_base))
        self.motor_socket.setblocking(False)

        # Build full wall segment list for ToF ray-casting (arena bounds + layout walls)
        self._tof_wall_segments = [
            (0.0, 0.0, ARENA_WIDTH, 0.0),
            (ARENA_WIDTH, 0.0, ARENA_WIDTH, ARENA_HEIGHT),
            (ARENA_WIDTH, ARENA_HEIGHT, 0.0, ARENA_HEIGHT),
            (0.0, ARENA_HEIGHT, 0.0, 0.0),
        ] + [(float(x1), float(y1), float(x2), float(y2)) for x1, y1, x2, y2 in self.walls]

        # Robot processes
        self.robot_processes: List[subprocess.Popen] = []
        self.log_files: List = []

        self.running = False
    
    def arena_to_screen(self, x: float, y: float) -> Tuple[int, int]:
        sx = int(self.offset_x + x * self.scale)
        sy = int(self.offset_y + (ARENA_HEIGHT - y) * self.scale)
        return (sx, sy)
    
    def _line_circle_collision(self, x1, y1, x2, y2, cx, cy, r):
        """Check if circle (cx,cy,r) overlaps line segment (x1,y1)-(x2,y2).
        Tangent contact (distance == radius) is NOT collision - this prevents
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
    
    # -------------------- A* Pathfinding --------------------
    def _wall_blocks_move(self, cx, cy, nx, ny):
        """Check if moving between adjacent cells crosses a wall."""
        ax = (cx + 0.5) * CELL_SIZE
        ay = (cy + 0.5) * CELL_SIZE
        bx = (nx + 0.5) * CELL_SIZE
        by = (ny + 0.5) * CELL_SIZE
        return self._path_crosses_wall(ax, ay, bx, by)

    def _build_nav_grid(self):
        cols = ARENA_WIDTH // CELL_SIZE
        rows = ARENA_HEIGHT // CELL_SIZE
        return cols, rows

    def _astar_cell(self, start_cell, goal_cell, cols, rows):
        if start_cell == goal_cell:
            return [start_cell]
        sx, sy = start_cell
        gx, gy = goal_cell
        if not (0 <= sx < cols and 0 <= sy < rows):
            return None
        if not (0 <= gx < cols and 0 <= gy < rows):
            return None
        open_set = [(abs(gx - sx) + abs(gy - sy), 0, sx, sy)]
        came_from = {}
        g_score = {start_cell: 0}
        while open_set:
            _, g, cx, cy = heapq.heappop(open_set)
            if (cx, cy) == goal_cell:
                path = []
                cur = goal_cell
                while cur is not None:
                    path.append(cur)
                    cur = came_from.get(cur)
                path.reverse()
                return path
            if g > g_score.get((cx, cy), float('inf')):
                continue
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx, ny = cx + dx, cy + dy
                if (0 <= nx < cols and 0 <= ny < rows
                        and not self._wall_blocks_move(cx, cy, nx, ny)):
                    ng = g + 1
                    if ng < g_score.get((nx, ny), float('inf')):
                        g_score[(nx, ny)] = ng
                        h = abs(gx - nx) + abs(gy - ny)
                        heapq.heappush(open_set, (ng + h, ng, nx, ny))
                        came_from[(nx, ny)] = (cx, cy)
        return None

    def _get_path_waypoints(self, x, y, target_x, target_y):
        cols, rows = self._build_nav_grid()
        sc = (max(0, min(cols - 1, int(x / CELL_SIZE))),
              max(0, min(rows - 1, int(y / CELL_SIZE))))
        gc = (max(0, min(cols - 1, int(target_x / CELL_SIZE))),
              max(0, min(rows - 1, int(target_y / CELL_SIZE))))
        path = self._astar_cell(sc, gc, cols, rows)
        if path is None or len(path) <= 1:
            return [(target_x, target_y)]
        raw = []
        for i, (cx, cy) in enumerate(path[1:], 1):
            if i == len(path) - 1:
                raw.append((target_x, target_y))
            else:
                raw.append(((cx + 0.5) * CELL_SIZE, (cy + 0.5) * CELL_SIZE))
        if len(raw) <= 2:
            return raw
        smoothed = [raw[0]]
        i = 0
        while i < len(raw) - 1:
            farthest = i + 1
            for j in range(i + 2, len(raw)):
                if not self._path_crosses_wall(smoothed[-1][0], smoothed[-1][1],
                                                raw[j][0], raw[j][1]):
                    farthest = j
            smoothed.append(raw[farthest])
            i = farthest
        return smoothed

    def start_robot_processes(self):
        """Start robot node scripts as subprocesses."""
        script = self.robot_script
        tag = os.path.splitext(os.path.basename(script))[0]
        print(f"Starting {tag} processes (port_base={self.port_base})...")

        project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        log_dir = os.path.join(project_root, "logs", f"{tag}_pb{self.port_base}")
        os.makedirs(log_dir, exist_ok=True)

        for i in range(1, self.num_robots + 1):
            log_file = open(os.path.join(log_dir, f"robot_{i}.log"), 'w')
            cmd = [sys.executable, "-u", script, str(i), "--sim"]
            if self.port_base > 0:
                cmd += ["--port-base", str(self.port_base)]
            proc = subprocess.Popen(
                cmd, stdout=log_file, stderr=subprocess.STDOUT,
                cwd=project_root)
            self.robot_processes.append(proc)
            self.log_files.append(log_file)
            print(f"  Robot {i}: PID {proc.pid}")
        time.sleep(1)  # Let them initialize
    
    def stop_robot_processes(self):
        """Stop all robot processes"""
        for proc in self.robot_processes:
            try:
                proc.terminate()
            except Exception:
                pass
        for proc in self.robot_processes:
            try:
                proc.wait(timeout=3)
            except Exception:
                proc.kill()
        for lf in self.log_files:
            try:
                lf.close()
            except Exception:
                pass
        self.robot_processes.clear()
        self.log_files.clear()

    def close_sockets(self):
        """Close all UDP sockets."""
        for s in (self.pos_socket, self.state_socket, self.fault_socket,
                  self.sensor_socket, self.motor_socket):
            try:
                s.close()
            except Exception:
                pass

    def reset(self, start_positions: Optional[Dict[int, Tuple[float, float, float]]] = None):
        """Kill processes, wipe state, respawn.  Used by live demo restart."""
        self.stop_robot_processes()

        # Re-init sim robots to starting positions
        explicit = start_positions or START_PRESETS.get(self.start_preset, {})
        for i in range(1, self.num_robots + 1):
            r = self.sim_robots[i]
            if i in explicit:
                r.x, r.y, r.theta = explicit[i]
            elif self.layout_name in ("multiroom", "multiroom2"):
                col = (i - 1) % 2
                row = (i - 1) // 2
                r.x, r.y, r.theta = 200 + col * 250, 200 + row * 300, 0
            else:
                r.x, r.y, r.theta = 200 + (i - 1) * 300, ARENA_HEIGHT / 2, 0
            r.target_x, r.target_y = r.x, r.y
            r.prev_x, r.prev_y = r.x, r.y
            r.motor_fault = None
            r.is_bad_leader = False
            r.stuck_counter = 0
            r.stuck_override = None
            r.path_waypoints = None
            r.path_target_key = None

        # Clear tracking
        self.robot_states.clear()
        self.visited_cells.clear()
        self.current_leader_id = None
        self._last_known_leader = None
        self.election_count = 0
        self.start_time = time.time()
        self.motor_intents.clear()
        self._motor_vel.clear()
        self._last_tick_time = time.time()

        # Drain stale UDP from state and motor sockets
        for sock in (self.state_socket, self.motor_socket):
            try:
                while True:
                    sock.recvfrom(8192)
            except (BlockingIOError, OSError):
                pass

        # Respawn
        self.start_robot_processes()
    
    def send_start(self):
        """Send the 'start' command to every robot node so they begin exploring.
        Nodes gate motor output on trial_started, which is set by this message."""
        for rid in range(1, self.num_robots + 1):
            msg = {
                'type': 'fault_inject',
                'robot_id': 0,
                'fault': 'start',
                'timestamp': time.time(),
            }
            port = UDP_FAULT_PORT + self.port_base + rid
            try:
                self.fault_socket.sendto(
                    json.dumps(msg).encode(), ('127.0.0.1', port))
            except Exception:
                pass
        # Send twice for reliability
        time.sleep(0.05)
        for rid in range(1, self.num_robots + 1):
            msg = {
                'type': 'fault_inject',
                'robot_id': 0,
                'fault': 'start',
                'timestamp': time.time(),
            }
            port = UDP_FAULT_PORT + self.port_base + rid
            try:
                self.fault_socket.sendto(
                    json.dumps(msg).encode(), ('127.0.0.1', port))
            except Exception:
                pass

    # -------------------- ToF Sensor Simulation --------------------
    @staticmethod
    def _ray_segment_distance(ox, oy, dx, dy, x1, y1, x2, y2):
        """Distance from ray origin (ox,oy)+t*(dx,dy) to segment (x1,y1)-(x2,y2).
        Returns t >= 0 if hit, else None."""
        rx = x2 - x1
        ry = y2 - y1
        denom = dx * ry - dy * rx
        if abs(denom) < 1e-9:
            return None
        qx = x1 - ox
        qy = y1 - oy
        t = (qx * ry - qy * rx) / denom
        u = (qx * dy - qy * dx) / denom
        if t >= 0.0 and 0.0 <= u <= 1.0:
            return t
        return None

    @staticmethod
    def _ray_circle_distance(ox, oy, dx, dy, cx, cy, radius):
        """Distance from ray origin to circle surface.  Returns nearest t >= 0 or None."""
        fx = ox - cx
        fy = oy - cy
        a = dx * dx + dy * dy
        b = 2.0 * (fx * dx + fy * dy)
        c = fx * fx + fy * fy - radius * radius
        disc = b * b - 4.0 * a * c
        if disc < 0.0:
            return None
        sqrt_disc = math.sqrt(disc)
        t1 = (-b - sqrt_disc) / (2.0 * a)
        t2 = (-b + sqrt_disc) / (2.0 * a)
        hits = [t for t in (t1, t2) if t >= 0.0]
        return min(hits) if hits else None

    def _compute_tof_for_robot(self, rid: int) -> Dict[str, int]:
        """Cast rays from robot *rid* through the arena and return simulated
        VL53L0X ToF readings (mm) for each sensor direction."""
        robot = self.sim_robots[rid]
        readings: Dict[str, int] = {}
        for sensor_name, rel_deg in TOF_SENSOR_ANGLES_DEG.items():
            angle = robot.theta + math.radians(rel_deg)
            ox = robot.x + TOF_SENSOR_OFFSET_MM * math.cos(angle)
            oy = robot.y + TOF_SENSOR_OFFSET_MM * math.sin(angle)
            dx = math.cos(angle)
            dy = math.sin(angle)
            best = TOF_RANGE_MM + 1.0
            # Walls (arena bounds + layout walls)
            for seg in self._tof_wall_segments:
                dist = self._ray_segment_distance(ox, oy, dx, dy, *seg)
                if dist is not None and dist < best:
                    best = dist
            # Other robots
            for other_id, other in self.sim_robots.items():
                if other_id == rid:
                    continue
                dist = self._ray_circle_distance(
                    ox, oy, dx, dy, other.x, other.y, BODY_RADIUS_MM)
                if dist is not None and dist < best:
                    best = dist
            readings[sensor_name] = int(round(best)) if best <= TOF_RANGE_MM else 9999
        return readings

    def send_sensor_feedback(self):
        """Send simulated ToF + encoder data to each robot node.

        Mirrors isef_experiments.py so robot nodes get the same local-sensor
        stream they expect in --sim mode.  Encoders are zeroed because the
        visual sim drives motion externally (encoder-based dead reckoning is
        not used in sim).
        """
        now = time.time()
        for rid in self.sim_robots:
            msg = {
                'type': 'sim_sensor',
                'robot_id': rid,
                'tof': self._compute_tof_for_robot(rid),
                'encoders': [0, 0],
                'timestamp': now,
            }
            port = SIM_SENSOR_PORT + self.port_base + rid
            try:
                self.sensor_socket.sendto(
                    json.dumps(msg).encode(), ('127.0.0.1', port))
            except Exception:
                pass

    def send_positions(self):
        """Send mock positions to robots"""
        now = time.time()
        for rid, robot in self.sim_robots.items():
            msg = {
                'type': 'position',
                'robot_id': rid,
                'x': robot.x,
                'y': robot.y,
                'theta': robot.theta,
                'timestamp': now
            }
            data = json.dumps(msg).encode()
            robot_pos_port = UDP_POSITION_PORT + self.port_base + rid
            try:
                self.pos_socket.sendto(data, ('127.0.0.1', robot_pos_port))
            except Exception as e:
                print(f"[ERROR] Failed to send position to robot {rid}: {e}")
    
    def _find_reachable_unexplored(self, robot):
        """Find nearest unexplored cell by BFS path distance (not Euclidean)."""
        cols, rows = self._build_nav_grid()
        sc = (max(0, min(cols - 1, int(robot.x / CELL_SIZE))),
              max(0, min(rows - 1, int(robot.y / CELL_SIZE))))
        queue = _deque([sc])
        visited_bfs = {sc}
        while queue:
            cx, cy = queue.popleft()
            if (cx, cy) not in self.visited_cells:
                tx = (cx + 0.5) * CELL_SIZE
                ty = (cy + 0.5) * CELL_SIZE
                return (tx, ty)
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx, ny = cx + dx, cy + dy
                if (0 <= nx < cols and 0 <= ny < rows
                        and (nx, ny) not in visited_bfs
                        and not self._wall_blocks_move(cx, cy, nx, ny)):
                    visited_bfs.add((nx, ny))
                    queue.append((nx, ny))
        return None

    def receive_motor_intents(self):
        """Receive motor intent messages from robot nodes.

        Robot nodes publish (left_pwm, right_pwm) to SIM_MOTOR_PORT every
        tick.  This is the ACTUAL motor output the robot computed using its
        full REIP/Raft logic (trust, impeachment, ToF avoidance, stuck
        escape, pivot turns -- everything).  We store it and apply it in
        update_sim_robots() via differential-drive integration.
        """
        try:
            for _ in range(200):
                data, _ = self.motor_socket.recvfrom(4096)
                msg = json.loads(data.decode())
                if msg.get('type') != 'motor_intent':
                    continue
                rid = msg.get('robot_id')
                if rid:
                    self.motor_intents[rid] = (
                        float(msg.get('left', 0.0)),
                        float(msg.get('right', 0.0)),
                    )
        except (socket.timeout, BlockingIOError, ConnectionResetError, OSError):
            pass

    @staticmethod
    def _wrap_angle(a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

    def update_sim_robots(self):
        """Drive robot positions from actual motor intents using differential
        drive kinematics -- the SAME physics model as isef_experiments.py
        hardware_clone mode.

        Robot nodes compute motor commands based on their full REIP/Raft
        logic.  We just integrate those commands here.  No A* pathfinding,
        no stuck detection, no target chasing -- all of that lives in the
        robot nodes where it belongs.
        """
        now = time.time()
        dt = now - self._last_tick_time
        dt = min(dt, 0.15)  # Cap to prevent huge jumps
        self._last_tick_time = now

        # Simplified hardware_clone constants (from isef_experiments.py)
        PWM_TO_MM_S = 0.9
        WHEEL_BASE_MM = 130.0  # 2 * body_half_width_mm (~65mm)
        MAX_ACCEL_MM_S2 = 450.0
        ROBOT_RADIUS = 75.0

        for rid, robot in self.sim_robots.items():
            left_pwm, right_pwm = self.motor_intents.get(rid, (0.0, 0.0))

            # Initialize velocity state if needed
            if rid not in self._motor_vel:
                self._motor_vel[rid] = [0.0, 0.0]

            vel = self._motor_vel[rid]

            # Convert PWM to target velocity (mm/s)
            target_left = left_pwm * PWM_TO_MM_S
            target_right = right_pwm * PWM_TO_MM_S

            # Acceleration-limited velocity update
            def step_vel(current, target):
                delta = target - current
                limit = MAX_ACCEL_MM_S2 * dt
                if delta > limit:
                    delta = limit
                elif delta < -limit:
                    delta = -limit
                return current + delta

            vel[0] = step_vel(vel[0], target_left)
            vel[1] = step_vel(vel[1], target_right)

            # Sub-step integration (prevents wall tunneling)
            substeps = max(1, int(dt / 0.01))
            sub_dt = dt / substeps if substeps > 0 else 0.0

            x, y, theta = robot.x, robot.y, robot.theta
            for _ in range(substeps):
                linear_mm_s = 0.5 * (vel[0] + vel[1])
                angular_rad_s = (vel[1] - vel[0]) / max(WHEEL_BASE_MM, 1.0)
                new_theta = theta + angular_rad_s * sub_dt
                mid_theta = theta + 0.5 * angular_rad_s * sub_dt
                cand_x = x + linear_mm_s * math.cos(mid_theta) * sub_dt
                cand_y = y + linear_mm_s * math.sin(mid_theta) * sub_dt

                moved = False
                if not self._collides_with_wall(cand_x, cand_y, ROBOT_RADIUS):
                    x, y = cand_x, cand_y
                    moved = True
                else:
                    # Slide along axes
                    if not self._collides_with_wall(cand_x, y, ROBOT_RADIUS):
                        x = cand_x
                        moved = True
                    if not self._collides_with_wall(x, cand_y, ROBOT_RADIUS):
                        y = cand_y
                        moved = True
                if moved:
                    theta = new_theta
                else:
                    vel[0] *= 0.5
                    vel[1] *= 0.5
                    theta = new_theta

                theta = self._wrap_angle(theta)

            # Clamp to arena bounds
            x = max(ROBOT_RADIUS, min(ARENA_WIDTH - ROBOT_RADIUS, x))
            y = max(ROBOT_RADIUS, min(ARENA_HEIGHT - ROBOT_RADIUS, y))

            robot.x, robot.y, robot.theta = x, y, theta

            # Track coverage
            cell = (int(robot.x / CELL_SIZE), int(robot.y / CELL_SIZE))
            self.visited_cells.add(cell)
    
    def receive_states(self):
        """Receive robot broadcasts from the relay port, relay them to every
        other robot's individual peer port, and extract state for the display.

        In sim mode, all robot messages go to SIM_PEER_RELAY_PORT.  The harness
        must relay them so that robots can hear each other.

        KEY: Each robot broadcasts its navigation_target -- the position it
        actually decided to go to.  We store it in robot.target_x/y for
        drawing target lines.  Movement is driven by motor intents (see
        receive_motor_intents + update_sim_robots), NOT by this target.
        """
        try:
            for _ in range(200):  # drain up to 200 msgs per tick
                data, _ = self.state_socket.recvfrom(8192)
                msg = json.loads(data.decode())
                msg_type = msg.get('type')
                src = (msg.get('robot_id') or msg.get('voter_id')
                       or msg.get('leader_id') or msg.get('candidate_id'))

                # ---- Relay to every other robot's individual peer port ----
                for peer_id in range(1, self.num_robots + 1):
                    if peer_id == src:
                        continue
                    try:
                        self.state_socket.sendto(
                            data, ('127.0.0.1',
                                   UDP_PEER_PORT + self.port_base + peer_id))
                    except Exception:
                        pass

                # ---- Extract display state from peer_state messages ----
                if msg_type == 'peer_state':
                    rid = msg.get('robot_id')
                    if not rid:
                        continue
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

                    if msg.get('state') == 'leader':
                        self.current_leader_id = rid

                    # Store navigation target for drawing target lines.
                    # Movement is driven by motor intents, not this.
                    nav = msg.get('navigation_target')
                    if nav and rid in self.sim_robots:
                        robot = self.sim_robots[rid]
                        robot.target_x = nav[0]
                        robot.target_y = nav[1]
        except (BlockingIOError, ConnectionResetError, OSError):
            pass
    
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
        
        # Send to robot node
        msg = {
            'type': 'fault_inject',
            'robot_id': robot_id,
            'fault': fault_type,
            'timestamp': time.time()
        }
        robot_fault_port = UDP_FAULT_PORT + self.port_base + robot_id
        self.fault_socket.sendto(json.dumps(msg).encode(), ('127.0.0.1', robot_fault_port))
        print(f"Fault: {fault_type} on Robot {robot_id} (port_base={self.port_base})")
    
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
        """Draw the GridWorld visualization.

        In standalone mode: full chrome (title bar, stats panel, controls).
        In embedded mode: just the grid + walls + robots (no chrome) so the
        caller can overlay its own clean HUD.
        """
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

        # ---- Avg trust (expose for caller) ----
        trusts = [s.trust_in_leader for s in self.robot_states.values()
                  if s.state != 'leader']
        self._avg_trust = sum(trusts) / len(trusts) if trusts else 1.0

        # ===================== BACKGROUND =====================
        self.screen.fill(COLOR_PANEL_BG)

        # ===================== TITLE BAR (standalone only) =====================
        if not self.embedded:
            pygame.draw.rect(self.screen, (245, 245, 245),
                             (0, 0, WINDOW_WIDTH, TITLE_HEIGHT))
            pygame.draw.line(self.screen, (180, 180, 180),
                             (0, TITLE_HEIGHT - 1), (WINDOW_WIDTH, TITLE_HEIGHT - 1), 1)
            title_parts = [
                f"t={elapsed:.0f}",
                f"N={self.num_robots}",
                f"Leader={current_leader or '?'}",
                f"Trust={self._avg_trust:.2f}",
                f"Elections={self.election_count}",
                f"Layout={self.layout_name}",
            ]
            title_str = "  |  ".join(title_parts)
            title_surf = self.font_title.render(title_str, True, (30, 30, 30))
            self.screen.blit(title_surf,
                             (WINDOW_WIDTH // 2 - title_surf.get_width() // 2,
                              TITLE_HEIGHT // 2 - title_surf.get_height() // 2))

        # ===================== GRID WORLD =====================
        arena_x1, arena_y1_top = self.arena_to_screen(0, ARENA_HEIGHT)
        arena_x2, arena_y2_bot = self.arena_to_screen(ARENA_WIDTH, 0)
        arena_rect = pygame.Rect(arena_x1, arena_y1_top,
                                 arena_x2 - arena_x1, arena_y2_bot - arena_y1_top)
        pygame.draw.rect(self.screen, COLOR_UNKNOWN, arena_rect)

        frontier_cells = self._get_frontier_cells()

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
                pygame.draw.rect(self.screen, COLOR_GRID_LINE, rect, 1)

        pygame.draw.rect(self.screen, (80, 80, 80), arena_rect, 2)

        # ---- Walls ----
        drawn_walls = set()
        for i, (wx1, wy1, wx2, wy2) in enumerate(self.walls):
            if i in drawn_walls:
                continue
            for j, (bx1, by1, bx2, by2) in enumerate(self.walls):
                if j <= i or j in drawn_walls:
                    continue
                if wy1 == by1 and wy2 == by2 and abs(wx1 - bx1) < 100:
                    tl = self.arena_to_screen(min(wx1, bx1), max(wy2, by2))
                    br = self.arena_to_screen(max(wx1, bx1), min(wy1, by1))
                    wr = pygame.Rect(tl[0], tl[1],
                                     max(4, br[0] - tl[0]),
                                     max(4, br[1] - tl[1]))
                    pygame.draw.rect(self.screen, COLOR_WALL, wr)
                    drawn_walls.update((i, j))
                    break
            else:
                p1 = self.arena_to_screen(wx1, wy1)
                p2 = self.arena_to_screen(wx2, wy2)
                pygame.draw.line(self.screen, COLOR_WALL, p1, p2, 5)

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

            # Gold leader halo
            if is_leader:
                pygame.draw.circle(self.screen, COLOR_LEADER_GOLD, pos, 20, 3)
                pygame.draw.circle(self.screen, (255, 200, 0, 180), pos, 24, 2)

            # Trust warning ring
            if not is_leader and state.trust_in_leader < 0.6:
                ring_col = COLOR_TRUST_CRITICAL if state.trust_in_leader < 0.3 \
                    else COLOR_TRUST_LOW
                pygame.draw.circle(self.screen, ring_col, pos, 22, 3)

            # Robot body
            marker_size = 14 if is_leader else 11
            marker_color = COLOR_LEADER_GOLD if is_leader else color
            pygame.draw.circle(self.screen, marker_color, pos, marker_size)
            pygame.draw.circle(self.screen, (0, 0, 0), pos, marker_size, 2)

            # Heading line
            hx = int(pos[0] + (marker_size + 10) * math.cos(theta))
            hy = int(pos[1] - (marker_size + 10) * math.sin(theta))
            pygame.draw.line(self.screen, (0, 0, 0), pos, (hx, hy), 2)

            # Target line
            if rid in self.sim_robots:
                sr = self.sim_robots[rid]
                tp = self.arena_to_screen(sr.target_x, sr.target_y)
                target_color = (200, 50, 50) if is_leader else (80, 80, 80)
                pygame.draw.line(self.screen, target_color, pos, tp, 1)

            # ID label
            id_surf = self.font_id.render(str(rid), True, (0, 0, 0))
            id_w, id_h = id_surf.get_size()
            label_x = pos[0] + marker_size + 2
            label_y = pos[1] - id_h - 2
            bg_rect = pygame.Rect(label_x - 2, label_y - 1, id_w + 4, id_h + 2)
            pygame.draw.rect(self.screen, (255, 255, 255), bg_rect)
            pygame.draw.rect(self.screen, (0, 0, 0), bg_rect, 1)
            self.screen.blit(id_surf, (label_x, label_y))

            # Fault indicator (red badge)
            if rid in self.sim_robots:
                sr = self.sim_robots[rid]
                if sr.motor_fault or sr.is_bad_leader:
                    fault_label = "BAD" if sr.is_bad_leader else sr.motor_fault.upper()
                    ft = self.font.render(fault_label, True, (255, 255, 255))
                    fw, fh = ft.get_size()
                    badge_rect = pygame.Rect(pos[0] - fw // 2 - 3, pos[1] - 38,
                                            fw + 6, fh + 2)
                    pygame.draw.rect(self.screen, (200, 40, 40),
                                     badge_rect, border_radius=3)
                    self.screen.blit(ft, (pos[0] - fw // 2, pos[1] - 37))

        # ===================== STATS PANEL (standalone only) =====================
        if not self.embedded:
            panel_y = WINDOW_HEIGHT - STATS_HEIGHT
            pygame.draw.rect(self.screen, COLOR_PANEL_BG,
                             (0, panel_y, WINDOW_WIDTH, STATS_HEIGHT))
            pygame.draw.line(self.screen, (180, 180, 180),
                             (0, panel_y), (WINDOW_WIDTH, panel_y), 2)

            y = panel_y + 8
            total_cells = cells_x * cells_y
            cov_pct = len(self.visited_cells) / total_cells * 100
            cov_str = f"Coverage: {len(self.visited_cells)}/{total_cells} ({cov_pct:.1f}%)"
            cov_surf = self.font_large.render(cov_str, True, COLOR_TEXT)
            self.screen.blit(cov_surf, (20, y))

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

            fr_str = f"Frontiers: {len(frontier_cells)}"
            fr_surf = self.font.render(fr_str, True, (120, 120, 0))
            self.screen.blit(fr_surf, (bar_x + bar_w + 15, y + 2))
            y += 28

            x_pos = 20
            for rid in sorted(self.robot_states.keys()):
                st = self.robot_states[rid]
                color = AGENT_COLORS[(rid - 1) % len(AGENT_COLORS)]
                is_ldr = (st.state == 'leader') or (current_leader == rid)
                pygame.draw.circle(self.screen, color, (x_pos + 6, y + 8), 6)
                pygame.draw.circle(self.screen, (0, 0, 0), (x_pos + 6, y + 8), 6, 1)
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

            x_pos = 20
            for rid in sorted(self.robot_states.keys()):
                st = self.robot_states[rid]
                if st.suspicion > 0.01:
                    sus_color = COLOR_TRUST_CRITICAL if st.suspicion > 1.0 else (180, 120, 0)
                    sus_surf = self.font.render(
                        f"R{rid} sus={st.suspicion:.2f}", True, sus_color)
                    self.screen.blit(sus_surf, (x_pos, y))
                    x_pos += 140
            y += 18

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
        # Bootstrap positions + sensor data then unlock motor output
        for _ in range(30):
            self.send_positions()
            self.send_sensor_feedback()
            time.sleep(0.05)
        self.send_start()
        
        print("\nVisual simulation running!")
        print("Keys: 1-5 = DENSE_EXPLORED (send to visited cells)")
        print("      SHIFT+1-5 = SPIN (motor fault)")  
        print("      C = clear all faults, Q = quit\n")
        
        try:
            while self.running:
                # Handle input
                if not self.handle_events():
                    break
                
                # Update simulation -- motor-intent driven
                self.receive_motor_intents()
                self.update_sim_robots()
                self.send_positions()
                self.send_sensor_feedback()
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
    start_preset = sys.argv[3] if len(sys.argv) > 3 else None
    
    if layout not in ARENA_LAYOUTS:
        print(f"Unknown layout '{layout}'. Available: {', '.join(ARENA_LAYOUTS.keys())}")
        sys.exit(1)
    if start_preset is not None and start_preset not in START_PRESETS:
        print(f"Unknown start preset '{start_preset}'. Available: {', '.join(START_PRESETS.keys())}")
        sys.exit(1)
    
    print(f"REIP Visual Simulation - {num_robots} robots, layout: {layout}")
    print("=" * 50)
    if layout != "open":
        print(f"  Walls: {len(ARENA_LAYOUTS[layout])} segments")
    if start_preset:
        print(f"  Start preset: {start_preset}")
    
    sim = VisualSimulation(num_robots, layout, start_preset=start_preset)
    sim.run()

if __name__ == "__main__":
    main()
